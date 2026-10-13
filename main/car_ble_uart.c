#include "car_ble_uart.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "car_ble";
/* Nordic UART Service UUIDs, stored least significant byte first. */
#define NUS_UUID(id) BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0, \
                                    0x93,0xf3,0xa3,0xb5,id,0x00,0x40,0x6e)
static const ble_uuid128_t service_uuid = NUS_UUID(0x01);
static const ble_uuid128_t rx_uuid = NUS_UUID(0x02);
static const ble_uuid128_t tx_uuid = NUS_UUID(0x03);
static QueueHandle_t rx_queue;
static uint8_t own_addr_type;
static uint16_t tx_handle;
static uint16_t connection = BLE_HS_CONN_HANDLE_NONE;
static uint16_t mtu = 23;
static bool subscribed;
static uint32_t session;
/* Access callbacks and GAP callbacks run in the same NimBLE host task. */
static bool discard_until_newline;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

uint32_t car_ble_uart_session(void)
{
    portENTER_CRITICAL(&state_lock);
    uint32_t value = session;
    portEXIT_CRITICAL(&state_lock);
    return value;
}

static void invalidate_stream(void)
{
    discard_until_newline = true;
    portENTER_CRITICAL(&state_lock);
    ++session;
    portEXIT_CRITICAL(&state_lock);
}

static int access_rx(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctx, void *arg)
{
    (void)conn; (void)attr; (void)arg;
    if (ctx->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    car_ble_packet_t packet = {.session = car_ble_uart_session()};
    uint16_t length = OS_MBUF_PKTLEN(ctx->om);
    if (length > sizeof(packet.data)) {
        invalidate_stream();
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ble_hs_mbuf_to_flat(ctx->om, packet.data, sizeof(packet.data), &packet.length) != 0) {
        invalidate_stream();
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (discard_until_newline) {
        unsigned offset = 0;
        while (offset < packet.length && packet.data[offset] != '\r' && packet.data[offset] != '\n')
            ++offset;
        if (offset == packet.length) return 0;
        discard_until_newline = false;
        ++offset;
        packet.length -= offset;
        memmove(packet.data, packet.data + offset, packet.length);
        if (!packet.length) return 0;
    }
    if (xQueueSend(rx_queue, &packet, 0) != pdTRUE) {
        invalidate_stream();
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

static int access_tx(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctx, void *arg)
{
    (void)conn; (void)attr; (void)ctx; (void)arg;
    return 0;
}

static const struct ble_gatt_svc_def services[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &service_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]) {
        {.uuid = &rx_uuid.u, .access_cb = access_rx,
         .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
        {.uuid = &tx_uuid.u, .access_cb = access_tx,
         .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &tx_handle},
        {0},
     }},
    {0},
};

static int gap_event(struct ble_gap_event *event, void *arg);

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .uuids128 = (ble_uuid128_t *)&service_uuid, .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    int rc = ble_gap_adv_set_fields(&fields);
    const char *name = ble_svc_gap_device_name();
    struct ble_hs_adv_fields response = {
        .name = (uint8_t *)name, .name_len = strlen(name), .name_is_complete = 1,
    };
    if (rc == 0) rc = ble_gap_adv_rsp_set_fields(&response);
    const struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND, .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    if (rc == 0) rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                                      &params, gap_event, NULL);
    if (rc != 0) ESP_LOGE(TAG, "Advertising failed: %d", rc);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            discard_until_newline = false;
            portENTER_CRITICAL(&state_lock);
            connection = event->connect.conn_handle;
            subscribed = false;
            mtu = 23;
            ++session;
            portEXIT_CRITICAL(&state_lock);
            ESP_LOGI(TAG, "BLE connected");
        } else advertise();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        portENTER_CRITICAL(&state_lock);
        connection = BLE_HS_CONN_HANDLE_NONE;
        subscribed = false;
        ++session;
        portEXIT_CRITICAL(&state_lock);
        ESP_LOGI(TAG, "BLE disconnected; motor command retained");
        advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == tx_handle) {
            portENTER_CRITICAL(&state_lock);
            subscribed = event->subscribe.cur_notify;
            portEXIT_CRITICAL(&state_lock);
        }
        break;
    case BLE_GAP_EVENT_MTU:
        portENTER_CRITICAL(&state_lock);
        mtu = event->mtu.value;
        portEXIT_CRITICAL(&state_lock);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc == 0) advertise();
    else ESP_LOGE(TAG, "BLE address init failed: %d", rc);
}

static void on_reset(int reason)
{
    portENTER_CRITICAL(&state_lock);
    connection = BLE_HS_CONN_HANDLE_NONE;
    subscribed = false;
    ++session;
    portEXIT_CRITICAL(&state_lock);
    ESP_LOGE(TAG, "BLE host reset: %d", reason);
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t car_ble_uart_start(void)
{
    esp_err_t err = nvs_flash_init();
    /* Preserve NVS on errors instead of silently erasing existing data. */
    if (err != ESP_OK) return err;
    rx_queue = xQueueCreate(8, sizeof(car_ble_packet_t));
    if (!rx_queue) return ESP_ERR_NO_MEM;
    err = nimble_port_init();
    if (err != ESP_OK) goto failed;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_svc_gap_device_name_set("Car-ESP32S3");
    if (rc == 0) rc = ble_gatts_count_cfg(services);
    if (rc == 0) rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        (void)nimble_port_deinit();
        err = ESP_FAIL;
        goto failed;
    }
    (void)ble_att_set_preferred_mtu(247);
    nimble_port_freertos_init(host_task);
    return ESP_OK;
failed:
    vQueueDelete(rx_queue);
    rx_queue = NULL;
    return err;
}

bool car_ble_uart_receive(car_ble_packet_t *out)
{
    return rx_queue && xQueueReceive(rx_queue, out, 0) == pdTRUE;
}

void car_ble_uart_send(const char *text)
{
    portENTER_CRITICAL(&state_lock);
    uint16_t conn = connection, payload = mtu - 3;
    bool ready = subscribed;
    uint32_t generation = session;
    portEXIT_CRITICAL(&state_lock);
    if (!ready || conn == BLE_HS_CONN_HANDLE_NONE) return;
    size_t remaining = strlen(text);
    while (remaining) {
        if (generation != car_ble_uart_session()) return;
        size_t length = remaining < payload ? remaining : payload;
        struct os_mbuf *buffer = ble_hs_mbuf_from_flat(text, length);
        if (!buffer) return;
        /* NimBLE consumes the mbuf even if notifying fails. */
        if (ble_gatts_notify_custom(conn, tx_handle, buffer) != 0) return;
        remaining -= length;
        text += length;
    }
}
