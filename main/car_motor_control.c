#include "car_motor_control.h"
#include "car_encoder.h"
#include "sdkconfig.h"
#if CONFIG_CAR_BLE_UART
#include "car_ble_uart.h"
#endif
#include "car_command_stream.h"
#include <stdarg.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_timer.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tb6612_command.h"
#include "tb6612fng.h"

static const char *TAG = "car_motor";
/* Keep A as the forward reference; the user's mirrored B motor is reversed. */
static const int motor_direction[2] = {1, -1};

typedef struct {
    tb6612fng_handle_t motor;
    bool enabled;
    tb6612_command_kind_t mode[2];
    int percent[2];
} control_t;

#if CONFIG_CAR_BLE_UART
static bool reply_to_ble;
#endif
static car_encoder_sample_t encoder;
static int64_t encoder_us;
static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static car_motor_snapshot_t snapshot;

static void publish_snapshot(const control_t *control)
{
    car_motor_snapshot_t next = {
        .encoder = encoder, .encoder_us = encoder_us, .enabled = control->enabled,
        .command_percent = {control->percent[0], control->percent[1]},
        .braking = {control->mode[0] == TB_CMD_BRAKE, control->mode[1] == TB_CMD_BRAKE},
    };
    portENTER_CRITICAL(&snapshot_lock);
    snapshot = next;
    portEXIT_CRITICAL(&snapshot_lock);
}

bool car_motor_control_snapshot(car_motor_snapshot_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&snapshot_lock);
    *out = snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
    return out->encoder_us > 0;
}

static void reply(const char *format, ...)
{
    char text[200];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text) - 3, format, args);
    va_end(args);
    size_t used = strlen(text);
    memcpy(text + used, "\r\n", 3);
    printf("%s", text);
#if CONFIG_CAR_BLE_UART
    if (reply_to_ble) car_ble_uart_send(text);
#endif
}

static void print_encoder(void)
{
    reply("enc_left=%" PRId64 ",enc_right=%" PRId64 ",cps_left=%.1f,cps_right=%.1f,enc_us=%" PRId64,
          encoder.count[0], encoder.count[1], (double)encoder.cps[0],
          (double)encoder.cps[1], encoder_us);
}

static void print_help(void)
{
    reply("Commands (CR or LF ends each line; integer percentages):");
    reply("help | status | enc | enczero | enable | standby");
    reply("a <percent> | b <percent> | ab <A-percent> <B-percent> (-100..100)");
    reply("coast a|b|all | brake a|b|all");
    reply("Boot: STANDBY; send enable, then ab 20 20 to start both motors");
    reply("PWM=20kHz, 0=coast, +/-100=HIGH; holds until next command");
    reply("Logical forward: A normal, B reversed; encoder signs still require confirmation");
}

static void print_status(const control_t *control)
{
    reply("Commanded state: %s, STBY=%d (not a hardware measurement)",
             control->enabled ? "ENABLED" : "STANDBY", control->enabled);
    for (int i = 0; i < 2; ++i) {
        if (!control->enabled) {
            reply("%c: IN=00, PWM=LOW", 'A' + i);
        } else if (control->mode[i] == TB_CMD_BRAKE) {
            reply("%c: BRAKE, IN=11, PWM=LOW", 'A' + i);
        } else if (control->mode[i] == TB_CMD_COAST || control->percent[i] == 0) {
            reply("%c: COAST, IN=00, PWM=HIGH", 'A' + i);
        } else {
            int p = control->percent[i];
            int physical = p * motor_direction[i];
            reply("%c: %+d%%, IN=%s, PWM=%s", 'A' + i, p,
                     physical > 0 ? "10" : "01", (p == 100 || p == -100) ? "HIGH" : "20kHz");
        }
    }
}

static void clear_commands(control_t *control)
{
    for (int i = 0; i < 2; ++i) {
        control->mode[i] = TB_CMD_COAST;
        control->percent[i] = 0;
    }
}

static esp_err_t apply_command(control_t *control, const tb6612_command_t *command)
{
    esp_err_t err = ESP_OK;
    if (command->kind == TB_CMD_HELP) { print_help(); return ESP_OK; }
    if (command->kind == TB_CMD_STATUS) { print_status(control); print_encoder(); return ESP_OK; }
    if (command->kind == TB_CMD_ENCODER) { print_encoder(); return ESP_OK; }
    if (command->kind == TB_CMD_ENCODER_ZERO) {
        err = car_encoder_zero();
        if (err != ESP_OK) return err;
        encoder = (car_encoder_sample_t){0};
        encoder_us = esp_timer_get_time();
        print_encoder();
        return ESP_OK;
    }
    if (command->kind == TB_CMD_ENABLE || command->kind == TB_CMD_STANDBY) {
        bool enable = command->kind == TB_CMD_ENABLE;
        err = tb6612fng_enable(control->motor, enable);
        if (err == ESP_OK) {
            if (!enable || !control->enabled) clear_commands(control);
            control->enabled = enable;
        }
    } else {
        if (!control->enabled) return ESP_ERR_INVALID_STATE;
        if (command->kind == TB_CMD_SPEED && command->motor_mask == 3) {
            err = tb6612fng_set_speeds(control->motor,
                                     command->a_percent * 10 * motor_direction[0],
                                     command->b_percent * 10 * motor_direction[1]);
        } else {
            for (int i = 0; i < 2 && err == ESP_OK; ++i) {
                if (!(command->motor_mask & (1U << i))) continue;
                tb6612fng_motor_t motor = (tb6612fng_motor_t)i;
                if (command->kind == TB_CMD_SPEED) {
                    int p = i == 0 ? command->a_percent : command->b_percent;
                    err = tb6612fng_set_speed(control->motor, motor, p * 10 * motor_direction[i]);
                } else if (command->kind == TB_CMD_COAST) {
                    err = tb6612fng_coast(control->motor, motor);
                } else {
                    err = tb6612fng_brake(control->motor, motor);
                }
            }
        }
        if (err == ESP_OK) {
            for (int i = 0; i < 2; ++i) {
                if (!(command->motor_mask & (1U << i))) continue;
                control->mode[i] = command->kind;
                control->percent[i] = command->kind == TB_CMD_SPEED ?
                    (i == 0 ? command->a_percent : command->b_percent) : 0;
            }
        }
    }
    if (err != ESP_OK) {
        (void)tb6612fng_enable(control->motor, false);
        control->enabled = false;
        clear_commands(control);
    }
    return err;
}

static void receive_line(const char *line, void *argument)
{
    control_t *control = argument;
    if (!line) { reply("ERR: discarded line (too long or invalid character)"); return; }
    tb6612_command_t command;
    esp_err_t err = tb6612_command_parse(line, &command);
    if (err != ESP_OK) { reply("ERR: invalid command; send help"); return; }
    err = apply_command(control, &command);
    publish_snapshot(control);
    if (err == ESP_ERR_INVALID_STATE) reply("ERR: in standby; send enable first");
    else if (err != ESP_OK) reply("ERR: %s", esp_err_to_name(err));
    else reply("OK: %s", line);
}

static bool poll_input(int fd, control_t *control, car_command_stream_t *input)
{
    uint8_t buffer[64];
    ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
    if (count > 0) car_command_stream_feed(input, buffer, count, receive_line, control);
    return true;
}

static void motor_task(void *argument)
{
    control_t control = {.motor = argument};
    car_command_stream_t primary = {0}, secondary = {0};
#if CONFIG_CAR_BLE_UART
    car_command_stream_t bluetooth = {0};
    uint32_t ble_session = car_ble_uart_session();
#endif
    clear_commands(&control);
    int usb_fd = -1;
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_fd = open("/dev/secondary", O_RDONLY | O_NONBLOCK);
    if (usb_fd < 0) ESP_LOGW(TAG, "USB command input unavailable; use UART console");
#endif
    /* Every power-on/reset waits for explicit enable and speed commands. */
    const tb6612_command_t boot_standby = {.kind = TB_CMD_STANDBY};
    esp_err_t err = apply_command(&control, &boot_standby);
    if (err != ESP_OK) goto stopped;
    reply("Motor control ready: STANDBY, STBY=0; waiting for serial commands");
    publish_snapshot(&control);
    print_help();
    int64_t last_sample_us = esp_timer_get_time();
    int64_t last_report_us = last_sample_us;
    while (1) {
#if CONFIG_CAR_BLE_UART
        reply_to_ble = false;
#endif
        if (!poll_input(STDIN_FILENO, &control, &primary) ||
            (usb_fd >= 0 && !poll_input(usb_fd, &control, &secondary))) {
            err = ESP_FAIL;
            break;
        }
#if CONFIG_CAR_BLE_UART
        /* Temporarily disabled by CONFIG_CAR_BLE_UART; keep for later reuse. */
        car_ble_packet_t packet;
        /* Bound work per iteration; a flooding client cannot starve sampling. */
        reply_to_ble = true;
        for (int n = 0; n < 8 && car_ble_uart_receive(&packet); ++n) {
            uint32_t current = car_ble_uart_session();
            if (ble_session != current) {
                bluetooth = (car_command_stream_t){0};
                ble_session = current;
            }
            if (packet.session != current) continue;
            car_command_stream_feed(&bluetooth, packet.data, packet.length, receive_line, &control);
        }
#endif
        int64_t now = esp_timer_get_time();
        if (now - last_sample_us >= 20000) {
            err = car_encoder_sample(&encoder);
            if (err != ESP_OK) break;
            encoder_us = esp_timer_get_time();
            publish_snapshot(&control);
            last_sample_us = now;
        }
        if (now - last_report_us >= 200000) {
            print_encoder();
            last_report_us = now;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
stopped:
    (void)tb6612fng_enable(control.motor, false);
    control.enabled = false;
    clear_commands(&control);
    encoder_us = 0;
    publish_snapshot(&control);
    ESP_LOGE(TAG, "Control failed: %s; both motors commanded to standby", esp_err_to_name(err));
    if (usb_fd >= 0) close(usb_fd);
    car_encoder_deinit();
    (void)tb6612fng_deinit(control.motor);
    vTaskDelete(NULL);
}

esp_err_t car_motor_control_start(void)
{
    const tb6612fng_config_t config = TB6612FNG_CONFIG_DEFAULT();
    tb6612fng_handle_t motor = NULL;
    esp_err_t err = tb6612fng_init(&config, &motor);
    if (err != ESP_OK) return err;
    err = car_encoder_init();
    if (err != ESP_OK) { (void)tb6612fng_deinit(motor); return err; }
#if CONFIG_CAR_BLE_UART
    /* BLE startup is excluded while the external wireless UART is in use. */
    err = car_ble_uart_start();
    if (err != ESP_OK) ESP_LOGE(TAG, "BLE unavailable: %s; wired commands remain available", esp_err_to_name(err));
#endif
#if CONFIG_ESP_CONSOLE_UART
    /* Buffered RX while the IMU prints to the same console. */
    bool installed = false;
    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
        err = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 1024, 0, 0, NULL, 0);
        if (err != ESP_OK) goto failed;
        installed = true;
    }
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);
#endif
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        err = ESP_FAIL;
        goto failed;
    }
    if (xTaskCreate(motor_task, "car_motor", 6144, motor, 5, NULL) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto failed;
    }
    return ESP_OK;
failed:
#if CONFIG_ESP_CONSOLE_UART
    if (installed) {
        uart_vfs_dev_use_nonblocking(CONFIG_ESP_CONSOLE_UART_NUM);
        (void)uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);
    }
#endif
    car_encoder_deinit();
    (void)tb6612fng_deinit(motor);
    return err;
}
