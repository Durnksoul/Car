/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2022 SEEKFREE
 * ESP-IDF adaptation of RT1064 zf_device_imu660rc.c, 2026.
 * Upstream: https://gitee.com/seekfree/RT1064_Library
 * See README.md for reference revision and original GPL license.
 */
#include "imu660rc.h"
#include "imu660rc_math.h"
#include <stdlib.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

enum {
    REG_FUNC_CFG_ACCESS = 0x01, REG_PAGE_SEL = 0x02,
    REG_EMB_FUNC_EN_A = 0x04, REG_FIFO_CTRL1 = 0x07,
    REG_PAGE_ADDRESS = 0x08, REG_PAGE_VALUE = 0x09,
    REG_FIFO_CTRL4 = 0x0a, REG_INT2_CTRL = 0x0e,
    REG_WHO_AM_I = 0x0f, REG_CTRL1 = 0x10, REG_CTRL2 = 0x11,
    REG_CTRL3 = 0x12, REG_CTRL4 = 0x13, REG_CTRL6 = 0x15,
    REG_CTRL7 = 0x16, REG_CTRL8 = 0x17, REG_CTRL9 = 0x18,
    REG_OUTX_L_G = 0x22, REG_PAGE_RW = 0x17,
    REG_EMB_FUNC_FIFO_EN_A = 0x44, REG_SFLP_ODR = 0x5e,
    BANK_MAIN = 0x00, BANK_EMBEDDED = 0x80,
    SFLP_RATE_120HZ = 3,
};

static const char *TAG = "imu660rc";

struct imu660rc_device {
    spi_device_handle_t spi;
    SemaphoreHandle_t ready;
    imu660rc_config_t config;
    bool bus_initialized;
    bool isr_added;
    bool gpio_configured;
};

static void delay_ms(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

static esp_err_t write_reg(imu660rc_handle_t dev, uint8_t reg, uint8_t value)
{
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,
        .tx_data = {reg & 0x7f, value},
    };
    return spi_device_polling_transmit(dev->spi, &transaction);
}

static esp_err_t read_regs(imu660rc_handle_t dev, uint8_t reg,
                           uint8_t *data, size_t length)
{
    /* DMA is disabled; buffers include the command byte's discarded reply. */
    if (length == 0 || length > 12) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t tx[13] = {reg | 0x80};
    uint8_t rx[13] = {0};
    spi_transaction_t transaction = {
        .length = (length + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(dev->spi, &transaction);
    if (err == ESP_OK) {
        for (size_t i = 0; i < length; ++i) {
            data[i] = rx[i + 1];
        }
    }
    return err;
}

static void data_ready_isr(void *arg)
{
    imu660rc_handle_t dev = arg;
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(dev->ready, &wake);
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t configure_sensor(imu660rc_handle_t dev)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(write_reg(dev, REG_FUNC_CFG_ACCESS, BANK_MAIN), TAG, "main bank");
    for (int attempt = 0; attempt < 25; ++attempt) {
        ESP_RETURN_ON_ERROR(read_regs(dev, REG_WHO_AM_I, &id, 1), TAG, "read chip ID");
        if (id == 0x70) {
            break;
        }
        delay_ms(10);
    }
    if (id != 0x70) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02x, expected 0x70; check power and SPI wiring", id);
        return ESP_ERR_NOT_FOUND;
    }
    /* Preserve the upstream boot/reset and SFLP setup sequence. */
    ESP_RETURN_ON_ERROR(write_reg(dev, REG_FUNC_CFG_ACCESS, 0x04), TAG, "reset");
    delay_ms(30);
    const uint8_t main_setup[][2] = {
        {REG_CTRL3, 0x44},               /* BDU + register auto-increment */
        {REG_CTRL8, 0x02},               /* +/-8 g */
        {REG_CTRL6, 0x04},               /* +/-2000 dps */
        {REG_CTRL1, 0x15}, {REG_CTRL2, 0x18},
        {REG_CTRL7, 0x01}, {REG_CTRL9, 0x08}, /* LPF1 / LPF2 */
        {REG_FIFO_CTRL1, 0x01}, {REG_FIFO_CTRL4, 0x06},
        {REG_INT2_CTRL, 0x80}, {REG_CTRL4, 0x08}, /* SFLP INT2, pulsed */
        {REG_CTRL1, 0x10 | (SFLP_RATE_120HZ + 3)},
        {REG_CTRL2, 0x10 | (SFLP_RATE_120HZ + 3)},
        {REG_FUNC_CFG_ACCESS, BANK_EMBEDDED},
        {REG_EMB_FUNC_FIFO_EN_A, 0x02},
        {REG_SFLP_ODR, 0x43 | (SFLP_RATE_120HZ << 3)},
        {REG_EMB_FUNC_EN_A, 0x02},
        {REG_PAGE_RW, 0x00},
        {REG_FUNC_CFG_ACCESS, BANK_MAIN},
    };
    for (size_t i = 0; i < sizeof(main_setup) / sizeof(main_setup[0]); ++i) {
        ESP_RETURN_ON_ERROR(write_reg(dev, main_setup[i][0], main_setup[i][1]),
                            TAG, "configure register 0x%02x", main_setup[i][0]);
    }
    ESP_LOGI(TAG, "LSM6DSV16X ID=0x70, SPI mode 0 / %d Hz, SFLP 120 Hz",
             dev->config.clock_speed_hz);
    return ESP_OK;
}

esp_err_t imu660rc_init(const imu660rc_config_t *config, imu660rc_handle_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;
    if (config == NULL || config->host == SPI1_HOST ||
        config->clock_speed_hz <= 0 || config->clock_speed_hz > 10000000 ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->sclk_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->mosi_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->cs_gpio) ||
        !GPIO_IS_VALID_GPIO(config->miso_gpio) ||
        !GPIO_IS_VALID_GPIO(config->int2_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    const gpio_num_t pins[] = {config->sclk_gpio, config->mosi_gpio,
                              config->miso_gpio, config->cs_gpio, config->int2_gpio};
    for (int i = 0; i < 5; ++i) {
        for (int j = i + 1; j < 5; ++j) {
            if (pins[i] == pins[j]) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    imu660rc_handle_t dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->config = *config;
    esp_err_t err = ESP_ERR_NO_MEM;
    dev->ready = xSemaphoreCreateBinary();
    if (dev->ready == NULL) {
        goto fail;
    }
    spi_bus_config_t bus = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = config->miso_gpio,
        .sclk_io_num = config->sclk_gpio,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 13,
    };
    err = spi_bus_initialize(config->host, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        goto fail;
    }
    dev->bus_initialized = true;
    spi_device_interface_config_t device = {
        .clock_speed_hz = config->clock_speed_hz,
        .mode = 0, .spics_io_num = config->cs_gpio, .queue_size = 1,
    };
    err = spi_bus_add_device(config->host, &device, &dev->spi);
    if (err != ESP_OK) {
        goto fail;
    }
    gpio_config_t irq = {
        .pin_bit_mask = 1ULL << config->int2_gpio,
        .mode = GPIO_MODE_INPUT, .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&irq);
    if (err != ESP_OK) {
        goto fail;
    }
    dev->gpio_configured = true;
    /* The global GPIO ISR service can be shared with other project modules. */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        goto fail;
    }
    err = gpio_isr_handler_add(config->int2_gpio, data_ready_isr, dev);
    if (err != ESP_OK) {
        goto fail;
    }
    dev->isr_added = true;
    err = gpio_set_intr_type(config->int2_gpio, GPIO_INTR_POSEDGE);
    if (err != ESP_OK) {
        goto fail;
    }
    err = gpio_intr_enable(config->int2_gpio);
    if (err != ESP_OK) {
        goto fail;
    }
    delay_ms(10);
    err = configure_sensor(dev);
    if (err != ESP_OK) {
        goto fail;
    }
    *out = dev;
    return ESP_OK;
fail:
    imu660rc_deinit(dev);
    return err;
}

static esp_err_t read_quaternion(imu660rc_handle_t dev, uint8_t data[8])
{
    esp_err_t err = write_reg(dev, REG_FUNC_CFG_ACCESS, BANK_EMBEDDED);
    if (err != ESP_OK) {
        return err;
    }
    err = write_reg(dev, REG_PAGE_RW, 0x20);
    if (err == ESP_OK) {
        err = write_reg(dev, REG_PAGE_SEL, 0x31);
    }
    for (int i = 0; i < 8 && err == ESP_OK; ++i) {
        err = write_reg(dev, REG_PAGE_ADDRESS, 0x4c + i);
        if (err == ESP_OK) {
            err = read_regs(dev, REG_PAGE_VALUE, &data[i], 1);
        }
    }
    /* Always try to leave page access and return to the main register bank. */
    esp_err_t page_err = write_reg(dev, REG_PAGE_RW, 0x00);
    esp_err_t bank_err = write_reg(dev, REG_FUNC_CFG_ACCESS, BANK_MAIN);
    return err != ESP_OK ? err : (page_err != ESP_OK ? page_err : bank_err);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

esp_err_t imu660rc_read(imu660rc_handle_t dev, imu660rc_sample_t *sample,
                       uint32_t timeout_ms)
{
    if (dev == NULL || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms > 0 && ticks == 0) {
        ticks = 1;
    }
    if (xSemaphoreTake(dev->ready, ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t quaternion[8];
    ESP_RETURN_ON_ERROR(read_quaternion(dev, quaternion), TAG, "read quaternion");
    float angles[3];
    if (!imu660rc_decode_angles(quaternion, angles)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t raw[12];
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_OUTX_L_G, raw, sizeof(raw)), TAG, "read sensors");
    imu660rc_sample_t next = {
        .roll_deg = angles[0], .pitch_deg = angles[1], .yaw_deg = angles[2],
    };
    for (int i = 0; i < 3; ++i) {
        next.gyro_raw[i] = read_i16_le(&raw[i * 2]);
        next.acc_raw[i] = read_i16_le(&raw[6 + i * 2]);
    }
    *sample = next;
    return ESP_OK;
}

esp_err_t imu660rc_deinit(imu660rc_handle_t dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = ESP_OK;
    if (dev->gpio_configured) {
        gpio_intr_disable(dev->config.int2_gpio);
    }
    if (dev->isr_added) {
        gpio_isr_handler_remove(dev->config.int2_gpio);
    }
    if (dev->gpio_configured) {
        gpio_reset_pin(dev->config.int2_gpio);
    }
    if (dev->spi != NULL) {
        /* Stop SFLP interrupts before releasing the SPI device. */
        write_reg(dev, REG_FUNC_CFG_ACCESS, BANK_MAIN);
        write_reg(dev, REG_INT2_CTRL, 0x00);
        result = spi_bus_remove_device(dev->spi);
    }
    if (dev->bus_initialized) {
        esp_err_t err = spi_bus_free(dev->config.host);
        if (result == ESP_OK) {
            result = err;
        }
    }
    if (dev->ready != NULL) {
        vSemaphoreDelete(dev->ready);
    }
    free(dev);
    return result;
}
