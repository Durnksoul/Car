/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2022 SEEKFREE
 * ESP-IDF port of SeekFree zf_device_imu660rc, 2026.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct imu660rc_device *imu660rc_handle_t;

typedef struct {
    spi_host_device_t host;
    gpio_num_t sclk_gpio;
    gpio_num_t mosi_gpio;
    gpio_num_t miso_gpio;
    gpio_num_t cs_gpio;
    gpio_num_t int2_gpio;
    int clock_speed_hz;
} imu660rc_config_t;

#define IMU660RC_CONFIG_DEFAULT() { \
    .host = SPI2_HOST, .sclk_gpio = GPIO_NUM_12, \
    .mosi_gpio = GPIO_NUM_11, .miso_gpio = GPIO_NUM_13, \
    .cs_gpio = GPIO_NUM_10, .int2_gpio = GPIO_NUM_9, \
    .clock_speed_hz = 10000000 \
}

typedef struct {
    float roll_deg;                /* -180 .. 180, SeekFree axis convention */
    float pitch_deg;               /* -90 .. 90 */
    float yaw_deg;                 /* 0 .. 360, relative heading; may drift */
    int16_t acc_raw[3];            /* X/Y/Z, +/-8 g; divide by 4098.36 for g */
    int16_t gyro_raw[3];           /* X/Y/Z, +/-2000 dps; divide by 14.2857 */
    float tilt_deg;                /* projected +Y wheel-axis tilt, [-180,180) */
    float tilt_rate_dps;           /* matching projected rate; ~gyro Y near upright */
    bool tilt_valid;               /* defined projection, not balance enablement */
} imu660rc_sample_t;

/* Owns a dedicated SPI bus. Enables hardware SFLP FIFO at 120 Hz.
 * INT2 FIFO watermark (one record) must be wired. Does not install UART. */
esp_err_t imu660rc_init(const imu660rc_config_t *config, imu660rc_handle_t *out);

/* Call from one task only, never an ISR. Drains the current FIFO snapshot,
 * selects the newest SFLP record, and reads current raw sensors. Waits on INT2
 * if empty. On error sample is unchanged. Raw sensors and FIFO attitude are
 * not timestamp-synchronized. This interface is not a lossless FIFO recorder. */
esp_err_t imu660rc_read(imu660rc_handle_t dev, imu660rc_sample_t *sample,
                       uint32_t timeout_ms);

/* Call after the reading task has stopped. Releases device, bus and INT2. */
esp_err_t imu660rc_deinit(imu660rc_handle_t dev);

#ifdef __cplusplus
}
#endif
