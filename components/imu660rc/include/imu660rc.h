/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2022 SEEKFREE
 * ESP-IDF port of SeekFree zf_device_imu660rc, 2026.
 */
#pragma once

#include <stdint.h>
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
} imu660rc_sample_t;

/* Owns a dedicated SPI bus. Enables hardware SFLP at 120 Hz, like the
 * upstream demo. INT2 must be wired. Does not install a UART driver. */
esp_err_t imu660rc_init(const imu660rc_config_t *config, imu660rc_handle_t *out);

/* Call from one task only, never an ISR. Waits for INT2, then reads the latest
 * quaternion and raw sensors. On error, sample is unchanged. Older interrupts
 * may coalesce; this interface is for current attitude, not FIFO recording. */
esp_err_t imu660rc_read(imu660rc_handle_t dev, imu660rc_sample_t *sample,
                       uint32_t timeout_ms);

/* Call after the reading task has stopped. Releases device, bus and INT2. */
esp_err_t imu660rc_deinit(imu660rc_handle_t dev);

#ifdef __cplusplus
}
#endif
