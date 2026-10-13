#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tb6612fng_device *tb6612fng_handle_t;

typedef enum {
    TB6612FNG_MOTOR_A = 0,
    TB6612FNG_MOTOR_B = 1,
} tb6612fng_motor_t;

typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    gpio_num_t pwm_gpio;
    ledc_channel_t ledc_channel;
} tb6612fng_motor_config_t;

typedef struct {
    tb6612fng_motor_config_t motor[2];
    gpio_num_t stby_gpio;
    ledc_timer_t ledc_timer;
    uint32_t pwm_freq_hz;
} tb6612fng_config_t;

/* ESP32-S3 defaults, separate from this project's IMU GPIO9..13.
 * Owns seven GPIOs, two low-speed LEDC channels and one dedicated timer.
 * Do not share these resources with another driver. Resolution is 10 bits. */
#define TB6612FNG_CONFIG_DEFAULT() { \
    .motor = { \
        { .in1_gpio = GPIO_NUM_4, .in2_gpio = GPIO_NUM_5, \
          .pwm_gpio = GPIO_NUM_6, .ledc_channel = LEDC_CHANNEL_0 }, \
        { .in1_gpio = GPIO_NUM_7, .in2_gpio = GPIO_NUM_8, \
          .pwm_gpio = GPIO_NUM_15, .ledc_channel = LEDC_CHANNEL_1 } \
    }, \
    .stby_gpio = GPIO_NUM_16, .ledc_timer = LEDC_TIMER_0, \
    .pwm_freq_hz = 20000 \
}

/* All calls must be serialized by the caller, from task context, never an ISR.
 * Initialization holds STBY low. No motor runs until enabled and commanded. */
esp_err_t tb6612fng_init(const tb6612fng_config_t *config,
                       tb6612fng_handle_t *out);

/* Disable clears both motor commands; enabling starts with both coasting.
 * Repeated enable(true) leaves existing commands unchanged. */
esp_err_t tb6612fng_enable(tb6612fng_handle_t dev, bool enable);

/* Requires enable(true). Signed PWM command -1000..1000 (per mille),
 * positive: IN1=1/IN2=0; negative: IN1=0/IN2=1. Zero means coast.
 * This controls duty, not measured RPM. Physical direction depends on wiring.
 * +/-1000 produces constant-high PWM (100% duty).
 * Stop/decelerate before reversing a loaded motor; no mechanical ramp is added. */
esp_err_t tb6612fng_set_speed(tb6612fng_handle_t dev,
                            tb6612fng_motor_t motor, int speed_permille);

/* Validates both commands before changing outputs. Updates A then B, not
 * simultaneously; a hardware API failure puts BOTH motors into standby. */
esp_err_t tb6612fng_set_speeds(tb6612fng_handle_t dev,
                             int a_permille, int b_permille);

/* Requires enable(true). Coast: IN1=IN2=0, PWM=1, outputs high impedance.
 * Brake: IN1=IN2=1, PWM=0, both outputs low (short brake). */
esp_err_t tb6612fng_coast(tb6612fng_handle_t dev, tb6612fng_motor_t motor);
esp_err_t tb6612fng_brake(tb6612fng_handle_t dev, tb6612fng_motor_t motor);

/* Stop the controlling task first. Stops PWM, releases the dedicated timer
 * and motor GPIOs, and leaves STBY configured as a low output. */
esp_err_t tb6612fng_deinit(tb6612fng_handle_t dev);

#ifdef __cplusplus
}
#endif
