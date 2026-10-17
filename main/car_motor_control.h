#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "car_encoder.h"

typedef struct {
    car_encoder_sample_t encoder;
    int64_t encoder_us;
    int command_percent[2]; /* Logical forward; B is inverted at driver boundary. */
    bool enabled, braking[2];
} car_motor_snapshot_t;

esp_err_t car_motor_control_start(void);
/* Thread-safe cached snapshot; never accesses PCNT from the IMU task. */
bool car_motor_control_snapshot(car_motor_snapshot_t *out);
