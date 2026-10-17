#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "car_encoder.h"
#include "balance_controller.h"

typedef struct {
    car_encoder_sample_t encoder;
    int64_t encoder_us;
    int command_percent[2]; /* Logical forward; B is inverted at driver boundary. */
    bool enabled, braking[2];
    balance_output_t balance;
    float balance_trim_deg;
} car_motor_snapshot_t;

esp_err_t car_motor_control_start(void);
/* Thread-safe cached snapshot; never accesses PCNT from the IMU task. */
bool car_motor_control_snapshot(car_motor_snapshot_t *out);
/* Latest sample mailbox; motor task owns all PID state and motor writes. */
void car_motor_control_imu(int64_t sample_us, float tilt_deg, float tilt_rate_dps,
                           float yaw_rate_dps, bool valid);
