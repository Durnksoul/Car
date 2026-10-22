#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "car_encoder.h"
#include "balance_controller.h"

typedef enum {
    CAR_MODE_STANDBY, CAR_MODE_BALANCE, CAR_MODE_HOLD, CAR_MODE_DRIVE, CAR_MODE_MANUAL,
} car_control_mode_t;

typedef struct {
    car_encoder_sample_t encoder;
    int64_t encoder_us;
    int command_percent[2]; /* Logical forward; B is inverted at driver boundary. */
    bool enabled, braking[2];
    bool telemetry_fast;
    balance_output_t balance;
    float balance_trim_deg;
    car_control_mode_t control_mode;
    float requested_cps, requested_yaw_dps;
    float target_cps, target_yaw_dps; /* Slew-limited targets actually sent to PID. */
} car_motor_snapshot_t;

esp_err_t car_motor_control_start(void);
/* Thread-safe cached snapshot; never accesses PCNT from the IMU task. */
bool car_motor_control_snapshot(car_motor_snapshot_t *out);
/* Gates the legacy IMU log task when tuner CSV is disabled. */
bool car_motor_control_telemetry_fast(void);
/* Latest sample mailbox; motor task owns all PID state and motor writes. */
void car_motor_control_imu(int64_t sample_us, float tilt_deg, float tilt_rate_dps,
                           float yaw_rate_dps, bool valid);
