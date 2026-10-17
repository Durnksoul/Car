#pragma once
#include <stdint.h>
#include "pid_controller.h"

typedef enum {
    BALANCE_DISARMED, BALANCE_ARMED, BALANCE_FAULT_INPUT,
    BALANCE_FAULT_STALE, BALANCE_FAULT_TIMING, BALANCE_FAULT_FALL,
} balance_state_t;

typedef struct {
    pid_config_t attitude; /* deg -> PWM percent; PD is a useful starting structure */
    pid_config_t speed;    /* counts/s -> target tilt deg; normally PI */
    pid_config_t turn;     /* yaw deg/s -> differential PWM percent */
    float trim_deg, arm_angle_deg, fall_angle_deg;
    float max_output_percent, max_step_s, max_imu_age_s, max_encoder_age_s;
    float speed_period_s;
    int encoder_sign[2];   /* Normalize physical forward to positive counts/s. */
    int speed_to_tilt_sign, attitude_to_motor_sign;
    bool mapping_confirmed;
} balance_config_t;

typedef struct {
    int64_t now_us, imu_us, encoder_us;
    /* Caller maps mounting axes/signs BEFORE calling. Tilt is relative to
     * nominal upright; tilt_rate must be its matching derivative. */
    float tilt_deg, tilt_rate_dps, yaw_rate_dps;
    float left_cps, right_cps;
    float target_cps, target_yaw_rate_dps;
} balance_input_t;

typedef struct {
    /* Logical wheel-forward percentages. Caller applies motor wiring signs. */
    float left_percent, right_percent, target_tilt_deg;
    float attitude_percent, turn_percent, average_cps;
    balance_state_t state;
} balance_output_t;

typedef struct {
    balance_config_t config;
    pid_controller_t attitude, speed, turn;
    balance_state_t state;
    int64_t previous_imu_us;
    float speed_elapsed_s, speed_tilt_deg;
    bool valid;
} balance_controller_t;

/* Zero gains, mapping unconfirmed, disarmed. Limits are scaffolding, not tuned gains. */
balance_config_t balance_config_default(void);
bool balance_controller_init(balance_controller_t *control, const balance_config_t *config);
/* Explicit call required; refuses unconfirmed mapping, zero attitude Kp,
 * stale sensors, or a body outside arm_angle_deg. */
bool balance_controller_arm(balance_controller_t *control, const balance_input_t *input);
void balance_controller_disarm(balance_controller_t *control);
/* One call per NEW IMU sample, using MCU timestamps. Faults latch and return
 * zero outputs. This pure calculation never changes GPIO or motor standby. */
bool balance_controller_update(balance_controller_t *control, const balance_input_t *input,
                               balance_output_t *output);
