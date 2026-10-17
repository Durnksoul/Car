#pragma once
#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float output_min, output_max;
    /* Integral term limits, in output units (already multiplied by Ki). */
    float integral_min, integral_max;
    float derivative_cutoff_hz; /* 0: unfiltered */
} pid_config_t;

typedef struct {
    pid_config_t config;
    float p, i, d, output;
    float previous_measurement, filtered_rate;
    bool initialized, valid;
} pid_controller_t;

bool pid_config_valid(const pid_config_t *config);
bool pid_controller_init(pid_controller_t *pid, const pid_config_t *config);
void pid_controller_reset(pid_controller_t *pid);
/* dt is in seconds. Derivative on measurement avoids setpoint kick.
 * Returns false and output=0 on invalid inputs or arithmetic overflow. */
bool pid_controller_update(pid_controller_t *pid, float target, float measurement,
                           float dt_s, float *output);
/* Use actual gyro angular velocity for the attitude loop's derivative.
 * measurement_rate must have the same sign as increasing measurement. */
bool pid_controller_update_with_rate(pid_controller_t *pid, float target,
                                     float measurement, float measurement_rate,
                                     float dt_s, float *output);
