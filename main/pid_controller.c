#include "pid_controller.h"
#include <math.h>
#include <stddef.h>

static float pid_clamp(float x, float low, float high)
{
    return fminf(high, fmaxf(low, x));
}

bool pid_config_valid(const pid_config_t *c)
{
    return c && isfinite(c->kp) && isfinite(c->ki) && isfinite(c->kd) &&
        isfinite(c->output_min) && isfinite(c->output_max) &&
        isfinite(c->integral_min) && isfinite(c->integral_max) &&
        isfinite(c->derivative_cutoff_hz) && c->derivative_cutoff_hz >= 0 &&
        c->output_min <= 0 && c->output_max >= 0 && c->output_min < c->output_max &&
        c->integral_min <= 0 && c->integral_max >= 0;
}

bool pid_controller_init(pid_controller_t *pid, const pid_config_t *config)
{
    if (!pid) return false;
    const bool valid = pid_config_valid(config);
    *pid = (pid_controller_t){0};
    if (valid) { pid->config = *config; pid->valid = true; }
    return valid;
}

void pid_controller_reset(pid_controller_t *pid)
{
    if (!pid) return;
    pid->p = pid->i = pid->d = pid->output = 0;
    pid->previous_measurement = pid->filtered_rate = 0;
    pid->initialized = false;
}

static bool fail(pid_controller_t *pid, float *output)
{
    pid_controller_reset(pid);
    if (output) *output = 0;
    return false;
}

bool pid_controller_update_with_rate(pid_controller_t *pid, float target,
                                     float measurement, float rate,
                                     float dt_s, float *output)
{
    if (!pid || !pid->valid || !output || !isfinite(target) ||
        !isfinite(measurement) || !isfinite(rate) || !isfinite(dt_s) || dt_s <= 0)
        return fail(pid, output);
    const pid_config_t *c = &pid->config;
    const float error = target - measurement;
    /* Backward-Euler first-order low-pass on measured derivative. */
    const float tau = c->derivative_cutoff_hz > 0 ?
        1.0f / (6.28318530718f * c->derivative_cutoff_hz) : 0;
    const float alpha = tau > 0 ? dt_s / (tau + dt_s) : 1;
    const float filtered = pid->filtered_rate + alpha * (rate - pid->filtered_rate);
    const float p = c->kp * error, d = -c->kd * filtered;
    const float increment = c->ki * error * dt_s;
    const float integral_sum = pid->i + increment;
    if (!isfinite(error) || !isfinite(filtered) || !isfinite(p) ||
        !isfinite(d) || !isfinite(increment) || !isfinite(integral_sum))
        return fail(pid, output);
    float integral = pid_clamp(integral_sum, c->integral_min, c->integral_max);
    const float candidate = p + integral + d;
    if (!isfinite(candidate)) return fail(pid, output);
    /* Conditional integration: allow unwinding, reject further windup. */
    if ((candidate > c->output_max && integral > pid->i) ||
        (candidate < c->output_min && integral < pid->i)) integral = pid->i;
    const float raw_output = p + integral + d;
    if (!isfinite(raw_output)) return fail(pid, output);
    pid->p = p; pid->i = integral; pid->d = d;
    pid->filtered_rate = filtered;
    pid->previous_measurement = measurement;
    pid->initialized = true;
    pid->output = pid_clamp(raw_output, c->output_min, c->output_max);
    *output = pid->output;
    return true;
}

bool pid_controller_update(pid_controller_t *pid, float target, float measurement,
                           float dt_s, float *output)
{
    float rate = 0;
    if (pid && pid->initialized && isfinite(dt_s) && dt_s > 0)
        rate = (measurement - pid->previous_measurement) / dt_s;
    return pid_controller_update_with_rate(pid, target, measurement, rate, dt_s, output);
}
