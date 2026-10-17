#include "balance_controller.h"
#include <math.h>
#include <stddef.h>

static bool sign_valid(int sign) { return sign == 1 || sign == -1; }
static bool positive(float x) { return isfinite(x) && x > 0; }
static float clamp(float x, float limit) { return fminf(limit, fmaxf(-limit, x)); }

balance_config_t balance_config_default(void)
{
    return (balance_config_t){
        .attitude = {.output_min = -30, .output_max = 30,
                     .integral_min = -10, .integral_max = 10, .derivative_cutoff_hz = 20},
        .speed = {.output_min = -6, .output_max = 6, .integral_min = -3, .integral_max = 3},
        .turn = {.output_min = -15, .output_max = 15, .integral_min = -5, .integral_max = 5},
        .arm_angle_deg = 5, .fall_angle_deg = 30, .max_output_percent = 30,
        .max_step_s = 0.05f, .max_imu_age_s = 0.05f, .max_encoder_age_s = 0.1f,
        .speed_period_s = 0.02f, .encoder_sign = {1, 1},
        .speed_to_tilt_sign = 1, .attitude_to_motor_sign = 1,
    };
}

bool balance_controller_init(balance_controller_t *control, const balance_config_t *c)
{
    if (!control) return false;
    *control = (balance_controller_t){0};
    if (!c || !pid_config_valid(&c->attitude) || !pid_config_valid(&c->speed) ||
        !pid_config_valid(&c->turn) || !isfinite(c->trim_deg) ||
        !positive(c->arm_angle_deg) || !positive(c->fall_angle_deg) ||
        c->arm_angle_deg >= c->fall_angle_deg || !positive(c->max_output_percent) ||
        c->max_output_percent > 100 || !positive(c->max_step_s) ||
        !positive(c->max_imu_age_s) || !positive(c->max_encoder_age_s) ||
        !positive(c->speed_period_s) || !sign_valid(c->encoder_sign[0]) ||
        !sign_valid(c->encoder_sign[1]) || !sign_valid(c->speed_to_tilt_sign) ||
        !sign_valid(c->attitude_to_motor_sign) ||
        c->attitude.output_min < -c->max_output_percent ||
        c->attitude.output_max > c->max_output_percent ||
        fmaxf(-c->speed.output_min, c->speed.output_max) >= c->fall_angle_deg)
        return false;
    control->config = *c;
    (void)pid_controller_init(&control->attitude, &c->attitude);
    (void)pid_controller_init(&control->speed, &c->speed);
    (void)pid_controller_init(&control->turn, &c->turn);
    control->valid = true;
    return true;
}

void balance_controller_disarm(balance_controller_t *control)
{
    if (!control) return;
    pid_controller_reset(&control->attitude);
    pid_controller_reset(&control->speed);
    pid_controller_reset(&control->turn);
    control->speed_elapsed_s = control->speed_tilt_deg = 0;
    control->previous_imu_us = 0;
    control->state = BALANCE_DISARMED;
}

static balance_state_t input_fault(const balance_controller_t *c, const balance_input_t *in)
{
    if (!in || !isfinite(in->tilt_deg) || !isfinite(in->tilt_rate_dps) ||
        !isfinite(in->yaw_rate_dps) || !isfinite(in->left_cps) ||
        !isfinite(in->right_cps) || !isfinite(in->target_cps) ||
        !isfinite(in->target_yaw_rate_dps)) return BALANCE_FAULT_INPUT;
    if (in->imu_us <= 0 || in->encoder_us <= 0 || in->now_us < in->imu_us ||
        in->now_us < in->encoder_us ||
        (double)(in->now_us - in->imu_us) * 1e-6 > c->config.max_imu_age_s ||
        (double)(in->now_us - in->encoder_us) * 1e-6 > c->config.max_encoder_age_s)
        return BALANCE_FAULT_STALE;
    if (fabsf(in->tilt_deg - c->config.trim_deg) >= c->config.fall_angle_deg)
        return BALANCE_FAULT_FALL;
    return BALANCE_ARMED;
}

bool balance_controller_arm(balance_controller_t *c, const balance_input_t *in)
{
    if (!c) return false;
    balance_controller_disarm(c);
    if (!c->valid || !c->config.mapping_confirmed || c->config.attitude.kp <= 0 ||
        input_fault(c, in) != BALANCE_ARMED ||
        fabsf(in->tilt_deg - c->config.trim_deg) > c->config.arm_angle_deg) return false;
    c->state = BALANCE_ARMED;
    c->previous_imu_us = in->imu_us;
    return true;
}

static bool fault(balance_controller_t *c, balance_state_t state, balance_output_t *out)
{
    balance_controller_disarm(c);
    c->state = state;
    *out = (balance_output_t){.state = state};
    return false;
}

bool balance_controller_update(balance_controller_t *c, const balance_input_t *in,
                               balance_output_t *out)
{
    if (!out) {
        if (c) { balance_controller_disarm(c); c->state = BALANCE_FAULT_INPUT; }
        return false;
    }
    *out = (balance_output_t){.state = c ? c->state : BALANCE_FAULT_INPUT};
    if (!c || !c->valid || c->state != BALANCE_ARMED) return false;
    balance_state_t state = input_fault(c, in);
    if (state != BALANCE_ARMED) return fault(c, state, out);
    if (in->imu_us <= c->previous_imu_us) return fault(c, BALANCE_FAULT_TIMING, out);
    float dt = (float)((double)(in->imu_us - c->previous_imu_us) * 1e-6);
    if (dt > c->config.max_step_s) return fault(c, BALANCE_FAULT_TIMING, out);
    c->previous_imu_us = in->imu_us;
    float speed = in->left_cps * (0.5f * c->config.encoder_sign[0]) +
                  in->right_cps * (0.5f * c->config.encoder_sign[1]);
    c->speed_elapsed_s += dt;
    if (c->speed_elapsed_s >= c->config.speed_period_s) {
        if (!pid_controller_update(&c->speed, in->target_cps, speed,
                                   c->speed_elapsed_s, &c->speed_tilt_deg))
            return fault(c, BALANCE_FAULT_INPUT, out);
        c->speed_elapsed_s = 0;
    }
    const float target = c->config.trim_deg + c->config.speed_to_tilt_sign * c->speed_tilt_deg;
    float attitude, turn;
    const float previous_turn_integral = c->turn.i;
    if (!isfinite(target) ||
        !pid_controller_update_with_rate(&c->attitude, target, in->tilt_deg,
                                         in->tilt_rate_dps, dt, &attitude) ||
        !pid_controller_update(&c->turn, in->target_yaw_rate_dps,
                               in->yaw_rate_dps, dt, &turn))
        return fault(c, BALANCE_FAULT_INPUT, out);
    attitude *= c->config.attitude_to_motor_sign;
    /* Balance has priority: turn may only use remaining motor headroom.
     * Roll back turn integration when the mixer cannot apply its full demand. */
    const float headroom = fmaxf(0, c->config.max_output_percent - fabsf(attitude));
    const float limited_turn = clamp(turn, headroom);
    if (limited_turn != turn) c->turn.i = previous_turn_integral;
    *out = (balance_output_t){
        .left_percent = clamp(attitude - limited_turn, c->config.max_output_percent),
        .right_percent = clamp(attitude + limited_turn, c->config.max_output_percent),
        .target_tilt_deg = target, .attitude_percent = attitude,
        .turn_percent = limited_turn, .average_cps = speed, .state = BALANCE_ARMED,
    };
    return true;
}
