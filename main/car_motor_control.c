#include "car_motor_control.h"
#include "car_log_mode.h"
#include "car_encoder.h"
#include "sdkconfig.h"
#if CONFIG_CAR_BLE_UART
#include "car_ble_uart.h"
#endif
#include "car_command_stream.h"
#include <stdarg.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "esp_timer.h"
#if !defined(CAR_HOST_TEST)
#include "nvs.h"
#include "nvs_flash.h"
#endif

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tb6612_command.h"
#include "tb6612fng.h"

static const char *TAG = "car_motor";
/* Keep A as the forward reference; the user's mirrored B motor is reversed. */
static const int motor_direction[2] = {1, -1};

/* Zero-initialized control objects start with the quiet, human-readable mode. */
typedef enum { TELEMETRY_SLOW, TELEMETRY_OFF, TELEMETRY_FAST } telemetry_mode_t;
#define TELEMETRY_SLOW_PERIOD_US INT64_C(1000000)
#define TELEMETRY_ENCODER_PERIOD_US INT64_C(200000)

typedef struct {
    tb6612fng_handle_t motor;
    bool enabled;
    tb6612_command_kind_t mode[2];
    int percent[2];
    balance_controller_t balance;
    balance_output_t balance_output;
    bool trim_confirmed;
    telemetry_mode_t telemetry_mode;
    int64_t telemetry_last_us;
    int64_t diag_end_us, diag_last_us, diag_sample_us;
    /* Exact IMU input used for the last successful PID/motor update. */
    int64_t control_sample_us;
    float control_tilt_deg, control_rate_dps;
    /* Owned only by the motor task; published copies are read-only. */
    car_control_mode_t control_mode;
    bool motion_owned; /* Explicit mode commands exclude the tuning host. */
    float requested_cps, requested_yaw_dps, target_cps, target_yaw_dps;
    int64_t last_move_us;
#if CONFIG_CAR_PID_TUNER
    /* Upright reference captured by balance zero, without the tuner offset.
     * The balance controller's trim is base + host setpoint offset. */
    float trim_base_deg, tuner_setpoint_deg;
#endif
} control_t;

/* ---- persisted balance configuration ------------------------------------
 * Gains and the upright reference survive power cycles so a tuned car does
 * not need to be recalibrated on every boot. Only validated values are
 * applied; anything missing or corrupt falls back to the compiled defaults. */
#define STORE_NAMESPACE "car_balance"
#define STORE_KEY "cfg"
#define STORE_MAGIC 0x43415231u /* "CAR1" */
#define STORE_VERSION 1u
#define STORE_FLAG_TRIM 0x0001u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    float trim_deg;
    float attitude_kp, attitude_ki, attitude_kd;
    float speed_kp, speed_ki, speed_kd;
    float turn_kp, turn_ki, turn_kd;
    int8_t motor_sign;
    uint8_t reserved[3];
} stored_config_t;

static bool stored_gain_ok(float value)
{
    return isfinite(value) && value >= 0.0f && value <= 100.0f;
}

#if !defined(CAR_HOST_TEST)
static bool store_load_config(balance_config_t *config, bool *trim_valid)
{
    nvs_handle_t handle;
    stored_config_t saved;
    size_t length = sizeof(saved);
    if (nvs_open(STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
    const esp_err_t err = nvs_get_blob(handle, STORE_KEY, &saved, &length);
    nvs_close(handle);
    if (err != ESP_OK || length != sizeof(saved)) return false;
    if (saved.magic != STORE_MAGIC || saved.version != STORE_VERSION) return false;
    if (!stored_gain_ok(saved.attitude_kp) || !stored_gain_ok(saved.attitude_ki) ||
        !stored_gain_ok(saved.attitude_kd) || !stored_gain_ok(saved.speed_kp) ||
        !stored_gain_ok(saved.speed_ki) || !stored_gain_ok(saved.speed_kd) ||
        !stored_gain_ok(saved.turn_kp) || !stored_gain_ok(saved.turn_ki) ||
        !stored_gain_ok(saved.turn_kd)) {
        return false;
    }
    if (!isfinite(saved.trim_deg) || fabsf(saved.trim_deg) > 90.0f) return false;
    if (saved.motor_sign != 1 && saved.motor_sign != -1) return false;
    config->attitude.kp = saved.attitude_kp;
    config->attitude.ki = saved.attitude_ki;
    config->attitude.kd = saved.attitude_kd;
    config->speed.kp = saved.speed_kp;
    config->speed.ki = saved.speed_ki;
    config->speed.kd = saved.speed_kd;
    config->turn.kp = saved.turn_kp;
    config->turn.ki = saved.turn_ki;
    config->turn.kd = saved.turn_kd;
    config->trim_deg = saved.trim_deg;
    config->attitude_to_motor_sign = saved.motor_sign;
    *trim_valid = (saved.flags & STORE_FLAG_TRIM) != 0;
    return true;
}

static void store_save_config(const balance_config_t *config, bool trim_valid)
{
    stored_config_t saved = {0};
    saved.magic = STORE_MAGIC;
    saved.version = STORE_VERSION;
    saved.flags = trim_valid ? STORE_FLAG_TRIM : 0;
    saved.trim_deg = config->trim_deg;
    saved.attitude_kp = config->attitude.kp;
    saved.attitude_ki = config->attitude.ki;
    saved.attitude_kd = config->attitude.kd;
    saved.speed_kp = config->speed.kp;
    saved.speed_ki = config->speed.ki;
    saved.speed_kd = config->speed.kd;
    saved.turn_kp = config->turn.kp;
    saved.turn_ki = config->turn.ki;
    saved.turn_kd = config->turn.kd;
    saved.motor_sign = (int8_t)config->attitude_to_motor_sign;
    nvs_handle_t handle;
    if (nvs_open(STORE_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;
    const esp_err_t err = nvs_set_blob(handle, STORE_KEY, &saved, sizeof(saved));
    if (err == ESP_OK) (void)nvs_commit(handle);
    nvs_close(handle);
}
#else
/* Host tests exercise control logic only; persistence is a no-op. */
static bool store_load_config(balance_config_t *config, bool *trim_valid)
{
    (void)config;
    *trim_valid = false;
    return false;
}

static void store_save_config(const balance_config_t *config, bool trim_valid)
{
    (void)config;
    (void)trim_valid;
}
#endif

#if CONFIG_CAR_BLE_UART
static bool reply_to_ble;
#endif
static car_encoder_sample_t encoder;
static int64_t encoder_us;
static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static car_motor_snapshot_t snapshot;
typedef struct {
    int64_t us;
    float tilt, rate, yaw_rate;
    bool valid;
} imu_input_t;
static imu_input_t latest_imu;

void car_motor_control_imu(int64_t sample_us, float tilt_deg, float tilt_rate_dps,
                           float yaw_rate_dps, bool valid)
{
    const imu_input_t next = {sample_us, tilt_deg, tilt_rate_dps, yaw_rate_dps, valid};
    portENTER_CRITICAL(&snapshot_lock);
    latest_imu = next;
    portEXIT_CRITICAL(&snapshot_lock);
}

static balance_input_t balance_input(bool *valid)
{
    imu_input_t imu;
    portENTER_CRITICAL(&snapshot_lock);
    imu = latest_imu;
    portEXIT_CRITICAL(&snapshot_lock);
    *valid = imu.valid;
    return (balance_input_t){
        .now_us = esp_timer_get_time(), .imu_us = imu.us, .encoder_us = encoder_us,
        .tilt_deg = imu.tilt, .tilt_rate_dps = imu.rate, .yaw_rate_dps = imu.yaw_rate,
        .left_cps = encoder.cps[0], .right_cps = encoder.cps[1],
    };
}

static void publish_snapshot(const control_t *control)
{
    car_motor_snapshot_t next = {
        .encoder = encoder, .encoder_us = encoder_us, .enabled = control->enabled,
        .command_percent = {control->percent[0], control->percent[1]},
        .braking = {control->mode[0] == TB_CMD_BRAKE, control->mode[1] == TB_CMD_BRAKE},
        .balance = control->balance_output,
        .balance_trim_deg = control->balance.config.trim_deg,
        .control_mode = control->control_mode,
        .telemetry_fast = control->telemetry_mode == TELEMETRY_FAST && control->diag_end_us == 0,
        .requested_cps = control->requested_cps, .requested_yaw_dps = control->requested_yaw_dps,
        .target_cps = control->target_cps, .target_yaw_dps = control->target_yaw_dps,
    };
    portENTER_CRITICAL(&snapshot_lock);
    snapshot = next;
    portEXIT_CRITICAL(&snapshot_lock);
}

bool car_motor_control_snapshot(car_motor_snapshot_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&snapshot_lock);
    *out = snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
    return out->encoder_us > 0;
}

bool car_motor_control_telemetry_fast(void)
{
    portENTER_CRITICAL(&snapshot_lock);
    const bool fast = snapshot.telemetry_fast;
    portEXIT_CRITICAL(&snapshot_lock);
    return fast;
}

static void reply(const char *format, ...)
{
    /* Commands still execute; temporarily emit encoder records only. */
    if (CAR_ENCODER_ONLY_LOG && strncmp(format, "enc_left=", 9) != 0) return;
    char text[200];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text) - 3, format, args);
    va_end(args);
    size_t used = strlen(text);
    memcpy(text + used, "\r\n", 3);
    printf("%s", text);
#if CONFIG_CAR_BLE_UART
    if (reply_to_ble) car_ble_uart_send(text);
#endif
}

static void print_encoder(void)
{
    reply("enc_left=%" PRId64 ",enc_right=%" PRId64 ",cps_left=%.1f,cps_right=%.1f,enc_us=%" PRId64,
          encoder.count[0], encoder.count[1], (double)encoder.cps[0],
          (double)encoder.cps[1], encoder_us);
}

static void print_help(void)
{
    reply("Commands (CR or LF ends each line; integer percentages):");
    reply("help | status | enc | enczero | enable | standby");
    reply("telemetry off|slow|fast|status (default slow: 1 Hz)");
    reply("diag start|stop (10-second diagnostic burst, up to 50 Hz)");
    reply("mode hold|drive|status|tune | move <counts/s> <yaw deg/s> (repeat at >=5 Hz)");
    reply("a <percent> | b <percent> | ab <A-percent> <B-percent> (-100..100)");
    reply("coast a|b|all | brake a|b|all");
    reply("Boot: STANDBY; send enable, then ab 20 20 to start both motors");
    reply("PWM=20kHz, 0=coast, +/-100=HIGH; holds until next command");
    reply("Logical forward: A normal, B reversed; both encoders forward-positive");
    reply("balance zero | arm | off | status | motor <1|-1>");
    reply("pid attitude|speed|turn <Kp> <Ki> <Kd> | pid show (standby only)");
}

static void print_status(const control_t *control)
{
    reply("Commanded state: %s, STBY=%d (not a hardware measurement)",
             control->enabled ? "ENABLED" : "STANDBY", control->enabled);
    for (int i = 0; i < 2; ++i) {
        if (!control->enabled) {
            reply("%c: IN=00, PWM=LOW", 'A' + i);
        } else if (control->mode[i] == TB_CMD_BRAKE) {
            reply("%c: BRAKE, IN=11, PWM=LOW", 'A' + i);
        } else if (control->mode[i] == TB_CMD_COAST || control->percent[i] == 0) {
            reply("%c: COAST, IN=00, PWM=HIGH", 'A' + i);
        } else {
            int p = control->percent[i];
            int physical = p * motor_direction[i];
            reply("%c: %+d%%, IN=%s, PWM=%s", 'A' + i, p,
                     physical > 0 ? "10" : "01", (p == 100 || p == -100) ? "HIGH" : "20kHz");
        }
    }
}

static void clear_commands(control_t *control)
{
    control->control_sample_us = 0;
    for (int i = 0; i < 2; ++i) {
        control->mode[i] = TB_CMD_COAST;
        control->percent[i] = 0;
    }
}

static esp_err_t balance_stop(control_t *control, balance_state_t state)
{
    balance_controller_disarm(&control->balance);
#if CONFIG_CAR_PID_TUNER
    /* Drop any host setpoint offset so a restart begins from the calibration. */
    if (control->trim_confirmed) control->balance.config.trim_deg = control->trim_base_deg;
    control->tuner_setpoint_deg = 0;
#endif
    control->control_mode = CAR_MODE_STANDBY;
    control->requested_cps = control->requested_yaw_dps = 0;
    control->target_cps = control->target_yaw_dps = 0;
    control->last_move_us = 0;
    control->balance.state = state;
    control->balance_output = (balance_output_t){.state = state};
    esp_err_t err = tb6612fng_enable(control->motor, false);
    control->enabled = false;
    clear_commands(control);
    return err;
}

/* Motion targets use encoder counts/s and gyro deg/s, never PWM percentages.
 * These are initial software limits, not measured motor/mechanical ratings. */
#define MOTION_MAX_CPS 300.0f
#define MOTION_MAX_YAW_DPS 45.0f
#define MOTION_ACCEL_CPS2 300.0f
#define MOTION_YAW_ACCEL_DPS2 90.0f
#define MOTION_COMMAND_TIMEOUT_US INT64_C(500000)

static const char *control_mode_name(car_control_mode_t mode)
{
    switch (mode) {
    case CAR_MODE_HOLD: return "HOLD";
    case CAR_MODE_DRIVE: return "DRIVE";
    case CAR_MODE_BALANCE: return "BALANCE";
    case CAR_MODE_MANUAL: return "MANUAL";
    default: return "STANDBY";
    }
}

static bool loop_enabled(const pid_config_t *pid)
{
    return pid->kp > 0 || pid->ki > 0 || pid->kd > 0;
}

static void motion_status(const control_t *c)
{
    reply("mode=%s,balance_state=%d,mode_owner=%d,speed_loop=%d,turn_loop=%d",
          control_mode_name(c->control_mode), c->balance.state, c->motion_owned,
          loop_enabled(&c->balance.config.speed), loop_enabled(&c->balance.config.turn));
    reply("requested_cps=%.2f,requested_yaw_dps=%.2f,target_cps=%.2f,target_yaw_dps=%.2f",
          (double)c->requested_cps, (double)c->requested_yaw_dps,
          (double)c->target_cps, (double)c->target_yaw_dps);
}

/* Starting from standby resets PID once. Switching HOLD/DRIVE never resets PID. */
static bool motion_start(control_t *c, car_control_mode_t mode)
{
    if (mode == CAR_MODE_DRIVE && !loop_enabled(&c->balance.config.speed)) {
        reply("ERR: speed PID is zero; configure pid speed before mode drive");
        return false;
    }
    if (c->enabled && c->balance.state != BALANCE_ARMED) {
        reply("ERR: manual motor control active; send standby first");
        return false;
    }
#if CONFIG_CAR_PID_TUNER
    if (c->tuner_setpoint_deg != 0) {
        reply("ERR: tuner tilt offset active; send SETPOINT:0 or standby first");
        return false;
    }
#endif
    if (!c->enabled) {
        bool valid;
        balance_input_t in = balance_input(&valid);
        if (!c->trim_confirmed || !valid || !balance_controller_arm(&c->balance, &in)) {
            reply("ERR: mode start needs saved upright zero, Kp>0 and fresh upright sensors");
            return false;
        }
        esp_err_t err = tb6612fng_enable(c->motor, false);
        if (err == ESP_OK) err = tb6612fng_enable(c->motor, true);
        if (err != ESP_OK) {
            (void)balance_stop(c, BALANCE_FAULT_INPUT);
            reply("ERR: mode motor %s", esp_err_to_name(err));
            return false;
        }
        clear_commands(c);
        c->enabled = true;
        c->balance_output = (balance_output_t){.state = BALANCE_ARMED};
        c->target_cps = c->target_yaw_dps = 0;
    }
    if (c->control_mode != mode || mode == CAR_MODE_HOLD) {
        c->requested_cps = c->requested_yaw_dps = 0;
        c->last_move_us = esp_timer_get_time();
    }
    c->motion_owned = true;
    c->control_mode = mode;
    return true;
}

static bool motion_command(control_t *c, const char *line)
{
    char verb[16], arg[16], extra;
    if (sscanf(line, "%15s", verb) != 1) return false;
    if (!strcmp(verb, "mode")) {
        if (sscanf(line, "%15s %15s %c", verb, arg, &extra) != 2) {
            reply("ERR: mode hold|drive|status|tune");
            return true;
        }
        if (!strcmp(arg, "status")) { motion_status(c); return true; }
        if (!strcmp(arg, "tune")) {
            if (c->enabled) reply("ERR: send standby before mode tune");
            else { c->motion_owned = false; reply("OK: tuning host enabled; no motor start"); }
            return true;
        }
        car_control_mode_t mode;
        if (!strcmp(arg, "hold")) mode = CAR_MODE_HOLD;
        else if (!strcmp(arg, "drive")) mode = CAR_MODE_DRIVE;
        else { reply("ERR: mode hold|drive|status|tune"); return true; }
        if (motion_start(c, mode)) {
            reply("OK: mode %s", control_mode_name(mode));
            if (mode == CAR_MODE_HOLD && !loop_enabled(&c->balance.config.speed))
                reply("NOTE: speed PID is zero; upright balance only, drift correction inactive");
            motion_status(c);
        }
        return true;
    }
    if (strcmp(verb, "move")) return false;
    float cps, yaw;
    if (sscanf(line, "%15s %f %f %c", verb, &cps, &yaw, &extra) != 3 ||
        !isfinite(cps) || !isfinite(yaw) || fabsf(cps) > MOTION_MAX_CPS ||
        fabsf(yaw) > MOTION_MAX_YAW_DPS) {
        reply("ERR: move <counts/s -300..300> <yaw deg/s -45..45>");
        return true;
    }
    if (!c->enabled || c->balance.state != BALANCE_ARMED || c->control_mode != CAR_MODE_DRIVE) {
        reply("ERR: move requires mode drive; command never arms motors");
        return true;
    }
    if (!loop_enabled(&c->balance.config.speed) ||
        (yaw != 0 && !loop_enabled(&c->balance.config.turn))) {
        reply("ERR: configure the corresponding speed/turn PID in standby first");
        return true;
    }
    c->requested_cps = cps;
    c->requested_yaw_dps = yaw;
    c->last_move_us = esp_timer_get_time();
    reply("OK: move %.2f %.2f", (double)cps, (double)yaw);
    return true;
}

static float motion_ramp(float actual, float requested, float step)
{
    return actual + fminf(step, fmaxf(-step, requested - actual));
}

/* Called once per new IMU sample, in the same task that owns the balance PID. */
static void motion_update(control_t *c, balance_input_t *in)
{
    if (c->control_mode != CAR_MODE_HOLD && c->control_mode != CAR_MODE_DRIVE) return;
    if (c->control_mode == CAR_MODE_DRIVE &&
        in->now_us - c->last_move_us >= MOTION_COMMAND_TIMEOUT_US) {
        c->control_mode = CAR_MODE_HOLD;
        c->requested_cps = c->requested_yaw_dps = 0;
        reply("NOTE: move timeout; slowing to HOLD, balance remains enabled");
    }
    const float dt = (float)((double)(in->imu_us - c->balance.previous_imu_us) * 1e-6);
    if (dt <= 0 || dt > c->balance.config.max_step_s) return;
    c->target_cps = motion_ramp(c->target_cps, c->requested_cps, MOTION_ACCEL_CPS2 * dt);
    c->target_yaw_dps = motion_ramp(c->target_yaw_dps, c->requested_yaw_dps, MOTION_YAW_ACCEL_DPS2 * dt);
    in->target_cps = c->target_cps;
    in->target_yaw_rate_dps = c->target_yaw_dps;
}

static void balance_status(const control_t *c)
{
    const balance_config_t *cfg = &c->balance.config;
    reply("balance_state=%d,trim=%.3f,trim_set=%d,motor_sign=%d,limit=%.1f",
          c->balance.state, (double)cfg->trim_deg, c->trim_confirmed,
          cfg->attitude_to_motor_sign, (double)cfg->max_output_percent);
    reply("pid attitude %.4f %.4f %.4f; speed %.5f %.5f %.5f",
          (double)cfg->attitude.kp, (double)cfg->attitude.ki, (double)cfg->attitude.kd,
          (double)cfg->speed.kp, (double)cfg->speed.ki, (double)cfg->speed.kd);
    reply("pid turn %.5f %.5f %.5f", (double)cfg->turn.kp, (double)cfg->turn.ki, (double)cfg->turn.kd);
}

#if CONFIG_CAR_PID_TUNER
/* ---- LLM-PID-Tuner host link ---------------------------------------------
 * Protocol of KINGSTON-115/llm-pid-tuner, generic_serial_csv profile:
 *   20 Hz CSV : timestamp_ms,setpoint,input,pwm,error,p,i,d
 *   host -> MCU: STATUS | SET P:.. I:.. D:.. | SET KP:.. KI:.. KD:..
 *                SETPOINT:<deg> | RESET
 * Commands are recognised in uppercase only, so the lowercase human commands
 * status/standby keep their old meaning. input is tilt relative to the
 * balance-zero reference, setpoint is the host offset (normally 0) and pwm is
 * the attitude-loop output in percent; gains are therefore deg -> % gains. */
#define TUNER_KP_MAX 50.0f
#define TUNER_KI_MAX 20.0f
#define TUNER_KD_MAX 5.0f
#define TUNER_SETPOINT_MAX_DEG 5.0f

static void tuner_status(const control_t *c)
{
    bool valid;
    balance_input_t in = balance_input(&valid);
    const pid_config_t *a = &c->balance.config.attitude;
    const float input = valid ? in.tilt_deg - c->trim_base_deg : 0.0f;
    const float setpoint = c->balance.state == BALANCE_ARMED ?
        c->balance_output.target_tilt_deg - c->trim_base_deg : c->tuner_setpoint_deg;
    reply("# STATUS: Kp=%.4f Ki=%.4f Kd=%.4f Setpoint=%.3f Temp=%.3f Error=%.3f "
          "Armed=%d State=%d Trim=%.3f",
          (double)a->kp, (double)a->ki, (double)a->kd, (double)setpoint,
          (double)input, (double)(setpoint - input), c->balance.state == BALANCE_ARMED,
          (int)c->balance.state, (double)c->trim_base_deg);
}

static bool tuner_apply_gains(control_t *c, float kp, float ki, float kd)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) return false;
    if (kp < 0 || ki < 0 || kd < 0) return false;
    /* Zero proportional gain while balancing would drop the car immediately. */
    if (c->enabled && kp <= 0) return false;
    pid_config_t next = c->balance.config.attitude;
    next.kp = fminf(kp, TUNER_KP_MAX);
    next.ki = fminf(ki, TUNER_KI_MAX);
    next.kd = fminf(kd, TUNER_KD_MAX);
    /* Reinitializing clears the I/D state but does NOT change the balance
     * state; this is what lets the host retune while the car stands. */
    if (!pid_controller_init(&c->balance.attitude, &next)) return false;
    c->balance.config.attitude = next;
    store_save_config(&c->balance.config, c->trim_confirmed);
    return true;
}

static bool tuner_try_arm(control_t *c)
{
    if (c->enabled) return true;
    bool valid;
    balance_input_t in = balance_input(&valid);
    if (!valid || !isfinite(in.tilt_deg)) return false;
    /* Same "held upright and still" gate as balance zero/arm. */
    if (fabsf(in.tilt_rate_dps) > 5.0f || fabsf(in.left_cps) > 10.0f ||
        fabsf(in.right_cps) > 10.0f) return false;
    if (!c->trim_confirmed) {
        if (fabsf(in.tilt_deg) > 10.0f) return false;
        balance_config_t cfg = c->balance.config;
        cfg.trim_deg = in.tilt_deg;
        if (!balance_controller_init(&c->balance, &cfg)) return false;
        c->trim_base_deg = in.tilt_deg;
        c->tuner_setpoint_deg = 0;
        c->trim_confirmed = true;
        store_save_config(&c->balance.config, true);
        reply("# tuner: upright reference set to %.3f deg", (double)in.tilt_deg);
    }
    if (c->balance.config.attitude.kp <= 0) return false;
    if (!balance_controller_arm(&c->balance, &in)) return false;
    /* Disabling also clears stale manual output before enabling the bridge. */
    esp_err_t err = tb6612fng_enable(c->motor, false);
    if (err == ESP_OK) err = tb6612fng_enable(c->motor, true);
    if (err != ESP_OK) {
        (void)balance_stop(c, BALANCE_FAULT_INPUT);
        reply("# ERROR: arm motor %s", esp_err_to_name(err));
        return false;
    }
    clear_commands(c);
    c->enabled = true;
    c->control_mode = CAR_MODE_BALANCE;
    c->balance_output = (balance_output_t){.state = BALANCE_ARMED};
    reply("# tuner: balance armed, output limit %.0f%%",
          (double)c->balance.config.max_output_percent);
    return true;
}

/* Returns true when the line belonged to the tuner protocol. */
static bool tuner_command(control_t *c, const char *line)
{
    if (!line) return false;
    char tail[2] = {0};
    float p, i, d;
    if (!strcmp(line, "STATUS")) {
        /* The tuning app always handshakes with STATUS: restore its normal CSV
         * rate automatically. Human queries use lowercase status/mode status. */
        c->telemetry_mode = TELEMETRY_FAST;
        c->telemetry_last_us = esp_timer_get_time();
        tuner_status(c);
        return true;
    }
    if (!strcmp(line, "RESET")) {
        if (c->motion_owned) { reply("# ERROR: explicit mode owns control; standby then mode tune"); return true; }
        pid_controller_reset(&c->balance.attitude);
        pid_controller_reset(&c->balance.speed);
        pid_controller_reset(&c->balance.turn);
        reply("# System Reset: integrators cleared, gains kept");
        return true;
    }
    if (!strcmp(line, "STANDBY")) {
        esp_err_t err = balance_stop(c, BALANCE_DISARMED);
        reply("# STANDBY: %s", esp_err_to_name(err));
        return true;
    }
    if (sscanf(line, "SET P:%f I:%f D:%f %1s", &p, &i, &d, tail) == 3 ||
        sscanf(line, "SET KP:%f KI:%f KD:%f %1s", &p, &i, &d, tail) == 3) {
        if (c->motion_owned) { reply("# ERROR: explicit mode owns control; standby then mode tune"); return true; }
        if (!tuner_apply_gains(c, p, i, d)) {
            reply("# ERROR: PID rejected (finite, non-negative, Kp>0 while armed)");
            return true;
        }
#if CONFIG_CAR_PID_TUNER_AUTO_ARM
        (void)tuner_try_arm(c);
#endif
        tuner_status(c);
        return true;
    }
    if (sscanf(line, "SETPOINT:%f %1s", &p, tail) == 1) {
        if (c->motion_owned) { reply("# ERROR: explicit mode owns control; standby then mode tune"); return true; }
        if (!isfinite(p) || fabsf(p) > 100.0f) {
            reply("# ERROR: setpoint rejected");
            return true;
        }
        const float offset = fminf(TUNER_SETPOINT_MAX_DEG, fmaxf(-TUNER_SETPOINT_MAX_DEG, p));
        c->tuner_setpoint_deg = offset;
        if (c->trim_confirmed) c->balance.config.trim_deg = c->trim_base_deg + offset;
        reply("# Setpoint Updated: %.3f deg from upright (requested %.3f)",
              (double)offset, (double)p);
        return true;
    }
    return false;
}
#endif

/* Exact token matching rejects trailing arguments rather than partly applying them. */
static bool balance_command(control_t *c, const char *line)
{
    char verb[16], arg[16], extra;
    int tokens = sscanf(line, "%15s %15s %c", verb, arg, &extra);
    if (tokens < 1 || (strcmp(verb, "balance") && strcmp(verb, "pid"))) return false;
    if (tokens == 2 && ((!strcmp(verb, "balance") && !strcmp(arg, "status")) ||
                         (!strcmp(verb, "pid") && !strcmp(arg, "show")))) {
        balance_status(c);
        return true;
    }
    if (tokens == 2 && !strcmp(verb, "balance") && !strcmp(arg, "off")) {
        esp_err_t err = balance_stop(c, BALANCE_DISARMED);
        reply("balance off: %s", esp_err_to_name(err));
        return true;
    }
    bool valid;
    balance_input_t in = balance_input(&valid);
    if (tokens == 2 && !strcmp(verb, "balance") && !strcmp(arg, "arm")) {
        if (c->motion_owned) { reply("ERR: use mode hold or mode drive to restart"); return true; }
        if (c->enabled || !c->trim_confirmed || !valid || !balance_controller_arm(&c->balance, &in)) {
            reply("ERR: arm requires standby, balance zero, Kp>0 and fresh upright sensors");
            return true;
        }
        /* Disabling also clears stale manual output before enabling the bridge. */
        esp_err_t err = tb6612fng_enable(c->motor, false);
        if (err == ESP_OK) err = tb6612fng_enable(c->motor, true);
        if (err != ESP_OK) {
            (void)balance_stop(c, BALANCE_FAULT_INPUT);
            reply("ERR: arm motor %s", esp_err_to_name(err));
        } else {
            clear_commands(c);
            c->enabled = true;
            c->control_mode = CAR_MODE_BALANCE;
            c->balance_output = (balance_output_t){.state = BALANCE_ARMED};
            reply("OK: balance armed; hold the car for direction test");
        }
        return true;
    }
    if (c->enabled) {
        reply("ERR: send standby or balance off before changing PID/zero");
        return true;
    }
    balance_config_t cfg = c->balance.config;
    if (tokens == 2 && !strcmp(verb, "balance") && !strcmp(arg, "zero")) {
        if (!valid || in.imu_us <= 0 || in.encoder_us <= 0 ||
            in.now_us < in.imu_us || in.now_us - in.imu_us > 50000 ||
            in.now_us < in.encoder_us || in.now_us - in.encoder_us > 100000 ||
            !isfinite(in.tilt_deg) || fabsf(in.tilt_deg) > 15 ||
            !isfinite(in.tilt_rate_dps) || fabsf(in.tilt_rate_dps) > 3 ||
            fabsf(in.left_cps) > 10 || fabsf(in.right_cps) > 10) {
            reply("ERR: hold car upright and stationary with fresh sensors for balance zero");
            return true;
        }
        cfg.trim_deg = in.tilt_deg;
        if (balance_controller_init(&c->balance, &cfg)) {
            c->trim_confirmed = true;
#if CONFIG_CAR_PID_TUNER
            c->trim_base_deg = in.tilt_deg;
            c->tuner_setpoint_deg = 0;
#endif
            c->balance_output = (balance_output_t){0};
            store_save_config(&c->balance.config, true);
            reply("OK: upright trim=%.3f deg (saved)", (double)cfg.trim_deg);
        }
        return true;
    }
    int sign;
    if (sscanf(line, "balance motor %d %c", &sign, &extra) == 1 &&
        (sign == -1 || sign == 1)) {
        cfg.attitude_to_motor_sign = sign;
    } else {
        float kp, ki, kd;
        if (sscanf(line, "pid %15s %f %f %f %c", arg, &kp, &ki, &kd, &extra) != 4 ||
            (strcmp(arg, "attitude") && strcmp(arg, "speed") && strcmp(arg, "turn")) ||
            !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
            kp < 0 || ki < 0 || kd < 0 || kp > 100 || ki > 100 || kd > 100) {
            reply("ERR: balance zero|arm|off|status|motor +/-1; pid attitude|speed|turn Kp Ki Kd");
            return true;
        }
        pid_config_t *pid = !strcmp(arg, "attitude") ? &cfg.attitude :
                            (!strcmp(arg, "speed") ? &cfg.speed : &cfg.turn);
        pid->kp = kp; pid->ki = ki; pid->kd = kd;
    }
    if (balance_controller_init(&c->balance, &cfg)) {
        c->balance_output = (balance_output_t){0};
        store_save_config(&c->balance.config, c->trim_confirmed);
        balance_status(c);
    } else reply("ERR: invalid balance configuration");
    return true;
}

static esp_err_t balance_tick(control_t *c)
{
    if (c->balance.state != BALANCE_ARMED) return ESP_OK;
    bool valid;
    balance_input_t in = balance_input(&valid);
    balance_state_t failure = BALANCE_DISARMED;
    if (!valid) failure = BALANCE_FAULT_INPUT;
    else if (in.now_us < in.imu_us || in.now_us - in.imu_us > 50000 ||
             in.now_us < in.encoder_us || in.now_us - in.encoder_us > 100000)
        failure = BALANCE_FAULT_STALE;
    if (failure != BALANCE_DISARMED) {
        reply("balance fault=%d; STANDBY", failure);
        return balance_stop(c, failure);
    }
    /* The mailbox may still contain the previous sample: do not integrate twice. */
    if (in.imu_us == c->balance.previous_imu_us) return ESP_OK;
    motion_update(c, &in);
    if (!balance_controller_update(&c->balance, &in, &c->balance_output)) {
        failure = c->balance.state;
        reply("balance fault=%d; STANDBY", failure);
        return balance_stop(c, failure);
    }
    int left = (int)lroundf(c->balance_output.left_percent * 10);
    int right = (int)lroundf(c->balance_output.right_percent * 10);
    esp_err_t err = tb6612fng_set_speeds(c->motor,
                                      left * motor_direction[0], right * motor_direction[1]);
    if (err != ESP_OK) return err;
    c->control_sample_us = in.imu_us;
    c->control_tilt_deg = in.tilt_deg;
    c->control_rate_dps = in.tilt_rate_dps;
    c->percent[0] = (int)lroundf(c->balance_output.left_percent);
    c->percent[1] = (int)lroundf(c->balance_output.right_percent);
    c->mode[0] = c->mode[1] = TB_CMD_SPEED;
    return ESP_OK;
}

static esp_err_t apply_command(control_t *control, const tb6612_command_t *command)
{
    esp_err_t err = ESP_OK;
    if (command->kind == TB_CMD_HELP) { print_help(); return ESP_OK; }
    if (command->kind == TB_CMD_STATUS) { print_status(control); print_encoder(); motion_status(control); return ESP_OK; }
    if (command->kind == TB_CMD_ENCODER) { print_encoder(); return ESP_OK; }
    if (command->kind == TB_CMD_STANDBY) return balance_stop(control, BALANCE_DISARMED);
    if (control->balance.state == BALANCE_ARMED || control->motion_owned) return ESP_ERR_INVALID_STATE;
    if (command->kind == TB_CMD_ENCODER_ZERO) {
        err = car_encoder_zero();
        if (err != ESP_OK) return err;
        encoder = (car_encoder_sample_t){0};
        encoder_us = esp_timer_get_time();
        print_encoder();
        return ESP_OK;
    }
    if (command->kind == TB_CMD_ENABLE || command->kind == TB_CMD_STANDBY) {
        bool enable = command->kind == TB_CMD_ENABLE;
        err = tb6612fng_enable(control->motor, enable);
        if (err == ESP_OK) {
            if (!enable || !control->enabled) clear_commands(control);
            control->enabled = enable;
            control->control_mode = enable ? CAR_MODE_MANUAL : CAR_MODE_STANDBY;
        }
    } else {
        if (!control->enabled) return ESP_ERR_INVALID_STATE;
        if (command->kind == TB_CMD_SPEED && command->motor_mask == 3) {
            err = tb6612fng_set_speeds(control->motor,
                                     command->a_percent * 10 * motor_direction[0],
                                     command->b_percent * 10 * motor_direction[1]);
        } else {
            for (int i = 0; i < 2 && err == ESP_OK; ++i) {
                if (!(command->motor_mask & (1U << i))) continue;
                tb6612fng_motor_t motor = (tb6612fng_motor_t)i;
                if (command->kind == TB_CMD_SPEED) {
                    int p = i == 0 ? command->a_percent : command->b_percent;
                    err = tb6612fng_set_speed(control->motor, motor, p * 10 * motor_direction[i]);
                } else if (command->kind == TB_CMD_COAST) {
                    err = tb6612fng_coast(control->motor, motor);
                } else {
                    err = tb6612fng_brake(control->motor, motor);
                }
            }
        }
        if (err == ESP_OK) {
            for (int i = 0; i < 2; ++i) {
                if (!(command->motor_mask & (1U << i))) continue;
                control->mode[i] = command->kind;
                control->percent[i] = command->kind == TB_CMD_SPEED ?
                    (i == 0 ? command->a_percent : command->b_percent) : 0;
            }
        }
    }
    if (err != ESP_OK) {
        (void)tb6612fng_enable(control->motor, false);
        control->enabled = false;
        control->control_mode = CAR_MODE_STANDBY;
        clear_commands(control);
    }
    return err;
}

#define DIAG_DURATION_US INT64_C(10000000)
#define DIAG_PERIOD_US INT64_C(20000)
#define DIAG_MAX_SAMPLE_AGE_US INT64_C(50000)

static bool diagnostic_command(control_t *c, const char *line)
{
    char verb[16], arg[16], extra;
    if (sscanf(line, "%15s", verb) != 1 || strcmp(verb, "diag")) return false;
    if (sscanf(line, "%15s %15s %c", verb, arg, &extra) != 2 ||
        (strcmp(arg, "start") && strcmp(arg, "stop"))) {
        reply("ERR: diag start|stop (10 seconds at up to 50 Hz; motors unchanged)");
        return true;
    }
    const int64_t now = esp_timer_get_time();
    c->diag_last_us = now;
    c->diag_sample_us = 0;
    c->diag_end_us = !strcmp(arg, "start") ? now + DIAG_DURATION_US : 0;
    c->telemetry_last_us = now;
    if (c->diag_end_us) {
        reply("# DIAG start: 10 s, up to 50 Hz; other periodic output paused");
        reply("# DIAG,imu_us,enc_us,tilt_deg,rate_dps,filtered_rate_dps,cpsL,cpsR,pwmL,pwmR,P,I,D,state");
    } else reply("# DIAG stopped; normal telemetry restored");
    return true;
}

static bool diagnostic_due(control_t *c, int64_t now_us, int64_t sample_us)
{
    if (!c->diag_end_us) return false;
    if (now_us >= c->diag_end_us) {
        c->diag_end_us = 0;
        c->telemetry_last_us = now_us;
        reply("# DIAG done; normal telemetry restored");
        return false;
    }
    if (now_us - c->diag_last_us < DIAG_PERIOD_US || sample_us <= 0 ||
        sample_us <= c->diag_sample_us || sample_us > now_us ||
        now_us - sample_us > DIAG_MAX_SAMPLE_AGE_US) return false;
    c->diag_last_us = now_us;
    c->diag_sample_us = sample_us;
    return true;
}

static bool telemetry_command(control_t *c, const char *line)
{
    char verb[16], arg[16], extra;
    if (sscanf(line, "%15s", verb) != 1 || strcmp(verb, "telemetry")) return false;
    if (sscanf(line, "%15s %15s %c", verb, arg, &extra) != 2) {
        reply("ERR: telemetry off|slow|fast|status");
        return true;
    }
    if (!strcmp(arg, "status")) {
        reply("telemetry=%s", c->telemetry_mode == TELEMETRY_OFF ? "off" :
              (c->telemetry_mode == TELEMETRY_FAST ? "fast" : "slow (1 Hz)"));
        return true;
    }
    telemetry_mode_t mode;
    if (!strcmp(arg, "off")) mode = TELEMETRY_OFF;
    else if (!strcmp(arg, "slow")) mode = TELEMETRY_SLOW;
    else if (!strcmp(arg, "fast")) mode = TELEMETRY_FAST;
    else { reply("ERR: telemetry off|slow|fast|status"); return true; }
    c->telemetry_mode = mode;
    if (mode == TELEMETRY_OFF) c->diag_end_us = 0;
    c->telemetry_last_us = esp_timer_get_time();
    reply("OK: telemetry %s", arg);
    return true;
}

/* Output cadence only: never changes PID state, target ramps or motor enable. */
static bool telemetry_due(control_t *c, int64_t now_us)
{
    if (c->telemetry_mode == TELEMETRY_OFF) return false;
    int64_t period = TELEMETRY_SLOW_PERIOD_US;
    if (c->telemetry_mode == TELEMETRY_FAST) {
#if CONFIG_CAR_PID_TUNER
        period = (int64_t)CONFIG_CAR_PID_TUNER_PERIOD_MS * 1000;
#else
        period = TELEMETRY_ENCODER_PERIOD_US;
#endif
    }
    if (now_us - c->telemetry_last_us < period) return false;
    c->telemetry_last_us = now_us;
    return true;
}

static void receive_line(const char *line, void *argument)
{
    control_t *control = argument;
    if (!line) { reply("ERR: discarded line (too long or invalid character)"); return; }
    if (diagnostic_command(control, line)) { publish_snapshot(control); return; }
    if (telemetry_command(control, line)) { publish_snapshot(control); return; }
    if (motion_command(control, line)) { publish_snapshot(control); return; }
#if CONFIG_CAR_PID_TUNER
    if (tuner_command(control, line)) { publish_snapshot(control); return; }
#endif
    if (balance_command(control, line)) { publish_snapshot(control); return; }
    tb6612_command_t command;
    esp_err_t err = tb6612_command_parse(line, &command);
    if (err != ESP_OK) { reply("ERR: invalid command; send help"); return; }
    err = apply_command(control, &command);
    publish_snapshot(control);
    if (err == ESP_ERR_INVALID_STATE) reply("ERR: standby requires enable; balance mode requires balance off before manual commands");
    else if (err != ESP_OK) reply("ERR: %s", esp_err_to_name(err));
    else reply("OK: %s", line);
}

static bool poll_input(int fd, control_t *control, car_command_stream_t *input)
{
    uint8_t buffer[64];
    ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
    if (count > 0) car_command_stream_feed(input, buffer, count, receive_line, control);
    return true;
}

#if CONFIG_CAR_PID_TUNER
/* One CSV line per CONFIG_CAR_PID_TUNER_PERIOD_MS (default 50 ms, 20 Hz). A
 * single printf keeps the line intact even if a higher-priority task preempts
 * us. Slower periods help wireless serial links that cannot sustain 20 Hz. */
static void tuner_stream(const control_t *control, int64_t now_us)
{
    imu_input_t imu;
    portENTER_CRITICAL(&snapshot_lock);
    imu = latest_imu;
    portEXIT_CRITICAL(&snapshot_lock);
    const bool armed = control->balance.state == BALANCE_ARMED;
    const float input = imu.tilt - control->trim_base_deg;
    const float setpoint = armed ?
        control->balance_output.target_tilt_deg - control->trim_base_deg :
        control->tuner_setpoint_deg;
    const float pwm = armed ? control->balance_output.attitude_percent : 0.0f;
    const pid_config_t *g = &control->balance.config.attitude;
    printf("%" PRId64 ",%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\r\n",
           now_us / 1000, (double)setpoint, (double)input, (double)pwm,
           (double)(setpoint - input), (double)g->kp, (double)g->ki, (double)g->kd);
}
#endif

/* Read-only diagnostic. While armed, log the input used by the PID rather
 * than a newer mailbox value, so the PID terms and motor demand match the row. */
static void diagnostic_report(control_t *c, int64_t now_us)
{
    if (now_us >= c->diag_end_us) { (void)diagnostic_due(c, now_us, 0); return; }
    imu_input_t imu;
    portENTER_CRITICAL(&snapshot_lock);
    imu = latest_imu;
    portEXIT_CRITICAL(&snapshot_lock);
    const bool armed = c->balance.state == BALANCE_ARMED;
    if (armed) {
        imu.us = c->control_sample_us;
        imu.tilt = c->control_tilt_deg;
        imu.rate = c->control_rate_dps;
    }
    if (!imu.valid || !isfinite(imu.tilt) || !isfinite(imu.rate) ||
        !diagnostic_due(c, now_us, imu.us)) return;
#if CONFIG_CAR_PID_TUNER
    const float upright = c->trim_base_deg;
#else
    const float upright = c->balance.config.trim_deg;
#endif
    const float left = armed ? c->balance_output.left_percent : c->percent[0];
    const float right = armed ? c->balance_output.right_percent : c->percent[1];
    printf("DIAG,%" PRId64 ",%" PRId64 ",%.3f,%.3f,%.3f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%d\r\n",
           imu.us, encoder_us, (double)(imu.tilt-upright), (double)imu.rate,
           (double)c->balance.attitude.filtered_rate, (double)encoder.cps[0], (double)encoder.cps[1],
           (double)left, (double)right, (double)c->balance.attitude.p,
           (double)c->balance.attitude.i, (double)c->balance.attitude.d, c->balance.state);
}

/* All periodic output is gated here; command replies/faults bypass this gate. */
static void telemetry_report(control_t *c, int64_t now_us)
{
    if (c->diag_end_us) { diagnostic_report(c, now_us); return; }
    if (!telemetry_due(c, now_us)) return;
    if (c->telemetry_mode == TELEMETRY_FAST) {
#if CONFIG_CAR_PID_TUNER
        tuner_stream(c, now_us);
#else
        print_encoder();
#endif
        return;
    }
    imu_input_t imu;
    portENTER_CRITICAL(&snapshot_lock);
    imu = latest_imu;
    portEXIT_CRITICAL(&snapshot_lock);
    const bool valid = imu.valid && imu.us > 0 && now_us >= imu.us &&
                       now_us - imu.us <= 50000 && isfinite(imu.tilt) && isfinite(imu.rate);
    if (!valid) {
        reply("# STATE mode=%s,balance_state=%d,imu_valid=0",
              control_mode_name(c->control_mode), c->balance.state);
        return;
    }
#if CONFIG_CAR_PID_TUNER
    const float upright = c->trim_base_deg;
#else
    const float upright = c->balance.config.trim_deg;
#endif
    const bool armed = c->balance.state == BALANCE_ARMED;
    const float left = armed ? c->balance_output.left_percent : c->percent[0];
    const float right = armed ? c->balance_output.right_percent : c->percent[1];
    reply("# STATE mode=%s,state=%d,tilt=%.2f,rate=%.2f,cpsL=%.1f,cpsR=%.1f,pwmL=%.1f,pwmR=%.1f",
          control_mode_name(c->control_mode), c->balance.state,
          (double)(imu.tilt - upright), (double)imu.rate,
          (double)encoder.cps[0], (double)encoder.cps[1], (double)left, (double)right);
}

static void motor_task(void *argument)
{
    control_t control = {.motor = argument};
    balance_config_t balance_cfg = balance_config_default();
    balance_cfg.mapping_confirmed = true;
    balance_cfg.attitude_to_motor_sign = -1; /* Forward tilt => forward wheel output. */
    balance_cfg.max_output_percent = 20;
    balance_cfg.attitude.output_min = -20;
    balance_cfg.attitude.output_max = 20;
    balance_cfg.fall_angle_deg = 25;
    (void)balance_controller_init(&control.balance, &balance_cfg);
#if !defined(CAR_HOST_TEST)
    /* Restore a previous tune so a power cycle does not wipe the gains and
     * upright reference. Faults and the armed state are never persisted. */
    {
        esp_err_t nvs_err = nvs_flash_init();
        if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            (void)nvs_flash_erase();
            nvs_err = nvs_flash_init();
        }
        bool trim_valid = false;
        if (nvs_err == ESP_OK && store_load_config(&balance_cfg, &trim_valid)) {
            (void)balance_controller_init(&control.balance, &balance_cfg);
            if (trim_valid) {
                control.trim_confirmed = true;
#if CONFIG_CAR_PID_TUNER
                control.trim_base_deg = balance_cfg.trim_deg;
                control.tuner_setpoint_deg = 0;
#endif
            }
            reply("# restored P=%.3f I=%.3f D=%.3f trim=%.3f%s",
                  (double)balance_cfg.attitude.kp, (double)balance_cfg.attitude.ki,
                  (double)balance_cfg.attitude.kd, (double)balance_cfg.trim_deg,
                  trim_valid ? "" : " (no trim)");
        } else if (nvs_err != ESP_OK) {
            ESP_LOGW(TAG, "NVS init failed: %s", esp_err_to_name(nvs_err));
        }
    }
#endif
    car_command_stream_t primary = {0}, secondary = {0};
#if CONFIG_CAR_BLE_UART
    car_command_stream_t bluetooth = {0};
    uint32_t ble_session = car_ble_uart_session();
#endif
    clear_commands(&control);
    int usb_fd = -1;
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_fd = open("/dev/secondary", O_RDONLY | O_NONBLOCK);
    if (usb_fd < 0) ESP_LOGW(TAG, "USB command input unavailable; use UART console");
#endif
    /* Every power-on/reset waits for explicit enable and speed commands. */
    const tb6612_command_t boot_standby = {.kind = TB_CMD_STANDBY};
    esp_err_t err = apply_command(&control, &boot_standby);
    if (err != ESP_OK) goto stopped;
    reply("Motor control ready: STANDBY, STBY=0; waiting for serial commands");
#if CONFIG_CAR_PID_TUNER
    reply("# LLM-PID-Tuner ready: fast CSV at %d Hz; boot output is slow (1 Hz)",
          1000 / CONFIG_CAR_PID_TUNER_PERIOD_MS);
    reply("# Uppercase: STATUS | SET P:.. I:.. D:.. | SETPOINT:<deg> | RESET | STANDBY");
#endif
    publish_snapshot(&control);
#if !CONFIG_CAR_PID_TUNER
    print_help();
#endif
    int64_t last_sample_us = esp_timer_get_time();
    control.telemetry_last_us = last_sample_us;
    reply("# telemetry slow: 1 Hz; use telemetry off|slow|fast|status");
    while (1) {
#if CONFIG_CAR_BLE_UART
        reply_to_ble = false;
#endif
        if (!poll_input(STDIN_FILENO, &control, &primary) ||
            (usb_fd >= 0 && !poll_input(usb_fd, &control, &secondary))) {
            err = ESP_FAIL;
            break;
        }
#if CONFIG_CAR_BLE_UART
        /* Temporarily disabled by CONFIG_CAR_BLE_UART; keep for later reuse. */
        car_ble_packet_t packet;
        /* Bound work per iteration; a flooding client cannot starve sampling. */
        reply_to_ble = true;
        for (int n = 0; n < 8 && car_ble_uart_receive(&packet); ++n) {
            uint32_t current = car_ble_uart_session();
            if (ble_session != current) {
                bluetooth = (car_command_stream_t){0};
                ble_session = current;
            }
            if (packet.session != current) continue;
            car_command_stream_feed(&bluetooth, packet.data, packet.length, receive_line, &control);
        }
#endif
        int64_t now = esp_timer_get_time();
        if (now - last_sample_us >= 20000) {
            err = car_encoder_sample(&encoder);
            if (err != ESP_OK) break;
            encoder_us = esp_timer_get_time();
            publish_snapshot(&control);
            last_sample_us = now;
        }
        err = balance_tick(&control);
        if (err != ESP_OK) break;
        publish_snapshot(&control);
        telemetry_report(&control, esp_timer_get_time());
        vTaskDelay(pdMS_TO_TICKS(2) > 0 ? pdMS_TO_TICKS(2) : 1);
    }
stopped:
    balance_controller_disarm(&control.balance);
    control.balance_output = (balance_output_t){.state = BALANCE_FAULT_INPUT};
    (void)tb6612fng_enable(control.motor, false);
    control.enabled = false;
    control.control_mode = CAR_MODE_STANDBY;
    control.requested_cps = control.requested_yaw_dps = 0;
    control.target_cps = control.target_yaw_dps = 0;
    clear_commands(&control);
    encoder_us = 0;
    publish_snapshot(&control);
    ESP_LOGE(TAG, "Control failed: %s; both motors commanded to standby", esp_err_to_name(err));
    if (usb_fd >= 0) close(usb_fd);
    car_encoder_deinit();
    (void)tb6612fng_deinit(control.motor);
    vTaskDelete(NULL);
}

esp_err_t car_motor_control_start(void)
{
    const tb6612fng_config_t config = TB6612FNG_CONFIG_DEFAULT();
    tb6612fng_handle_t motor = NULL;
    esp_err_t err = tb6612fng_init(&config, &motor);
    if (err != ESP_OK) return err;
    err = car_encoder_init();
    if (err != ESP_OK) { (void)tb6612fng_deinit(motor); return err; }
#if CONFIG_CAR_BLE_UART
    /* BLE startup is excluded while the external wireless UART is in use. */
    err = car_ble_uart_start();
    if (err != ESP_OK) ESP_LOGE(TAG, "BLE unavailable: %s; wired commands remain available", esp_err_to_name(err));
#endif
#if CONFIG_ESP_CONSOLE_UART
    /* Buffered RX while the IMU prints to the same console. The TX ring buffer
     * keeps the 20 Hz tuner CSV line from blocking the control task. */
    bool installed = false;
    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
        err = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 1024, 2048, 0, NULL, 0);
        if (err != ESP_OK) goto failed;
        installed = true;
    }
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);
#endif
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        err = ESP_FAIL;
        goto failed;
    }
    if (xTaskCreate(motor_task, "car_motor", 6144, motor, 5, NULL) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto failed;
    }
    return ESP_OK;
failed:
#if CONFIG_ESP_CONSOLE_UART
    if (installed) {
        uart_vfs_dev_use_nonblocking(CONFIG_ESP_CONSOLE_UART_NUM);
        (void)uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);
    }
#endif
    car_encoder_deinit();
    (void)tb6612fng_deinit(motor);
    return err;
}
