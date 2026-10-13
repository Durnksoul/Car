#include "tb6612fng.h"

#include <stdlib.h>

#define PWM_SCALE 1024U
#define TRY(call) do { esp_err_t result_ = (call); \
    if (result_ != ESP_OK) return result_; } while (0)

struct tb6612fng_device {
    tb6612fng_config_t config;
    bool enabled;
    bool stby_ready;
    bool motor_gpios_ready;
    bool timer_ready;
    unsigned channels_ready;
    int direction[2];
};

static bool valid_motor(tb6612fng_motor_t motor)
{
    return motor == TB6612FNG_MOTOR_A || motor == TB6612FNG_MOTOR_B;
}

static bool valid_speed(int speed)
{
    return speed >= -1000 && speed <= 1000;
}

static esp_err_t validate_config(const tb6612fng_config_t *config)
{
    if (!config || config->pwm_freq_hz == 0 || config->pwm_freq_hz > 100000 ||
        (unsigned)config->ledc_timer >= LEDC_TIMER_MAX ||
        config->motor[0].ledc_channel == config->motor[1].ledc_channel) {
        return ESP_ERR_INVALID_ARG;
    }
    gpio_num_t pins[7] = { config->stby_gpio };
    for (int i = 0; i < 2; ++i) {
        const tb6612fng_motor_config_t *m = &config->motor[i];
        if ((unsigned)m->ledc_channel >= LEDC_CHANNEL_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
        pins[1 + i * 3] = m->in1_gpio;
        pins[2 + i * 3] = m->in2_gpio;
        pins[3 + i * 3] = m->pwm_gpio;
    }
    for (int i = 0; i < 7; ++i) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) return ESP_ERR_INVALID_ARG;
        for (int j = 0; j < i; ++j) {
            if (pins[i] == pins[j]) return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

/* Keep shutdown best-effort: attempt all outputs even if one API fails. */
static esp_err_t enter_standby(tb6612fng_handle_t dev)
{
    esp_err_t first = ESP_OK;
#define RECORD(call) do { esp_err_t e_ = (call); \
    if (first == ESP_OK && e_ != ESP_OK) first = e_; } while (0)
    if (dev->stby_ready) RECORD(gpio_set_level(dev->config.stby_gpio, 0));
    dev->enabled = false;
    for (int i = 0; i < 2; ++i) {
        const tb6612fng_motor_config_t *m = &dev->config.motor[i];
        if (dev->channels_ready & (1U << i)) {
            RECORD(ledc_stop(LEDC_LOW_SPEED_MODE, m->ledc_channel, 0));
        }
        if (dev->motor_gpios_ready) {
            RECORD(gpio_set_level(m->in1_gpio, 0));
            RECORD(gpio_set_level(m->in2_gpio, 0));
        }
        dev->direction[i] = 0;
    }
#undef RECORD
    return first;
}

static esp_err_t checked_result(tb6612fng_handle_t dev, esp_err_t result)
{
    if (result != ESP_OK) (void)enter_standby(dev);
    return result;
}

static esp_err_t configure_device(tb6612fng_handle_t dev)
{
    /* Assert standby before configuring any direction or PWM pin. */
    TRY(gpio_set_level(dev->config.stby_gpio, 0));
    const gpio_config_t stby = {
        .pin_bit_mask = 1ULL << dev->config.stby_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    TRY(gpio_config(&stby));
    dev->stby_ready = true;
    TRY(gpio_set_level(dev->config.stby_gpio, 0));

    uint64_t mask = 0;
    for (int i = 0; i < 2; ++i) {
        const tb6612fng_motor_config_t *m = &dev->config.motor[i];
        TRY(gpio_set_level(m->in1_gpio, 0));
        TRY(gpio_set_level(m->in2_gpio, 0));
        TRY(gpio_set_level(m->pwm_gpio, 0));
        mask |= (1ULL << m->in1_gpio) | (1ULL << m->in2_gpio) | (1ULL << m->pwm_gpio);
    }
    const gpio_config_t outputs = {
        .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    TRY(gpio_config(&outputs));
    dev->motor_gpios_ready = true;
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = dev->config.ledc_timer,
        .freq_hz = dev->config.pwm_freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    TRY(ledc_timer_config(&timer));
    dev->timer_ready = true;
    for (int i = 0; i < 2; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = dev->config.motor[i].pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = dev->config.motor[i].ledc_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = dev->config.ledc_timer,
            .duty = 0, .hpoint = 0,
        };
        TRY(ledc_channel_config(&channel));
        dev->channels_ready |= 1U << i;
    }
    return enter_standby(dev);
}

esp_err_t tb6612fng_init(const tb6612fng_config_t *config, tb6612fng_handle_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    *out = NULL;
    esp_err_t err = validate_config(config);
    if (err != ESP_OK) return err;
    tb6612fng_handle_t dev = calloc(1, sizeof(*dev));
    if (!dev) return ESP_ERR_NO_MEM;
    dev->config = *config;
    err = configure_device(dev);
    if (err != ESP_OK) {
        (void)tb6612fng_deinit(dev);
        return err;
    }
    *out = dev;
    return ESP_OK;
}

static esp_err_t set_direction(tb6612fng_handle_t dev, int motor, int direction)
{
    const tb6612fng_motor_config_t *m = &dev->config.motor[motor];
    TRY(ledc_stop(LEDC_LOW_SPEED_MODE, m->ledc_channel, 0));
    /* Clear the old direction first so reversal never passes through IN1=IN2=1. */
    TRY(gpio_set_level(m->in1_gpio, 0));
    TRY(gpio_set_level(m->in2_gpio, 0));
    TRY(gpio_set_level(m->in1_gpio, direction > 0));
    TRY(gpio_set_level(m->in2_gpio, direction < 0));
    dev->direction[motor] = direction;
    return ESP_OK;
}

static esp_err_t coast_motor(tb6612fng_handle_t dev, int motor)
{
    TRY(set_direction(dev, motor, 0));
    /* PWM must be HIGH for the datasheet's IN1=IN2=0 STOP state. */
    return ledc_stop(LEDC_LOW_SPEED_MODE, dev->config.motor[motor].ledc_channel, 1);
}

esp_err_t tb6612fng_enable(tb6612fng_handle_t dev, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!enable) return enter_standby(dev);
    if (dev->enabled) return ESP_OK;
    esp_err_t err = coast_motor(dev, 0);
    if (err == ESP_OK) err = coast_motor(dev, 1);
    if (err == ESP_OK) err = gpio_set_level(dev->config.stby_gpio, 1);
    if (err == ESP_OK) dev->enabled = true;
    return checked_result(dev, err);
}

static esp_err_t speed_motor(tb6612fng_handle_t dev, int motor, int speed)
{
    if (speed == 0) return coast_motor(dev, motor);
    const int direction = speed > 0 ? 1 : -1;
    const ledc_channel_t channel = dev->config.motor[motor].ledc_channel;
    if (dev->direction[motor] != direction) TRY(set_direction(dev, motor, direction));
    const unsigned magnitude = speed > 0 ? (unsigned)speed : (unsigned)-speed;
    if (magnitude == 1000) return ledc_stop(LEDC_LOW_SPEED_MODE, channel, 1);
    const uint32_t duty = (magnitude * PWM_SCALE + 500U) / 1000U;
    TRY(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

esp_err_t tb6612fng_set_speed(tb6612fng_handle_t dev, tb6612fng_motor_t motor, int speed)
{
    if (!dev || !valid_motor(motor) || !valid_speed(speed)) return ESP_ERR_INVALID_ARG;
    if (!dev->enabled) return ESP_ERR_INVALID_STATE;
    return checked_result(dev, speed_motor(dev, motor, speed));
}

esp_err_t tb6612fng_set_speeds(tb6612fng_handle_t dev, int a, int b)
{
    if (!dev || !valid_speed(a) || !valid_speed(b)) return ESP_ERR_INVALID_ARG;
    if (!dev->enabled) return ESP_ERR_INVALID_STATE;
    esp_err_t err = speed_motor(dev, 0, a);
    if (err == ESP_OK) err = speed_motor(dev, 1, b);
    return checked_result(dev, err);
}

esp_err_t tb6612fng_coast(tb6612fng_handle_t dev, tb6612fng_motor_t motor)
{
    if (!dev || !valid_motor(motor)) return ESP_ERR_INVALID_ARG;
    if (!dev->enabled) return ESP_ERR_INVALID_STATE;
    return checked_result(dev, coast_motor(dev, motor));
}

static esp_err_t brake_motor(tb6612fng_handle_t dev, int motor)
{
    TRY(set_direction(dev, motor, 0));
    TRY(gpio_set_level(dev->config.motor[motor].in1_gpio, 1));
    return gpio_set_level(dev->config.motor[motor].in2_gpio, 1);
}

esp_err_t tb6612fng_brake(tb6612fng_handle_t dev, tb6612fng_motor_t motor)
{
    if (!dev || !valid_motor(motor)) return ESP_ERR_INVALID_ARG;
    if (!dev->enabled) return ESP_ERR_INVALID_STATE;
    return checked_result(dev, brake_motor(dev, motor));
}

esp_err_t tb6612fng_deinit(tb6612fng_handle_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t first = enter_standby(dev);
#define RECORD(call) do { esp_err_t e_ = (call); \
    if (first == ESP_OK && e_ != ESP_OK) first = e_; } while (0)
    if (dev->motor_gpios_ready) {
        for (int i = 0; i < 2; ++i) {
            RECORD(gpio_reset_pin(dev->config.motor[i].in1_gpio));
            RECORD(gpio_reset_pin(dev->config.motor[i].in2_gpio));
            RECORD(gpio_reset_pin(dev->config.motor[i].pwm_gpio));
        }
    }
    if (dev->timer_ready) {
        RECORD(ledc_timer_pause(LEDC_LOW_SPEED_MODE, dev->config.ledc_timer));
        const ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = dev->config.ledc_timer,
            .deconfigure = true,
        };
        RECORD(ledc_timer_config(&timer));
    }
#undef RECORD
    free(dev);
    return first;
}
