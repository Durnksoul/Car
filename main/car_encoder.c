#include "car_encoder.h"
#include <stdbool.h>
#include <stddef.h>
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* Motor A = left wheel, motor B = right wheel. */
static const int pins[2][2] = {{17, 18}, {1, 2}};
static pcnt_unit_handle_t units[2];
static pcnt_channel_handle_t channels[2][2];
static bool enabled[2], started[2];
static int previous[2];
static int64_t totals[2], last_us;

void car_encoder_deinit(void)
{
    for (int i = 0; i < 2; ++i) {
        if (started[i]) (void)pcnt_unit_stop(units[i]);
        if (enabled[i]) (void)pcnt_unit_disable(units[i]);
        for (int j = 0; j < 2; ++j) {
            if (channels[i][j]) (void)pcnt_del_channel(channels[i][j]);
            channels[i][j] = NULL;
        }
        if (units[i]) (void)pcnt_del_unit(units[i]);
        units[i] = NULL;
        enabled[i] = started[i] = false;
    }
}

esp_err_t car_encoder_init(void)
{
    esp_err_t err;
#define TRY(call) do { err = (call); if (err != ESP_OK) goto failed; } while (0)
    for (int i = 0; i < 2; ++i) {
        const pcnt_unit_config_t unit = {
            .low_limit = -30000, .high_limit = 30000,
            .flags.accum_count = true,
        };
        TRY(pcnt_new_unit(&unit, &units[i]));
        const pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
        TRY(pcnt_unit_set_glitch_filter(units[i], &filter));
        for (int j = 0; j < 2; ++j) {
            const pcnt_chan_config_t channel = {
                .edge_gpio_num = pins[i][j], .level_gpio_num = pins[i][1 - j],
            };
            TRY(pcnt_new_channel(units[i], &channel, &channels[i][j]));
            TRY(pcnt_channel_set_edge_action(channels[i][j],
                j == 0 ? PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                j == 0 ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE));
            TRY(pcnt_channel_set_level_action(channels[i][j],
                PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            TRY(gpio_set_pull_mode(pins[i][j], GPIO_PULLUP_ONLY));
        }
        /* Both limits are needed for the driver's overflow accumulator. */
        TRY(pcnt_unit_add_watch_point(units[i], -30000));
        TRY(pcnt_unit_add_watch_point(units[i], 30000));
        TRY(pcnt_unit_enable(units[i]));
        enabled[i] = true;
        TRY(pcnt_unit_clear_count(units[i]));
        TRY(pcnt_unit_start(units[i]));
        started[i] = true;
        previous[i] = 0;
        totals[i] = 0;
    }
    last_us = esp_timer_get_time();
    return ESP_OK;
failed:
    car_encoder_deinit();
    return err;
#undef TRY
}

esp_err_t car_encoder_sample(car_encoder_sample_t *out)
{
    if (!out || !started[0] || !started[1]) return ESP_ERR_INVALID_STATE;
    int raw[2];
    for (int i = 0; i < 2; ++i) {
        esp_err_t err = pcnt_unit_get_count(units[i], &raw[i]);
        if (err != ESP_OK) return err;
    }
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - last_us;
    for (int i = 0; i < 2; ++i) {
        /* Modular subtraction also handles the 32-bit driver counter wrapping. */
        uint32_t difference = (uint32_t)raw[i] - (uint32_t)previous[i];
        int64_t delta = difference <= INT32_MAX ? (int64_t)difference :
                        (int64_t)difference - INT64_C(4294967296);
        totals[i] += delta;
        previous[i] = raw[i];
        out->count[i] = totals[i];
        out->cps[i] = elapsed > 0 ? (float)((double)delta * 1000000.0 / elapsed) : 0;
    }
    last_us = now;
    return ESP_OK;
}

esp_err_t car_encoder_zero(void)
{
    /* Software zero: keep hardware running so no pulses are lost by clearing. */
    car_encoder_sample_t sample;
    esp_err_t err = car_encoder_sample(&sample);
    if (err == ESP_OK) totals[0] = totals[1] = 0;
    return err;
}
