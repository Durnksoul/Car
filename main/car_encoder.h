#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int64_t count[2];
    float cps[2]; /* signed quadrature counts per second, not RPM */
} car_encoder_sample_t;

esp_err_t car_encoder_init(void);
esp_err_t car_encoder_sample(car_encoder_sample_t *out);
esp_err_t car_encoder_zero(void);
void car_encoder_deinit(void);
