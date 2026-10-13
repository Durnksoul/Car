#pragma once

#include "esp_err.h"

#define TB6612_COMMAND_LINE_SIZE 64

typedef enum {
    TB_CMD_HELP, TB_CMD_STATUS, TB_CMD_ENABLE, TB_CMD_STANDBY,
    TB_CMD_SPEED, TB_CMD_COAST, TB_CMD_BRAKE, TB_CMD_ENCODER, TB_CMD_ENCODER_ZERO,
} tb6612_command_kind_t;

typedef struct {
    tb6612_command_kind_t kind;
    unsigned motor_mask; /* A=1, B=2, both=3 */
    int a_percent;
    int b_percent;
} tb6612_command_t;

/* Strict, case-insensitive ASCII parser. Integer percentages -100..100.
 * No hardware operations. On invalid input, output is unchanged. */
esp_err_t tb6612_command_parse(const char *line, tb6612_command_t *out);
