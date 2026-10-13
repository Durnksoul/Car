#include "tb6612_command.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct { const char *text; size_t length; } token_t;

static bool space(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static bool equal(token_t token, const char *word)
{
    size_t i = 0;
    for (; i < token.length && word[i]; ++i) {
        char c = token.text[i];
        if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
        if (c != word[i]) return false;
    }
    return i == token.length && word[i] == '\0';
}

static bool percent(token_t token, int *out)
{
    size_t i = 0;
    int sign = 1, value = 0;
    if (!token.length) return false;
    if (token.text[0] == '-' || token.text[0] == '+') {
        sign = token.text[0] == '-' ? -1 : 1;
        ++i;
    }
    if (i == token.length) return false;
    for (; i < token.length; ++i) {
        char c = token.text[i];
        if (c < '0' || c > '9') return false;
        value = value * 10 + c - '0';
        if (value > 100) return false;
    }
    *out = value * sign;
    return true;
}

esp_err_t tb6612_command_parse(const char *line, tb6612_command_t *out)
{
    if (!line || !out) return ESP_ERR_INVALID_ARG;
    token_t tokens[3];
    size_t position = 0;
    unsigned count = 0;
    while (line[position]) {
        if (position >= TB6612_COMMAND_LINE_SIZE - 1) return ESP_ERR_INVALID_ARG;
        if (space(line[position])) { ++position; continue; }
        if (count == 3) return ESP_ERR_INVALID_ARG;
        size_t start = position;
        while (line[position] && !space(line[position])) {
            if (position >= TB6612_COMMAND_LINE_SIZE - 1) return ESP_ERR_INVALID_ARG;
            ++position;
        }
        tokens[count++] = (token_t){ .text = line + start, .length = position - start };
    }
    if (!count) return ESP_ERR_INVALID_ARG;
    tb6612_command_t command = {0};
    if (count == 1) {
        if (equal(tokens[0], "help")) command.kind = TB_CMD_HELP;
        else if (equal(tokens[0], "status")) command.kind = TB_CMD_STATUS;
        else if (equal(tokens[0], "enable")) command.kind = TB_CMD_ENABLE;
        else if (equal(tokens[0], "standby")) command.kind = TB_CMD_STANDBY;
        else if (equal(tokens[0], "enc")) command.kind = TB_CMD_ENCODER;
        else if (equal(tokens[0], "enczero")) command.kind = TB_CMD_ENCODER_ZERO;
        else return ESP_ERR_INVALID_ARG;
    } else if (count == 2 && (equal(tokens[0], "coast") || equal(tokens[0], "brake"))) {
        command.kind = equal(tokens[0], "coast") ? TB_CMD_COAST : TB_CMD_BRAKE;
        if (equal(tokens[1], "a")) command.motor_mask = 1;
        else if (equal(tokens[1], "b")) command.motor_mask = 2;
        else if (equal(tokens[1], "all")) command.motor_mask = 3;
        else return ESP_ERR_INVALID_ARG;
    } else if (count == 2 && (equal(tokens[0], "a") || equal(tokens[0], "b"))) {
        command.kind = TB_CMD_SPEED;
        if (equal(tokens[0], "a")) {
            command.motor_mask = 1;
            if (!percent(tokens[1], &command.a_percent)) return ESP_ERR_INVALID_ARG;
        } else {
            command.motor_mask = 2;
            if (!percent(tokens[1], &command.b_percent)) return ESP_ERR_INVALID_ARG;
        }
    } else if (count == 3 && equal(tokens[0], "ab")) {
        command.kind = TB_CMD_SPEED;
        command.motor_mask = 3;
        if (!percent(tokens[1], &command.a_percent) || !percent(tokens[2], &command.b_percent)) {
            return ESP_ERR_INVALID_ARG;
        }
    } else return ESP_ERR_INVALID_ARG;
    *out = command;
    return ESP_OK;
}
