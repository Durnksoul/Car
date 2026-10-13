#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tb6612_command.h"
typedef struct {
    char line[TB6612_COMMAND_LINE_SIZE];
    size_t used;
    bool discard;
} car_command_stream_t;
typedef void (*car_command_line_fn)(const char *line, void *ctx);
/* NULL line reports a discarded command; callback runs synchronously. */
void car_command_stream_feed(car_command_stream_t *stream, const uint8_t *data,
                             size_t length, car_command_line_fn callback, void *ctx);
