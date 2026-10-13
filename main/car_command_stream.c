#include "car_command_stream.h"
void car_command_stream_feed(car_command_stream_t *stream, const uint8_t *data,
                             size_t length, car_command_line_fn callback, void *ctx)
{
    for (size_t i = 0; i < length; ++i) {
        uint8_t c = data[i];
        if (c == '\r' || c == '\n') {
            if (stream->discard) callback(NULL, ctx);
            else if (stream->used) {
                stream->line[stream->used] = '\0';
                callback(stream->line, ctx);
            }
            stream->used = 0;
            stream->discard = false;
        } else if (c == 8 || c == 127) {
            if (!stream->discard && stream->used) --stream->used;
        } else if (!stream->discard) {
            if ((c < 32 && c != '\t') || c > 126 || stream->used >= sizeof(stream->line) - 1)
                stream->discard = true;
            else stream->line[stream->used++] = (char)c;
        }
    }
}
