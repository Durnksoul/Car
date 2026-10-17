/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "imu660rc_fifo.h"
#include "imu660rc_math.h"
#include <stdbool.h>
#include <string.h>

esp_err_t imu660rc_fifo_read(imu660rc_register_read_fn read, void *context,
                            float angles[3], uint8_t last_packet[7], uint8_t status[2])
{
    esp_err_t err = read(context, 0x1b, status, 2); /* FIFO_STATUS1/2 */
    if (err != ESP_OK) return err;
    const unsigned count = status[0] | ((unsigned)(status[1] & 1U) << 8);
    bool found = false, valid = false;
    float latest[3];
    /* Never chase a moving FIFO indefinitely. One count snapshot is at most
     * 511 records. New arrivals are handled by the next read call. */
    for (unsigned i = 0; i < count; ++i) {
        uint8_t packet[7];
        err = read(context, 0x78, packet, sizeof(packet));
        if (err != ESP_OK) return err;
        if ((packet[0] >> 3) != 0x13) continue;
        memcpy(last_packet, packet, sizeof(packet));
        found = true;
        valid = imu660rc_decode_fifo_angles(packet + 1, latest);
    }
    /* Overflow reported for this snapshot means records were lost. Reject
     * the batch even if one frame happened to decode. */
    if (status[1] & 0x48U) return ESP_ERR_INVALID_STATE;
    if (!found) return ESP_ERR_NOT_FOUND;
    if (!valid) return ESP_ERR_INVALID_RESPONSE;
    memcpy(angles, latest, sizeof(latest));
    return ESP_OK;
}
