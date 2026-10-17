/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef esp_err_t (*imu660rc_register_read_fn)(void *context, uint8_t reg,
                                              uint8_t *data, size_t length);
/* Drain a bounded snapshot of FIFO records, select the NEWEST SFLP record.
 * NOT_FOUND: no SFLP; INVALID_RESPONSE: newest SFLP invalid;
 * INVALID_STATE: overrun detected (snapshot drained, sample discarded).
 * Raw packet/status remain available for diagnostics; angles change only on OK. */
esp_err_t imu660rc_fifo_read(imu660rc_register_read_fn read, void *context,
                            float angles[3], uint8_t last_packet[7], uint8_t status[2]);
