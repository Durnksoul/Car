/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Decode four little-endian binary16 values from embedded page 3, 0x4c.
 * Retains SeekFree's component reorder, signs, and Euler axis convention. */
bool imu660rc_decode_angles(const uint8_t data[8], float angles_deg[3]);
