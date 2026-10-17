/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Decode four little-endian binary16 values from embedded page 3, 0x4c.
 * Retains SeekFree's component reorder, signs, and Euler axis convention. */
bool imu660rc_decode_angles(const uint8_t data[8], float angles_deg[3]);

/* FIFO SFLP tag 0x13 contains native X/Y/Z binary16, without W. Reconstruct
 * positive W per ST's example and retain the existing Euler axis convention.
 * A zero vector is identity. Reject non-finite/non-unit data, but tolerate
 * binary16 rounding at 180 degrees. Output is unchanged on failure. */
bool imu660rc_decode_fifo_angles(const uint8_t data[6], float angles_deg[3]);

/* Project world-up into the sensor X/Z plane and measure rotation about +Y.
 * Independent of world heading. Includes off-axis gyro terms in its rate;
 * for pure wheel-axis rotation the rate equals gyro_dps[Y]. No startup zero
 * or mechanical trim is applied. Returns false near a vertical Y axle or on
 * invalid inputs; outputs remain unchanged. */
bool imu660rc_decode_balance(const uint8_t data[6], const float gyro_dps[3],
                             float *tilt_deg, float *tilt_rate_dps);
