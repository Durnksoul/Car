/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2022 SEEKFREE
 * Adapted from zf_device_imu660rc.c: binary16 quaternion / Euler conversion.
 */
#include "imu660rc_math.h"
#include <math.h>
#include <string.h>

static float half_to_float(uint16_t h)
{
    uint32_t sign = ((uint32_t)h & 0x8000u) << 16;
    uint32_t exponent = (h >> 10) & 0x1fu;
    uint32_t fraction = h & 0x3ffu;
    uint32_t bits;
    if (exponent == 0) {
        /* Exact conversion also handles signed zero and subnormals. */
        float value = ldexpf((float)fraction, -24);
        return sign ? -value : value;
    }
    if (exponent == 31) {
        bits = sign | 0x7f800000u | (fraction << 13);
    } else {
        bits = sign | ((exponent + 112u) << 23) | (fraction << 13);
    }
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool imu660rc_decode_angles(const uint8_t data[8], float angles_deg[3])
{
    float raw[4];
    float norm2 = 0.0f;
    for (int i = 0; i < 4; ++i) {
        raw[i] = half_to_float((uint16_t)data[i * 2] |
                               ((uint16_t)data[i * 2 + 1] << 8));
        if (!isfinite(raw[i])) {
            return false;
        }
        norm2 += raw[i] * raw[i];
    }
    if (!isfinite(norm2) || norm2 <= 0.000001f) {
        return false;
    }
    float norm = sqrtf(norm2);
    if (raw[3] < 0.0f) {
        norm = -norm;
    }
    const float x = raw[1] / norm;
    const float y = raw[2] / norm;
    const float z = raw[0] / norm;
    const float w = raw[3] / norm;
    float sin_pitch = 2.0f * (x * w - y * z);
    sin_pitch = fmaxf(-1.0f, fminf(1.0f, sin_pitch));
    const float rad_to_deg = 57.29577951308232f;
    angles_deg[0] = atan2f(2.0f * (y * w + x * z),
                          1.0f - 2.0f * (y * y + x * x)) * rad_to_deg;
    angles_deg[1] = -asinf(sin_pitch) * rad_to_deg;
    angles_deg[2] = atan2f(2.0f * (x * y + z * w),
                          1.0f - 2.0f * (x * x + z * z)) * rad_to_deg;
    if (angles_deg[2] < 0.0f) {
        angles_deg[2] += 360.0f;
    }
    return true;
}
