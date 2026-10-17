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

static void quaternion_to_angles(float x, float y, float z, float w, float angles_deg[3])
{
    float sin_pitch = 2.0f * (x * w - y * z);
    sin_pitch = fmaxf(-1.0f, fminf(1.0f, sin_pitch));
    const float rad_to_deg = 57.29577951308232f;
    angles_deg[0] = atan2f(2.0f * (y * w + x * z),
                          1.0f - 2.0f * (y * y + x * x)) * rad_to_deg;
    angles_deg[1] = -asinf(sin_pitch) * rad_to_deg;
    angles_deg[2] = atan2f(2.0f * (x * y + z * w),
                          1.0f - 2.0f * (x * x + z * z)) * rad_to_deg;
    if (angles_deg[2] < 0.0f) angles_deg[2] += 360.0f;
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
    quaternion_to_angles(x, y, z, w, angles_deg);
    return true;
}

static bool fifo_to_quaternion(const uint8_t data[6], float q[4])
{
    float sum = 0;
    for (int i = 0; i < 3; ++i) {
        q[i] = half_to_float((uint16_t)data[2 * i] | ((uint16_t)data[2 * i + 1] << 8));
        if (!isfinite(q[i])) return false;
        sum += q[i] * q[i];
    }
    /* Three rounded binary16 unit components can sum slightly above 1.
     * A large excess is corrupted data, not a quaternion to normalize. */
    if (!isfinite(sum) || sum > 1.002f) return false;
    if (sum > 1.0f) {
        const float norm = sqrtf(sum);
        for (int i = 0; i < 3; ++i) q[i] /= norm;
        sum = 1.0f;
    }
    q[3] = sqrtf(1.0f - sum);
    return true;
}

bool imu660rc_decode_fifo_angles(const uint8_t data[6], float angles_deg[3])
{
    float q[4];
    if (!fifo_to_quaternion(data, q)) return false;
    quaternion_to_angles(q[0], q[1], q[2], q[3], angles_deg);
    return true;
}

bool imu660rc_decode_balance(const uint8_t data[6], const float gyro_dps[3],
                             float *tilt_deg, float *tilt_rate_dps)
{
    if (!data || !gyro_dps || !tilt_deg || !tilt_rate_dps) return false;
    for (int i = 0; i < 3; ++i) if (!isfinite(gyro_dps[i])) return false;
    float q[4];
    if (!fifo_to_quaternion(data, q)) return false;
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    /* q rotates body vectors into world coordinates. The last row of R(q)
     * is world-up expressed in body coordinates: R(q)^T * [0,0,1]. */
    const float up_x = 2.0f * (x * z - w * y);
    const float up_y = 2.0f * (y * z + w * x);
    const float up_z = 1.0f - 2.0f * (x * x + y * y);
    const float projection2 = up_x * up_x + up_z * up_z;
    /* A nearly vertical wheel axle makes this projected angle undefined. */
    if (projection2 < 0.000001f) return false;
    float tilt = atan2f(-up_x, up_z) * 57.29577951308232f;
    if (tilt >= 180.0f) tilt -= 360.0f;
    /* up_dot = -omega cross up, theta = atan2(-up_x, up_z).
     * This derivative is in deg/s when omega is supplied in deg/s. */
    const float rate = gyro_dps[1] - up_y *
        (gyro_dps[0] * up_x + gyro_dps[2] * up_z) / projection2;
    if (!isfinite(tilt) || !isfinite(rate)) return false;
    *tilt_deg = tilt;
    *tilt_rate_dps = rate;
    return true;
}
