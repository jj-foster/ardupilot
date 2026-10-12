/*
 * Native symmetric X-tail allocation. GPL-3.0-or-later.
 * Pure arithmetic: no parameters, servo writes or aerodynamic assumptions.
 */
#pragma once
#include <cmath>

namespace XTailAllocation {
struct Result {
    float requested[4] {};
    float output[4] {};
    float requested_axes[3] {};
    float applied_axes[3] {};
    float residual[3] {};
    float lateral_scale = 1.0f;
    bool valid = true;
};

inline float clamp(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

inline Result allocate(float roll, float pitch, float yaw, float gr, float gp, float gy)
{
    Result result;
    const float inputs[] = {roll, pitch, yaw, gr, gp, gy};
    for (float value : inputs) {
        if (!std::isfinite(value)) {
            result.valid = false;
            return result;
        }
    }
    if (gr < 0 || gp < 0 || gy < 0 || gr > 2 || gp > 2 || gy > 2) {
        result.valid = false;
        return result;
    }
    // Demands may exceed one after upstream tailsitter surface scaling.
    const float r = roll * gr;
    const float p = pitch * gp;
    const float y = yaw * gy;
    if (!std::isfinite(r) || !std::isfinite(p) || !std::isfinite(y)) {
        result.valid = false;
        return result;
    }
    const float lateral[] = {r-y, -r+y, r+y, -r-y};
    const float applied_pitch = clamp(p, -1, 1);
    const float pitch_output = -applied_pitch;
    for (unsigned i = 0; i < 4; i++) {
        result.requested[i] = -p + lateral[i];
        if (!std::isfinite(result.requested[i])) {
            result = Result{};
            result.valid = false;
            return result;
        }
        if (lateral[i] > 0) {
            result.lateral_scale = fminf(result.lateral_scale, (1-pitch_output)/lateral[i]);
        } else if (lateral[i] < 0) {
            result.lateral_scale = fminf(result.lateral_scale, (-1-pitch_output)/lateral[i]);
        }
    }
    result.lateral_scale = clamp(result.lateral_scale, 0, 1);
    for (unsigned i = 0; i < 4; i++) {
        result.output[i] = clamp(pitch_output + result.lateral_scale*lateral[i], -1, 1);
    }
    result.requested_axes[0] = r;
    result.requested_axes[1] = p;
    result.requested_axes[2] = y;
    result.applied_axes[0] = result.lateral_scale*r;
    result.applied_axes[1] = applied_pitch;
    result.applied_axes[2] = result.lateral_scale*y;
    for (unsigned i = 0; i < 3; i++) {
        result.residual[i] = result.requested_axes[i] - result.applied_axes[i];
    }
    return result;
}
} // namespace XTailAllocation
