/* GPL-3.0-or-later. Signed actuator-limit feedback shared by FW controllers. */
#pragma once

namespace AP_ExternalLimit {
struct Result {
    float integrator;
    bool limited;
};

inline Result apply(float previous, float requested, float residual)
{
    // A stored I opposing the saturated command can unwind all the way to zero.
    // Once at zero, forbid growth in the saturated direction, including a step
    // that crosses through zero. Growth away from the limit remains permitted.
    const float upper = previous > 0 ? previous : 0;
    const float lower = previous < 0 ? previous : 0;
    if (residual > 0 && requested > upper) {
        return {upper, true};
    }
    if (residual < 0 && requested < lower) {
        return {lower, true};
    }
    return {requested, false};
}
} // namespace AP_ExternalLimit
