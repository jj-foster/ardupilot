/* GPL-3.0-or-later. Tests exercise the actual ArduPlane allocation helper. */
#include <AP_gtest.h>
#include "../../../ArduPlane/xtail_allocation.h"
#include <limits>

using XTailAllocation::allocate;

TEST(XTailAllocation, NeutralAndAxisSigns)
{
    const auto zero = allocate(0, 0, 0, .5f, .5f, .5f);
    for (float value : zero.output) {
        EXPECT_FLOAT_EQ(value, 0);
    }
    const float signs[3][4] = {{1, -1, 1, -1}, {-1, -1, -1, -1}, {-1, 1, 1, -1}};
    for (unsigned axis = 0; axis < 3; axis++) {
        for (float sign : {-1.0f, 1.0f}) {
            float inputs[3] {};
            inputs[axis] = sign;
            const auto result = allocate(inputs[0], inputs[1], inputs[2], .5f, .5f, .5f);
            for (unsigned tail = 0; tail < 4; tail++) {
                EXPECT_FLOAT_EQ(result.output[tail], .5f * sign * signs[axis][tail]);
            }
        }
    }
}

TEST(XTailAllocation, PitchPriorityAndLimitRelease)
{
    const auto mixed = allocate(1, 1, 1, .5f, .5f, .5f);
    EXPECT_FLOAT_EQ(mixed.lateral_scale, .5f);
    EXPECT_FLOAT_EQ(mixed.applied_axes[1], .5f);
    EXPECT_FLOAT_EQ(mixed.residual[0], .25f);
    EXPECT_FLOAT_EQ(mixed.residual[2], .25f);
    const auto full_pitch = allocate(1, 1, 1, 1, 1, 1);
    EXPECT_FLOAT_EQ(full_pitch.lateral_scale, 0);
    for (float value : full_pitch.output) {
        EXPECT_FLOAT_EQ(value, -1);
    }
    const auto released = allocate(0, 0, 0, 1, 1, 1);
    EXPECT_FLOAT_EQ(released.lateral_scale, 1);
    for (float residual : released.residual) {
        EXPECT_FLOAT_EQ(residual, 0);
    }
    const auto reversed = allocate(-1, -1, -1, .5f, .5f, .5f);
    for (unsigned i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(reversed.output[i], -mixed.output[i]);
    }
}

TEST(XTailAllocation, ZeroGainsAndNonfiniteInputs)
{
    const auto no_roll = allocate(2, 0, 0, 0, .5f, .5f);
    EXPECT_FLOAT_EQ(no_roll.residual[0], 0);
    const float bad[] = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
    for (float value : bad) {
        EXPECT_FALSE(allocate(value, 0, 0, 1, 1, 1).valid);
        EXPECT_FALSE(allocate(0, 0, 0, 1, value, 1).valid);
    }
    EXPECT_FALSE(allocate(0, 0, 0, -1, 1, 1).valid);
    EXPECT_FALSE(allocate(0, 0, 0, 1, 3, 1).valid);
}

TEST(XTailAllocation, ExhaustiveTravelAndAxisRecovery)
{
    for (int ri = -12; ri <= 12; ri++) {
        for (int pi = -12; pi <= 12; pi++) {
            for (int yi = -12; yi <= 12; yi++) {
                const float r = ri * .1f;
                const float p = pi * .1f;
                const float y = yi * .1f;
                const auto result = allocate(r, p, y, 1, 1, 1);
                ASSERT_TRUE(result.valid);
                for (float value : result.output) {
                    ASSERT_TRUE(std::isfinite(value));
                    EXPECT_LE(fabsf(value), 1);
                }
                const auto *out = result.output;
                EXPECT_NEAR((out[0]-out[1]+out[2]-out[3])*.25f, result.applied_axes[0], 1.0e-6f);
                EXPECT_NEAR(-(out[0]+out[1]+out[2]+out[3])*.25f, XTailAllocation::clamp(p, -1, 1), 1.0e-6f);
                EXPECT_NEAR((-out[0]+out[1]+out[2]-out[3])*.25f, result.applied_axes[2], 1.0e-6f);
                EXPECT_NEAR(result.applied_axes[0]*y, result.applied_axes[2]*r, 1.0e-6f);
            }
        }
    }
}

AP_GTEST_MAIN()
