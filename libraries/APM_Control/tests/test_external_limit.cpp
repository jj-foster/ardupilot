/* GPL-3.0-or-later. Real helper used in FW rate and yaw damping paths. */
#include <AP_gtest.h>
#include "../AP_ExternalLimit.h"

TEST(ExternalLimit, GrowthAndUnwindingForBothSigns)
{
    for (float sign : {-1.0f, 1.0f}) {
        auto growth = AP_ExternalLimit::apply(sign*.1f, sign*.2f, sign);
        EXPECT_TRUE(growth.limited);
        EXPECT_FLOAT_EQ(growth.integrator, sign*.1f);
        auto unwind = AP_ExternalLimit::apply(sign*.1f, sign*.05f, sign);
        EXPECT_FALSE(unwind.limited);
        EXPECT_FLOAT_EQ(unwind.integrator, sign*.05f);
        auto oppose = AP_ExternalLimit::apply(-sign*.2f, -sign*.1f, sign);
        EXPECT_FALSE(oppose.limited);
        EXPECT_FLOAT_EQ(oppose.integrator, -sign*.1f);
        auto crossing = AP_ExternalLimit::apply(-sign*.1f, sign*.1f, sign);
        EXPECT_TRUE(crossing.limited);
        EXPECT_FLOAT_EQ(crossing.integrator, 0);
    }
}

TEST(ExternalLimit, SustainedDemandReversalAndRelease)
{
    float integrator = .1f;
    for (unsigned i = 0; i < 1000; i++) {
        integrator = AP_ExternalLimit::apply(integrator, integrator+.001f, 1).integrator;
    }
    EXPECT_FLOAT_EQ(integrator, .1f);
    for (unsigned i = 0; i < 1000; i++) {
        integrator = AP_ExternalLimit::apply(integrator, integrator-.001f, -1).integrator;
    }
    EXPECT_FLOAT_EQ(integrator, 0);
    integrator = AP_ExternalLimit::apply(integrator, .05f, 0).integrator;
    EXPECT_FLOAT_EQ(integrator, .05f);
}

AP_GTEST_MAIN()
