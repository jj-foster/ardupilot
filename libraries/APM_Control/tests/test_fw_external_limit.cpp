/* GPL-3.0-or-later. Actual fixed-wing controller and AC_PID backend. */
#include <AP_gtest.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "../AP_FW_Controller.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
static AP_AHRS ahrs;
static AP_Scheduler scheduler;
static AP_FixedWing parameters;

class TestFWController : public AP_FW_Controller {
public:
    TestFWController() : AP_FW_Controller(parameters, {0, 1, 0, 0, .5f, 0, 0, 0, 0, 1, 0}, AP_AutoTune::AUTOTUNE_ROLL)
    {
        _last_out = 0;
        autotune = nullptr;
        failed_autotune_alloc = false;
        angle_err_deg = 0;
        rate_pid.reset_I();
    }
    float get_servo_out(int32_t, float scaler, bool disable, bool ground) override
    {
        return _get_rate_out(30, scaler, disable, 20, ground);
    }
    void set_i(float value) { rate_pid.set_integrator(value); }
    float i() const { return rate_pid.get_i(); }
    bool underspeed = false;
protected:
    bool is_underspeed(float) const override { return underspeed; }
    float get_airspeed() const override { return 20; }
    float get_measured_rate() const override { return 0; }
};

TEST(FWExternalLimit, SaturationBlocksStateAndActualOutput)
{
    scheduler.init(nullptr, 0, 0);
    for (float sign : {-1.0f, 1.0f}) {
        // AP controller backends rely on zero-initialized vehicle storage.
        static TestFWController controller;
        controller.reset_I();
        controller.underspeed = false;
        for (unsigned i = 0; i < 100; i++) {
            controller.set_external_limit(sign*.1f);
            const float output = controller.get_rate_out(sign*30, 1);
            EXPECT_FLOAT_EQ(output, 0);
            EXPECT_FLOAT_EQ(controller.i(), 0);
            EXPECT_FLOAT_EQ(controller.get_pid_info().I, 0);
        }
        controller.set_external_limit(0);
        const float released = sign*controller.get_rate_out(sign*30, 1);
        EXPECT_GT(released, 0);
    }
}

TEST(FWExternalLimit, OpposingStoredIUnwindsAndUnderspeedStillFreezes)
{
    scheduler.init(nullptr, 0, 0);
    for (float sign : {-1.0f, 1.0f}) {
        // AP controller backends rely on zero-initialized vehicle storage.
        static TestFWController controller;
        controller.reset_I();
        controller.underspeed = false;
        controller.set_i(-sign*.1f);
        for (unsigned i = 0; i < 200; i++) {
            controller.set_external_limit(sign*.1f);
            controller.get_rate_out(sign*30, 1);
        }
        EXPECT_FLOAT_EQ(controller.i(), 0);
        controller.set_i(sign*.1f);
        controller.underspeed = true;
        for (unsigned i = 0; i < 20; i++) {
            controller.set_external_limit(0);
            controller.get_rate_out(sign*30, 1);
            EXPECT_FLOAT_EQ(controller.i(), sign*.1f);
        }
    }
}

AP_GTEST_MAIN()
