/* GPL-3.0-or-later. Actual multicopter rate PID limit and release behavior. */
#include <AP_gtest.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
static AP_AHRS ahrs;
static AP_AHRS_View view(ahrs, ROTATION_PITCH_90);
static AP_Scheduler scheduler;
static AP_MotorsMatrix motors;

class TestMultiController : public AC_AttitudeControl_Multi {
public:
    TestMultiController() : AC_AttitudeControl_Multi(view, motors) {}
    void demand(const Vector3f &value) { _ang_vel_body_rads = value; }
    void pure_integral()
    {
        set_dt_s(.0025f);
        AC_PID *pids[] = {&get_rate_roll_pid(), &get_rate_pitch_pid(), &get_rate_yaw_pid()};
        for (auto *pid : pids) {
            pid->set_kP(0);
            pid->set_kI(1);
            pid->set_kD(0);
            pid->set_ff(0);
            pid->set_kDff(0);
            pid->set_imax(.5f);
            pid->set_filt_T_hz(0);
            pid->set_filt_E_hz(0);
            pid->set_filt_D_hz(0);
            pid->reset_I();
            pid->reset_filter();
        }
    }
};

TEST(VTOLExternalLimit, SurfaceLimitAddsToMotorFlagsAndExpires)
{
    scheduler.init(nullptr, 0, 0);
    static TestMultiController controller;
    controller.pure_integral();
    motors.limit.roll = true;
    motors.limit.pitch = false;
    motors.limit.yaw = true;
    controller.demand({.5f, .5f, .5f});
    for (unsigned i = 0; i < 1000; i++) {
        controller.set_external_limits(false, true, false);
        controller.rate_controller_run_dt({}, .0025f);
        EXPECT_FLOAT_EQ(controller.get_rate_pitch_pid().get_i(), 0);
        EXPECT_FLOAT_EQ(motors.get_pitch(), 0);
        EXPECT_TRUE(controller.get_rate_pitch_pid().get_pid_info().limit);
        EXPECT_TRUE(motors.limit.roll);
        EXPECT_FALSE(motors.limit.pitch);
        EXPECT_TRUE(motors.limit.yaw);
    }
    controller.rate_controller_run_dt({}, .0025f);
    EXPECT_GT(controller.get_rate_pitch_pid().get_i(), 0);
    EXPECT_FALSE(controller.get_rate_pitch_pid().get_pid_info().limit);
    EXPECT_TRUE(motors.limit.roll);
    EXPECT_TRUE(motors.limit.yaw);
}

TEST(VTOLExternalLimit, ReversalPermitsUnwindingWithMotorAuthorityRemaining)
{
    scheduler.init(nullptr, 0, 0);
    static TestMultiController controller;
    controller.pure_integral();
    motors.limit.set_all(false);
    for (float sign : {-1.0f, 1.0f}) {
        controller.get_rate_pitch_pid().set_integrator(sign*.1f);
        controller.demand({0, -sign*.5f, 0});
        for (unsigned i = 0; i < 20; i++) {
            const float previous = fabsf(controller.get_rate_pitch_pid().get_i());
            controller.set_external_limits(false, true, false);
            controller.rate_controller_run_dt({}, .0025f);
            EXPECT_LT(fabsf(controller.get_rate_pitch_pid().get_i()), previous);
            EXPECT_FALSE(motors.limit.pitch);
        }
    }
}

AP_GTEST_MAIN()
