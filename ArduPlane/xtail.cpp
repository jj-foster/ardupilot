/* Native ArduPlane X-tail mixer. GPL-3.0-or-later. */
#include "Plane.h"
#include "xtail.h"

#if AP_PLANE_XTAIL_ENABLED
const AP_Param::GroupInfo XTail::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Native X-tail mixer enable
    // @Description: Enable symmetric four-tail roll, pitch and yaw allocation after tailsitter axis conversion. Requires exactly one output for each X-tail function.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, XTail, _enable, 0, AP_PARAM_FLAG_ENABLE),
    // @Param: R_GAIN
    // @DisplayName: X-tail roll contribution
    // @Description: Non-negative roll allocation gain. Full normalized output is calibrated by SERVOn_MIN/TRIM/MAX. Equal axis gains do not imply equal moment authority.
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("R_GAIN", 2, XTail, _roll_gain, 0.5),
    // @Param: P_GAIN
    // @DisplayName: X-tail pitch contribution
    // @Description: Non-negative pitch allocation gain. Pitch has priority over the shared roll and yaw contribution when travel is exhausted.
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("P_GAIN", 3, XTail, _pitch_gain, 0.5),
    // @Param: Y_GAIN
    // @DisplayName: X-tail yaw contribution
    // @Description: Non-negative yaw allocation gain. Roll and yaw are scaled together when travel is exhausted.
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("Y_GAIN", 4, XTail, _yaw_gain, 0.5),
    AP_GROUPEND
};

const SRV_Channel::Function XTail::functions[4] = {
    SRV_Channel::k_xtail_upper_left, SRV_Channel::k_xtail_upper_right,
    SRV_Channel::k_xtail_lower_left, SRV_Channel::k_xtail_lower_right
};

XTail::XTail()
{
    AP_Param::setup_object_defaults(this, var_info);
}

const char *XTail::configuration_error() const
{
    uint8_t counts[4] {};
    bool invalid_calibration = false;
    bool legacy_surface = false;
    for (uint8_t channel = 0; channel < NUM_SERVO_CHANNELS; channel++) {
        const auto *servo = SRV_Channels::srv_channel(channel);
        if (servo == nullptr) {
            continue;
        }
        const auto fn = servo->get_function();
        for (uint8_t i = 0; i < 4; i++) {
            if (fn == functions[i]) {
                counts[i]++;
                invalid_calibration |= !(servo->get_output_min() < servo->get_trim() &&
                                         servo->get_trim() < servo->get_output_max());
            }
        }
        legacy_surface |= fn == SRV_Channel::k_aileron || fn == SRV_Channel::k_elevator ||
                          fn == SRV_Channel::k_rudder || fn == SRV_Channel::k_elevon_left ||
                          fn == SRV_Channel::k_elevon_right || fn == SRV_Channel::k_vtail_left ||
                          fn == SRV_Channel::k_vtail_right || fn == SRV_Channel::k_dspoilerLeft1 ||
                          fn == SRV_Channel::k_dspoilerLeft2 || fn == SRV_Channel::k_dspoilerRight1 ||
                          fn == SRV_Channel::k_dspoilerRight2;
    }
    if (_enable != 0 && _enable != 1) {
        return "XT_ENABLE must be 0 or 1";
    }
    for (uint8_t count : counts) {
        if (!enabled() && count != 0) {
            return "X-tail outputs require XT_ENABLE=1";
        }
        if (enabled() && count != 1) {
            return "X-tail requires one of each function";
        }
    }
    if (!enabled()) {
        return nullptr;
    }
    if (invalid_calibration) {
        return "X-tail needs MIN < TRIM < MAX";
    }
    if (legacy_surface) {
        return "X-tail conflicts with legacy mixers";
    }
    const float gains[] = {_roll_gain, _pitch_gain, _yaw_gain};
    for (float gain : gains) {
        if (!std::isfinite(gain) || gain < 0 || gain > 2) {
            return "X-tail gains must be finite, 0..2";
        }
    }
    return nullptr;
}

bool XTail::complete() const
{
    return enabled() && configuration_error() == nullptr;
}

uint32_t XTail::channel_mask() const
{
    uint32_t mask = 0;
    for (uint8_t channel = 0; channel < NUM_SERVO_CHANNELS; channel++) {
        for (auto fn : functions) {
            if (SRV_Channels::channel_function(channel) == fn) {
                mask |= 1U << channel;
            }
        }
    }
    return mask;
}

bool XTail::vtol_controller() const
{
#if HAL_QUADPLANE_ENABLED
    auto &q = plane.quadplane;
    return q.initialised && q.tailsitter.enabled() &&
           ((q.tailsitter.active() && !q.tailsitter.in_vtol_transition()) ||
            (q.assisted_flight && !q.option_is_set(QuadPlane::Option::TAILSIT_Q_ASSIST_MOTORS_ONLY)));
#else
    return false;
#endif
}

void XTail::prepare_feedback()
{
    const bool vtol = vtol_controller();
    if (!complete() || _last_mode != uint8_t(plane.control_mode->mode_number()) || _last_vtol != vtol || _termination) {
        _last = XTailAllocation::Result{};
    }
    if (is_zero(_roll_gain)) {
        _last.residual[0] = 0;
    }
    if (is_zero(_pitch_gain)) {
        _last.residual[1] = 0;
    }
    if (is_zero(_yaw_gain)) {
        _last.residual[2] = 0;
    }
    plane.rollController.set_external_limit(vtol ? 0 : _last.residual[0]);
    plane.pitchController.set_external_limit(vtol ? 0 : _last.residual[1]);
    plane.yawController.set_external_limit(vtol ? 0 : _last.residual[2]);
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.initialised) {
        // Plane yaw -> VTOL roll; plane pitch -> VTOL pitch; plane roll -> negative VTOL yaw.
        // OR at the PID input, never overwrite or clear a motor saturation flag.
        plane.quadplane.attitude_control->set_external_limits(
            vtol && fabsf(_last.residual[2]) > 1.0e-6f,
            vtol && fabsf(_last.residual[1]) > 1.0e-6f,
            vtol && fabsf(_last.residual[0]) > 1.0e-6f);
    }
#endif
}

void XTail::output()
{
    if (!enabled()) {
        _last = XTailAllocation::Result{};
        // Prevent stale surface commands if enable was changed without reboot.
        for (auto fn : functions) {
            SRV_Channels::set_output_scaled(fn, 0);
        }
        return;
    }
    // ArduPlane logical outputs: positive aileron=right-wing-down, elevator=nose-up,
    // rudder=nose-right. VTOL conversion and gain scaling have already happened.
    const float r = _termination ? 1 : SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) / 4500.0f;
    const float p = _termination ? 1 : SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / 4500.0f;
    const float y = _termination ? 1 : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / 4500.0f;
    _last = complete() ? XTailAllocation::allocate(r, p, y, _roll_gain, _pitch_gain, _yaw_gain) : XTailAllocation::Result{};
    for (uint8_t i = 0; i < 4; i++) {
        SRV_Channels::set_output_scaled(functions[i], 4500.0f * _last.output[i]);
    }
    _last_mode = uint8_t(plane.control_mode->mode_number());
    _last_vtol = vtol_controller();
#if HAL_LOGGING_ENABLED
    const uint32_t now = AP_HAL::millis();
    if (now - _last_log_ms >= 50) {
        _last_log_ms = now;
        // 20 Hz rather than the 400 Hz output cycle; correlate with PID/RCOU/TSIT.
        AP::logger().WriteStreaming("XTAL", "TimeUS,Mode,Q,R,P,Y,UL,UR,LL,LR,Lam,RR,PR,YR",
                                   "QBBfffffffffff", AP_HAL::micros64(), _last_mode, uint8_t(_last_vtol),
                                   double(r), double(p), double(y), double(_last.output[0]), double(_last.output[1]),
                                   double(_last.output[2]), double(_last.output[3]), double(_last.lateral_scale),
                                   double(_last.residual[0]), double(_last.residual[1]), double(_last.residual[2]));
        AP::logger().WriteStreaming("XTRQ", "TimeUS,UL,UR,LL,LR,R,P,Y,RA,PA,YA",
                                   "Qffffffffff", AP_HAL::micros64(), double(_last.requested[0]),
                                   double(_last.requested[1]), double(_last.requested[2]), double(_last.requested[3]),
                                   double(_last.requested_axes[0]), double(_last.requested_axes[1]), double(_last.requested_axes[2]),
                                   double(_last.applied_axes[0]), double(_last.applied_axes[1]), double(_last.applied_axes[2]));
    }
#endif
}

void XTail::setup_IO_failsafe() const
{
    if (!enabled()) {
        return;
    }
    const auto termination = XTailAllocation::allocate(1, 1, 1, _roll_gain, _pitch_gain, _yaw_gain);
    for (uint8_t channel = 0; channel < NUM_SERVO_CHANNELS; channel++) {
        const auto *servo = SRV_Channels::srv_channel(channel);
        for (uint8_t i = 0; i < 4; i++) {
            if (servo != nullptr && servo->get_function() == functions[i]) {
                const float value = servo->get_reversed() ? -termination.output[i] : termination.output[i];
                const float span = value < 0 ? servo->get_trim() - servo->get_output_min() :
                                   servo->get_output_max() - servo->get_trim();
                const uint16_t pwm = servo->get_trim() + value * span;
                // Match the calibrated allocator output, including intermediate PWM.
                hal.rcout->set_failsafe_pwm(1U << channel, pwm);
            }
        }
    }
}
#endif // AP_PLANE_XTAIL_ENABLED
