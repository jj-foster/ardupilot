/* Native ArduPlane X-tail mixer. GPL-3.0-or-later. */
#pragma once
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>
#include "xtail_allocation.h"

#ifndef AP_PLANE_XTAIL_ENABLED
#define AP_PLANE_XTAIL_ENABLED 1
#endif

#if AP_PLANE_XTAIL_ENABLED
class XTail {
public:
    XTail();
    bool enabled() const { return _enable == 1; }
    bool complete() const;
    const char *configuration_error() const;
    uint32_t channel_mask() const;
    void prepare_feedback();
    void output();
    void set_termination() { _termination = true; }
    void setup_IO_failsafe() const;
    static const AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _enable;
    AP_Float _roll_gain;
    AP_Float _pitch_gain;
    AP_Float _yaw_gain;
    XTailAllocation::Result _last;
    uint8_t _last_mode = 255;
    bool _last_vtol = false;
    bool _termination = false;
    uint32_t _last_log_ms = 0;
    static const SRV_Channel::Function functions[4];
    bool vtol_controller() const;
};
#endif // AP_PLANE_XTAIL_ENABLED
