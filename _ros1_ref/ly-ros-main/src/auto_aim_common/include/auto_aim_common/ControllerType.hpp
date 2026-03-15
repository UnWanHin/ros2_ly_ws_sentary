#pragma once
#include <cstdint>

namespace ly_auto_aim::inline controller
{
    struct ControlResult
    {
        uint8_t shoot_flag;  /// depprecated in sentry
        float pitch_setpoint;
        float yaw_setpoint;
        float pitch_actual_want;
        float yaw_actual_want;
        bool valid=true;  /// deprecated in sentry
    };
}