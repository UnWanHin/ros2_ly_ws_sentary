#pragma once

#include <cmath>
#include <cstdint>

#include "aim_msgs/msg/control_angles.hpp"
#include "BasicTypes.hpp"

namespace LangYa::mpc_gimbal_protocol {

inline bool IsFiniteTrajectory(const aim_msgs::msg::ControlAngles & msg) noexcept
{
    return std::isfinite(msg.yaw) &&
           std::isfinite(msg.pitch) &&
           std::isfinite(msg.yaw_omega) &&
           std::isfinite(msg.pitch_omega) &&
           std::isfinite(msg.yaw_alpha) &&
           std::isfinite(msg.pitch_alpha);
}

inline GimbalTrajectoryFrame ToTrajectoryFrame(
    const aim_msgs::msg::ControlAngles & msg) noexcept
{
    GimbalTrajectoryFrame frame;
    frame.Yaw = msg.yaw;
    frame.Pitch = msg.pitch;
    frame.YawOmega = msg.yaw_omega;
    frame.PitchOmega = msg.pitch_omega;
    frame.YawAlpha = msg.yaw_alpha;
    frame.PitchAlpha = msg.pitch_alpha;
    return frame;
}

inline bool IsNewerSampleTick(
    const std::uint32_t current,
    const std::uint32_t previous) noexcept
{
    const auto delta = current - previous;
    return delta != 0U && delta < 0x80000000U;
}

}  // namespace LangYa::mpc_gimbal_protocol
