#pragma once

#include <chrono>
#include <cmath>
#include <optional>

#include "../module/BasicTypes.hpp"
#include "gimbal_driver/msg/gimbal_trajectory.hpp"

namespace BehaviorTree {

struct AimSourceView {
    const LangYa::AimData* Active{nullptr};
    const LangYa::AimData* AutoAim{nullptr};
    const LangYa::AimData* Buff{nullptr};
    const LangYa::AimData* Outpost{nullptr};
    bool ExternalAimActive{false};
};

inline AimSourceView MakeAimSourceView(
    const LangYa::AimData& external_aim) noexcept {
    return AimSourceView{
        .Active = &external_aim,
        .AutoAim = &external_aim,
        .Buff = &external_aim,
        .Outpost = &external_aim,
        .ExternalAimActive = true,
    };
}

inline bool AimFreshAndValid(const LangYa::AimData& data) noexcept {
    return data.Fresh && data.Valid;
}

inline std::optional<gimbal_driver::msg::GimbalTrajectory> MakeGimbalTrajectory(
    const LangYa::AimData& data) noexcept {
    if (!data.Valid ||
        !std::isfinite(data.Angles.Yaw) ||
        !std::isfinite(data.Angles.Pitch) ||
        !std::isfinite(data.YawOmega) ||
        !std::isfinite(data.PitchOmega) ||
        !std::isfinite(data.YawAlpha) ||
        !std::isfinite(data.PitchAlpha)) {
        return std::nullopt;
    }

    gimbal_driver::msg::GimbalTrajectory trajectory;
    trajectory.yaw = data.Angles.Yaw;
    trajectory.pitch = data.Angles.Pitch;
    trajectory.yaw_omega = data.YawOmega;
    trajectory.pitch_omega = data.PitchOmega;
    trajectory.yaw_alpha = data.YawAlpha;
    trajectory.pitch_alpha = data.PitchAlpha;
    return trajectory;
}

inline bool AimBuffTargetLocked(
    const LangYa::AimData& data,
    const bool external_aim_active) noexcept {
    return AimFreshAndValid(data) && (external_aim_active || data.BuffFollow);
}

inline bool AimBuffFireReady(
    const LangYa::AimData& data,
    const bool external_aim_active) noexcept {
    return AimBuffTargetLocked(data, external_aim_active) && data.FireStatus;
}

inline bool AimLatchedRecently(
    const LangYa::AimData& data,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds hold) noexcept {
    if (!data.HasLatchedAngles ||
        data.LastValidTime.time_since_epoch().count() == 0 ||
        hold.count() <= 0) {
        return false;
    }
    return now - data.LastValidTime <= hold;
}

inline bool AimFreshOrLatchedRecently(
    const LangYa::AimData& data,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds hold) noexcept {
    return AimFreshAndValid(data) || AimLatchedRecently(data, now, hold);
}

inline bool AimTargetForAngles(
    const LangYa::AimData& data,
    const bool callback_seen,
    const bool reuse_latched,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds hold,
    bool* fresh_target = nullptr,
    bool* latched_target = nullptr) noexcept {
    const bool fresh = callback_seen && AimFreshAndValid(data);
    const bool latched = !fresh && reuse_latched && AimLatchedRecently(data, now, hold);
    if (fresh_target != nullptr) {
        *fresh_target = fresh;
    }
    if (latched_target != nullptr) {
        *latched_target = latched;
    }
    return fresh || latched;
}

}  // namespace BehaviorTree
