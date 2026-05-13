// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/FaceModeManager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace BehaviorTree {

void FaceModeManager::ResetControl() noexcept {
    control_ = {};
}

void FaceModeManager::SetControl(
    const bool active,
    const bool use_face_mode,
    const RegionalAreaTaskPhase phase) noexcept {
    control_.Active = active;
    control_.UseFaceMode = active && use_face_mode;
    control_.Phase = phase;
}

bool FaceModeManager::Requested(const LangYa::FaceModeSetting& setting) const noexcept {
    return control_.Active && control_.UseFaceMode && setting.Enable;
}

bool FaceModeManager::Active(
    const LangYa::FaceModeSetting& setting,
    const bool visual_target_has_priority) const noexcept {
    return Requested(setting) && !visual_target_has_priority;
}

void FaceModeManager::CacheAngles(
    LangYa::AimData& data,
    const float yaw_deg,
    const float pitch_deg,
    const AreaTimePoint now) const noexcept {
    const bool angles_valid = std::isfinite(yaw_deg) && std::isfinite(pitch_deg);
    data.Angles = LangYa::GimbalAnglesType{
        static_cast<LangYa::AngleType>(yaw_deg),
        static_cast<LangYa::AngleType>(pitch_deg)
    };
    data.FireStatus = false;
    data.BuffFollow = false;
    data.Valid = angles_valid;
    data.Fresh = angles_valid;
    if (angles_valid) {
        data.HasLatchedAngles = true;
        data.LastValidTime = now;
    }
}

std::optional<LangYa::GimbalAnglesType> FaceModeManager::SelectAngles(
    const LangYa::AimData& data,
    const LangYa::FaceModeSetting& setting,
    const AreaTimePoint now) const noexcept {
    if (!Requested(setting) ||
        !data.Valid ||
        !data.HasLatchedAngles ||
        data.LastValidTime.time_since_epoch().count() == 0) {
        return std::nullopt;
    }

    const int hold_ms = std::max(0, setting.LostTargetHoldMs);
    if (data.Fresh ||
        (hold_ms > 0 && now - data.LastValidTime <= std::chrono::milliseconds(hold_ms))) {
        return data.Angles;
    }
    return std::nullopt;
}

bool FaceModeManager::PublishAimTarget(
    const LangYa::AimMode aim_mode,
    const LangYa::UnitTeam target_team,
    const TargetPublisher::SharedPtr& publisher,
    const std::optional<Area::Point3<double>>& outpost_manual_target) {
    if (aim_mode != LangYa::AimMode::Buff && aim_mode != LangYa::AimMode::Outpost) {
        return false;
    }

    SetControl(true, true, RegionalAreaTaskPhase::Idle);
    if (!publisher) {
        return false;
    }

    const auto target = aim_mode == LangYa::AimMode::Buff
        ? Area::BuffPose(target_team)
        : outpost_manual_target.value_or(Area::OutpostPose(target_team));
    publisher->publish(BuildTargetMessage(target));
    return true;
}

bool FaceModeManager::ApplyRegionalTaskResult(
    const RegionalAreaTaskTickResult& result,
    const TargetPublisher::SharedPtr& publisher) {
    SetControl(result.Active, result.UseFaceMode, result.Phase);
    if (!result.Active || !result.PublishFaceTarget || !publisher) {
        return false;
    }

    const auto face_target =
        AreaManager::GoalPointByBaseId(result.FaceTargetBaseGoalId, result.GoalTeam);
    publisher->publish(BuildTargetMessage(face_target, result.FaceTargetZCm));
    return true;
}

std_msgs::msg::UInt16MultiArray FaceModeManager::BuildTargetMessage(
    const Area::Point3<double>& point) {
    std_msgs::msg::UInt16MultiArray msg;
    msg.data = {
        ClampCmToU16(point.x),
        ClampCmToU16(point.y),
        ClampCmToU16(point.z)
    };
    return msg;
}

std_msgs::msg::UInt16MultiArray FaceModeManager::BuildTargetMessage(
    const Area::Point<std::uint16_t>& point,
    const int z_cm) {
    std_msgs::msg::UInt16MultiArray msg;
    msg.data = {
        point.x,
        point.y,
        ClampCmToU16(z_cm)
    };
    return msg;
}

std::uint16_t FaceModeManager::ClampCmToU16(const double value) noexcept {
    return static_cast<std::uint16_t>(
        std::clamp(static_cast<int>(std::lround(value)), 0, 65535));
}

std::uint16_t FaceModeManager::ClampCmToU16(const int value) noexcept {
    return static_cast<std::uint16_t>(std::clamp(value, 0, 65535));
}

}  // namespace BehaviorTree
