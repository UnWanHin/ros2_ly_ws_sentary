// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/FaceModeManager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace BehaviorTree {

void FaceModeManager::BeginCycle() noexcept {
    control_ = {};
}

bool FaceModeManager::RegisterRequest(
    const Source source,
    const bool active,
    const bool use_face_mode,
    const RegionalAreaTaskPhase phase,
    const std::uint8_t priority) noexcept {
    if (!active || (control_.Active && priority < control_.Priority)) {
        return false;
    }
    control_.Active = active;
    control_.UseFaceMode = active && use_face_mode;
    control_.Phase = phase;
    control_.RequestSource = source;
    control_.Priority = priority;
    return true;
}

bool FaceModeManager::Requested(const LangYa::FaceModeSetting& setting) const noexcept {
    return control_.Active && control_.UseFaceMode && setting.Enable;
}

FaceModeManager::Decision FaceModeManager::Resolve(
    const LangYa::AimData& data,
    const LangYa::FaceModeSetting& setting,
    const LangYa::PatrolScanSetting& patrol_scan,
    const bool visual_target_has_priority,
    const bool navigation_release_request,
    const bool clear_regional_when_navigation_releases,
    const AreaTimePoint now) const noexcept {
    Decision decision;
    decision.Requested = Requested(setting);
    decision.RequestSource = control_.RequestSource;
    decision.Phase = control_.Phase;
    const bool suppress_regional_request =
        navigation_release_request &&
        clear_regional_when_navigation_releases &&
        control_.RequestSource == Source::Regional;
    if (!decision.Requested || visual_target_has_priority || suppress_regional_request) {
        return decision;
    }

    decision.Angles = SelectAngles(data, setting, now);
    decision.UsePatrolFallback = !decision.Angles.has_value() && patrol_scan.FaceModeFallbackEnable;
    decision.Active = !decision.UsePatrolFallback;
    decision.SuppressFire = decision.Active && setting.SuppressFire;
    return decision;
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

bool FaceModeManager::RequestAimTarget(
    const LangYa::AimMode aim_mode,
    const LangYa::UnitTeam target_team,
    const TargetPublisher::SharedPtr& publisher) {
    if (aim_mode != LangYa::AimMode::Buff && aim_mode != LangYa::AimMode::Outpost) {
        return false;
    }

    const auto source = aim_mode == LangYa::AimMode::Buff ? Source::Buff : Source::Outpost;
    if (!RegisterRequest(source, true, true, RegionalAreaTaskPhase::Idle, kAimTaskPriority)) {
        return false;
    }
    if (!publisher) {
        return false;
    }

    const auto target = aim_mode == LangYa::AimMode::Buff
        ? Area::BuffPose(target_team)
        : Area::OutpostPose(target_team);
    publisher->publish(BuildTargetMessage(target));
    return true;
}

bool FaceModeManager::RequestStartGateOutpost(
    const LangYa::UnitTeam enemy_team,
    const TargetPublisher::SharedPtr& publisher) {
    if (!RegisterRequest(
            Source::StartGate,
            true,
            true,
            RegionalAreaTaskPhase::Idle,
            kStartGatePriority)) {
        return false;
    }
    if (!publisher) {
        return false;
    }

    publisher->publish(BuildTargetMessage(Area::OutpostPose(enemy_team)));
    return true;
}

bool FaceModeManager::StartGateOutpostSolutionReady(
    const bool status_fresh,
    const bool solver_function,
    const bool manual_target,
    const std::uint32_t target_update_count,
    const std::uint32_t target_update_count_floor,
    const bool face_mode_angles_fresh) noexcept {
    return DiagnoseStartGateOutpostSolution(
               true,
               status_fresh,
               solver_function,
               manual_target,
               target_update_count,
               target_update_count_floor,
               face_mode_angles_fresh) == StartGateOutpostReadiness::Ready;
}

FaceModeManager::StartGateOutpostReadiness
FaceModeManager::DiagnoseStartGateOutpostSolution(
    const bool status_received,
    const bool status_fresh,
    const bool solver_function,
    const bool manual_target,
    const std::uint32_t target_update_count,
    const std::uint32_t target_update_count_floor,
    const bool face_mode_angles_fresh) noexcept {
    if (!status_received) {
        return StartGateOutpostReadiness::StatusMissing;
    }
    if (!status_fresh) {
        return StartGateOutpostReadiness::StatusStale;
    }
    if (!solver_function) {
        return StartGateOutpostReadiness::SolverFunctionDisabled;
    }
    if (manual_target) {
        return StartGateOutpostReadiness::ManualTarget;
    }
    if (target_update_count <= target_update_count_floor) {
        return StartGateOutpostReadiness::TargetNotUpdated;
    }
    if (!face_mode_angles_fresh) {
        return StartGateOutpostReadiness::AnglesNotFresh;
    }
    return StartGateOutpostReadiness::Ready;
}

const char* FaceModeManager::StartGateOutpostReadinessName(
    const StartGateOutpostReadiness readiness) noexcept {
    switch (readiness) {
    case StartGateOutpostReadiness::Ready:
        return "accepted";
    case StartGateOutpostReadiness::StatusMissing:
        return "solver_status_missing";
    case StartGateOutpostReadiness::StatusStale:
        return "solver_status_stale";
    case StartGateOutpostReadiness::SolverFunctionDisabled:
        return "solver_function_false";
    case StartGateOutpostReadiness::ManualTarget:
        return "solver_manual_target";
    case StartGateOutpostReadiness::TargetNotUpdated:
        return "target_generation_not_advanced";
    case StartGateOutpostReadiness::AnglesNotFresh:
        return "angles_missing_or_stale";
    }
    return "unknown";
}

bool FaceModeManager::RequestRegionalTask(
    const RegionalAreaTaskTickResult& result,
    const TargetPublisher::SharedPtr& publisher) {
    if (!RegisterRequest(
            Source::Regional,
            result.Active,
            result.UseFaceMode,
            result.Phase,
            kRegionalPriority) ||
        !result.PublishFaceTarget || !publisher) {
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
