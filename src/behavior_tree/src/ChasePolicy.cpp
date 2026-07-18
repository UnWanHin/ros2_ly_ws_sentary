#include "../include/ChasePolicy.hpp"

namespace BehaviorTree {
namespace {

bool IsTeamKnown(const LangYa::UnitTeam team) noexcept {
    return team == LangYa::UnitTeam::Red || team == LangYa::UnitTeam::Blue;
}

bool IsPlannedAreaEnabled(
    const LangYa::ChasePolicySetting& setting,
    const AreaKey& area) noexcept {
    if (area.Side == AreaSide::Common && area.Kind == Area::MainAreaKind::Central) {
        return setting.CommonCentral;
    }
    if (area.Side != AreaSide::My) {
        return false;
    }
    switch (area.Kind) {
        case Area::MainAreaKind::Base: return setting.MyBase;
        case Area::MainAreaKind::Highland: return setting.MyHighland;
        case Area::MainAreaKind::PreRoadland: return setting.MyPreRoadland;
        case Area::MainAreaKind::ReadyRoadland: return setting.MyReadyRoadland;
        case Area::MainAreaKind::Central: return false;
        default: return false;
    }
}

}  // namespace

std::optional<AreaKey> PlannedAreaKeyForChase(
    const RegionalAreaTaskRuntime& task) noexcept {
    if (!task.Active) {
        return std::nullopt;
    }

    if (task.Type == RegionalAreaTaskType::CommonCentral) {
        return AreaKey{
            .Side = AreaSide::Common,
            .Kind = Area::MainAreaKind::Central,
            .Team = LangYa::UnitTeam::Unknown};
    }

    if (!IsTeamKnown(task.OwnerTeam)) {
        return std::nullopt;
    }

    Area::MainAreaKind kind = Area::MainAreaKind::Base;
    switch (task.Type) {
        case RegionalAreaTaskType::MyBase:
            kind = Area::MainAreaKind::Base;
            break;
        case RegionalAreaTaskType::MyHighland:
            kind = Area::MainAreaKind::Highland;
            break;
        case RegionalAreaTaskType::MyPreRoadland:
            kind = Area::MainAreaKind::PreRoadland;
            break;
        case RegionalAreaTaskType::MyReadyRoadland:
            kind = Area::MainAreaKind::ReadyRoadland;
            break;
        default:
            return std::nullopt;
    }

    return AreaKey{.Side = AreaSide::My, .Kind = kind, .Team = task.OwnerTeam};
}

ChasePolicyResult EvaluateRegionalChasePolicy(
    const LangYa::ChasePolicySetting& setting,
    const ChasePolicyContext& context) noexcept {
    if (!context.RegionalProfile) {
        return ChasePolicyResult{.Allowed = true, .Reason = ChasePolicyReason::ProfileBypass};
    }
    if (!setting.Enable) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::Disabled};
    }
    if (!context.YieldablePlan || !context.Plan.has_value()) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::NoYieldablePlan};
    }

    const auto planned_area = PlannedAreaKeyForChase(*context.Plan);
    if (!planned_area.has_value()) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::NoPlannedArea};
    }
    if (!context.TargetPositionFresh) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::TargetPositionStale};
    }
    if (!context.TargetArea.has_value()) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::TargetAreaUnknown};
    }
    if (context.TargetArea->UsedNearestFallback) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::TargetAreaNearestFallback};
    }
    if (context.TargetArea->Key != *planned_area) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::TargetAreaMismatch};
    }
    if (!IsPlannedAreaEnabled(setting, *planned_area)) {
        return ChasePolicyResult{.Allowed = false, .Reason = ChasePolicyReason::PlannedAreaDisabled};
    }
    return ChasePolicyResult{.Allowed = true, .Reason = ChasePolicyReason::Allowed};
}

const char* ChasePolicyReasonToString(const ChasePolicyReason reason) noexcept {
    switch (reason) {
        case ChasePolicyReason::Allowed: return "allowed";
        case ChasePolicyReason::ProfileBypass: return "profile_bypass";
        case ChasePolicyReason::Disabled: return "disabled";
        case ChasePolicyReason::NoYieldablePlan: return "no_yieldable_plan";
        case ChasePolicyReason::NoPlannedArea: return "no_planned_area";
        case ChasePolicyReason::TargetPositionStale: return "target_position_stale";
        case ChasePolicyReason::TargetAreaUnknown: return "target_area_unknown";
        case ChasePolicyReason::TargetAreaNearestFallback: return "target_area_nearest_fallback";
        case ChasePolicyReason::TargetAreaMismatch: return "target_area_mismatch";
        case ChasePolicyReason::PlannedAreaDisabled: return "planned_area_disabled";
        default: return "unknown";
    }
}

}  // namespace BehaviorTree
