#pragma once

#include <cstdint>
#include <optional>

#include "AreaManager.hpp"

namespace BehaviorTree {

enum class ChasePolicyReason : std::uint8_t {
    Allowed = 0,
    ProfileBypass,
    Disabled,
    NoYieldablePlan,
    NoPlannedArea,
    TargetPositionStale,
    TargetAreaUnknown,
    TargetAreaNearestFallback,
    TargetAreaMismatch,
    PlannedAreaDisabled
};

struct ChasePolicyContext {
    bool RegionalProfile{false};
    bool YieldablePlan{false};
    std::optional<RegionalAreaTaskRuntime> Plan{};
    // Tactical defense may authorize exactly one area without inventing a
    // synthetic Default plan. The same target freshness/area/YAML gates apply.
    std::optional<AreaKey> ExplicitAllowedArea{};
    bool TargetPositionFresh{false};
    std::optional<ResolvedAreaKey> TargetArea{};
};

struct ChasePolicyResult {
    bool Allowed{false};
    ChasePolicyReason Reason{ChasePolicyReason::Disabled};
};

std::optional<AreaKey> PlannedAreaKeyForChase(
    const RegionalAreaTaskRuntime& task) noexcept;

ChasePolicyResult EvaluateRegionalChasePolicy(
    const LangYa::ChasePolicySetting& setting,
    const ChasePolicyContext& context) noexcept;

const char* ChasePolicyReasonToString(ChasePolicyReason reason) noexcept;

}  // namespace BehaviorTree
