#include "../include/DefaultStrategyManager.hpp"

#include <algorithm>
#include <cmath>

namespace BehaviorTree {
namespace {

constexpr bool IsKnownTeam(const LangYa::UnitTeam team) noexcept {
    return team == LangYa::UnitTeam::Red || team == LangYa::UnitTeam::Blue;
}

std::uint16_t ClampThreshold(const int value) noexcept {
    return static_cast<std::uint16_t>(std::max(0, value));
}

bool MeetsFreshResource(
    const bool fresh,
    const std::uint16_t value,
    const int threshold) noexcept {
    return fresh && value >= ClampThreshold(threshold);
}

double DistancePenalty(
    const DefaultRegionalPolicyInput& input,
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team,
    const double penalty_per_meter) {
    if (!input.HasSelfPosition ||
        input.SelfX <= 0 ||
        input.SelfY <= 0 ||
        !IsKnownTeam(goal_team) ||
        penalty_per_meter <= 0.0) {
        return 0.0;
    }

    const auto goal = AreaManager::GoalPointByBaseId(base_goal_id, goal_team);
    const double dist_cm = std::sqrt(AreaManager::DistanceSq(
        input.SelfX,
        input.SelfY,
        static_cast<int>(goal.x),
        static_cast<int>(goal.y)));
    return (dist_cm / 100.0) * penalty_per_meter;
}

bool SameArea(
    const AreaRuntime& runtime,
    const AreaSide side,
    const Area::MainAreaKind kind) noexcept {
    return runtime.Current.has_value() &&
        runtime.Current->Side == side &&
        runtime.Current->Kind == kind;
}

double BaseWeight(
    const LangYa::DefaultPolicyScoreSetting& score,
    const RegionalAreaTaskType type) noexcept {
    switch (type) {
        case RegionalAreaTaskType::MyBase:
            return score.WeightMyBase;
        case RegionalAreaTaskType::MyHighland:
            return score.WeightMyHighland;
        case RegionalAreaTaskType::MyPreRoadland:
            return score.WeightMyPreRoadland;
        case RegionalAreaTaskType::MyRoadland:
            return score.WeightMyRoadland;
        case RegionalAreaTaskType::CommonCentral:
            return score.WeightCommonCentral;
        default:
            return 0.0;
    }
}

}  // namespace

bool DefaultStrategyManager::AreaScopeAllows(
    const LangYa::NaviGoalAutonomySetting& navi_goal,
    const std::vector<std::string>& scope,
    const Area::MainAreaKind kind) {
    if (!navi_goal.UseAreaScope) {
        return true;
    }
    return std::any_of(
        scope.begin(),
        scope.end(),
        [kind](const std::string& token) {
            const auto parsed = AreaManager::MainAreaKindFromToken(token);
            return parsed.has_value() && *parsed == kind;
        });
}

std::size_t DefaultStrategyManager::TaskIndex(
    const RegionalAreaTaskType type) noexcept {
    const auto raw = static_cast<std::size_t>(type);
    return raw < 6U ? raw : 0U;
}

bool DefaultStrategyManager::IsResultFailure(
    const std::string_view reason) noexcept {
    return reason == "unreachable" ||
           reason == "timeout" ||
           reason == "unhealthy" ||
           reason == "canceled";
}

std::vector<DefaultRegionalAreaCandidate> DefaultStrategyManager::BuildRegionalAreaCandidates(
    const DefaultRegionalPolicyInput& input) const {
    if (input.Config == nullptr || !IsKnownTeam(input.MyTeam)) {
        return {};
    }

    const auto& config = *input.Config;
    const auto& task = config.RegionalAreaTaskSettings;
    const auto& policy = task.DefaultPolicy;
    const auto& navi_goal = config.DecisionAutonomySettings.NaviGoal;
    if (!task.Enable || !policy.Enable) {
        return {};
    }

    std::vector<DefaultRegionalAreaCandidate> candidates;
    candidates.reserve(5);

    const bool low_resource =
        (input.HealthFresh &&
         input.Health < ClampThreshold(policy.Health.LowResourceFallbackHp)) ||
        (input.AmmoFresh &&
         input.Ammo < ClampThreshold(policy.Ammo.LowResourceFallbackAmmo)) ||
        !input.HealthFresh ||
        !input.AmmoFresh;

    auto add_candidate = [&](DefaultRegionalAreaCandidate candidate,
                             const bool enabled,
                             const std::vector<std::string>& scope,
                             const bool require_resource,
                             const int hp_min,
                             const int ammo_min) {
        if (!enabled || !AreaScopeAllows(navi_goal, scope, candidate.Kind)) {
            return;
        }
        const auto& runtime = task_runtime_[TaskIndex(candidate.TaskType)];
        if (runtime.CooldownUntil.time_since_epoch().count() != 0 &&
            input.Now < runtime.CooldownUntil) {
            return;
        }
        if (require_resource &&
            (!MeetsFreshResource(input.HealthFresh, input.Health, hp_min) ||
             !MeetsFreshResource(input.AmmoFresh, input.Ammo, ammo_min))) {
            return;
        }

        candidate.Score = BaseWeight(policy.Score, candidate.TaskType);
        candidate.Score -= DistancePenalty(
            input,
            candidate.BaseGoalId,
            candidate.GoalTeam,
            policy.Score.DistancePenaltyPerMeter);
        if (SameArea(input.SelfArea, candidate.Side, candidate.Kind)) {
            candidate.Score -= policy.Score.CurrentAreaPenalty;
        }
        if (candidate.TaskType == last_selected_task_) {
            candidate.Score -= policy.Score.LastAreaPenalty;
        }
        if (last_completed_task_ == RegionalAreaTaskType::MyHighland) {
            if (candidate.TaskType == RegionalAreaTaskType::MyBase) {
                candidate.Score += policy.Score.AfterHighlandMyBaseBonus;
            } else if (candidate.TaskType == RegionalAreaTaskType::MyRoadland) {
                candidate.Score += policy.Score.AfterHighlandMyRoadlandBonus;
            }
        }
        if (low_resource && candidate.TaskType == RegionalAreaTaskType::MyBase) {
            candidate.Score += policy.Score.LowResourceMyBaseBonus;
        }
        candidates.push_back(candidate);
    };

    add_candidate(
        DefaultRegionalAreaCandidate{
            .Name = "MyBase",
            .TaskType = RegionalAreaTaskType::MyBase,
            .Side = AreaSide::My,
            .Kind = Area::MainAreaKind::Base,
            .BaseGoalId = LangYa::CastleLeft2.ID,
            .GoalTeam = input.MyTeam
        },
        task.MyBase.Enable,
        navi_goal.MyArea,
        true,
        policy.Health.MyAreaHpMin,
        policy.Ammo.MyAreaAmmoMin);

    add_candidate(
        DefaultRegionalAreaCandidate{
            .Name = "MyHighland",
            .TaskType = RegionalAreaTaskType::MyHighland,
            .Side = AreaSide::My,
            .Kind = Area::MainAreaKind::Highland,
            .BaseGoalId = LangYa::Highland.ID,
            .GoalTeam = input.MyTeam
        },
        task.MyHighland.Enable,
        navi_goal.MyArea,
        true,
        policy.Health.MyAreaHpMin,
        policy.Ammo.MyAreaAmmoMin);

    add_candidate(
        DefaultRegionalAreaCandidate{
            .Name = "MyPreRoadland",
            .TaskType = RegionalAreaTaskType::MyPreRoadland,
            .Side = AreaSide::My,
            .Kind = Area::MainAreaKind::PreRoadland,
            .BaseGoalId = LangYa::PreRoadland.ID,
            .GoalTeam = input.MyTeam
        },
        task.MyPreRoadland.Enable,
        navi_goal.MyArea,
        true,
        policy.Health.MyAreaHpMin,
        policy.Ammo.MyAreaAmmoMin);

    add_candidate(
        DefaultRegionalAreaCandidate{
            .Name = "MyRoadland",
            .TaskType = RegionalAreaTaskType::MyRoadland,
            .Side = AreaSide::My,
            .Kind = Area::MainAreaKind::Roadland,
            .BaseGoalId = LangYa::CentralToBase.ID,
            .GoalTeam = input.MyTeam
        },
        task.MyRoadland.Enable,
        navi_goal.MyArea,
        true,
        std::max(policy.Health.MyAreaHpMin, task.MyRoadland.HealthyHpMin),
        std::max(policy.Ammo.MyAreaAmmoMin, task.MyRoadland.HealthyAmmoMin));

    add_candidate(
        DefaultRegionalAreaCandidate{
            .Name = "CommonCentral",
            .TaskType = RegionalAreaTaskType::CommonCentral,
            .Side = AreaSide::Common,
            .Kind = Area::MainAreaKind::Central,
            .BaseGoalId = LangYa::OutpostArea.ID,
            .GoalTeam = input.MyTeam
        },
        task.CommonCentral.Enable,
        navi_goal.CommonArea,
        true,
        std::max(policy.Health.CommonCentralHpMin, task.CommonCentral.HealthyHpMin),
        std::max(policy.Ammo.CommonCentralAmmoMin, task.CommonCentral.HealthyAmmoMin));

    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const DefaultRegionalAreaCandidate& lhs, const DefaultRegionalAreaCandidate& rhs) {
            return lhs.Score > rhs.Score;
        });

    return candidates;
}

void DefaultStrategyManager::CommitRegionalAreaSelection(
    const DefaultRegionalAreaCandidate& candidate,
    const AreaTimePoint) noexcept {
    last_selected_task_ = candidate.TaskType;
}

void DefaultStrategyManager::RecordRegionalAreaResult(
    const RegionalAreaTaskType type,
    const std::string_view reason,
    const AreaTimePoint now,
    const LangYa::DefaultPolicySetting& setting) noexcept {
    if (type == RegionalAreaTaskType::None) {
        return;
    }

    auto& runtime = task_runtime_[TaskIndex(type)];
    const bool failure = IsResultFailure(reason);
    if (failure) {
        runtime.ConsecutiveFailures += 1;
        last_completed_task_ = RegionalAreaTaskType::None;
        const int cooldown_sec = reason == "unreachable"
            ? setting.Retry.UnreachableCooldownSec
            : setting.Retry.FailureCooldownSec;
        runtime.CooldownUntil =
            now + std::chrono::seconds(std::max(0, cooldown_sec));
        if (setting.Retry.MaxRetry > 0 &&
            runtime.ConsecutiveFailures >= setting.Retry.MaxRetry) {
            runtime.CooldownUntil =
                now + std::chrono::seconds(std::max(0, setting.Retry.UnreachableCooldownSec));
        }
    } else {
        runtime.ConsecutiveFailures = 0;
        runtime.CooldownUntil =
            now + std::chrono::seconds(std::max(0, setting.Retry.CompleteCooldownSec));
        last_completed_task_ = type;
    }
}

void DefaultStrategyManager::ResetRegionalPolicy() noexcept {
    last_selected_task_ = RegionalAreaTaskType::None;
    last_completed_task_ = RegionalAreaTaskType::None;
}

}  // namespace BehaviorTree
