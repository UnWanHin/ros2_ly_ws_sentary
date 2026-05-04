#include "../include/DefaultStrategyManager.hpp"

#include <algorithm>

namespace BehaviorTree {

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

std::vector<DefaultRegionalAreaCandidate> DefaultStrategyManager::BuildRegionalAreaCandidates(
    const LangYa::Config& config,
    const LangYa::UnitTeam my_team) const {
    const auto& task = config.RegionalAreaTaskSettings;
    const auto& navi_goal = config.DecisionAutonomySettings.NaviGoal;

    std::vector<DefaultRegionalAreaCandidate> candidates;
    candidates.reserve(4);

    if (task.MyHighland.Enable &&
        AreaScopeAllows(navi_goal, navi_goal.MyArea, Area::MainAreaKind::Highland)) {
        candidates.push_back(DefaultRegionalAreaCandidate{
            .Name = "MyHighland",
            .BaseGoalId = LangYa::Highland.ID,
            .GoalTeam = my_team
        });
    }
    if (task.MyBase.Enable &&
        AreaScopeAllows(navi_goal, navi_goal.MyArea, Area::MainAreaKind::Base)) {
        candidates.push_back(DefaultRegionalAreaCandidate{
            .Name = "MyBase",
            .BaseGoalId = LangYa::CastleLeft2.ID,
            .GoalTeam = my_team
        });
    }
    if (task.MyRoadland.Enable &&
        AreaScopeAllows(navi_goal, navi_goal.MyArea, Area::MainAreaKind::Roadland)) {
        candidates.push_back(DefaultRegionalAreaCandidate{
            .Name = "MyRoadland",
            .BaseGoalId = LangYa::CentralToBase.ID,
            .GoalTeam = my_team
        });
    }
    if (task.CommonCentral.Enable &&
        AreaScopeAllows(navi_goal, navi_goal.CommonArea, Area::MainAreaKind::Central)) {
        candidates.push_back(DefaultRegionalAreaCandidate{
            .Name = "CommonCentral",
            .BaseGoalId = LangYa::OutpostArea.ID,
            .GoalTeam = my_team
        });
    }

    return candidates;
}

std::vector<std::size_t> DefaultStrategyManager::BuildAttemptOrder(
    const std::size_t candidate_count) const {
    std::vector<std::size_t> order;
    if (candidate_count == 0U) {
        return order;
    }

    order.reserve(candidate_count);
    const auto start_index = regional_area_initialized_
        ? (regional_area_index_ + 1U) % candidate_count
        : 0U;
    for (std::size_t attempt = 0; attempt < candidate_count; ++attempt) {
        order.push_back((start_index + attempt) % candidate_count);
    }
    return order;
}

void DefaultStrategyManager::CommitRegionalAreaSelection(const std::size_t index) noexcept {
    regional_area_index_ = index;
    regional_area_initialized_ = true;
}

void DefaultStrategyManager::ResetRegionalAreaRotation() noexcept {
    regional_area_index_ = 0;
    regional_area_initialized_ = false;
}

}  // namespace BehaviorTree
