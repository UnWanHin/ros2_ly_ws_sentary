// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "AreaManager.hpp"
#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

struct DefaultRegionalAreaCandidate {
    const char* Name{""};
    std::uint8_t BaseGoalId{LangYa::Home.ID};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
};

class DefaultStrategyManager {
public:
    std::vector<DefaultRegionalAreaCandidate> BuildRegionalAreaCandidates(
        const LangYa::Config& config,
        LangYa::UnitTeam my_team) const;

    std::vector<std::size_t> BuildAttemptOrder(std::size_t candidate_count) const;
    void CommitRegionalAreaSelection(std::size_t index) noexcept;
    void ResetRegionalAreaRotation() noexcept;

private:
    static bool AreaScopeAllows(
        const LangYa::NaviGoalAutonomySetting& navi_goal,
        const std::vector<std::string>& scope,
        Area::MainAreaKind kind);

    std::size_t regional_area_index_{0};
    bool regional_area_initialized_{false};
};

}  // namespace BehaviorTree
