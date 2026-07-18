// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "AreaManager.hpp"
#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

struct DefaultRegionalPolicyInput {
    const LangYa::Config* Config{nullptr};
    LangYa::UnitTeam MyTeam{LangYa::UnitTeam::Unknown};
    LangYa::UnitTeam EnemyTeam{LangYa::UnitTeam::Unknown};
    bool HealthFresh{false};
    bool AmmoFresh{false};
    std::uint16_t Health{0};
    std::uint16_t Ammo{0};
    bool HasSelfPosition{false};
    int SelfX{0};
    int SelfY{0};
    AreaRuntime SelfArea{};
    AreaTimePoint Now{};
};

struct DefaultRegionalAreaCandidate {
    const char* Name{""};
    RegionalAreaTaskType TaskType{RegionalAreaTaskType::None};
    AreaSide Side{AreaSide::Unknown};
    Area::MainAreaKind Kind{Area::MainAreaKind::Base};
    std::uint8_t BaseGoalId{LangYa::Home.ID};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    double Score{0.0};
};

class DefaultStrategyManager {
public:
    std::vector<DefaultRegionalAreaCandidate> BuildRegionalAreaCandidates(
        const DefaultRegionalPolicyInput& input);

    void CommitRegionalAreaSelection(
        const DefaultRegionalAreaCandidate& candidate,
        AreaTimePoint now) noexcept;
    void RecordRegionalAreaResult(
        RegionalAreaTaskType type,
        std::string_view reason,
        AreaTimePoint now,
        const LangYa::DefaultPolicySetting& setting) noexcept;
    void ResetRegionalPolicy() noexcept;

private:
    struct TaskRuntime {
        AreaTimePoint CooldownUntil{};
        int ConsecutiveFailures{0};
    };

    static bool AreaScopeAllows(
        const LangYa::NaviGoalAutonomySetting& navi_goal,
        const std::vector<std::string>& scope,
        Area::MainAreaKind kind);
    static std::size_t TaskIndex(RegionalAreaTaskType type) noexcept;
    static bool IsResultFailure(std::string_view reason) noexcept;

    std::array<TaskRuntime, 6> task_runtime_{};
    RegionalAreaTaskType last_selected_task_{RegionalAreaTaskType::None};
    RegionalAreaTaskType last_completed_task_{RegionalAreaTaskType::None};
    RegionalAreaTaskType preempted_task_{RegionalAreaTaskType::None};
};

}  // namespace BehaviorTree
