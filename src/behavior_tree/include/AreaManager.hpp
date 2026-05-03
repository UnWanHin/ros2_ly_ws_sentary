// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "../module/Area.hpp"
#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

using AreaTimePoint = std::chrono::steady_clock::time_point;

enum class AreaSide : std::uint8_t {
    Unknown = 0,
    My = 1,
    Enemy = 2,
    Common = 3
};

enum class AreaState : std::uint8_t {
    Unknown = 0,
    Outside = 1,
    Entering = 2,
    Inside = 3,
    Leaving = 4,
    Blocked = 5
};

struct AreaKey {
    AreaSide Side{AreaSide::Unknown};
    Area::MainAreaKind Kind{Area::MainAreaKind::Base};
    LangYa::UnitTeam Team{LangYa::UnitTeam::Unknown};

    friend bool operator==(const AreaKey&, const AreaKey&) = default;
};

struct AreaRuntime {
    AreaState State{AreaState::Unknown};
    std::optional<AreaKey> Current{};
    std::optional<AreaKey> Previous{};
    bool FirstEnter{false};
    AreaTimePoint StateSince{};
    AreaTimePoint LastEnterTime{};
    AreaTimePoint LastLeaveTime{};
};

struct ResolvedMainArea {
    Area::MainAreaKind Kind{Area::MainAreaKind::Base};
    bool UsedNearestFallback{false};
};

struct NaviGoalAreaScopeResult {
    bool Allowed{false};
    bool ScopeEnabled{false};
    std::optional<ResolvedMainArea> ResolvedArea{};
    const char* ScopeName{"unknown"};
};

enum class NaviAreaTransitionKind : std::uint8_t {
    None = 0,
    EnterMyHighland = 1,
    ViaHighland = 2,
    LeaveMyHighland = 3,
    LeaveMyHighlandViaCastleLeft = 4
};

const char* NaviAreaTransitionKindToString(NaviAreaTransitionKind kind);

struct NaviAreaTransitionRuntime {
    NaviAreaTransitionKind Kind{NaviAreaTransitionKind::None};
    bool Active{false};
    bool HasPendingGoal{false};
    std::uint8_t ViaBaseGoal{LangYa::Highland.ID};
    std::uint8_t PendingBaseGoal{LangYa::Home.ID};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{true};
    AreaTimePoint StartTime{};

    void Clear() noexcept;
};

struct NaviAreaTransitionPlan {
    NaviAreaTransitionKind Kind{NaviAreaTransitionKind::None};
    std::uint8_t ViaBaseGoal{LangYa::Highland.ID};
    bool HasPendingGoal{false};
    std::uint8_t PendingBaseGoal{LangYa::Home.ID};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{true};
    bool CheckViaAlreadyArrived{false};
};

enum class NaviAreaTransitionTickAction : std::uint8_t {
    Idle = 0,
    ContinueVia = 1,
    FinishNoPending = 2,
    FinishWithPending = 3
};

struct NaviAreaTransitionTickResult {
    NaviAreaTransitionTickAction Action{NaviAreaTransitionTickAction::Idle};
    NaviAreaTransitionKind Kind{NaviAreaTransitionKind::None};
    std::uint8_t ViaBaseGoal{LangYa::Highland.ID};
    bool HasPendingGoal{false};
    std::uint8_t PendingBaseGoal{LangYa::Home.ID};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{true};
    bool FollowMode{false};
    std::string CompletionReason{};
};

struct RegionalIdlePatrolCandidate {
    std::uint8_t BaseGoalId{LangYa::Home.ID};
    std::size_t Index{0};
};

struct RegionalDefenseThreat {
    int OwnBaseCount{0};
    int OwnHighlandCount{0};
    int OwnRoadlandCount{0};
    int EnemyHighlandCount{0};
    int EnemyRoadlandCount{0};
    bool HardThreat{false};
    bool SoftEnemySideThreat{false};
};

struct RegionalDefenseEnemyPosition {
    int X{0};
    int Y{0};
};

struct NaviProgressWatchdogRuntime {
    bool Active{false};
    std::uint8_t GoalId{0};
    std::uint8_t BaseGoal{LangYa::Home.ID};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{true};
    Area::Point<std::uint16_t> GoalPosition{};
    int LastX{0};
    int LastY{0};
    AreaTimePoint GoalStartTime{};
    AreaTimePoint LastMoveTime{};
    AreaTimePoint FallbackCooldownUntil{};
    std::uint8_t CooldownBaseGoal{LangYa::Home.ID};
    LangYa::UnitTeam CooldownTeam{LangYa::UnitTeam::Unknown};

    void Clear() noexcept;
};

struct NaviProgressWatchdogInput {
    bool Enabled{false};
    bool BlockedByAreaTransition{false};
    bool HasSelfPosition{false};
    int SelfX{0};
    int SelfY{0};
    std::optional<bool> ExternalReach{};
    std::optional<bool> ExternalReachable{};
    LangYa::NaviProgressWatchdogSetting Setting{};
    AreaTimePoint Now{};
};

struct NaviProgressWatchdogDecision {
    bool NeedFallback{false};
    bool ExternalUnreachable{false};
    std::uint8_t OriginalBaseGoal{LangYa::Home.ID};
    std::uint8_t OriginalGoalId{0};
    LangYa::UnitTeam OriginalGoalTeam{LangYa::UnitTeam::Unknown};
    bool OriginalApplyTeamOffset{true};
    std::vector<std::uint8_t> FallbackCandidates{};
};

class AreaManager {
public:
    void Configure(const LangYa::NaviGoalAutonomySetting& navi_goal_setting);
    void Reset(AreaTimePoint now);

    void TickSelfArea(
        AreaTimePoint now,
        bool has_position,
        int x,
        int y,
        LangYa::UnitTeam my_team,
        LangYa::UnitTeam enemy_team);
    const AreaRuntime& SelfAreaRuntime() const noexcept { return self_area_; }

    bool IsGoalAreaScopeEnabled() const noexcept;
    NaviGoalAreaScopeResult CheckGoalAreaScope(
        std::uint8_t base_goal_id,
        LangYa::UnitTeam goal_team,
        LangYa::UnitTeam my_team,
        LangYa::UnitTeam enemy_team) const;

    bool IsHighlandCompatEnabled() const noexcept;
    bool IsHighlandCompatTarget(std::uint8_t base_goal_id, LangYa::UnitTeam goal_team) const;
    bool HighlandTransitionActive() const noexcept { return transition_.Active; }
    const NaviAreaTransitionRuntime& TransitionRuntime() const noexcept { return transition_; }
    std::optional<NaviAreaTransitionPlan> PlanHighlandTransition(
        std::uint8_t base_goal_id,
        LangYa::UnitTeam goal_team,
        LangYa::UnitTeam my_team,
        bool apply_team_offset,
        std::uint8_t current_goal_id,
        bool goal_highland_arrived,
        bool self_in_my_highland) const;
    void StartHighlandTransition(const NaviAreaTransitionPlan& plan, AreaTimePoint now);
    NaviAreaTransitionTickResult TickHighlandTransition(
        AreaTimePoint now,
        bool route_unreachable,
        bool arrived);
    void ClearHighlandTransition() noexcept { transition_.Clear(); }

    std::vector<RegionalIdlePatrolCandidate> BuildRegionalIdlePatrolCandidates(
        const std::vector<std::uint8_t>& goals,
        std::uint8_t current_goal_id,
        LangYa::UnitTeam my_team);
    void CommitRegionalIdlePatrolCandidate(std::size_t index) noexcept;
    void ResetRegionalIdlePatrol() noexcept;

    RegionalDefenseThreat AnalyzeRegionalDefenseThreat(
        LangYa::UnitTeam my_team,
        LangYa::UnitTeam enemy_team,
        bool enable_soft_enemy_side_threat,
        const std::vector<RegionalDefenseEnemyPosition>& enemies) const;
    void StartRegionalDefenseSuppress(AreaTimePoint now, int hold_sec);
    bool IsRegionalDefenseAimSuppressActive(AreaTimePoint now) const noexcept;

    void UpdateProgressWatchdogGoal(
        std::uint8_t goal_id,
        std::uint8_t base_goal_id,
        LangYa::UnitTeam goal_team,
        bool apply_team_offset,
        Area::Point<std::uint16_t> goal_position,
        int self_x,
        int self_y,
        AreaTimePoint now);
    const NaviProgressWatchdogRuntime& ProgressWatchdogRuntime() const noexcept {
        return progress_watchdog_;
    }
    NaviProgressWatchdogDecision TickProgressWatchdog(const NaviProgressWatchdogInput& input);
    void CommitProgressWatchdogFallback(
        AreaTimePoint now,
        int fallback_cooldown_sec,
        std::uint8_t cooldown_base_goal,
        LangYa::UnitTeam cooldown_team);
    void MarkProgressWatchdogFallbackFailed(AreaTimePoint now, bool has_self_position, int self_x, int self_y);

    static constexpr std::uint8_t MaxBaseGoalId() noexcept { return LangYa::Highland.ID; }
    static bool IsValidBaseGoalId(std::uint8_t base_goal_id) noexcept;
    static bool IsReservedNonCombatGoalId(std::uint8_t base_goal_id) noexcept;
    static std::uint8_t ResolveGoalId(
        std::uint8_t base_goal_id,
        LangYa::UnitTeam team,
        bool apply_team_offset = true) noexcept;
    static Area::Point<std::uint16_t> GoalPointByBaseId(
        std::uint8_t base_goal_id,
        LangYa::UnitTeam goal_team);
    static double DistanceSq(int ax, int ay, int bx, int by) noexcept;
    static std::optional<Area::MainAreaKind> MainAreaKindFromToken(std::string_view token);
    static std::optional<Area::MainAreaKind> ResolvePointMainAreaExact(
        LangYa::UnitTeam area_team,
        int x,
        int y);
    static std::optional<ResolvedMainArea> ResolveGoalMainArea(
        std::uint8_t base_goal_id,
        LangYa::UnitTeam goal_team);
    static bool IsPositionInMainArea(
        LangYa::UnitTeam area_team,
        Area::MainAreaKind kind,
        int x,
        int y);
    static std::optional<AreaKey> ResolveAreaKeyForPoint(
        LangYa::UnitTeam my_team,
        LangYa::UnitTeam enemy_team,
        int x,
        int y);

private:
    LangYa::NaviGoalAutonomySetting navi_goal_config_{};
    AreaRuntime self_area_{};
    NaviAreaTransitionRuntime transition_{};

    struct RegionalIdlePatrolRuntime {
        std::size_t GoalIndex{0};
        bool Initialized{false};
    };
    RegionalIdlePatrolRuntime idle_patrol_{};

    NaviProgressWatchdogRuntime progress_watchdog_{};
    AreaTimePoint regional_defense_suppress_until_{};
};

}  // namespace BehaviorTree
