// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/AreaManager.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iterator>
#include <limits>

namespace BehaviorTree {
namespace {

constexpr std::array<Area::MainAreaKind, 5> kAllMainAreas{
    Area::MainAreaKind::Base,
    Area::MainAreaKind::Highland,
    Area::MainAreaKind::Roadland,
    Area::MainAreaKind::PreRoadland,
    Area::MainAreaKind::Central
};

constexpr std::array<Area::MainAreaKind, 4> kSideMainAreas{
    Area::MainAreaKind::Base,
    Area::MainAreaKind::Highland,
    Area::MainAreaKind::Roadland,
    Area::MainAreaKind::PreRoadland
};

struct CentralPatrolGoalSpec {
    std::uint8_t BaseGoalId{LangYa::OutpostArea.ID};
    bool EnemySide{false};
};

constexpr std::array<CentralPatrolGoalSpec, 8> kCommonCentralPatrolGoals{{
    {LangYa::OutpostArea.ID, false},
    {LangYa::RightShoot.ID, false},
    {LangYa::BuffAround2.ID, false},
    {LangYa::LeftShoot.ID, false},
    {LangYa::OutpostShoot.ID, false},
    {LangYa::RightShoot.ID, true},
    {LangYa::OccupyArea.ID, true},
    {LangYa::OutpostShoot.ID, true}
}};

std::string NormalizeAreaToken(std::string_view token) {
    std::string normalized(token);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) {
            if (c == '-' || c == ' ') {
                return '_';
            }
            return static_cast<char>(std::tolower(c));
        });
    return normalized;
}

Area::Point<double> MainAreaCentroid(
    const LangYa::UnitTeam team,
    const Area::MainAreaKind kind) {
    const auto& boundary = Area::MainAreaBoundary(team, kind);
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto& point : boundary) {
        sum_x += static_cast<double>(point.x);
        sum_y += static_cast<double>(point.y);
    }
    const double count = boundary.empty() ? 1.0 : static_cast<double>(boundary.size());
    return Area::Point<double>{sum_x / count, sum_y / count};
}

std::vector<std::uint8_t> ProgressFallbackCandidatesForArea(const Area::MainAreaKind kind) {
    switch (kind) {
        case Area::MainAreaKind::Base:
            return {
                LangYa::Castle.ID,
                LangYa::CastleLeft1.ID,
                LangYa::CastleLeft2.ID,
                LangYa::CastleRight1.ID,
                LangYa::CastleRight2.ID
            };
        case Area::MainAreaKind::Highland:
            return {LangYa::Highland.ID, LangYa::BuffShoot.ID, LangYa::HoleRoad.ID};
        case Area::MainAreaKind::Roadland:
            return {LangYa::Highland.ID, LangYa::HoleRoad.ID, LangYa::FlyRoad.ID};
        case Area::MainAreaKind::Central:
            return {
                LangYa::MidShoot.ID,
                LangYa::BuffAround1.ID,
                LangYa::BuffAround2.ID,
                LangYa::RightShoot.ID
            };
        default:
            return {};
    }
}

LangYa::UnitTeam OppositeTeam(const LangYa::UnitTeam team) noexcept {
    if (team == LangYa::UnitTeam::Red) {
        return LangYa::UnitTeam::Blue;
    }
    if (team == LangYa::UnitTeam::Blue) {
        return LangYa::UnitTeam::Red;
    }
    return LangYa::UnitTeam::Unknown;
}

LangYa::UnitTeam CommonCentralPatrolGoalTeam(
    const LangYa::UnitTeam owner_team,
    const CentralPatrolGoalSpec& spec) noexcept {
    return spec.EnemySide ? OppositeTeam(owner_team) : owner_team;
}

std::size_t NextCommonCentralPatrolIndex(const std::size_t current_index) noexcept {
    return (current_index + 1U) % kCommonCentralPatrolGoals.size();
}

std::size_t NearestCommonCentralPatrolIndex(
    const LangYa::UnitTeam owner_team,
    const bool has_self_position,
    const int self_x,
    const int self_y) {
    if (!has_self_position || self_x <= 0 || self_y <= 0) {
        return 0U;
    }

    std::size_t nearest_index = 0U;
    double nearest_dist_sq = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < kCommonCentralPatrolGoals.size(); ++index) {
        const auto& spec = kCommonCentralPatrolGoals[index];
        const auto goal_team = CommonCentralPatrolGoalTeam(owner_team, spec);
        if (goal_team != LangYa::UnitTeam::Red && goal_team != LangYa::UnitTeam::Blue) {
            continue;
        }
        const auto goal_point = AreaManager::GoalPointByBaseId(spec.BaseGoalId, goal_team);
        const double dist_sq = AreaManager::DistanceSq(
            self_x,
            self_y,
            static_cast<int>(goal_point.x),
            static_cast<int>(goal_point.y));
        if (dist_sq < nearest_dist_sq) {
            nearest_dist_sq = dist_sq;
            nearest_index = index;
        }
    }
    return nearest_index;
}

std::uint8_t SelectMyBasePatrolGoal(
    const LangYa::UnitTeam goal_team,
    const LangYa::MyBaseAreaTaskSetting& setting,
    const LangYa::PatrolGoalSelectionSetting& selection,
    const bool has_self_position,
    const int self_x,
    const int self_y,
    const std::uint8_t current_base_goal,
    const bool avoid_current_goal,
    const AreaTimePoint now,
    const std::array<AreaTimePoint, 256>& last_arrived) {
    const bool can_use_position = has_self_position && self_x > 0 && self_y > 0;
    const bool can_use_time = now.time_since_epoch().count() != 0;
    const auto valid_goal_count = std::count_if(
        setting.PatrolGoals.begin(),
        setting.PatrolGoals.end(),
        [](const LangYa::MyBasePatrolGoalSetting& goal) {
            return goal.Weight > 0.0 &&
                   AreaManager::IsValidBaseGoalId(goal.BaseGoalId) &&
                   !AreaManager::IsReservedNonCombatGoalId(goal.BaseGoalId);
        });

    std::uint8_t best_goal = LangYa::CastleLeft2.ID;
    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;
    for (const auto& candidate : setting.PatrolGoals) {
        if (candidate.Weight <= 0.0 ||
            !AreaManager::IsValidBaseGoalId(candidate.BaseGoalId) ||
            AreaManager::IsReservedNonCombatGoalId(candidate.BaseGoalId)) {
            continue;
        }
        if (avoid_current_goal &&
            selection.AvoidCurrentGoal &&
            valid_goal_count > 1 &&
            candidate.BaseGoalId == current_base_goal) {
            continue;
        }

        double score = candidate.Weight;
        if (can_use_position) {
            const auto goal_point = AreaManager::GoalPointByBaseId(candidate.BaseGoalId, goal_team);
            const double distance_cm = std::sqrt(AreaManager::DistanceSq(
                self_x,
                self_y,
                static_cast<int>(goal_point.x),
                static_cast<int>(goal_point.y)));
            score -= (distance_cm / 100.0) * std::max(0.0, selection.DistancePenaltyPerMeter);
        }
        if (candidate.BaseGoalId == current_base_goal) {
            score -= std::max(0.0, selection.CurrentGoalPenalty);
        }
        if (can_use_time) {
            const auto arrived = last_arrived[candidate.BaseGoalId];
            if (arrived.time_since_epoch().count() == 0) {
                score += std::max(0.0, selection.UnvisitedBonus);
            } else {
                const auto age = std::chrono::duration_cast<std::chrono::seconds>(now - arrived);
                const auto age_sec = std::max(0, static_cast<int>(age.count()));
                if (selection.FreshnessTimeoutSec > 0) {
                    const double ratio = std::min(
                        1.0,
                        static_cast<double>(age_sec) /
                            static_cast<double>(selection.FreshnessTimeoutSec));
                    score += std::max(0.0, selection.FreshnessBonusMax) * ratio;
                }
                if (selection.RecentVisitPenaltySec > 0 &&
                    age_sec < selection.RecentVisitPenaltySec) {
                    score -= std::max(0.0, selection.RecentVisitPenalty);
                }
            }
        }
        if (!found || score > best_score) {
            best_score = score;
            best_goal = candidate.BaseGoalId;
            found = true;
        }
    }
    return found ? best_goal : LangYa::CastleLeft2.ID;
}

}  // namespace

const char* NaviAreaTransitionKindToString(const NaviAreaTransitionKind kind) {
    switch (kind) {
        case NaviAreaTransitionKind::None: return "None";
        case NaviAreaTransitionKind::EnterMyHighland: return "EnterMyHighland";
        case NaviAreaTransitionKind::ViaHighland: return "ViaHighland";
        case NaviAreaTransitionKind::LeaveMyHighland: return "LeaveMyHighland";
        case NaviAreaTransitionKind::LeaveMyHighlandViaCastleLeft1: return "LeaveMyHighlandViaCastleLeft1";
        case NaviAreaTransitionKind::BuffOutpostViaHoleRoad: return "BuffOutpostViaHoleRoad";
        case NaviAreaTransitionKind::LeaveBuffOutpostViaHoleRoad: return "LeaveBuffOutpostViaHoleRoad";
        default: return "Unknown";
    }
}

const char* RegionalAreaTaskTypeToString(const RegionalAreaTaskType type) {
    switch (type) {
        case RegionalAreaTaskType::None: return "None";
        case RegionalAreaTaskType::MyHighland: return "MyHighland";
        case RegionalAreaTaskType::MyBase: return "MyBase";
        case RegionalAreaTaskType::MyPreRoadland: return "MyPreRoadland";
        case RegionalAreaTaskType::MyRoadland: return "MyRoadland";
        case RegionalAreaTaskType::CommonCentral: return "CommonCentral";
        default: return "Unknown";
    }
}

const char* RegionalAreaTaskPhaseToString(const RegionalAreaTaskPhase phase) {
    switch (phase) {
        case RegionalAreaTaskPhase::Idle: return "Idle";
        case RegionalAreaTaskPhase::ApproachHighland: return "ApproachHighland";
        case RegionalAreaTaskPhase::HighlandPatrol: return "HighlandPatrol";
        case RegionalAreaTaskPhase::ToBuffShoot: return "ToBuffShoot";
        case RegionalAreaTaskPhase::BuffShootHold: return "BuffShootHold";
        case RegionalAreaTaskPhase::LeaveViaHoleRoad: return "LeaveViaHoleRoad";
        case RegionalAreaTaskPhase::BasePatrol: return "BasePatrol";
        case RegionalAreaTaskPhase::RoadlandApproachCentralToBase: return "RoadlandApproachCentralToBase";
        case RegionalAreaTaskPhase::RoadlandCrossToBaseToCentral: return "RoadlandCrossToBaseToCentral";
        case RegionalAreaTaskPhase::RoadlandHoldBaseToCentral: return "RoadlandHoldBaseToCentral";
        case RegionalAreaTaskPhase::RoadlandCrossToCentralToBase: return "RoadlandCrossToCentralToBase";
        case RegionalAreaTaskPhase::RoadlandReturnToCentralToBase: return "RoadlandReturnToCentralToBase";
        case RegionalAreaTaskPhase::CentralPatrol: return "CentralPatrol";
        case RegionalAreaTaskPhase::PreRoadlandApproach: return "PreRoadlandApproach";
        case RegionalAreaTaskPhase::PreRoadlandHold: return "PreRoadlandHold";
        default: return "Unknown";
    }
}

void NaviAreaTransitionRuntime::Clear() noexcept {
    Kind = NaviAreaTransitionKind::None;
    Active = false;
    HasPendingGoal = false;
    ViaBaseGoal = LangYa::Highland.ID;
    PendingBaseGoal = LangYa::Home.ID;
    GoalTeam = LangYa::UnitTeam::Unknown;
    ApplyTeamOffset = true;
    StartTime = AreaTimePoint{};
}

void RegionalAreaTaskRuntime::Clear() noexcept {
    Active = false;
    Type = RegionalAreaTaskType::None;
    Phase = RegionalAreaTaskPhase::Idle;
    GoalTeam = LangYa::UnitTeam::Unknown;
    ApplyTeamOffset = true;
    TriggerBaseGoal = LangYa::Highland.ID;
    CurrentBaseGoal = LangYa::Highland.ID;
    StartTime = AreaTimePoint{};
    PhaseStartTime = AreaTimePoint{};
    BaseGoalArrivedTime = AreaTimePoint{};
    OwnerTeam = LangYa::UnitTeam::Unknown;
    PatrolIndex = 0U;
    PatrolStepCount = 0;
}

void NaviProgressWatchdogRuntime::Clear() noexcept {
    Active = false;
    GoalId = 0;
    BaseGoal = LangYa::Home.ID;
    GoalTeam = LangYa::UnitTeam::Unknown;
    ApplyTeamOffset = true;
    GoalPosition = {};
    LastX = 0;
    LastY = 0;
    GoalStartTime = AreaTimePoint{};
    LastMoveTime = AreaTimePoint{};
    FallbackCooldownUntil = AreaTimePoint{};
    CooldownBaseGoal = LangYa::Home.ID;
    CooldownTeam = LangYa::UnitTeam::Unknown;
}

void AreaManager::Configure(const LangYa::NaviGoalAutonomySetting& navi_goal_setting) {
    navi_goal_config_ = navi_goal_setting;
}

void AreaManager::Reset(const AreaTimePoint now) {
    self_area_ = {};
    self_area_.StateSince = now;
    transition_.Clear();
    idle_patrol_ = {};
    progress_watchdog_.Clear();
    regional_defense_suppress_until_ = AreaTimePoint{};
    regional_area_task_.Clear();
}

void AreaManager::TickSelfArea(
    const AreaTimePoint now,
    const bool has_position,
    const int x,
    const int y,
    const LangYa::UnitTeam my_team,
    const LangYa::UnitTeam enemy_team) {
    const auto next_area = has_position
        ? ResolveAreaKeyForPoint(my_team, enemy_team, x, y)
        : std::optional<AreaKey>{};

    if (!next_area.has_value()) {
        if (self_area_.Current.has_value()) {
            self_area_.Previous = self_area_.Current;
            self_area_.Current = std::nullopt;
            self_area_.State = AreaState::Leaving;
            self_area_.FirstEnter = false;
            self_area_.LastLeaveTime = now;
            self_area_.StateSince = now;
            return;
        }
        self_area_.State = AreaState::Outside;
        self_area_.FirstEnter = false;
        if (self_area_.StateSince.time_since_epoch().count() == 0) {
            self_area_.StateSince = now;
        }
        return;
    }

    if (!self_area_.Current.has_value() || self_area_.Current.value() != next_area.value()) {
        self_area_.Previous = self_area_.Current;
        self_area_.Current = next_area;
        self_area_.State = AreaState::Entering;
        self_area_.FirstEnter = true;
        self_area_.LastEnterTime = now;
        self_area_.StateSince = now;
        return;
    }

    self_area_.State = AreaState::Inside;
    self_area_.FirstEnter = false;
}

bool AreaManager::IsGoalAreaScopeEnabled() const noexcept {
    return navi_goal_config_.UseAreaScope;
}

NaviGoalAreaScopeResult AreaManager::CheckGoalAreaScope(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team,
    const LangYa::UnitTeam my_team,
    const LangYa::UnitTeam enemy_team) const {
    NaviGoalAreaScopeResult result{};
    result.ScopeEnabled = IsGoalAreaScopeEnabled();
    if (!result.ScopeEnabled) {
        result.Allowed = true;
        return result;
    }
    if (!IsValidBaseGoalId(base_goal_id)) {
        return result;
    }
    if (goal_team != LangYa::UnitTeam::Red && goal_team != LangYa::UnitTeam::Blue) {
        return result;
    }

    result.ResolvedArea = ResolveGoalMainArea(base_goal_id, goal_team);
    if (!result.ResolvedArea.has_value()) {
        return result;
    }

    const std::vector<std::string>* allowed_areas = nullptr;
    if (result.ResolvedArea->Kind == Area::MainAreaKind::Central) {
        allowed_areas = &navi_goal_config_.CommonArea;
        result.ScopeName = "common";
    } else if (goal_team == my_team) {
        allowed_areas = &navi_goal_config_.MyArea;
        result.ScopeName = "my";
    } else if (goal_team == enemy_team) {
        allowed_areas = &navi_goal_config_.EnemyArea;
        result.ScopeName = "enemy";
    } else {
        return result;
    }

    if (allowed_areas == nullptr || allowed_areas->empty()) {
        return result;
    }
    for (const auto& area_token : *allowed_areas) {
        const auto area_kind = MainAreaKindFromToken(area_token);
        if (area_kind.has_value() && *area_kind == result.ResolvedArea->Kind) {
            result.Allowed = true;
            return result;
        }
    }
    return result;
}

bool AreaManager::IsHighlandCompatEnabled() const noexcept {
    return navi_goal_config_.HighlandCompatEnable;
}

bool AreaManager::IsBuffOutpostCompatEnabled() const noexcept {
    return navi_goal_config_.BuffOutpostCompatEnable;
}

bool AreaManager::IsNaviAreaTransitionCompatEnabled() const noexcept {
    return IsHighlandCompatEnabled() || IsBuffOutpostCompatEnabled();
}

bool AreaManager::IsHighlandCompatTarget(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team) const {
    if (!IsHighlandCompatEnabled() ||
        base_goal_id == LangYa::Highland.ID ||
        !IsValidBaseGoalId(base_goal_id)) {
        return false;
    }
    const auto resolved_area = ResolveGoalMainArea(base_goal_id, goal_team);
    return resolved_area.has_value() &&
        !resolved_area->UsedNearestFallback &&
        resolved_area->Kind == Area::MainAreaKind::Highland;
}

std::optional<NaviAreaTransitionPlan> AreaManager::PlanHighlandTransition(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team,
    const LangYa::UnitTeam my_team,
    const bool apply_team_offset,
    const std::uint8_t current_goal_id,
    const bool goal_highland_arrived,
    const bool hole_road_arrived,
    const bool buff_outpost_arrived,
    const bool self_in_my_highland) const {
    if (transition_.Active ||
        !IsNaviAreaTransitionCompatEnabled() ||
        !IsValidBaseGoalId(base_goal_id) ||
        goal_team == LangYa::UnitTeam::Unknown ||
        my_team == LangYa::UnitTeam::Unknown) {
        return std::nullopt;
    }

    NaviAreaTransitionPlan plan{};
    plan.GoalTeam = goal_team;
    plan.ApplyTeamOffset = apply_team_offset;

    if (IsBuffOutpostCompatEnabled() && goal_team == my_team) {
        const bool target_is_buff_outpost = base_goal_id == LangYa::BuffOutpost.ID;
        const bool target_is_hole_road = base_goal_id == LangYa::HoleRoad.ID;
        const bool current_is_buff_outpost =
            buff_outpost_arrived ||
            current_goal_id == ResolveGoalId(LangYa::BuffOutpost.ID, my_team, apply_team_offset) ||
            current_goal_id == LangYa::BuffOutpost.ID;

        if (target_is_buff_outpost && !current_is_buff_outpost && !hole_road_arrived) {
            plan.Kind = NaviAreaTransitionKind::BuffOutpostViaHoleRoad;
            plan.ViaBaseGoal = LangYa::HoleRoad.ID;
            plan.HasPendingGoal = true;
            plan.PendingBaseGoal = LangYa::BuffOutpost.ID;
            plan.CheckViaAlreadyArrived = true;
            return plan;
        }

        if (!target_is_buff_outpost &&
            !target_is_hole_road &&
            current_is_buff_outpost &&
            !hole_road_arrived) {
            plan.Kind = NaviAreaTransitionKind::LeaveBuffOutpostViaHoleRoad;
            plan.ViaBaseGoal = LangYa::HoleRoad.ID;
            plan.HasPendingGoal = true;
            plan.PendingBaseGoal = base_goal_id;
            plan.CheckViaAlreadyArrived = true;
            return plan;
        }
    }

    const bool target_is_my_highland =
        goal_team == my_team &&
        base_goal_id == LangYa::Highland.ID &&
        !goal_highland_arrived;

    if (target_is_my_highland) {
        plan.Kind = NaviAreaTransitionKind::EnterMyHighland;
        plan.ViaBaseGoal = LangYa::Highland.ID;
        plan.HasPendingGoal = false;
        plan.PendingBaseGoal = LangYa::Home.ID;
        return plan;
    }

    if (IsHighlandCompatTarget(base_goal_id, goal_team) &&
        current_goal_id != ResolveGoalId(base_goal_id, goal_team, apply_team_offset) &&
        !goal_highland_arrived) {
        plan.Kind = NaviAreaTransitionKind::ViaHighland;
        plan.ViaBaseGoal = LangYa::Highland.ID;
        plan.HasPendingGoal = true;
        plan.PendingBaseGoal = base_goal_id;
        return plan;
    }

    if (goal_team == my_team &&
        base_goal_id != LangYa::Highland.ID &&
        self_in_my_highland) {
        const auto resolved_target_area = ResolveGoalMainArea(base_goal_id, goal_team);
        const bool target_is_base_area =
            resolved_target_area.has_value() &&
            !resolved_target_area->UsedNearestFallback &&
            resolved_target_area->Kind == Area::MainAreaKind::Base;
        plan.Kind = target_is_base_area
            ? NaviAreaTransitionKind::LeaveMyHighlandViaCastleLeft1
            : NaviAreaTransitionKind::LeaveMyHighland;
        plan.ViaBaseGoal = target_is_base_area ? LangYa::CastleLeft1.ID : base_goal_id;
        plan.HasPendingGoal = plan.ViaBaseGoal != base_goal_id;
        plan.PendingBaseGoal = plan.HasPendingGoal ? base_goal_id : LangYa::Home.ID;
        plan.CheckViaAlreadyArrived = true;
        return plan;
    }

    return std::nullopt;
}

void AreaManager::StartHighlandTransition(
    const NaviAreaTransitionPlan& plan,
    const AreaTimePoint now) {
    transition_.Active = true;
    transition_.Kind = plan.Kind;
    transition_.HasPendingGoal = plan.HasPendingGoal;
    transition_.ViaBaseGoal = plan.ViaBaseGoal;
    transition_.PendingBaseGoal = plan.PendingBaseGoal;
    transition_.GoalTeam = plan.GoalTeam;
    transition_.ApplyTeamOffset = plan.ApplyTeamOffset;
    transition_.StartTime = now;
}

NaviAreaTransitionTickResult AreaManager::TickHighlandTransition(
    const AreaTimePoint now,
    const bool route_unreachable,
    const bool arrived) {
    NaviAreaTransitionTickResult result{};
    if (!transition_.Active) {
        return result;
    }

    const bool buff_outpost_transition =
        transition_.Kind == NaviAreaTransitionKind::BuffOutpostViaHoleRoad ||
        transition_.Kind == NaviAreaTransitionKind::LeaveBuffOutpostViaHoleRoad;
    const bool transition_enabled = buff_outpost_transition
        ? IsBuffOutpostCompatEnabled()
        : IsHighlandCompatEnabled();
    if (!transition_enabled) {
        return result;
    }

    const int timeout_sec = buff_outpost_transition
        ? navi_goal_config_.BuffOutpostCompatTimeoutSec
        : navi_goal_config_.HighlandCompatTimeoutSec;
    const auto timeout = std::chrono::seconds(std::max(1, timeout_sec));
    const bool timed_out =
        transition_.StartTime.time_since_epoch().count() != 0 &&
        now - transition_.StartTime >= timeout;

    result.Kind = transition_.Kind;
    result.ViaBaseGoal = IsValidBaseGoalId(transition_.ViaBaseGoal)
        ? transition_.ViaBaseGoal
        : LangYa::Highland.ID;
    result.HasPendingGoal = transition_.HasPendingGoal;
    result.PendingBaseGoal = transition_.PendingBaseGoal;
    result.GoalTeam = transition_.GoalTeam;
    result.ApplyTeamOffset = transition_.ApplyTeamOffset;

    if (!arrived && !timed_out && !route_unreachable) {
        result.Action = NaviAreaTransitionTickAction::ContinueVia;
        result.FollowMode = true;
        return result;
    }

    result.CompletionReason = route_unreachable ? "unreachable" : (arrived ? "arrived" : "timeout");
    result.FollowMode = false;
    transition_.Clear();
    result.Action = result.HasPendingGoal
        ? NaviAreaTransitionTickAction::FinishWithPending
        : NaviAreaTransitionTickAction::FinishNoPending;
    return result;
}

std::vector<RegionalIdlePatrolCandidate> AreaManager::BuildRegionalIdlePatrolCandidates(
    const std::vector<std::uint8_t>& goals,
    const std::uint8_t current_goal_id,
    const LangYa::UnitTeam my_team) {
    std::vector<RegionalIdlePatrolCandidate> candidates;
    if (goals.empty()) {
        return candidates;
    }

    constexpr bool apply_team_offset = true;
    const auto current_it = std::find_if(
        goals.begin(),
        goals.end(),
        [&](const std::uint8_t goal_id) {
            return current_goal_id == ResolveGoalId(goal_id, my_team, apply_team_offset);
        });

    std::size_t next_index = 0U;
    if (idle_patrol_.Initialized && current_it != goals.end()) {
        idle_patrol_.GoalIndex =
            static_cast<std::size_t>(std::distance(goals.begin(), current_it));
        next_index = (idle_patrol_.GoalIndex + 1U) % goals.size();
    }

    candidates.reserve(goals.size());
    for (std::size_t attempt = 0; attempt < goals.size(); ++attempt) {
        const std::size_t candidate_index = (next_index + attempt) % goals.size();
        candidates.push_back(RegionalIdlePatrolCandidate{
            .BaseGoalId = goals[candidate_index],
            .Index = candidate_index
        });
    }
    return candidates;
}

void AreaManager::CommitRegionalIdlePatrolCandidate(const std::size_t index) noexcept {
    idle_patrol_.GoalIndex = index;
    idle_patrol_.Initialized = true;
}

void AreaManager::ResetRegionalIdlePatrol() noexcept {
    idle_patrol_ = {};
}

RegionalDefenseThreat AreaManager::AnalyzeRegionalDefenseThreat(
    const LangYa::UnitTeam my_team,
    const LangYa::UnitTeam enemy_team,
    const bool enable_soft_enemy_side_threat,
    const std::vector<RegionalDefenseEnemyPosition>& enemies) const {
    RegionalDefenseThreat threat{};
    for (const auto& enemy : enemies) {
        if (enemy.X <= 0 || enemy.Y <= 0) {
            continue;
        }

        const auto own_area = ResolvePointMainAreaExact(my_team, enemy.X, enemy.Y);
        if (own_area.has_value()) {
            switch (*own_area) {
                case Area::MainAreaKind::Base:
                    ++threat.OwnBaseCount;
                    break;
                case Area::MainAreaKind::Highland:
                    ++threat.OwnHighlandCount;
                    break;
                case Area::MainAreaKind::Roadland:
                    ++threat.OwnRoadlandCount;
                    break;
                case Area::MainAreaKind::Central:
                    ++threat.CommonCentralCount;
                    break;
                default:
                    break;
            }
        }

        const auto enemy_area = ResolvePointMainAreaExact(enemy_team, enemy.X, enemy.Y);
        if (enemy_area.has_value()) {
            if (*enemy_area == Area::MainAreaKind::Highland) {
                ++threat.EnemyHighlandCount;
            } else if (*enemy_area == Area::MainAreaKind::Roadland) {
                ++threat.EnemyRoadlandCount;
            }
        }
    }

    threat.HardThreat =
        threat.OwnBaseCount > 0 ||
        threat.OwnHighlandCount > 0 ||
        threat.OwnRoadlandCount > 0 ||
        threat.CommonCentralCount > 0;
    threat.SoftEnemySideThreat =
        enable_soft_enemy_side_threat &&
        !threat.HardThreat &&
        (threat.EnemyHighlandCount > 0 || threat.EnemyRoadlandCount > 0);
    return threat;
}

void AreaManager::StartRegionalDefenseSuppress(
    const AreaTimePoint now,
    const int hold_sec) {
    regional_defense_suppress_until_ = now + std::chrono::seconds(std::max(1, hold_sec));
}

bool AreaManager::IsRegionalDefenseAimSuppressActive(const AreaTimePoint now) const noexcept {
    return regional_defense_suppress_until_.time_since_epoch().count() != 0 &&
        now < regional_defense_suppress_until_;
}

void AreaManager::UpdateProgressWatchdogGoal(
    const std::uint8_t goal_id,
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team,
    const bool apply_team_offset,
    const Area::Point<std::uint16_t> goal_position,
    const int self_x,
    const int self_y,
    const AreaTimePoint now) {
    const bool same_goal =
        progress_watchdog_.Active &&
        progress_watchdog_.GoalId == goal_id &&
        progress_watchdog_.GoalPosition.x == goal_position.x &&
        progress_watchdog_.GoalPosition.y == goal_position.y;
    if (same_goal) {
        return;
    }

    progress_watchdog_.Active = true;
    progress_watchdog_.GoalId = goal_id;
    progress_watchdog_.BaseGoal = base_goal_id;
    progress_watchdog_.GoalTeam = goal_team;
    progress_watchdog_.ApplyTeamOffset = apply_team_offset;
    progress_watchdog_.GoalPosition = goal_position;
    progress_watchdog_.GoalStartTime = now;
    progress_watchdog_.LastMoveTime = now;
    progress_watchdog_.LastX = self_x;
    progress_watchdog_.LastY = self_y;
}

NaviProgressWatchdogDecision AreaManager::TickProgressWatchdog(
    const NaviProgressWatchdogInput& input) {
    NaviProgressWatchdogDecision decision{};
    if (!input.Enabled || !progress_watchdog_.Active || input.BlockedByAreaTransition) {
        return decision;
    }
    if (progress_watchdog_.BaseGoal == LangYa::Home.ID ||
        progress_watchdog_.BaseGoal == LangYa::Recovery.ID) {
        return decision;
    }

    if (input.ExternalReach.has_value() && *input.ExternalReach) {
        progress_watchdog_.LastMoveTime = input.Now;
        return decision;
    }

    const bool external_unreachable =
        input.ExternalReachable.has_value() && !*input.ExternalReachable;

    if (!external_unreachable) {
        if (!input.HasSelfPosition) {
            return decision;
        }

        const int arrive_cm = std::max(1, input.Setting.ArriveDistanceCm);
        const double distance_to_goal_sq = DistanceSq(
            input.SelfX,
            input.SelfY,
            static_cast<int>(progress_watchdog_.GoalPosition.x),
            static_cast<int>(progress_watchdog_.GoalPosition.y));
        if (distance_to_goal_sq <= static_cast<double>(arrive_cm * arrive_cm)) {
            progress_watchdog_.LastMoveTime = input.Now;
            progress_watchdog_.LastX = input.SelfX;
            progress_watchdog_.LastY = input.SelfY;
            return decision;
        }

        const int move_progress_cm = std::max(1, input.Setting.MoveProgressCm);
        if (DistanceSq(input.SelfX, input.SelfY, progress_watchdog_.LastX, progress_watchdog_.LastY) >=
            static_cast<double>(move_progress_cm * move_progress_cm)) {
            progress_watchdog_.LastMoveTime = input.Now;
            progress_watchdog_.LastX = input.SelfX;
            progress_watchdog_.LastY = input.SelfY;
            return decision;
        }

        const auto no_move_timeout =
            std::chrono::seconds(std::max(1, input.Setting.NoMoveTimeoutSec));
        if (input.Now - progress_watchdog_.GoalStartTime < no_move_timeout ||
            input.Now - progress_watchdog_.LastMoveTime < no_move_timeout) {
            return decision;
        }
    }

    if (progress_watchdog_.FallbackCooldownUntil.time_since_epoch().count() != 0 &&
        input.Now < progress_watchdog_.FallbackCooldownUntil &&
        progress_watchdog_.CooldownBaseGoal == progress_watchdog_.BaseGoal &&
        progress_watchdog_.CooldownTeam == progress_watchdog_.GoalTeam) {
        return decision;
    }

    const auto resolved_area =
        ResolveGoalMainArea(progress_watchdog_.BaseGoal, progress_watchdog_.GoalTeam);
    if (!resolved_area.has_value()) {
        progress_watchdog_.LastMoveTime = input.Now;
        return decision;
    }

    decision.NeedFallback = true;
    decision.ExternalUnreachable = external_unreachable;
    decision.OriginalBaseGoal = progress_watchdog_.BaseGoal;
    decision.OriginalGoalId = progress_watchdog_.GoalId;
    decision.OriginalGoalTeam = progress_watchdog_.GoalTeam;
    decision.OriginalApplyTeamOffset = progress_watchdog_.ApplyTeamOffset;
    decision.FallbackCandidates = ProgressFallbackCandidatesForArea(resolved_area->Kind);
    decision.FallbackCandidates.erase(
        std::remove(
            decision.FallbackCandidates.begin(),
            decision.FallbackCandidates.end(),
            decision.OriginalBaseGoal),
        decision.FallbackCandidates.end());
    if (decision.FallbackCandidates.empty()) {
        decision.NeedFallback = false;
        progress_watchdog_.LastMoveTime = input.Now;
    }
    return decision;
}

void AreaManager::CommitProgressWatchdogFallback(
    const AreaTimePoint now,
    const int fallback_cooldown_sec,
    const std::uint8_t cooldown_base_goal,
    const LangYa::UnitTeam cooldown_team) {
    progress_watchdog_.CooldownBaseGoal = cooldown_base_goal;
    progress_watchdog_.CooldownTeam = cooldown_team;
    progress_watchdog_.FallbackCooldownUntil =
        now + std::chrono::seconds(std::max(0, fallback_cooldown_sec));
}

void AreaManager::MarkProgressWatchdogFallbackFailed(
    const AreaTimePoint now,
    const bool has_self_position,
    const int self_x,
    const int self_y) {
    progress_watchdog_.LastMoveTime = now;
    if (has_self_position && self_x > 0 && self_y > 0) {
        progress_watchdog_.LastX = self_x;
        progress_watchdog_.LastY = self_y;
    }
}

std::optional<RegionalAreaTaskPlan> AreaManager::PlanRegionalAreaTaskForGoal(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team,
    const LangYa::UnitTeam my_team,
    const bool apply_team_offset,
    const bool has_self_position,
    const int self_x,
    const int self_y,
    const LangYa::MyBaseAreaTaskSetting& my_base_setting,
    const LangYa::PatrolGoalSelectionSetting& patrol_selection,
    const AreaTimePoint now,
    const bool self_in_my_highland) const {
    if (regional_area_task_.Active ||
        !IsValidBaseGoalId(base_goal_id) ||
        (goal_team != LangYa::UnitTeam::Red && goal_team != LangYa::UnitTeam::Blue) ||
        (my_team != LangYa::UnitTeam::Red && my_team != LangYa::UnitTeam::Blue)) {
        return std::nullopt;
    }

    const auto resolved_area = ResolveGoalMainArea(base_goal_id, goal_team);
    if (!resolved_area.has_value() ||
        resolved_area->UsedNearestFallback) {
        return std::nullopt;
    }

    if (resolved_area->Kind == Area::MainAreaKind::Central) {
        const auto patrol_index =
            NearestCommonCentralPatrolIndex(my_team, has_self_position, self_x, self_y);
        const auto& initial_goal = kCommonCentralPatrolGoals[patrol_index];
        return RegionalAreaTaskPlan{
            .Type = RegionalAreaTaskType::CommonCentral,
            .GoalTeam = my_team,
            .ApplyTeamOffset = apply_team_offset,
            .TriggerBaseGoal = base_goal_id,
            .InitialBaseGoal = initial_goal.BaseGoalId,
            .InitialGoalTeam = CommonCentralPatrolGoalTeam(my_team, initial_goal),
            .InitialPatrolIndex = patrol_index
        };
    }

    if (goal_team != my_team) {
        return std::nullopt;
    }

    if (resolved_area->Kind == Area::MainAreaKind::Highland) {
        return RegionalAreaTaskPlan{
            .Type = RegionalAreaTaskType::MyHighland,
            .GoalTeam = goal_team,
            .ApplyTeamOffset = apply_team_offset,
            .TriggerBaseGoal = base_goal_id,
            .InitialBaseGoal = LangYa::Highland.ID
        };
    }

    if (resolved_area->Kind == Area::MainAreaKind::Base && !self_in_my_highland) {
        return RegionalAreaTaskPlan{
            .Type = RegionalAreaTaskType::MyBase,
            .GoalTeam = goal_team,
            .ApplyTeamOffset = apply_team_offset,
            .TriggerBaseGoal = base_goal_id,
            .InitialBaseGoal = SelectMyBasePatrolGoal(
                goal_team,
                my_base_setting,
                patrol_selection,
                has_self_position,
                self_x,
                self_y,
                LangYa::Home.ID,
                false,
                now,
                patrol_goal_last_arrived_)
        };
    }

    if (resolved_area->Kind == Area::MainAreaKind::Roadland && !self_in_my_highland) {
        return RegionalAreaTaskPlan{
            .Type = RegionalAreaTaskType::MyRoadland,
            .GoalTeam = goal_team,
            .ApplyTeamOffset = apply_team_offset,
            .TriggerBaseGoal = base_goal_id,
            .InitialBaseGoal = LangYa::CentralToBase.ID
        };
    }

    if (resolved_area->Kind == Area::MainAreaKind::PreRoadland && !self_in_my_highland) {
        return RegionalAreaTaskPlan{
            .Type = RegionalAreaTaskType::MyPreRoadland,
            .GoalTeam = goal_team,
            .ApplyTeamOffset = apply_team_offset,
            .TriggerBaseGoal = base_goal_id,
            .InitialBaseGoal = LangYa::PreRoadland.ID
        };
    }

    return std::nullopt;
}

void AreaManager::StartRegionalAreaTask(
    const RegionalAreaTaskPlan& plan,
    const AreaTimePoint now) {
    regional_area_task_.Active = true;
    regional_area_task_.Type = plan.Type;
    if (plan.Type == RegionalAreaTaskType::MyBase) {
        regional_area_task_.Phase = RegionalAreaTaskPhase::BasePatrol;
    } else if (plan.Type == RegionalAreaTaskType::MyPreRoadland) {
        regional_area_task_.Phase = RegionalAreaTaskPhase::PreRoadlandApproach;
    } else if (plan.Type == RegionalAreaTaskType::MyRoadland) {
        regional_area_task_.Phase = RegionalAreaTaskPhase::RoadlandApproachCentralToBase;
    } else if (plan.Type == RegionalAreaTaskType::CommonCentral) {
        regional_area_task_.Phase = RegionalAreaTaskPhase::CentralPatrol;
    } else {
        regional_area_task_.Phase = RegionalAreaTaskPhase::ApproachHighland;
    }
    regional_area_task_.GoalTeam = plan.InitialGoalTeam == LangYa::UnitTeam::Unknown
        ? plan.GoalTeam
        : plan.InitialGoalTeam;
    regional_area_task_.ApplyTeamOffset = plan.ApplyTeamOffset;
    regional_area_task_.TriggerBaseGoal = plan.TriggerBaseGoal;
    regional_area_task_.OwnerTeam = plan.GoalTeam;
    regional_area_task_.PatrolIndex = plan.InitialPatrolIndex;
    regional_area_task_.PatrolStepCount = 0;
    regional_area_task_.CurrentBaseGoal =
        (plan.Type == RegionalAreaTaskType::MyBase ||
         plan.Type == RegionalAreaTaskType::MyPreRoadland ||
         plan.Type == RegionalAreaTaskType::MyRoadland ||
         plan.Type == RegionalAreaTaskType::CommonCentral)
            ? plan.InitialBaseGoal
            : LangYa::Highland.ID;
    regional_area_task_.StartTime = now;
    regional_area_task_.PhaseStartTime = now;
    regional_area_task_.BaseGoalArrivedTime = AreaTimePoint{};
}

bool AreaManager::RegionalAreaTaskCriticalControlActive() const noexcept {
    if (!regional_area_task_.Active ||
        regional_area_task_.Type != RegionalAreaTaskType::MyRoadland) {
        return false;
    }
    return regional_area_task_.Phase == RegionalAreaTaskPhase::RoadlandCrossToBaseToCentral ||
           regional_area_task_.Phase == RegionalAreaTaskPhase::RoadlandCrossToCentralToBase;
}

bool AreaManager::RegionalAreaTaskCanYieldToHigherPriority() const noexcept {
    return !RegionalAreaTaskCriticalControlActive();
}

void AreaManager::RequestRoadlandReturnToBase(const AreaTimePoint now) noexcept {
    if (!regional_area_task_.Active ||
        regional_area_task_.Type != RegionalAreaTaskType::MyRoadland) {
        return;
    }
    if (RegionalAreaTaskCriticalControlActive()) {
        return;
    }
    if (regional_area_task_.Phase == RegionalAreaTaskPhase::RoadlandApproachCentralToBase) {
        regional_area_task_.Phase = RegionalAreaTaskPhase::RoadlandReturnToCentralToBase;
        regional_area_task_.CurrentBaseGoal = LangYa::CentralToBase.ID;
        regional_area_task_.PhaseStartTime = now;
        return;
    }
    if (regional_area_task_.Phase == RegionalAreaTaskPhase::RoadlandCrossToCentralToBase ||
        regional_area_task_.Phase == RegionalAreaTaskPhase::RoadlandReturnToCentralToBase) {
        return;
    }
    regional_area_task_.Phase = RegionalAreaTaskPhase::RoadlandCrossToCentralToBase;
    regional_area_task_.CurrentBaseGoal = LangYa::CentralToBase.ID;
    regional_area_task_.PhaseStartTime = now;
}

RegionalAreaTaskTickResult AreaManager::TickRegionalAreaTask(
    const RegionalAreaTaskTickInput& input) {
    RegionalAreaTaskTickResult result{};
    if (!regional_area_task_.Active ||
        !input.Setting.Enable) {
        return result;
    }

    if (regional_area_task_.Type == RegionalAreaTaskType::MyPreRoadland) {
        if (!input.Setting.MyPreRoadland.Enable) {
            return result;
        }

        const auto& setting = input.Setting.MyPreRoadland;
        const auto phase_elapsed = [&]() {
            return regional_area_task_.PhaseStartTime.time_since_epoch().count() == 0
                ? std::chrono::seconds{0}
                : std::chrono::duration_cast<std::chrono::seconds>(
                    input.Now - regional_area_task_.PhaseStartTime);
        };
        auto complete_task = [&](const char* reason) {
            result.Completed = true;
            result.Type = regional_area_task_.Type;
            result.Phase = regional_area_task_.Phase;
            result.Reason = reason;
            regional_area_task_.Clear();
        };

        if (regional_area_task_.Phase == RegionalAreaTaskPhase::PreRoadlandApproach) {
            if (input.CurrentBaseGoalUnreachable) {
                complete_task("unreachable");
                return result;
            }
            if (setting.TravelTimeoutSec > 0 &&
                phase_elapsed() >= std::chrono::seconds(setting.TravelTimeoutSec)) {
                complete_task("timeout");
                return result;
            }
            if (input.CurrentBaseGoalArrived) {
                regional_area_task_.Phase = RegionalAreaTaskPhase::PreRoadlandHold;
                regional_area_task_.PhaseStartTime = input.Now;
            }
        }

        if (regional_area_task_.Phase == RegionalAreaTaskPhase::PreRoadlandHold &&
            setting.GoalHoldSec >= 0 &&
            phase_elapsed() >= std::chrono::seconds(setting.GoalHoldSec)) {
            complete_task("complete");
            return result;
        }

        result.Active = true;
        result.SetGoal = true;
        result.BaseGoalId = LangYa::PreRoadland.ID;
        result.GoalTeam = regional_area_task_.GoalTeam;
        result.ApplyTeamOffset = regional_area_task_.ApplyTeamOffset;
        result.SpeedLevel = std::clamp(setting.SpeedLevel, 0, 255);
        result.ResetNaviHold = true;
        result.NaviHoldSec = std::max(1, setting.CommandHoldSec);
        result.Type = regional_area_task_.Type;
        result.Phase = regional_area_task_.Phase;
        return result;
    }

    if (regional_area_task_.Type == RegionalAreaTaskType::MyBase) {
        if (!input.Setting.MyBase.Enable) {
            return result;
        }

        const auto& base_setting = input.Setting.MyBase;
        auto phase_elapsed = [&]() {
            if (regional_area_task_.PhaseStartTime.time_since_epoch().count() == 0) {
                return std::chrono::seconds{0};
            }
            return std::chrono::duration_cast<std::chrono::seconds>(
                input.Now - regional_area_task_.PhaseStartTime);
        };
        auto complete_base_task = [&](const char* reason) {
            result.Completed = true;
            result.Type = regional_area_task_.Type;
            result.Phase = regional_area_task_.Phase;
            result.Reason = reason;
            regional_area_task_.Clear();
        };
        auto switch_to_next_base_goal = [&](const char* complete_reason) {
            ++regional_area_task_.PatrolStepCount;
            if (base_setting.MaxPatrolSteps > 0 &&
                regional_area_task_.PatrolStepCount >= base_setting.MaxPatrolSteps) {
                complete_base_task(complete_reason);
                return false;
            }
            regional_area_task_.CurrentBaseGoal = SelectMyBasePatrolGoal(
                regional_area_task_.GoalTeam,
                base_setting,
                input.Setting.PatrolSelection,
                input.HasSelfPosition,
                input.SelfX,
                input.SelfY,
                regional_area_task_.CurrentBaseGoal,
                true,
                input.Now,
                patrol_goal_last_arrived_);
            regional_area_task_.PhaseStartTime = input.Now;
            regional_area_task_.BaseGoalArrivedTime = AreaTimePoint{};
            return true;
        };

        if (regional_area_task_.CurrentBaseGoal == LangYa::Home.ID ||
            regional_area_task_.Phase != RegionalAreaTaskPhase::BasePatrol) {
            regional_area_task_.Phase = RegionalAreaTaskPhase::BasePatrol;
            regional_area_task_.CurrentBaseGoal = SelectMyBasePatrolGoal(
                regional_area_task_.GoalTeam,
                base_setting,
                input.Setting.PatrolSelection,
                input.HasSelfPosition,
                input.SelfX,
                input.SelfY,
                LangYa::Home.ID,
                false,
                input.Now,
                patrol_goal_last_arrived_);
            regional_area_task_.PhaseStartTime = input.Now;
            regional_area_task_.BaseGoalArrivedTime = AreaTimePoint{};
            regional_area_task_.PatrolStepCount = 0;
        } else {
            const bool hold_started =
                regional_area_task_.BaseGoalArrivedTime.time_since_epoch().count() != 0;
            const bool current_goal_arrived = hold_started || input.CurrentBaseGoalArrived;
            const bool travel_timed_out =
                !current_goal_arrived &&
                !input.HoldCurrentBaseGoal &&
                base_setting.TravelTimeoutSec > 0 &&
                phase_elapsed() >= std::chrono::seconds(base_setting.TravelTimeoutSec);
            if (input.CurrentBaseGoalUnreachable || travel_timed_out) {
                if (!switch_to_next_base_goal(input.CurrentBaseGoalUnreachable
                    ? "unreachable"
                    : "timeout")) {
                    return result;
                }
            } else if (current_goal_arrived) {
                if (!hold_started) {
                    regional_area_task_.BaseGoalArrivedTime = input.Now;
                    patrol_goal_last_arrived_[regional_area_task_.CurrentBaseGoal] = input.Now;
                }
                const auto hold_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    input.Now - regional_area_task_.BaseGoalArrivedTime);
                if (!input.HoldCurrentBaseGoal &&
                    (base_setting.GoalHoldSec <= 0 ||
                     hold_elapsed >= std::chrono::seconds(base_setting.GoalHoldSec))) {
                    if (!switch_to_next_base_goal("patrol_complete")) {
                        return result;
                    }
                }
            }
        }

        result.Active = true;
        result.SetGoal = true;
        result.BaseGoalId = regional_area_task_.CurrentBaseGoal;
        result.GoalTeam = regional_area_task_.GoalTeam;
        result.ApplyTeamOffset = regional_area_task_.ApplyTeamOffset;
        result.Type = regional_area_task_.Type;
        result.Phase = regional_area_task_.Phase;
        result.ResetNaviHold = true;
        result.NaviHoldSec = std::max(1, base_setting.CommandHoldSec);
        return result;
    }

    if (regional_area_task_.Type == RegionalAreaTaskType::MyRoadland) {
        if (!input.Setting.MyRoadland.Enable) {
            return result;
        }

        const auto& roadland_setting = input.Setting.MyRoadland;
        auto start_phase = [&](const RegionalAreaTaskPhase phase, const std::uint8_t goal) {
            regional_area_task_.Phase = phase;
            regional_area_task_.CurrentBaseGoal = goal;
            regional_area_task_.PhaseStartTime = input.Now;
        };
        auto phase_elapsed = [&]() {
            if (regional_area_task_.PhaseStartTime.time_since_epoch().count() == 0) {
                return std::chrono::seconds{0};
            }
            return std::chrono::duration_cast<std::chrono::seconds>(
                input.Now - regional_area_task_.PhaseStartTime);
        };
        auto phase_timed_out = [&](const int timeout_sec) {
            return timeout_sec > 0 && phase_elapsed() >= std::chrono::seconds(timeout_sec);
        };

        while (regional_area_task_.Active) {
            switch (regional_area_task_.Phase) {
                case RegionalAreaTaskPhase::RoadlandApproachCentralToBase:
                    if (input.RoadlandCentralToBaseArrived ||
                        input.RoadlandCentralToBaseUnreachable ||
                        phase_timed_out(roadland_setting.TravelTimeoutSec)) {
                        start_phase(
                            RegionalAreaTaskPhase::RoadlandCrossToBaseToCentral,
                            LangYa::BaseToCentral.ID);
                        continue;
                    }
                    break;
                case RegionalAreaTaskPhase::RoadlandCrossToBaseToCentral:
                    if (input.RoadlandBaseToCentralArrived ||
                        input.RoadlandBaseToCentralUnreachable ||
                        phase_timed_out(roadland_setting.CrossTimeoutSec)) {
                        start_phase(
                            RegionalAreaTaskPhase::RoadlandHoldBaseToCentral,
                            LangYa::BaseToCentral.ID);
                        continue;
                    }
                    break;
                case RegionalAreaTaskPhase::RoadlandHoldBaseToCentral:
                    if (input.RoadlandShouldLeave ||
                        phase_timed_out(roadland_setting.GuardHoldSec)) {
                        start_phase(
                            RegionalAreaTaskPhase::RoadlandCrossToCentralToBase,
                            LangYa::CentralToBase.ID);
                        continue;
                    }
                    break;
                case RegionalAreaTaskPhase::RoadlandCrossToCentralToBase:
                    if (input.RoadlandCentralToBaseArrived ||
                        input.RoadlandCentralToBaseUnreachable ||
                        phase_timed_out(roadland_setting.CrossTimeoutSec)) {
                        result.Completed = true;
                        result.Type = regional_area_task_.Type;
                        result.Phase = regional_area_task_.Phase;
                        result.Reason = input.RoadlandCentralToBaseArrived
                            ? "arrived"
                            : (input.RoadlandCentralToBaseUnreachable ? "unreachable" : "timeout");
                        regional_area_task_.Clear();
                        return result;
                    }
                    break;
                case RegionalAreaTaskPhase::RoadlandReturnToCentralToBase:
                    if (input.RoadlandCentralToBaseArrived ||
                        input.RoadlandCentralToBaseUnreachable ||
                        phase_timed_out(roadland_setting.TravelTimeoutSec)) {
                        result.Completed = true;
                        result.Type = regional_area_task_.Type;
                        result.Phase = regional_area_task_.Phase;
                        result.Reason = input.RoadlandCentralToBaseArrived
                            ? "arrived"
                            : (input.RoadlandCentralToBaseUnreachable ? "unreachable" : "timeout");
                        regional_area_task_.Clear();
                        return result;
                    }
                    break;
                default:
                    start_phase(
                        RegionalAreaTaskPhase::RoadlandApproachCentralToBase,
                        LangYa::CentralToBase.ID);
                    continue;
            }
            break;
        }

        result.Active = true;
        result.SetGoal = true;
        result.BaseGoalId = regional_area_task_.CurrentBaseGoal;
        result.GoalTeam = regional_area_task_.GoalTeam;
        result.ApplyTeamOffset = regional_area_task_.ApplyTeamOffset;
        result.Type = regional_area_task_.Type;
        result.Phase = regional_area_task_.Phase;
        result.ResetNaviHold = true;
        result.NaviHoldSec =
            regional_area_task_.Phase == RegionalAreaTaskPhase::RoadlandHoldBaseToCentral
                ? std::max(1, roadland_setting.GuardHoldSec)
                : std::max(1, roadland_setting.CommandHoldSec);
        if (RegionalAreaTaskCriticalControlActive()) {
            result.FollowMode = true;
            result.UseFaceMode = roadland_setting.UseFaceMode;
            result.SuppressFire = true;
            result.PublishFaceTarget = roadland_setting.UseFaceMode;
            result.FaceTargetBaseGoalId = regional_area_task_.CurrentBaseGoal;
            result.FaceTargetZCm = roadland_setting.FaceTargetZCm;
        }
        return result;
    }

    if (regional_area_task_.Type == RegionalAreaTaskType::CommonCentral) {
        if (!input.Setting.CommonCentral.Enable) {
            return result;
        }

        const auto& central_setting = input.Setting.CommonCentral;
        auto phase_elapsed = [&]() {
            if (regional_area_task_.PhaseStartTime.time_since_epoch().count() == 0) {
                return std::chrono::seconds{0};
            }
            return std::chrono::duration_cast<std::chrono::seconds>(
                input.Now - regional_area_task_.PhaseStartTime);
        };
        const bool travel_timed_out =
            central_setting.TravelTimeoutSec > 0 &&
            phase_elapsed() >= std::chrono::seconds(central_setting.TravelTimeoutSec);
        auto complete_central_task = [&](const char* reason) {
            result.Completed = true;
            result.Type = regional_area_task_.Type;
            result.Phase = regional_area_task_.Phase;
            result.Reason = reason;
            regional_area_task_.Clear();
        };
        auto set_patrol_goal = [&](std::size_t patrol_index) {
            patrol_index %= kCommonCentralPatrolGoals.size();
            const auto& spec = kCommonCentralPatrolGoals[patrol_index];
            regional_area_task_.PatrolIndex = patrol_index;
            regional_area_task_.Phase = RegionalAreaTaskPhase::CentralPatrol;
            regional_area_task_.CurrentBaseGoal = spec.BaseGoalId;
            regional_area_task_.GoalTeam =
                CommonCentralPatrolGoalTeam(regional_area_task_.OwnerTeam, spec);
            regional_area_task_.PhaseStartTime = input.Now;
        };

        if (input.CentralShouldLeave) {
            result.Completed = true;
            result.Type = regional_area_task_.Type;
            result.Phase = regional_area_task_.Phase;
            result.Reason = "unhealthy";
            regional_area_task_.Clear();
            return result;
        }

        if (regional_area_task_.OwnerTeam != LangYa::UnitTeam::Red &&
            regional_area_task_.OwnerTeam != LangYa::UnitTeam::Blue) {
            regional_area_task_.OwnerTeam = regional_area_task_.GoalTeam;
        }

        if (regional_area_task_.CurrentBaseGoal == LangYa::Home.ID ||
            regional_area_task_.Phase != RegionalAreaTaskPhase::CentralPatrol) {
            set_patrol_goal(regional_area_task_.PatrolIndex);
            regional_area_task_.PatrolStepCount = 0;
        } else if (input.CurrentBaseGoalArrived ||
                   input.CurrentBaseGoalUnreachable ||
                   travel_timed_out) {
            ++regional_area_task_.PatrolStepCount;
            if (central_setting.MaxPatrolSteps > 0 &&
                regional_area_task_.PatrolStepCount >= central_setting.MaxPatrolSteps) {
                complete_central_task(input.CurrentBaseGoalUnreachable
                    ? "unreachable"
                    : (travel_timed_out ? "timeout" : "patrol_complete"));
                return result;
            }
            set_patrol_goal(NextCommonCentralPatrolIndex(regional_area_task_.PatrolIndex));
        }

        result.Active = true;
        result.SetGoal = true;
        result.BaseGoalId = regional_area_task_.CurrentBaseGoal;
        result.GoalTeam = regional_area_task_.GoalTeam;
        result.ApplyTeamOffset = regional_area_task_.ApplyTeamOffset;
        result.Type = regional_area_task_.Type;
        result.Phase = regional_area_task_.Phase;
        result.ResetNaviHold = true;
        result.NaviHoldSec = std::max(1, central_setting.CommandHoldSec);
        return result;
    }

    if (regional_area_task_.Type != RegionalAreaTaskType::MyHighland ||
        !input.Setting.MyHighland.Enable) {
        return result;
    }

    const auto& setting = input.Setting.MyHighland;
    auto start_phase = [&](const RegionalAreaTaskPhase phase) {
        regional_area_task_.Phase = phase;
        regional_area_task_.PhaseStartTime = input.Now;
    };
    auto phase_elapsed = [&]() {
        if (regional_area_task_.PhaseStartTime.time_since_epoch().count() == 0) {
            return std::chrono::seconds{0};
        }
        return std::chrono::duration_cast<std::chrono::seconds>(
            input.Now - regional_area_task_.PhaseStartTime);
    };
    auto phase_timed_out = [&](const int timeout_sec) {
        return timeout_sec > 0 && phase_elapsed() >= std::chrono::seconds(timeout_sec);
    };

    while (regional_area_task_.Active) {
        switch (regional_area_task_.Phase) {
            case RegionalAreaTaskPhase::ApproachHighland:
                if (input.HighlandArrived ||
                    input.HighlandUnreachable ||
                    phase_timed_out(setting.ApproachTimeoutSec)) {
                    start_phase(RegionalAreaTaskPhase::HighlandPatrol);
                    continue;
                }
                break;
            case RegionalAreaTaskPhase::HighlandPatrol:
                if (phase_elapsed() >= std::chrono::seconds(std::max(0, setting.HighlandPatrolHoldSec))) {
                    start_phase(RegionalAreaTaskPhase::ToBuffShoot);
                    continue;
                }
                break;
            case RegionalAreaTaskPhase::ToBuffShoot:
                if (input.BuffShootArrived ||
                    input.BuffShootUnreachable ||
                    phase_timed_out(setting.BuffShootTravelTimeoutSec)) {
                    start_phase(RegionalAreaTaskPhase::BuffShootHold);
                    continue;
                }
                break;
            case RegionalAreaTaskPhase::BuffShootHold:
                if (phase_elapsed() >= std::chrono::seconds(std::max(0, setting.BuffShootHoldSec))) {
                    start_phase(RegionalAreaTaskPhase::LeaveViaHoleRoad);
                    continue;
                }
                break;
            case RegionalAreaTaskPhase::LeaveViaHoleRoad:
                if (input.HoleRoadArrived ||
                    input.HoleRoadUnreachable ||
                    phase_timed_out(setting.LeaveTimeoutSec)) {
                    result.Completed = true;
                    result.Type = regional_area_task_.Type;
                    result.Phase = regional_area_task_.Phase;
                    result.Reason = input.HoleRoadArrived
                        ? "arrived"
                        : (input.HoleRoadUnreachable ? "unreachable" : "timeout");
                    regional_area_task_.Clear();
                    return result;
                }
                break;
            default:
                regional_area_task_.Clear();
                return result;
        }
        break;
    }

    if (!regional_area_task_.Active) {
        return result;
    }

    result.Active = true;
    result.SetGoal = true;
    result.GoalTeam = regional_area_task_.GoalTeam;
    result.ApplyTeamOffset = regional_area_task_.ApplyTeamOffset;
    result.Type = regional_area_task_.Type;
    result.Phase = regional_area_task_.Phase;
    result.ResetNaviHold = true;
    result.NaviHoldSec = 1;

    switch (regional_area_task_.Phase) {
        case RegionalAreaTaskPhase::ApproachHighland:
            result.BaseGoalId = LangYa::Highland.ID;
            result.FollowMode = true;
            result.UseFaceMode = setting.UseFaceMode;
            result.SuppressFire = true;
            break;
        case RegionalAreaTaskPhase::HighlandPatrol:
            result.BaseGoalId = LangYa::Highland.ID;
            result.NaviHoldSec = std::max(1, setting.HighlandPatrolHoldSec);
            break;
        case RegionalAreaTaskPhase::ToBuffShoot:
            result.BaseGoalId = LangYa::BuffShoot.ID;
            break;
        case RegionalAreaTaskPhase::BuffShootHold:
            result.BaseGoalId = LangYa::BuffShoot.ID;
            result.NaviHoldSec = std::max(1, setting.BuffShootHoldSec);
            break;
        case RegionalAreaTaskPhase::LeaveViaHoleRoad:
            result.BaseGoalId = LangYa::HoleRoad.ID;
            result.FollowMode = true;
            result.UseFaceMode = setting.UseFaceMode;
            result.SuppressFire = true;
            break;
        default:
            result.Active = false;
            result.SetGoal = false;
            break;
    }
    return result;
}

bool AreaManager::IsValidBaseGoalId(const std::uint8_t base_goal_id) noexcept {
    return base_goal_id <= MaxBaseGoalId();
}

bool AreaManager::IsReservedNonCombatGoalId(const std::uint8_t base_goal_id) noexcept {
    return base_goal_id == LangYa::Home.ID ||
           base_goal_id == LangYa::Base.ID ||
           base_goal_id == LangYa::Recovery.ID;
}

std::uint8_t AreaManager::ResolveGoalId(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam team,
    const bool apply_team_offset) noexcept {
    if (!apply_team_offset) {
        return base_goal_id;
    }
    return team == LangYa::UnitTeam::Blue
        ? static_cast<std::uint8_t>(base_goal_id + LangYa::TeamedLocation::LocationCount)
        : base_goal_id;
}

Area::Point<std::uint16_t> AreaManager::GoalPointByBaseId(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team) {
    switch (base_goal_id) {
        case LangYa::Home.ID: return Area::Home(goal_team);
        case LangYa::Base.ID: return Area::Base(goal_team);
        case LangYa::Recovery.ID: return Area::Recovery(goal_team);
        case LangYa::BuffShoot.ID: return Area::BuffShoot(goal_team);
        case LangYa::LeftHighLand.ID: return Area::LeftHighLand(goal_team);
        case LangYa::CastleLeft1.ID: return Area::CastleLeft1(goal_team);
        case LangYa::CastleLeft2.ID: return Area::CastleLeft2(goal_team);
        case LangYa::Castle.ID: return Area::Castle(goal_team);
        case LangYa::CastleRight1.ID: return Area::CastleRight1(goal_team);
        case LangYa::CastleRight2.ID: return Area::CastleRight2(goal_team);
        case LangYa::FlyRoad.ID: return Area::FlyRoad(goal_team);
        case LangYa::OutpostArea.ID: return Area::OutpostArea(goal_team);
        case LangYa::MidShoot.ID: return Area::MidShoot(goal_team);
        case LangYa::LeftShoot.ID: return Area::LeftShoot(goal_team);
        case LangYa::OutpostShoot.ID: return Area::OutpostShoot(goal_team);
        case LangYa::BuffAround1.ID: return Area::BuffAround1(goal_team);
        case LangYa::BuffAround2.ID: return Area::BuffAround2(goal_team);
        case LangYa::RightShoot.ID: return Area::RightShoot(goal_team);
        case LangYa::HoleRoad.ID: return Area::HoleRoad(goal_team);
        case LangYa::OccupyArea.ID: return Area::OccupyArea(goal_team);
        case LangYa::Highland.ID: return Area::Highland(goal_team);
        case LangYa::BaseToCentral.ID: return Area::BaseToCentral(goal_team);
        case LangYa::CentralToBase.ID: return Area::CentralToBase(goal_team);
        case LangYa::BuffOutpost.ID: return Area::BuffOutpost(goal_team);
        case LangYa::OutpostGuard.ID: return Area::OutpostGuard(goal_team);
        case LangYa::PreRoadland.ID: return Area::PreRoadland(goal_team);
        case LangYa::CentralLeftA.ID: return Area::CentralLeft.A(goal_team);
        case LangYa::CentralLeftB.ID: return Area::CentralLeft.B(goal_team);
        default: return Area::Home(goal_team);
    }
}

double AreaManager::DistanceSq(
    const int ax,
    const int ay,
    const int bx,
    const int by) noexcept {
    const double dx = static_cast<double>(ax - bx);
    const double dy = static_cast<double>(ay - by);
    return dx * dx + dy * dy;
}

std::optional<Area::MainAreaKind> AreaManager::MainAreaKindFromToken(std::string_view token) {
    const auto normalized = NormalizeAreaToken(token);
    if (normalized == "base") {
        return Area::MainAreaKind::Base;
    }
    if (normalized == "highland" || normalized == "high_land" || normalized == "high") {
        return Area::MainAreaKind::Highland;
    }
    if (normalized == "pre_roadland" || normalized == "preroadland" ||
        normalized == "pre_road" || normalized == "pre_road_land") {
        return Area::MainAreaKind::PreRoadland;
    }
    if (normalized == "roadland" || normalized == "road_land" || normalized == "road") {
        return Area::MainAreaKind::Roadland;
    }
    if (normalized == "central" || normalized == "center" ||
        normalized == "centre" || normalized == "middle") {
        return Area::MainAreaKind::Central;
    }
    return std::nullopt;
}

std::optional<Area::MainAreaKind> AreaManager::ResolvePointMainAreaExact(
    const LangYa::UnitTeam area_team,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return std::nullopt;
    }
    for (const auto area_kind : kAllMainAreas) {
        if (Area::IsPointInsideMainArea(area_team, area_kind, x, y)) {
            return area_kind;
        }
    }
    return std::nullopt;
}

std::optional<ResolvedMainArea> AreaManager::ResolveGoalMainArea(
    const std::uint8_t base_goal_id,
    const LangYa::UnitTeam goal_team) {
    if (!IsValidBaseGoalId(base_goal_id)) {
        return std::nullopt;
    }
    if (goal_team != LangYa::UnitTeam::Red && goal_team != LangYa::UnitTeam::Blue) {
        return std::nullopt;
    }

    const auto goal_point = GoalPointByBaseId(base_goal_id, goal_team);
    const int goal_x = static_cast<int>(goal_point.x);
    const int goal_y = static_cast<int>(goal_point.y);

    for (const auto area_kind : kAllMainAreas) {
        if (Area::IsPointInsideMainArea(goal_team, area_kind, goal_x, goal_y)) {
            return ResolvedMainArea{.Kind = area_kind, .UsedNearestFallback = false};
        }
    }

    Area::MainAreaKind nearest_kind = Area::MainAreaKind::Base;
    double nearest_dist_sq = std::numeric_limits<double>::infinity();
    for (const auto area_kind : kAllMainAreas) {
        const auto centroid = MainAreaCentroid(goal_team, area_kind);
        const double dx = static_cast<double>(goal_x) - centroid.x;
        const double dy = static_cast<double>(goal_y) - centroid.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < nearest_dist_sq) {
            nearest_dist_sq = dist_sq;
            nearest_kind = area_kind;
        }
    }
    return ResolvedMainArea{.Kind = nearest_kind, .UsedNearestFallback = true};
}

bool AreaManager::IsPositionInMainArea(
    const LangYa::UnitTeam area_team,
    const Area::MainAreaKind kind,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return false;
    }
    return Area::IsPointInsideMainArea(area_team, kind, x, y);
}

bool AreaManager::IsPositionInRoadlandFollowModeArea(
    const LangYa::UnitTeam area_team,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return false;
    }
    return Area::IsPointInsideRoadlandFollowModeArea(area_team, x, y);
}

bool AreaManager::IsPositionInPreRoadlandArea(
    const LangYa::UnitTeam area_team,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return false;
    }
    return Area::IsPointInsidePreRoadlandArea(area_team, x, y);
}

bool AreaManager::IsPositionInRecoveryArea(
    const LangYa::UnitTeam area_team,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return false;
    }
    return Area::IsPointInsideRecoveryArea(area_team, x, y);
}

bool AreaManager::IsPositionInCentralLeftLineArea(
    const LangYa::UnitTeam area_team,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return false;
    }
    return Area::IsPointInsideCentralLeftLineArea(area_team, x, y);
}

bool AreaManager::IsPositionInSettleArea(
    const LangYa::UnitTeam area_team,
    const int x,
    const int y) {
    if (area_team != LangYa::UnitTeam::Red && area_team != LangYa::UnitTeam::Blue) {
        return false;
    }
    return Area::IsPointInsideSettleArea(area_team, x, y);
}

std::optional<AreaKey> AreaManager::ResolveAreaKeyForPoint(
    const LangYa::UnitTeam my_team,
    const LangYa::UnitTeam enemy_team,
    const int x,
    const int y) {
    if (my_team != LangYa::UnitTeam::Red && my_team != LangYa::UnitTeam::Blue) {
        return std::nullopt;
    }
    if (Area::IsPointInsideMainArea(my_team, Area::MainAreaKind::Central, x, y)) {
        return AreaKey{
            .Side = AreaSide::Common,
            .Kind = Area::MainAreaKind::Central,
            .Team = LangYa::UnitTeam::Unknown
        };
    }
    for (const auto area_kind : kSideMainAreas) {
        if (Area::IsPointInsideMainArea(my_team, area_kind, x, y)) {
            return AreaKey{.Side = AreaSide::My, .Kind = area_kind, .Team = my_team};
        }
    }
    if (enemy_team != LangYa::UnitTeam::Red && enemy_team != LangYa::UnitTeam::Blue) {
        return std::nullopt;
    }
    for (const auto area_kind : kSideMainAreas) {
        if (Area::IsPointInsideMainArea(enemy_team, area_kind, x, y)) {
            return AreaKey{.Side = AreaSide::Enemy, .Kind = area_kind, .Team = enemy_team};
        }
    }
    return std::nullopt;
}

std::optional<ResolvedAreaKey> AreaManager::ResolveAreaKeyForPointWithNearest(
    const LangYa::UnitTeam my_team,
    const LangYa::UnitTeam enemy_team,
    const int x,
    const int y) {
    const auto exact = ResolveAreaKeyForPoint(my_team, enemy_team, x, y);
    if (exact.has_value()) {
        return ResolvedAreaKey{.Key = *exact, .UsedNearestFallback = false};
    }
    if (my_team != LangYa::UnitTeam::Red && my_team != LangYa::UnitTeam::Blue) {
        return std::nullopt;
    }

    struct Candidate {
        AreaKey Key{};
        Area::Point<double> Centroid{};
    };
    std::vector<Candidate> candidates;
    candidates.reserve(7);
    for (const auto area_kind : kSideMainAreas) {
        candidates.push_back(Candidate{
            .Key = AreaKey{.Side = AreaSide::My, .Kind = area_kind, .Team = my_team},
            .Centroid = MainAreaCentroid(my_team, area_kind)
        });
    }
    candidates.push_back(Candidate{
        .Key = AreaKey{
            .Side = AreaSide::Common,
            .Kind = Area::MainAreaKind::Central,
            .Team = LangYa::UnitTeam::Unknown
        },
        .Centroid = MainAreaCentroid(my_team, Area::MainAreaKind::Central)
    });
    if (enemy_team == LangYa::UnitTeam::Red || enemy_team == LangYa::UnitTeam::Blue) {
        for (const auto area_kind : kSideMainAreas) {
            candidates.push_back(Candidate{
                .Key = AreaKey{.Side = AreaSide::Enemy, .Kind = area_kind, .Team = enemy_team},
                .Centroid = MainAreaCentroid(enemy_team, area_kind)
            });
        }
    }
    if (candidates.empty()) {
        return std::nullopt;
    }

    const Candidate* best = nullptr;
    double best_dist_sq = std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates) {
        const double dx = static_cast<double>(x) - candidate.Centroid.x;
        const double dy = static_cast<double>(y) - candidate.Centroid.y;
        const double dist_sq = dx * dx + dy * dy;
        if (best == nullptr || dist_sq < best_dist_sq) {
            best = &candidate;
            best_dist_sq = dist_sq;
        }
    }
    if (best == nullptr) {
        return std::nullopt;
    }
    return ResolvedAreaKey{.Key = best->Key, .UsedNearestFallback = true};
}

}  // namespace BehaviorTree
