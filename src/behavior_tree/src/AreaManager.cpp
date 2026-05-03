// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/AreaManager.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>

namespace BehaviorTree {
namespace {

constexpr std::array<Area::MainAreaKind, 4> kAllMainAreas{
    Area::MainAreaKind::Base,
    Area::MainAreaKind::Highland,
    Area::MainAreaKind::Roadland,
    Area::MainAreaKind::Central
};

constexpr std::array<Area::MainAreaKind, 3> kSideMainAreas{
    Area::MainAreaKind::Base,
    Area::MainAreaKind::Highland,
    Area::MainAreaKind::Roadland
};

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
                LangYa::CastleLeft.ID,
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

}  // namespace

const char* NaviAreaTransitionKindToString(const NaviAreaTransitionKind kind) {
    switch (kind) {
        case NaviAreaTransitionKind::None: return "None";
        case NaviAreaTransitionKind::EnterMyHighland: return "EnterMyHighland";
        case NaviAreaTransitionKind::ViaHighland: return "ViaHighland";
        case NaviAreaTransitionKind::LeaveMyHighland: return "LeaveMyHighland";
        case NaviAreaTransitionKind::LeaveMyHighlandViaCastleLeft: return "LeaveMyHighlandViaCastleLeft";
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
    const bool self_in_my_highland) const {
    if (transition_.Active ||
        !IsHighlandCompatEnabled() ||
        !IsValidBaseGoalId(base_goal_id) ||
        goal_team == LangYa::UnitTeam::Unknown ||
        my_team == LangYa::UnitTeam::Unknown) {
        return std::nullopt;
    }

    NaviAreaTransitionPlan plan{};
    plan.GoalTeam = goal_team;
    plan.ApplyTeamOffset = apply_team_offset;

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
            ? NaviAreaTransitionKind::LeaveMyHighlandViaCastleLeft
            : NaviAreaTransitionKind::LeaveMyHighland;
        plan.ViaBaseGoal = target_is_base_area ? LangYa::CastleLeft.ID : base_goal_id;
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
    if (!IsHighlandCompatEnabled() || !transition_.Active) {
        return result;
    }

    const auto timeout = std::chrono::seconds(std::max(1, navi_goal_config_.HighlandCompatTimeoutSec));
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
        threat.OwnRoadlandCount > 0;
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
        case LangYa::CastleLeft.ID: return Area::CastleLeft(goal_team);
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

}  // namespace BehaviorTree
