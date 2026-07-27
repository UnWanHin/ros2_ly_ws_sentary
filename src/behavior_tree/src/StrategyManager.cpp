#include "../include/Application.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <string>

namespace BehaviorTree {

const char* StrategyLayerName(const StrategyLayer layer) noexcept {
    switch (layer) {
        case StrategyLayer::Hard: return "Hard";
        case StrategyLayer::Default: return "Default";
        case StrategyLayer::Task: return "Task";
        case StrategyLayer::Tactical: return "Tactical";
        case StrategyLayer::Special: return "Special";
        case StrategyLayer::Finalizer: return "Finalizer";
        default: return "Unknown";
    }
}

void StrategyManager::PublishRuntimeToBlackboards(Application& app) const noexcept {
    if (app.TickBlackboard_) {
        app.TickBlackboard_->set("StrategyLayerHandled", handled_);
        app.TickBlackboard_->set("StrategyLayerHardLock", hard_lock_);
        app.TickBlackboard_->set("StrategyLayerDefaultRequested", default_requested_);
        app.TickBlackboard_->set("StrategyLayerHandledBy", std::string{handled_by_});
        app.TickBlackboard_->set("ChaseTacticalAllowed", app.IsChaseTacticalAllowed());
    }
    if (app.GlobalBlackboard_) {
        app.GlobalBlackboard_->set("StrategyLayerHandledBy", std::string{handled_by_});
        app.GlobalBlackboard_->set("StrategyLayerHardLock", hard_lock_);
        app.GlobalBlackboard_->set("StrategyLayerDefaultRequested", default_requested_);
        app.GlobalBlackboard_->set("ChaseTacticalAllowed", app.IsChaseTacticalAllowed());
    }
}

void StrategyManager::Reset(Application& app) noexcept {
    handled_ = false;
    hard_lock_ = false;
    default_requested_ = false;
    default_goal_commanded_ = false;
    handled_layer_ = StrategyLayer::Finalizer;
    handled_by_ = "none";
    app.ResetChaseTacticalAuthorization();
    PublishRuntimeToBlackboards(app);
}

void StrategyManager::MarkHandled(
    Application& app,
    const StrategyLayer layer,
    const bool hard_lock) noexcept {
    handled_ = true;
    hard_lock_ = hard_lock_ || hard_lock;
    handled_layer_ = layer;
    handled_by_ = StrategyLayerName(layer);
    PublishRuntimeToBlackboards(app);
}

bool Application::CanAuthorizeChaseTactical() const noexcept {
    const bool regional_task_blocks_chase =
        areaManager_.RegionalAreaTaskActive() &&
        !areaManager_.RegionalAreaTaskCanYieldToHigherPriority();
    if (!config.ChaseSettings.Enable ||
        !config.ChaseSettings.FollowAimTarget ||
        regional_task_blocks_chase ||
        areaManager_.HighlandTransitionActive() ||
        outpostVisualScoutNavigationActive_ ||
        ShouldSuppressChaseForOutpostTask() ||
        ShouldSuppressChaseForSpecialPatrol()) {
        return false;
    }

    switch (aimMode) {
        case AimMode::AutoAim:
            return config.ChaseSettings.EnableInAutoAim;
        case AimMode::RotateScan:
            return config.ChaseSettings.EnableInRotateScan;
        case AimMode::Outpost:
            return config.ChaseSettings.EnableInOutpostMode;
        case AimMode::Buff:
            return config.ChaseSettings.EnableInBuffMode;
        default:
            return false;
    }
}

bool StrategyManager::RunHard(Application& app) {
    Reset(app);

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    const bool bt_owns_protect_hero_enhanced_defense =
        app.postureTaskIntent_.Intent == TaskPostureIntent::ProtectHeroEnhancedDefense &&
        app.postureTaskIntent_.OwnsCurrentGoal;
    const bool protect_hero_enhanced_defense_active =
        ShouldDeferRecoveryForProtectHeroEnhancedDefense(
            app.protectHeroActive_ &&
                app.lastDecisionIntent_.Reason == DecisionReason::ProtectHero &&
                bt_owns_protect_hero_enhanced_defense,
            app.postureManager_.IsSwitchCooldownReady(std::chrono::steady_clock::now()),
            app.postureManager_.Runtime());
    app.protectHeroEnhancedDefenseRecoveryDeferred_ = protect_hero_enhanced_defense_active;
    if (!protect_hero_enhanced_defense_active && app.CheckPositionRecovery()) {
        app.CancelMapCommandTask();
        MarkHandled(app, StrategyLayer::Hard);
        return true;
    }

    if (app.areaManager_.RegionalAreaTaskActive() &&
        app.areaManager_.RegionalAreaTask().Type == RegionalAreaTaskType::MyReadyRoadland &&
        !app.areaManager_.RegionalAreaTaskCanYieldToHigherPriority()) {
        app.CancelMapCommandTask();
        if (app.TickRegionalAreaTask(my_team, enemy_team)) {
            MarkHandled(app, StrategyLayer::Hard, true);
            return true;
        }
    }
    return false;
}

bool StrategyManager::RunDefault(Application& app) {
    if (handled_) {
        return true;
    }

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    if (app.areaManager_.RegionalAreaTaskActive()) {
        default_requested_ = true;
        (void)app.TickRegionalAreaTask(my_team, enemy_team);
        MarkHandled(app, StrategyLayer::Default);
        return true;
    }

    default_requested_ = app.IsDefaultRegionalDecisionReady(my_team, enemy_team);
    if (!default_requested_) {
        PublishRuntimeToBlackboards(app);
        return false;
    }

    if (app.naviCommandIntervalClock.trigger()) {
        default_goal_commanded_ = app.TrySetDefaultRegionalGoal(my_team, enemy_team);
    }
    MarkHandled(app, StrategyLayer::Default);
    return true;
}

bool StrategyManager::RunTask(Application& app) {
    if (handled_ &&
        (handled_layer_ != StrategyLayer::Default || default_goal_commanded_)) {
        return true;
    }

    if (app.TrySetMapCommandGoal()) {
        MarkHandled(app, StrategyLayer::Task);
        return true;
    }

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    if (app.TickNaviAreaTransition() ||
        app.TickNaviProgressWatchdog(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Task);
        return true;
    }
    return false;
}

bool Application::TrySetMapCommandGoal() {
    const auto now = std::chrono::steady_clock::now();
    if (handledMapCommandRxSequence_ != mapCommandRxSequence_) {
        handledMapCommandRxSequence_ = mapCommandRxSequence_;
        if (mapCommandTask_.Observe(
                {.HasTargetPosition = mapCommand.has_target_position,
                 .XMeter = mapCommand.target_position_x_m,
                 .YMeter = mapCommand.target_position_y_m},
                config.TaskSettings.MapCommand,
                now)) {
            mapCommandGoalPublishPending_ = true;
            const auto raw_goal = mapCommandTask_.ActiveGoal(now);
            if (raw_goal.has_value()) {
                lastDecisionIntent_ = DecisionIntent{
                    .Layer = DecisionLayer::Task,
                    .Reason = DecisionReason::MapCommand,
                    .BaseGoalId = std::numeric_limits<std::uint8_t>::max(),
                    .ResolvedGoalId = std::numeric_limits<std::uint8_t>::max(),
                    .GoalTeam = UnitTeam::Unknown,
                    .ApplyTeamOffset = false,
                    .Priority = DecisionPriorityForReason(DecisionReason::MapCommand),
                    .Detail = "raw_cm=" + std::to_string(raw_goal->XCentimeter) + "," +
                        std::to_string(raw_goal->YCentimeter)};
            }
        }
    }

    const auto raw_goal = mapCommandTask_.ActiveGoal(now);
    if (!raw_goal.has_value()) {
        activeMapCommandGoal_.reset();
        mapCommandGoalPublishPending_ = false;
        return false;
    }

    if (!activeMapCommandGoal_.has_value() ||
        activeMapCommandGoal_->XCentimeter != raw_goal->XCentimeter ||
        activeMapCommandGoal_->YCentimeter != raw_goal->YCentimeter) {
        mapCommandGoalPublishPending_ = true;
    }
    activeMapCommandGoal_ = raw_goal;
    naviExternalStatusGoalInitialized_ = false;
    naviGoalPublishAllowed_ = true;
    return true;
}

void Application::CancelMapCommandTask() noexcept {
    if (handledMapCommandRxSequence_ != mapCommandRxSequence_) {
        handledMapCommandRxSequence_ = mapCommandRxSequence_;
        (void)mapCommandTask_.Observe(
            {.HasTargetPosition = mapCommand.has_target_position,
             .XMeter = mapCommand.target_position_x_m,
             .YMeter = mapCommand.target_position_y_m},
            config.TaskSettings.MapCommand,
            std::chrono::steady_clock::now());
    }
    mapCommandTask_.Cancel();
    activeMapCommandGoal_.reset();
    mapCommandGoalPublishPending_ = false;
}

bool StrategyManager::RunTactical(Application& app) {
    if (handled_) {
        if (handled_layer_ == StrategyLayer::Default &&
            !default_goal_commanded_ &&
            app.TryApplyChaseTactical()) {
            PublishRuntimeToBlackboards(app);
        }
        return true;
    }

    if (app.IsLeagueProfile()) {
        app.SetPositionLeagueSimple();
        app.TryApplyChaseTactical();
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.IsShowcasePatrolEnabled()) {
        app.SetPositionShowcasePatrol();
        app.TryApplyChaseTactical();
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    const auto opening_defense_threat = app.IsOutpostOpeningHighPriorityActive()
        ? app.EvaluateRegionalDefenseThreat(my_team, enemy_team)
        : std::optional<RegionalDefenseThreat>{};
    const bool opening_base_defense_required =
        opening_defense_threat.has_value() &&
        opening_defense_threat->OwnBaseCount > 0;

    if (opening_base_defense_required && app.TrySetRegionalDefenseGoal(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.aimMode == AimMode::Buff) {
        if (app.naviCommandIntervalClock.trigger()) {
            app.TrySetAimModeTaskGoal(my_team, enemy_team, "regional_tactical_buff_mode");
        }
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.IsOutpostOpeningHighPriorityActive()) {
        if (app.IsOutpostVisualScoutNavigationActive() && app.aimMode != AimMode::Outpost) {
            if (app.naviCommandIntervalClock.trigger()) {
                app.TrySetOutpostVisualScoutTravelGoal(
                    my_team,
                    enemy_team,
                    "regional_tactical_opening_outpost_scout_travel");
            }
            MarkHandled(app, StrategyLayer::Tactical);
            return true;
        }

        if (app.aimMode == AimMode::Outpost) {
            if (app.naviCommandIntervalClock.trigger()) {
                app.TrySetAimModeTaskGoal(
                    my_team,
                    enemy_team,
                    "regional_tactical_opening_outpost_aim");
            }
            MarkHandled(app, StrategyLayer::Tactical);
            return true;
        }
    }

    if (app.IsOutpostVisualScoutNavigationActive() && app.aimMode != AimMode::Outpost) {
        if (app.naviCommandIntervalClock.trigger()) {
            app.TrySetOutpostVisualScoutTravelGoal(
                my_team,
                enemy_team,
                "regional_tactical_outpost_scout_travel");
        }
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.aimMode == AimMode::Outpost) {
        if (app.naviCommandIntervalClock.trigger()) {
            app.TrySetAimModeTaskGoal(my_team, enemy_team, "regional_tactical_aim_mode");
        }
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.TickNaviProgressWatchdog(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    enum class TacticalPriorityAction : std::uint8_t {
        ProtectCastle = 0,
        ProtectOutpost = 1,
        ProtectHero = 2,
        Chase = 3,
    };
    struct TacticalPriorityCandidate {
        TacticalPriorityAction Action;
        int Priority;
        int TieBreak;
    };
    const auto& priority = app.config.TacticalSettings.Priority;
    std::array<TacticalPriorityCandidate, 4> candidates{{
        {TacticalPriorityAction::ProtectCastle, priority.ProtectCastle, 0},
        {TacticalPriorityAction::ProtectOutpost, priority.ProtectOutpost, 1},
        {TacticalPriorityAction::ProtectHero, priority.ProtectHero, 2},
        {TacticalPriorityAction::Chase, priority.Chase, 3},
    }};
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const TacticalPriorityCandidate& lhs, const TacticalPriorityCandidate& rhs) {
            if (lhs.Priority != rhs.Priority) {
                return lhs.Priority < rhs.Priority;
            }
            return lhs.TieBreak < rhs.TieBreak;
        });

    for (const auto candidate : candidates) {
        bool selected = false;
        switch (candidate.Action) {
            case TacticalPriorityAction::ProtectCastle:
                selected = app.TrySetRegionalDefenseGoal(my_team, enemy_team);
                break;
            case TacticalPriorityAction::ProtectOutpost:
                selected = app.TrySetProtectOutpostGoal(my_team, enemy_team);
                break;
            case TacticalPriorityAction::ProtectHero:
                selected = app.TrySetProtectHeroGoal(my_team, enemy_team);
                break;
            case TacticalPriorityAction::Chase:
                selected = app.TryApplyChaseTactical();
                break;
        }
        if (selected) {
            MarkHandled(app, StrategyLayer::Tactical);
            return true;
        }
    }

    return false;
}

bool StrategyManager::RunSpecial(Application& app) {
    if (handled_) {
        return true;
    }

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    if (app.TrySetSpecialPatrolGoal(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Special);
        return true;
    }
    return false;
}

bool StrategyManager::RunFinalizer(Application& app) {
    if (!handled_) {
        MarkHandled(app, StrategyLayer::Finalizer);
    } else {
        PublishRuntimeToBlackboards(app);
    }
    return true;
}

bool Application::RunStrategyLayerHard() {
    return strategyManager_.RunHard(*this);
}

bool Application::RunStrategyLayerDefault() {
    return strategyManager_.RunDefault(*this);
}

bool Application::RunStrategyLayerTask() {
    return strategyManager_.RunTask(*this);
}

bool Application::RunStrategyLayerTactical() {
    return strategyManager_.RunTactical(*this);
}

bool Application::RunStrategyLayerSpecial() {
    return strategyManager_.RunSpecial(*this);
}

bool Application::RunStrategyLayerFinalizer() {
    return strategyManager_.RunFinalizer(*this);
}

}  // namespace BehaviorTree
