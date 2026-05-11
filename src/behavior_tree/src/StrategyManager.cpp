#include "../include/Application.hpp"

#include <string>

namespace BehaviorTree {

const char* StrategyLayerName(const StrategyLayer layer) noexcept {
    switch (layer) {
        case StrategyLayer::Hard: return "Hard";
        case StrategyLayer::Default: return "Default";
        case StrategyLayer::Task: return "Task";
        case StrategyLayer::Tactical: return "Tactical";
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
    }
    if (app.GlobalBlackboard_) {
        app.GlobalBlackboard_->set("StrategyLayerHandledBy", std::string{handled_by_});
        app.GlobalBlackboard_->set("StrategyLayerHardLock", hard_lock_);
        app.GlobalBlackboard_->set("StrategyLayerDefaultRequested", default_requested_);
    }
}

void StrategyManager::Reset(Application& app) noexcept {
    handled_ = false;
    hard_lock_ = false;
    default_requested_ = false;
    default_goal_commanded_ = false;
    handled_layer_ = StrategyLayer::Finalizer;
    handled_by_ = "none";
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

bool StrategyManager::RunHard(Application& app) {
    Reset(app);

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    if (app.CheckPositionRecovery()) {
        MarkHandled(app, StrategyLayer::Hard);
        return true;
    }

    if (app.areaManager_.RegionalAreaTaskActive() &&
        app.areaManager_.RegionalAreaTask().Type == RegionalAreaTaskType::MyRoadland &&
        !app.areaManager_.RegionalAreaTaskCanYieldToHigherPriority()) {
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

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    if (app.TickRegionalAreaTask(my_team, enemy_team) ||
        app.TickNaviAreaTransition() ||
        app.TickNaviProgressWatchdog(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Task);
        return true;
    }
    return false;
}

bool StrategyManager::RunTactical(Application& app) {
    if (handled_) {
        return true;
    }

    if (app.IsLeagueProfile()) {
        app.SetPositionLeagueSimple();
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.IsShowcasePatrolEnabled()) {
        app.SetPositionShowcasePatrol();
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    const UnitTeam my_team = app.team;
    const UnitTeam enemy_team = app.team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;

    if (app.aimMode == AimMode::Buff) {
        if (app.naviCommandIntervalClock.trigger()) {
            app.TrySetAimModeTaskGoal(my_team, enemy_team, "regional_tactical_buff_mode");
        }
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.TrySetRegionalDefenseGoal(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
    }

    if (app.TrySetProtectHeroGoal(my_team, enemy_team)) {
        MarkHandled(app, StrategyLayer::Tactical);
        return true;
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

bool Application::RunStrategyLayerFinalizer() {
    return strategyManager_.RunFinalizer(*this);
}

}  // namespace BehaviorTree
