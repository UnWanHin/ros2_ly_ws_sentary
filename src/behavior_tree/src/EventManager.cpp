// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/EventManager.hpp"

#include <algorithm>
#include <chrono>

namespace BehaviorTree {

bool EventManager::Fresh(
    const bool received,
    const AreaTimePoint last_rx,
    const AreaTimePoint now,
    const int timeout_ms) noexcept {
    if (!received || last_rx.time_since_epoch().count() == 0) {
        return false;
    }
    return now - last_rx <= std::chrono::milliseconds(std::max(0, timeout_ms));
}

EventSnapshot EventManager::Evaluate(const EventEvaluateInput& input) const noexcept {
    EventSnapshot snapshot;
    snapshot.EvaluatedAt = input.Now;

    snapshot.EventDataFresh = Fresh(
        input.HasEventData,
        input.LastEventDataRxTime,
        input.Now,
        input.RefereeFreshTimeoutMs);
    snapshot.SentryInfoFresh = Fresh(
        input.HasSentryInfo,
        input.LastSentryInfoRxTime,
        input.Now,
        input.RefereeFreshTimeoutMs);
    snapshot.SelfHealthFresh = Fresh(
        input.HasSelfHealth,
        input.LastSelfHealthRxTime,
        input.Now,
        input.RefereeFreshTimeoutMs);
    snapshot.AmmoFresh = Fresh(
        input.HasAmmo,
        input.LastAmmoRxTime,
        input.Now,
        input.RefereeFreshTimeoutMs);
    snapshot.EnemyOutpostHealthFresh = Fresh(
        input.HasEnemyOutpostHealth,
        input.LastEnemyOutpostHealthRxTime,
        input.Now,
        input.RefereeFreshTimeoutMs);

    snapshot.BuffTaskEnabled = input.BuffTaskEnabled;
    snapshot.SelfSmallEnergyStatus = input.SelfSmallEnergyStatus;
    snapshot.SelfLargeEnergyStatus = input.SelfLargeEnergyStatus;
    snapshot.BuffCanActivate =
        snapshot.SentryInfoFresh && input.SentryCanActivateEnergyMechanism;
    snapshot.BuffActivating =
        snapshot.EventDataFresh &&
        (input.SelfSmallEnergyStatus == 2 || input.SelfLargeEnergyStatus == 2);
    snapshot.BuffActivated =
        snapshot.EventDataFresh &&
        (input.SelfSmallEnergyStatus == 1 || input.SelfLargeEnergyStatus == 1);

    snapshot.OutpostTaskEnabled = input.OutpostTaskEnabled;
    snapshot.EnemyOutpostAlive =
        snapshot.EnemyOutpostHealthFresh && input.EnemyOutpostHealth > 0U;
    snapshot.OutpostAttackWindowOpen =
        input.OutpostMaxGameTimeSec <= 0 ||
        input.ElapsedGameSec < input.OutpostMaxGameTimeSec;

    snapshot.SelfFortressGainPointStatus = input.SelfFortressGainPointStatus;
    snapshot.SelfOutpostGainPointStatus = input.SelfOutpostGainPointStatus;
    snapshot.SelfBaseGainPointStatus = input.SelfBaseGainPointStatus;

    snapshot.SelfLowHp =
        snapshot.SelfHealthFresh &&
        input.LowHpThreshold > 0U &&
        input.SelfHealth < input.LowHpThreshold;
    snapshot.SelfLowAmmo =
        snapshot.AmmoFresh &&
        input.LowAmmoThreshold > 0U &&
        input.Ammo <= input.LowAmmoThreshold;
    snapshot.RecentDamageOver30 = input.RecentDamageOver30;

    snapshot.ArmorTargetVisible = input.ArmorTargetVisible;
    snapshot.BuffTargetLocked = input.BuffTargetLocked;
    snapshot.OutpostTargetLocked = input.OutpostTargetLocked;

    snapshot.NaviReachFresh = Fresh(
        input.HasNaviReach,
        input.LastNaviReachRxTime,
        input.Now,
        input.NaviStatusFreshTimeoutMs);
    snapshot.NaviReachableFresh = Fresh(
        input.HasNaviReachable,
        input.LastNaviReachableRxTime,
        input.Now,
        input.NaviStatusFreshTimeoutMs);
    snapshot.GoalReached = input.CompositeGoalReached;
    snapshot.GoalUnreachable = input.CompositeGoalUnreachable;

    if (input.RegionalDefense.has_value()) {
        snapshot.RegionalDefenseActive = true;
        snapshot.RegionalDefense = *input.RegionalDefense;
    }
    return snapshot;
}

}  // namespace BehaviorTree
