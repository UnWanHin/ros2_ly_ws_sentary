// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "../module/BasicTypes.hpp"
#include "AreaManager.hpp"

namespace BehaviorTree {

struct EventSnapshot {
    AreaTimePoint EvaluatedAt{};

    bool EventDataFresh{false};
    bool SentryInfoFresh{false};
    bool SelfHealthFresh{false};
    bool AmmoFresh{false};
    bool EnemyOutpostHealthFresh{false};

    bool BuffTaskEnabled{false};
    bool BuffCanActivate{false};
    bool BuffActivating{false};
    bool BuffActivated{false};
    std::uint8_t SelfSmallEnergyStatus{0};
    std::uint8_t SelfLargeEnergyStatus{0};

    bool OutpostTaskEnabled{false};
    bool EnemyOutpostAlive{false};
    bool OutpostAttackWindowOpen{false};

    std::uint8_t SelfFortressGainPointStatus{0};
    std::uint8_t SelfOutpostGainPointStatus{0};
    bool SelfBaseGainPointStatus{false};

    bool SelfLowHp{false};
    bool SelfLowAmmo{false};
    bool RecentDamageOver30{false};

    bool ArmorTargetVisible{false};
    bool BuffTargetLocked{false};
    bool OutpostTargetLocked{false};

    bool NaviReachFresh{false};
    bool NaviReachableFresh{false};
    bool GoalReached{false};
    bool GoalUnreachable{false};

    bool RegionalDefenseActive{false};
    RegionalDefenseThreat RegionalDefense{};
};

struct EventEvaluateInput {
    AreaTimePoint Now{};

    int RefereeFreshTimeoutMs{2000};
    bool HasEventData{false};
    AreaTimePoint LastEventDataRxTime{};
    std::uint8_t SelfSmallEnergyStatus{0};
    std::uint8_t SelfLargeEnergyStatus{0};
    std::uint8_t SelfFortressGainPointStatus{0};
    std::uint8_t SelfOutpostGainPointStatus{0};
    bool SelfBaseGainPointStatus{false};

    bool HasSentryInfo{false};
    AreaTimePoint LastSentryInfoRxTime{};
    bool SentryCanActivateEnergyMechanism{false};

    bool BuffTaskEnabled{false};
    bool OutpostTaskEnabled{false};
    int OutpostMaxGameTimeSec{90};
    int ElapsedGameSec{0};

    bool HasEnemyOutpostHealth{false};
    AreaTimePoint LastEnemyOutpostHealthRxTime{};
    std::uint16_t EnemyOutpostHealth{0};

    bool HasSelfHealth{false};
    AreaTimePoint LastSelfHealthRxTime{};
    std::uint16_t SelfHealth{0};
    std::uint16_t LowHpThreshold{0};

    bool HasAmmo{false};
    AreaTimePoint LastAmmoRxTime{};
    std::uint16_t Ammo{0};
    std::uint16_t LowAmmoThreshold{0};

    bool RecentDamageOver30{false};
    bool ArmorTargetVisible{false};
    bool BuffTargetLocked{false};
    bool OutpostTargetLocked{false};

    int NaviStatusFreshTimeoutMs{2000};
    bool HasNaviReach{false};
    bool NaviReach{false};
    AreaTimePoint LastNaviReachRxTime{};
    bool HasNaviReachable{false};
    bool NaviReachable{true};
    AreaTimePoint LastNaviReachableRxTime{};

    std::optional<RegionalDefenseThreat> RegionalDefense{};
};

class EventManager {
public:
    EventSnapshot Evaluate(const EventEvaluateInput& input) const noexcept;

private:
    static bool Fresh(
        bool received,
        AreaTimePoint last_rx,
        AreaTimePoint now,
        int timeout_ms) noexcept;
};

}  // namespace BehaviorTree
