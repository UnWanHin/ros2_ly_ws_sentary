#include <gtest/gtest.h>

#include <chrono>

#include "TacticalProtectionPolicy.hpp"

namespace {

using namespace std::chrono_literals;

BehaviorTree::ProtectOutpostState ActiveProtectOutpostTravelState() {
    BehaviorTree::ProtectOutpostState state;
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1500, true, start));
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1480, true, start + 1s, 2s, 20));
    return state;
}

TEST(TacticalProtectionPolicy, ProtectOutpostStartsOnlyAfterThresholdDamageWithinWindow) {
    BehaviorTree::ProtectOutpostState state;
    const auto now = std::chrono::steady_clock::time_point{} + 1s;

    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1500, true, now));
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1490, true, now + 1s, 2s, 20));
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1481, false, now + 2s, 2s, 20));
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1481, true, now + 2s, 2s, 20));
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1480, true, now + 2s, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Travel);
    const auto generation = state.ActiveEventGeneration;
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1479, true, now + 3s, 2s, 20));
    EXPECT_EQ(state.ActiveEventGeneration, generation);
}

TEST(TacticalProtectionPolicy, ProtectOutpostExpiresDamageWindowBeforeThreshold) {
    BehaviorTree::ProtectOutpostState state;
    const auto now = std::chrono::steady_clock::time_point{} + 1s;

    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1500, true, now));
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1485, true, now + 3s, 2s, 20));
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1470, true, now + 4s, 2s, 20));
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1465, true, now + 5s, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Travel);
}

TEST(TacticalProtectionPolicy, ProtectOutpostZeroHealthCancelsAndRebuildCanRetrigger) {
    auto state = ActiveProtectOutpostTravelState();
    const auto now = std::chrono::steady_clock::time_point{} + 10s;

    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 0, true, now, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Idle);
    EXPECT_FALSE(state.HasSample);
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1500, true, now + 1s, 2s, 20));
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1480, true, now + 2s, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Travel);
}

TEST(TacticalProtectionPolicy, ProtectOutpostHoldsAfterArrivalAndConsumesEvent) {
    auto state = ActiveProtectOutpostTravelState();
    const auto now = std::chrono::steady_clock::time_point{} + 10s;

    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, true, false, now, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::SearchHold);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, true, false, now + 29s, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::SearchHold);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, true, false, now + 30s, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::Complete);
}

TEST(TacticalProtectionPolicy, ProtectOutpostCompleteNeedsAnotherStrictDecreaseToRestart) {
    auto state = ActiveProtectOutpostTravelState();
    const auto now = std::chrono::steady_clock::time_point{} + 10s;
    BehaviorTree::TickProtectOutpost(state, true, true, false, now, 30s);
    BehaviorTree::TickProtectOutpost(state, true, true, false, now + 30s, 30s);

    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Complete);
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1500, true, now + 31s, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Complete);
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1480, true, now + 32s, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Travel);
}

TEST(TacticalProtectionPolicy, ProtectOutpostFreshDamageWhileHoldingRestartsSearchHold) {
    auto state = ActiveProtectOutpostTravelState();
    const auto now = std::chrono::steady_clock::time_point{} + 10s;
    BehaviorTree::TickProtectOutpost(state, true, true, false, now, 30s);

    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1480, true, now + 19s, 2s, 20));
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1460, true, now + 20s, 2s, 20));
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::SearchHold);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, true, false, now + 49s, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::SearchHold);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, true, false, now + 50s, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::Complete);
}

TEST(TacticalProtectionPolicy, ProtectOutpostStaleHealthPreservesSampleAndActiveEvent) {
    auto state = ActiveProtectOutpostTravelState();
    const auto sampled_health = state.LastHealth;
    const auto active_generation = state.ActiveEventGeneration;
    const auto now = std::chrono::steady_clock::time_point{} + 10s;

    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1400, false, now));
    EXPECT_EQ(state.LastHealth, sampled_health);
    EXPECT_EQ(state.ActiveEventGeneration, active_generation);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, false, false, false, now, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::Travel);
}

TEST(TacticalProtectionPolicy, ProtectOutpostUnreachableCoolsDownThenCompletes) {
    auto state = ActiveProtectOutpostTravelState();
    const auto now = std::chrono::steady_clock::time_point{} + 10s;

    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, false, true, now, 30s, 5s).Phase,
              BehaviorTree::ProtectOutpostPhase::Cooldown);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, false, false, now + 4s, 30s, 5s).Phase,
              BehaviorTree::ProtectOutpostPhase::Cooldown);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, false, false, now + 5s, 30s, 5s).Phase,
              BehaviorTree::ProtectOutpostPhase::Complete);
}

TEST(TacticalProtectionPolicy, ProtectOutpostDamageDuringCooldownQueuesTravelUntilExpiry) {
    auto state = ActiveProtectOutpostTravelState();
    const auto now = std::chrono::steady_clock::time_point{} + 10s;

    BehaviorTree::TickProtectOutpost(state, true, false, true, now, 30s, 5s);
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Cooldown);
    EXPECT_FALSE(BehaviorTree::ObserveProtectOutpostHealth(state, 1480, true, now, 2s, 20));
    EXPECT_TRUE(BehaviorTree::ObserveProtectOutpostHealth(state, 1460, true, now + 1s, 2s, 20));
    EXPECT_EQ(state.LastHealth, 1460);
    EXPECT_EQ(state.Phase, BehaviorTree::ProtectOutpostPhase::Cooldown);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, false, false, now + 4s, 30s, 5s).Phase,
              BehaviorTree::ProtectOutpostPhase::Cooldown);
    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, false, false, now + 5s, 30s, 5s).Phase,
              BehaviorTree::ProtectOutpostPhase::Travel);
}

TEST(TacticalProtectionPolicy, ProtectOutpostPreemptedTickRetainsActiveEvent) {
    auto state = ActiveProtectOutpostTravelState();
    const auto generation = state.ActiveEventGeneration;
    const auto now = std::chrono::steady_clock::time_point{} + 10s;

    EXPECT_EQ(BehaviorTree::TickProtectOutpost(state, true, false, false, now, 30s).Phase,
              BehaviorTree::ProtectOutpostPhase::Travel);
    EXPECT_EQ(state.ActiveEventGeneration, generation);
}

TEST(TacticalProtectionPolicy, PreservesProfileBaselineWhenYamlDoesNotSetTheSwitch) {
    EXPECT_FALSE(BehaviorTree::ResolveTacticalFeatureEnable(false, false, true));
    EXPECT_TRUE(BehaviorTree::ResolveTacticalFeatureEnable(true, false, false));
}

TEST(TacticalProtectionPolicy, ExplicitYamlValueOverridesTheProfileBaseline) {
    EXPECT_TRUE(BehaviorTree::ResolveTacticalFeatureEnable(false, true, true));
    EXPECT_FALSE(BehaviorTree::ResolveTacticalFeatureEnable(true, true, false));
}

TEST(TacticalProtectionPolicy, ProtectCastleRfidRequiresMasterSourceAndEvent) {
    EXPECT_TRUE(BehaviorTree::IsProtectCastleRfidEventEnabled(true, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidEventEnabled(false, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidEventEnabled(true, false, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidEventEnabled(true, true, false));
}

TEST(TacticalProtectionPolicy, ProtectCastleRfidStayRequiresFreshRfidEventAndEveryGate) {
    EXPECT_TRUE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(false, true, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, false, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, false, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, true, false));
}

TEST(TacticalProtectionPolicy, ProtectCastleEnemyPositionRequiresMasterAndSource) {
    EXPECT_TRUE(BehaviorTree::IsProtectCastleEnemyPositionEnabled(true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleEnemyPositionEnabled(false, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleEnemyPositionEnabled(true, false));
}

TEST(TacticalProtectionPolicy, BaseDamageRequiresMasterAndBaseGate) {
    using namespace std::chrono_literals;
    BehaviorTree::BaseDamageWindowState state;
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    EXPECT_FALSE(BehaviorTree::ObserveBaseHealthForProtection(state, 5000, start));
    EXPECT_TRUE(BehaviorTree::ObserveBaseHealthForProtection(state, 4900, start + 1s));
    EXPECT_TRUE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 2s));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        false, true, true, state, start + 2s));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, false, true, state, start + 2s));
}

TEST(TacticalProtectionPolicy, BaseDamageFirstSampleDoesNotActivateAndDecreaseRenewsEightSeconds) {
    using namespace std::chrono_literals;
    BehaviorTree::BaseDamageWindowState state;
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    EXPECT_FALSE(BehaviorTree::ObserveBaseHealthForProtection(state, 5000, start));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 7s));

    EXPECT_TRUE(BehaviorTree::ObserveBaseHealthForProtection(state, 4950, start + 1s));
    EXPECT_TRUE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 8s));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 9s));

    EXPECT_TRUE(BehaviorTree::ObserveBaseHealthForProtection(state, 4900, start + 10s));
    EXPECT_TRUE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 17s));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 18s));
}

TEST(TacticalProtectionPolicy, BaseDamageIgnoresEqualIncreaseZeroAndStaleSamples) {
    using namespace std::chrono_literals;
    BehaviorTree::BaseDamageWindowState state;
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    EXPECT_FALSE(BehaviorTree::ObserveBaseHealthForProtection(state, 5000, start));
    EXPECT_TRUE(BehaviorTree::ObserveBaseHealthForProtection(state, 4900, start + 1s));
    EXPECT_FALSE(BehaviorTree::ObserveBaseHealthForProtection(state, 4900, start + 2s));
    EXPECT_FALSE(BehaviorTree::ObserveBaseHealthForProtection(state, 4950, start + 3s));
    EXPECT_FALSE(BehaviorTree::ObserveBaseHealthForProtection(state, 0, start + 4s));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, true, state, start + 5s));

    EXPECT_FALSE(BehaviorTree::IsProtectCastleBaseDamageActive(
        true, true, false, state, start + 5s));
}

TEST(TacticalProtectionPolicy, CastleOccupancyApproachesWhenRefereeSaysNoneOrEnemy) {
    for (const auto status : {0U, 2U}) {
        const auto result = BehaviorTree::ResolveCastleOccupancy({
            .RefereeFresh = true,
            .RefereeStatus = static_cast<std::uint8_t>(status),
        });
        EXPECT_EQ(result.Action, BehaviorTree::CastleOccupancyAction::ApproachCastle);
    }
}

TEST(TacticalProtectionPolicy, CastleOccupancyUsesPerimeterWhenFriendlyStatusAndSelfAbsent) {
    const auto result = BehaviorTree::ResolveCastleOccupancy({
        .RefereeFresh = true,
        .RefereeStatus = 1,
        .TeammatePositionAtCastle = true,
    });
    EXPECT_EQ(result.Action, BehaviorTree::CastleOccupancyAction::PerimeterDefense);
    EXPECT_TRUE(result.TeammateLikelyAtCastle);
    EXPECT_TRUE(result.TeamOrAmbiguousOccupant);
}

TEST(TacticalProtectionPolicy, CastleOccupancyHoldsOnlyForConfirmedSelfCapture) {
    const auto confirmed_capture = BehaviorTree::ResolveCastleOccupancy({
        .RefereeFresh = true,
        .RefereeStatus = 3,
        .SelfRfidAtCastle = true,
        .SelfCaptureConfirmed = true,
    });
    EXPECT_EQ(confirmed_capture.Action, BehaviorTree::CastleOccupancyAction::HoldCastle);
    EXPECT_TRUE(confirmed_capture.SelfLikelyAtCastle);

    const auto direct_presence_without_transition = BehaviorTree::ResolveCastleOccupancy({
        .RefereeFresh = true,
        .RefereeStatus = 3,
        .SelfRfidAtCastle = true,
    });
    EXPECT_EQ(direct_presence_without_transition.Action, BehaviorTree::CastleOccupancyAction::PerimeterDefense);
    EXPECT_FALSE(direct_presence_without_transition.SelfLikelyAtCastle);

    const auto grace_only = BehaviorTree::ResolveCastleOccupancy({
        .RefereeFresh = true,
        .RefereeStatus = 3,
        .ReachedCastleGrace = true,
    });
    EXPECT_EQ(grace_only.Action, BehaviorTree::CastleOccupancyAction::PerimeterDefense);
    EXPECT_FALSE(grace_only.SelfLikelyAtCastle);
    EXPECT_TRUE(grace_only.TeamOrAmbiguousOccupant);
}

TEST(TacticalProtectionPolicy, CastleOccupancyAttributesCaptureToSelfOnlyAcrossRfidBackedTransition) {
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    BehaviorTree::CastleCaptureAttributionState state;

    BehaviorTree::ObserveCastleCaptureAttribution(state, 2U, true, start, 3s);
    EXPECT_FALSE(state.SelfCaptureConfirmed);

    BehaviorTree::ObserveCastleCaptureAttribution(state, 1U, true, start + 2s, 3s);
    EXPECT_TRUE(state.SelfCaptureConfirmed);

    BehaviorTree::ObserveCastleCaptureAttribution(state, 2U, true, start + 3s, 3s);
    EXPECT_FALSE(state.SelfCaptureConfirmed);
}

TEST(TacticalProtectionPolicy, CastleOccupancyDoesNotAttributeTeammateCaptureAfterLateRfidArrival) {
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    BehaviorTree::CastleCaptureAttributionState state;

    BehaviorTree::ObserveCastleCaptureAttribution(state, 2U, false, start, 3s);
    BehaviorTree::ObserveCastleCaptureAttribution(state, 1U, false, start + 1s, 3s);
    EXPECT_FALSE(state.SelfCaptureConfirmed);

    // The referee already reports team occupation when this sentry enters, so
    // this is not evidence that this sentry performed the capture.
    BehaviorTree::ObserveCastleCaptureAttribution(state, 1U, true, start + 2s, 3s);
    EXPECT_FALSE(state.SelfCaptureConfirmed);
}

TEST(TacticalProtectionPolicy, CastleOccupancyDoesNotAttributeCaptureAfterTransitionWindowExpires) {
    const auto start = std::chrono::steady_clock::time_point{} + 1s;
    BehaviorTree::CastleCaptureAttributionState state;

    BehaviorTree::ObserveCastleCaptureAttribution(state, 0U, true, start, 3s);
    BehaviorTree::ObserveCastleCaptureAttribution(state, 3U, true, start + 3001ms, 3s);

    EXPECT_FALSE(state.SelfCaptureConfirmed);
}

}  // namespace
