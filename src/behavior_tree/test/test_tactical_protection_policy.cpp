#include <gtest/gtest.h>

#include <chrono>

#include "TacticalProtectionPolicy.hpp"

namespace {

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

TEST(TacticalProtectionPolicy, CastleOccupancyHoldsOnlyForDirectSelfPresence) {
    const auto direct_presence = BehaviorTree::ResolveCastleOccupancy({
        .RefereeFresh = true,
        .RefereeStatus = 3,
        .SelfRfidAtCastle = true,
    });
    EXPECT_EQ(direct_presence.Action, BehaviorTree::CastleOccupancyAction::HoldCastle);
    EXPECT_TRUE(direct_presence.SelfLikelyAtCastle);

    const auto grace_only = BehaviorTree::ResolveCastleOccupancy({
        .RefereeFresh = true,
        .RefereeStatus = 3,
        .ReachedCastleGrace = true,
    });
    EXPECT_EQ(grace_only.Action, BehaviorTree::CastleOccupancyAction::PerimeterDefense);
    EXPECT_FALSE(grace_only.SelfLikelyAtCastle);
    EXPECT_TRUE(grace_only.TeamOrAmbiguousOccupant);
}

}  // namespace
