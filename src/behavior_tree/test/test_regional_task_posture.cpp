#include <gtest/gtest.h>

#include "../include/PostureTypes.hpp"

namespace {

BehaviorTree::PostureRuntime FreshRuntime(
    const std::uint8_t attack,
    const std::uint8_t defense,
    const std::uint8_t move) {
    BehaviorTree::PostureRuntime runtime;
    runtime.UsingRefereeTimer = true;
    runtime.RefereeRemainingSec = {0, attack, defense, move};
    return runtime;
}

}  // namespace

TEST(RegionalTaskPostureTest, TransitUsesMoveWhileOfficialBudgetIsHealthy) {
    const auto runtime = FreshRuntime(80, 60, 30);

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, 20),
        BehaviorTree::SentryPosture::Move);
}

TEST(RegionalTaskPostureTest, TransitReservesLowMoveBudgetForBetterDefense) {
    const auto runtime = FreshRuntime(50, 60, 10);

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, 20),
        BehaviorTree::SentryPosture::Defense);
}

TEST(RegionalTaskPostureTest, TransitUsesAttackWhenItHasTheLargestUsableBudget) {
    const auto runtime = FreshRuntime(80, 60, 10);

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, 20),
        BehaviorTree::SentryPosture::Attack);
}

TEST(RegionalTaskPostureTest, TransitKeepsMoveWhenOfficialMoveBudgetIsExhausted) {
    const auto runtime = FreshRuntime(80, 60, 0);

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, 20),
        BehaviorTree::SentryPosture::Move);
}

TEST(RegionalTaskPostureTest, TransitFallsBackToMoveWithoutFreshOfficialTimer) {
    auto runtime = FreshRuntime(80, 60, 10);
    runtime.UsingRefereeTimer = false;

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, 20),
        BehaviorTree::SentryPosture::Move);
}
