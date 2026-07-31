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

TEST(RegionalTaskPostureTest, DynamicReserveUsesMoveForShortRemainingTravel) {
    const auto runtime = FreshRuntime(180, 180, 60);
    BehaviorTree::TransitPostureContext context;
    context.Enabled = true;
    context.HasFreshDistance = true;
    context.DistanceCm = 1000.0;
    context.NominalSpeedMps = 1.0;
    context.SafetyFactor = 1.5;
    context.ArrivalBufferSec = 5;
    context.MinReserveSec = 30;
    context.MaxReserveSec = 120;
    context.FallbackReserveSec = 45;

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, context),
        BehaviorTree::SentryPosture::Move);
}

TEST(RegionalTaskPostureTest, DynamicReserveLeavesMoveForLongRemainingTravel) {
    const auto runtime = FreshRuntime(180, 180, 60);
    BehaviorTree::TransitPostureContext context;
    context.Enabled = true;
    context.HasFreshDistance = true;
    context.DistanceCm = 6000.0;
    context.NominalSpeedMps = 1.0;
    context.SafetyFactor = 1.5;
    context.ArrivalBufferSec = 5;
    context.MinReserveSec = 30;
    context.MaxReserveSec = 120;
    context.FallbackReserveSec = 45;

    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, context),
        BehaviorTree::SentryPosture::Defense);
}

TEST(RegionalTaskPostureTest, FreshNavigationVelocityReducesDynamicReserve) {
    const auto runtime = FreshRuntime(180, 180, 60);
    BehaviorTree::TransitPostureContext context;
    context.Enabled = true;
    context.HasFreshDistance = true;
    context.DistanceCm = 6000.0;
    context.HasFreshVelocity = true;
    context.VelocityMps = 2.0;
    context.NominalSpeedMps = 0.8;
    context.SafetyFactor = 1.5;
    context.ArrivalBufferSec = 5;
    context.MinReserveSec = 30;
    context.MaxReserveSec = 120;
    context.FallbackReserveSec = 45;

    EXPECT_EQ(50, BehaviorTree::ComputeTransitMoveReserveSec(context));
    EXPECT_EQ(
        BehaviorTree::SelectTransitPosture(runtime, context),
        BehaviorTree::SentryPosture::Move);
}

TEST(RegionalTaskPostureTest, TargetTransitCanUseAttackInsteadOfMove) {
    BehaviorTree::PostureRuntime runtime;
    runtime.UsingRefereeTimer = true;
    runtime.RefereeRemainingSec = {0U, 180U, 180U, 60U};
    BehaviorTree::TransitPostureContext context;
    context.Enabled = true;
    context.AllowAttackDuringTransit = true;

    const auto request = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::SoftTransit,
        BehaviorTree::SentryPosture::Attack,
        runtime,
        context);

    EXPECT_EQ(BehaviorTree::SentryPosture::Attack, request.Mode.Base);
}
