#include "../include/RegionalDefenseSearchPolicy.hpp"
#include "../module/Area.hpp"

#include <gtest/gtest.h>

namespace {

TEST(RegionalDefenseSearchPolicy, UsesDedicatedCentralHighAndLowGoalIds) {
    EXPECT_EQ(::LangYa::CentralLeftA.ID, 26U);
    EXPECT_EQ(::LangYa::CentralLeftB.ID, 27U);
    EXPECT_EQ(::LangYa::CentralHigh.ID, 29U);
    EXPECT_EQ(::LangYa::CentralLow.ID, 30U);
    EXPECT_EQ(
        (BehaviorTree::kCommonCentralSearchGoals),
        (std::array<std::uint8_t, 4>{17U, 29U, 30U, 24U}));

    const auto central_high_red = BehaviorTree::Area::CentralHigh(::LangYa::UnitTeam::Red);
    const auto central_high_blue = BehaviorTree::Area::CentralHigh(::LangYa::UnitTeam::Blue);
    const auto central_low_red = BehaviorTree::Area::CentralLow(::LangYa::UnitTeam::Red);
    const auto central_low_blue = BehaviorTree::Area::CentralLow(::LangYa::UnitTeam::Blue);

    EXPECT_EQ(central_high_red.x, 1000U);
    EXPECT_EQ(central_high_red.y, 1007U);
    EXPECT_EQ(central_high_blue.x, 1800U);
    EXPECT_EQ(central_high_blue.y, 493U);
    EXPECT_EQ(central_low_red.x, 989U);
    EXPECT_EQ(central_low_red.y, 496U);
    EXPECT_EQ(central_low_blue.x, 1811U);
    EXPECT_EQ(central_low_blue.y, 1004U);
}

TEST(RegionalDefenseSearchPolicy, KeepsTravelingGoalUntilArrivalOrWatchdog) {
    EXPECT_FALSE(BehaviorTree::ShouldAdvanceCommonCentralSearch({
        .GoalReached = false,
        .GoalUnreachable = false,
        .NoProgressTimeout = false,
        .HoldElapsed = true,
        .VisualTargetRecentlySeen = false}));
    EXPECT_TRUE(BehaviorTree::ShouldAdvanceCommonCentralSearch({
        .GoalReached = false,
        .GoalUnreachable = false,
        .NoProgressTimeout = true,
        .HoldElapsed = false,
        .VisualTargetRecentlySeen = false}));
    EXPECT_TRUE(BehaviorTree::ShouldAdvanceCommonCentralSearch({
        .GoalReached = false,
        .GoalUnreachable = true,
        .NoProgressTimeout = false,
        .HoldElapsed = false,
        .VisualTargetRecentlySeen = true}));
}

TEST(RegionalDefenseSearchPolicy, HoldsAfterArrivalBeforeAdvancing) {
    EXPECT_FALSE(BehaviorTree::ShouldAdvanceCommonCentralSearch({
        .GoalReached = true,
        .GoalUnreachable = false,
        .NoProgressTimeout = false,
        .HoldElapsed = false,
        .VisualTargetRecentlySeen = false}));
    EXPECT_TRUE(BehaviorTree::ShouldAdvanceCommonCentralSearch({
        .GoalReached = true,
        .GoalUnreachable = false,
        .NoProgressTimeout = false,
        .HoldElapsed = true,
        .VisualTargetRecentlySeen = false}));
    EXPECT_FALSE(BehaviorTree::ShouldAdvanceCommonCentralSearch({
        .GoalReached = true,
        .GoalUnreachable = false,
        .NoProgressTimeout = false,
        .HoldElapsed = true,
        .VisualTargetRecentlySeen = true}));
}

}  // namespace
