#include "../include/NearGoalArrivalConfirm.hpp"

#include <chrono>

#include <gtest/gtest.h>

namespace {

using BehaviorTree::NearGoalArrivalConfirm;
using Point = BehaviorTree::Area::Point<std::uint16_t>;
using Clock = std::chrono::steady_clock;

TEST(NearGoalArrivalConfirm, HoldsInsideRadiusUntilWaitExpires) {
    NearGoalArrivalConfirm confirm;
    const auto start = Clock::time_point{} + std::chrono::seconds(1);
    const Point goal{100, 200};

    EXPECT_FALSE(confirm.Observe(7, goal, true, start, 1500));
    EXPECT_FALSE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(1499), 1500));
    EXPECT_TRUE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(1500), 1500));
}

TEST(NearGoalArrivalConfirm, LeavingRadiusRestartsTheConfirmationTimer) {
    NearGoalArrivalConfirm confirm;
    const auto start = Clock::time_point{} + std::chrono::seconds(1);
    const Point goal{100, 200};

    EXPECT_FALSE(confirm.Observe(7, goal, true, start, 1500));
    EXPECT_FALSE(confirm.Observe(7, goal, false, start + std::chrono::milliseconds(900), 1500));
    EXPECT_FALSE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(1000), 1500));
    EXPECT_FALSE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(2499), 1500));
    EXPECT_TRUE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(2500), 1500));
}

TEST(NearGoalArrivalConfirm, ChangingGoalRestartsTheConfirmationTimer) {
    NearGoalArrivalConfirm confirm;
    const auto start = Clock::time_point{} + std::chrono::seconds(1);
    const Point first_goal{100, 200};
    const Point second_goal{300, 400};

    EXPECT_FALSE(confirm.Observe(7, first_goal, true, start, 1500));
    EXPECT_FALSE(confirm.Observe(8, second_goal, true, start + std::chrono::milliseconds(1400), 1500));
    EXPECT_FALSE(confirm.Observe(8, second_goal, true, start + std::chrono::milliseconds(2899), 1500));
    EXPECT_TRUE(confirm.Observe(8, second_goal, true, start + std::chrono::milliseconds(2900), 1500));
}

TEST(NearGoalArrivalConfirm, ZeroWaitKeepsImmediateDistanceFallback) {
    NearGoalArrivalConfirm confirm;
    const auto now = Clock::time_point{} + std::chrono::seconds(1);
    const Point goal{100, 200};

    EXPECT_TRUE(confirm.Observe(7, goal, true, now, 0));
    EXPECT_FALSE(confirm.Observe(7, goal, false, now, 0));
}

}  // namespace
