#include "../include/OutpostEngagementLock.hpp"

#include <chrono>

#include <gtest/gtest.h>

namespace {

BehaviorTree::OutpostEngagementInput HealthyOutpostInput() {
    BehaviorTree::OutpostEngagementInput input{};
    input.Target7Fresh = true;
    input.SelectedTarget7 = true;
    input.EnemyHpFresh = true;
    input.EnemyHp = 1000;
    input.SelfHpFresh = true;
    input.SelfHp = 400;
    input.NavigationReachable = true;
    input.Posture.Current = {BehaviorTree::SentryPosture::Attack, false};
    input.Posture.Desired = {BehaviorTree::SentryPosture::Attack, false};
    input.Posture.Pending = {BehaviorTree::SentryPosture::Unknown, false};
    input.PostureCooldownReady = true;
    input.EnhancedAttackRemainingFresh = true;
    input.EnhancedAttackRemainingSec = 15;
    return input;
}

}  // namespace

TEST(OutpostEngagementLockTest, TargetSevenRequestsNormalAttack) {
    BehaviorTree::OutpostEngagementLock lock;
    auto input = HealthyOutpostInput();
    input.Posture.Current = {BehaviorTree::SentryPosture::Move, false};

    const auto out = lock.Tick(BehaviorTree::OutpostEngagementLock::TimePoint{}, input);

    ASSERT_TRUE(out.Active);
    ASSERT_TRUE(out.HoldTarget);
    ASSERT_TRUE(out.Intent.has_value());
    EXPECT_EQ(1U, BehaviorTree::ToPostureCommandValue(*out.Intent));
}

TEST(OutpostEngagementLockTest, FreshHpDropArmsExactlyOneEnhancedAttack) {
    BehaviorTree::OutpostEngagementLock lock;
    const auto now = BehaviorTree::OutpostEngagementLock::TimePoint{};
    auto input = HealthyOutpostInput();

    lock.Tick(now, input);
    input.EnemyHp = 999;
    const auto armed = lock.Tick(now + std::chrono::milliseconds(1), input);

    ASSERT_TRUE(armed.EnhancedArmed);
    ASSERT_TRUE(armed.Intent.has_value());
    EXPECT_EQ(4U, BehaviorTree::ToPostureCommandValue(*armed.Intent));

    input.EnemyHp = 998;
    input.Posture.Current = {BehaviorTree::SentryPosture::Attack, true};
    const auto active = lock.Tick(now + std::chrono::milliseconds(2), input);

    EXPECT_TRUE(active.EnhancedActive);
    EXPECT_FALSE(active.EnhancedArmed);
    EXPECT_FALSE(active.Intent.has_value());
}

TEST(OutpostEngagementLockTest, LockThresholdsAre200And250) {
    const auto now = BehaviorTree::OutpostEngagementLock::TimePoint{};

    BehaviorTree::OutpostEngagementLock normal_lock;
    auto normal = HealthyOutpostInput();
    normal.SelfHp = 201;
    EXPECT_TRUE(normal_lock.Tick(now, normal).Active);
    normal.SelfHp = 200;
    const auto normal_exit = normal_lock.Tick(now + std::chrono::milliseconds(1), normal);
    EXPECT_FALSE(normal_exit.Active);
    EXPECT_EQ(BehaviorTree::OutpostEngagementExitReason::NormalHealthThreshold, normal_exit.ExitReason);

    BehaviorTree::OutpostEngagementLock enhanced_lock;
    auto enhanced = HealthyOutpostInput();
    enhanced_lock.Tick(now, enhanced);
    enhanced.EnemyHp = 999;
    enhanced.Posture.HasPending = true;
    enhanced.Posture.Pending = {BehaviorTree::SentryPosture::Attack, true};
    enhanced.SelfHp = 251;
    EXPECT_TRUE(enhanced_lock.Tick(now + std::chrono::milliseconds(1), enhanced).Active);
    enhanced.SelfHp = 250;
    const auto enhanced_exit = enhanced_lock.Tick(now + std::chrono::milliseconds(2), enhanced);
    EXPECT_FALSE(enhanced_exit.Active);
    EXPECT_EQ(BehaviorTree::OutpostEngagementExitReason::EnhancedHealthThreshold, enhanced_exit.ExitReason);
}

TEST(OutpostEngagementLockTest, StaleZeroAndUnreachableCancelPending) {
    for (const auto reason : {0, 1, 2}) {
        BehaviorTree::OutpostEngagementLock lock;
        const auto now = BehaviorTree::OutpostEngagementLock::TimePoint{};
        auto input = HealthyOutpostInput();
        lock.Tick(now, input);

        if (reason == 0) input.EnemyHpFresh = false;
        if (reason == 1) input.EnemyHp = 0;
        if (reason == 2) input.NavigationReachable = false;
        const auto out = lock.Tick(now + std::chrono::milliseconds(1), input);

        EXPECT_FALSE(out.Active);
        EXPECT_FALSE(out.HoldTarget);
        EXPECT_TRUE(out.CancelPending);
    }
}
