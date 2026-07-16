#include "../include/PostureManager.hpp"

#include <chrono>

#include <gtest/gtest.h>

using namespace std::chrono_literals;

namespace {

LangYa::PostureSetting EnabledPostureSetting() {
    LangYa::PostureSetting setting;
    setting.Enable = true;
    setting.SwitchCooldownSec = 5;
    setting.MinHoldSec = 0;
    setting.PendingAckTimeoutMs = 600;
    setting.RetryIntervalMs = 300;
    setting.MaxRetryCount = 3;
    setting.OptimisticAck = true;
    return setting;
}

BehaviorTree::PostureFeedback FreshAttackFeedback(const bool enhanced) {
    return {1U, enhanced, true, true};
}

}  // namespace

TEST(PostureManagerTest, EnhancedAttackNeedsEnhancedFeedbackForAck) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);

    EXPECT_EQ(4U, manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Attack, true},
        FreshAttackFeedback(false), {},
        BehaviorTree::PostureRequestPolicy::OutpostLock()).Command);
    EXPECT_TRUE(manager.Runtime().HasPending);

    manager.Tick(
        now + 5s + 1ms,
        {BehaviorTree::SentryPosture::Attack, true},
        FreshAttackFeedback(true), {},
        BehaviorTree::PostureRequestPolicy::OutpostLock());

    EXPECT_FALSE(manager.Runtime().HasPending);
    EXPECT_TRUE(manager.Runtime().Current.Enhanced);
}

TEST(PostureManagerTest, FailedOutpostRequestDoesNotIssueDefenseOrMove) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);
    const BehaviorTree::PostureMode enhanced_attack{BehaviorTree::SentryPosture::Attack, true};
    const auto policy = BehaviorTree::PostureRequestPolicy::OutpostLock();

    const auto first = manager.Tick(now + 5s, enhanced_attack, FreshAttackFeedback(false), {}, policy);
    const auto retry_one = manager.Tick(now + 5600ms, enhanced_attack, FreshAttackFeedback(false), {}, policy);
    const auto retry_two = manager.Tick(now + 5900ms, enhanced_attack, FreshAttackFeedback(false), {}, policy);
    const auto exhausted = manager.Tick(now + 6200ms, enhanced_attack, FreshAttackFeedback(false), {}, policy);

    EXPECT_EQ(4U, first.Command);
    EXPECT_EQ(4U, retry_one.Command);
    EXPECT_EQ(4U, retry_two.Command);
    EXPECT_EQ(0U, exhausted.Command);
    EXPECT_EQ(BehaviorTree::SentryPosture::Attack, manager.Runtime().Current.Base);
    EXPECT_FALSE(manager.Runtime().Current.Enhanced);
    EXPECT_FALSE(manager.Runtime().HasPending);
    EXPECT_STREQ("pending_preserved", exhausted.Reason);
}

TEST(PostureManagerTest, CooldownBeginsAtCompositeAck) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);
    const auto policy = BehaviorTree::PostureRequestPolicy::OutpostLock();

    manager.Tick(now + 5s, {BehaviorTree::SentryPosture::Attack, true}, FreshAttackFeedback(false), {}, policy);
    manager.Tick(now + 5200ms, {BehaviorTree::SentryPosture::Attack, true}, FreshAttackFeedback(true), {}, policy);
    const auto decision = manager.Tick(
        now + 10199ms,
        {BehaviorTree::SentryPosture::Move, false},
        FreshAttackFeedback(true), {},
        BehaviorTree::PostureRequestPolicy{});

    EXPECT_EQ(0U, decision.Command);
    EXPECT_STREQ("cooldown", decision.Reason);
}
