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

BehaviorTree::PostureFeedback FreshAttackFeedback(
    const bool enhanced,
    const BehaviorTree::PostureManager::TimePoint received_at) {
    return {1U, enhanced, true, true, received_at};
}

}  // namespace

TEST(PostureFeedbackFreshnessTest, RequiresReceiptAndExpiresAtConfiguredDeadline) {
    const auto now = BehaviorTree::PostureManager::TimePoint{};

    EXPECT_FALSE(BehaviorTree::IsPostureFeedbackFresh(false, now, 1000, now));
    EXPECT_TRUE(BehaviorTree::IsPostureFeedbackFresh(true, now, 1000, now + 1000ms));
    EXPECT_FALSE(BehaviorTree::IsPostureFeedbackFresh(true, now, 1000, now + 1001ms));
}

TEST(PostureManagerTest, ExpiredFeedbackImmediatelyMarksReadbackStale) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);

    manager.Tick(now + 1ms, {BehaviorTree::SentryPosture::Attack, false}, FreshAttackFeedback(false, now + 1ms), {});
    manager.Tick(now + 1002ms, {BehaviorTree::SentryPosture::Attack, false}, {1U, false, false, false}, {});

    EXPECT_TRUE(manager.Runtime().FeedbackStale);
}

TEST(PostureManagerTest, ReadbackReceivedBeforePendingCannotConfirmMatchingRequest) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);

    const auto request = manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Defense, false},
        {1U, false, true, false, now + 1ms},
        {});
    EXPECT_EQ(2U, request.Command);
    EXPECT_TRUE(manager.Runtime().HasPending);

    const auto stale_queue = manager.Tick(
        now + 5s + 1ms,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, false, true, false, now + 4999ms},
        {});
    EXPECT_TRUE(manager.Runtime().HasPending);
    EXPECT_STREQ("pending_wait", stale_queue.Reason);

    manager.Tick(
        now + 5s + 2ms,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, false, true, false, now + 5s + 2ms},
        {});
    EXPECT_FALSE(manager.Runtime().HasPending);
}

TEST(PostureManagerTest, ExpiredMatchingReadbackCannotConfirmPendingRequest) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);

    manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Defense, false},
        {1U, false, true, false, now + 1ms},
        {});
    EXPECT_TRUE(manager.Runtime().HasPending);

    const auto expired = manager.Tick(
        now + 5s + 1ms,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, false, false, false, now + 5s + 1ms},
        {});
    EXPECT_TRUE(manager.Runtime().HasPending);
    EXPECT_STREQ("pending_wait", expired.Reason);
}

TEST(PostureManagerTest, EnhancedAttackNeedsEnhancedFeedbackForAck) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);

    EXPECT_EQ(4U, manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Attack, true},
        FreshAttackFeedback(false, now + 5s), {},
        BehaviorTree::PostureRequestPolicy::OutpostLock()).Command);
    EXPECT_TRUE(manager.Runtime().HasPending);

    manager.Tick(
        now + 5s + 1ms,
        {BehaviorTree::SentryPosture::Attack, true},
        FreshAttackFeedback(true, now + 5s + 1ms), {},
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

    const auto first = manager.Tick(now + 5s, enhanced_attack, FreshAttackFeedback(false, now + 5s), {}, policy);
    const auto retry_one = manager.Tick(now + 5600ms, enhanced_attack, FreshAttackFeedback(false, now + 5600ms), {}, policy);
    const auto retry_two = manager.Tick(now + 5900ms, enhanced_attack, FreshAttackFeedback(false, now + 5900ms), {}, policy);
    const auto exhausted = manager.Tick(now + 6200ms, enhanced_attack, FreshAttackFeedback(false, now + 6200ms), {}, policy);

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

    manager.Tick(now + 5s, {BehaviorTree::SentryPosture::Attack, true}, FreshAttackFeedback(false, now + 5s), {}, policy);
    manager.Tick(now + 5200ms, {BehaviorTree::SentryPosture::Attack, true}, FreshAttackFeedback(true, now + 5200ms), {}, policy);
    const auto decision = manager.Tick(
        now + 10199ms,
        {BehaviorTree::SentryPosture::Move, false},
        FreshAttackFeedback(true, now + 10199ms), {},
        BehaviorTree::PostureRequestPolicy{});

    EXPECT_EQ(0U, decision.Command);
    EXPECT_STREQ("cooldown", decision.Reason);
}
