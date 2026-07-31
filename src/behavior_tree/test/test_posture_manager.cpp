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

TEST(PostureManagerTest, EnhancedDefenseNeedsEnhancedFeedbackForAck) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Defense);

    EXPECT_EQ(5U, manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Defense, true},
        {2U, false, true, true, now + 5s}, {},
        BehaviorTree::PostureRequestPolicy::RequiredPosture()).Command);
    EXPECT_TRUE(manager.Runtime().HasPending);

    manager.Tick(
        now + 5s + 1ms,
        {BehaviorTree::SentryPosture::Defense, true},
        {2U, true, true, true, now + 5s + 1ms}, {},
        BehaviorTree::PostureRequestPolicy::RequiredPosture());

    EXPECT_FALSE(manager.Runtime().HasPending);
    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, manager.Runtime().Current.Base);
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

TEST(PostureManagerTest, RequiredDegradedMoveDoesNotAutoRotate) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Move);

    BehaviorTree::PostureRefereeTimer timer;
    timer.HasInfo3 = true;
    timer.Fresh = true;
    timer.RemainingSec = {0U, 180U, 180U, 0U};
    const auto decision = manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Move, false},
        {3U, false, true, false, now + 5s},
        timer,
        BehaviorTree::PostureRequestPolicy::RequiredPosture());

    EXPECT_EQ(0U, decision.Command);
    EXPECT_EQ(BehaviorTree::SentryPosture::Move, manager.Runtime().Desired.Base);
    EXPECT_STREQ("hold", decision.Reason);
}

TEST(PostureManagerTest, HardMoveSupersedesPendingAttack) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Move);

    const auto attack = manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Attack, false},
        {3U, false, true, false, now + 5s},
        {});
    ASSERT_EQ(1U, attack.Command);
    ASSERT_TRUE(manager.Runtime().HasPending);

    const auto hard_move = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::HardMove,
        BehaviorTree::SentryPosture::Attack,
        manager.Runtime(),
        20);
    const auto move = manager.Tick(
        now + 5s + 700ms,
        hard_move.Mode,
        {3U, false, true, false, now + 5s + 700ms},
        {},
        hard_move.Policy);

    EXPECT_EQ(3U, move.Command);
    EXPECT_STREQ("pending_superseded", move.Reason);
    EXPECT_EQ(BehaviorTree::SentryPosture::Move, manager.Runtime().Pending.Base);
}

TEST(PostureManagerTest, CancelPendingClearsPendingRequestMetadata) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Move);

    const auto attack = manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Attack, false},
        {3U, false, true, false, now + 5s},
        {},
        BehaviorTree::PostureRequestPolicy::RequiredPosture());
    ASSERT_EQ(1U, attack.Command);
    ASSERT_TRUE(manager.Runtime().HasPending);
    ASSERT_EQ(
        BehaviorTree::PostureRequestPriority::Required,
        manager.Runtime().PendingPriority);
    ASSERT_STREQ("required", manager.Runtime().PendingSource);

    manager.CancelPending();

    EXPECT_FALSE(manager.Runtime().HasPending);
    EXPECT_EQ(
        BehaviorTree::PostureRequestPriority::Scored,
        manager.Runtime().PendingPriority);
    EXPECT_STREQ("none", manager.Runtime().PendingSource);
}

TEST(TaskPostureIntentTest, SoftTransitReservesMoveAndDisablesEarlyRotation) {
    BehaviorTree::PostureRuntime runtime;
    runtime.UsingRefereeTimer = true;
    runtime.RefereeRemainingSec = {0U, 100U, 90U, 0U};

    const auto request = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::SoftTransit,
        BehaviorTree::SentryPosture::Attack,
        runtime,
        20);

    EXPECT_EQ(BehaviorTree::SentryPosture::Move, request.Mode.Base);
    EXPECT_FALSE(request.Policy.AllowEarlyRotate);
}

TEST(TaskPostureIntentTest, SoftTransitDoesNotOverrideSafetyDefense) {
    BehaviorTree::PostureRuntime runtime;

    const auto request = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::SoftTransit,
        BehaviorTree::SentryPosture::Defense,
        runtime,
        20);

    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, request.Mode.Base);
    EXPECT_FALSE(request.Policy.AllowEarlyRotate);
}

TEST(TaskPostureIntentTest, SoftArrivedKeepsNormalScoringAndRotationPolicy) {
    BehaviorTree::PostureRuntime runtime;

    const auto request = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::SoftArrived,
        BehaviorTree::SentryPosture::Defense,
        runtime,
        20);

    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, request.Mode.Base);
    EXPECT_TRUE(request.Policy.AllowEarlyRotate);
}

TEST(TaskPostureIntentTest, HardDefenseOverridesScoreAndDisablesEarlyRotation) {
    BehaviorTree::PostureRuntime runtime;

    const auto request = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::HardDefense,
        BehaviorTree::SentryPosture::Move,
        runtime,
        20);

    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, request.Mode.Base);
    EXPECT_FALSE(request.Policy.AllowEarlyRotate);
}

TEST(TaskPostureIntentTest, DamageBurstForcesDefenseExceptDuringRecovery) {
    EXPECT_TRUE(BehaviorTree::ShouldForceDamageBurstDefense(false, true));
    EXPECT_FALSE(BehaviorTree::ShouldForceDamageBurstDefense(true, true));
    EXPECT_FALSE(BehaviorTree::ShouldForceDamageBurstDefense(false, false));
}

TEST(TaskPostureIntentTest, ProtectHeroHoldUsesEnhancedDefenseOnlyWithFreshBudget) {
    BehaviorTree::PostureRuntime runtime;
    runtime.UsingRefereeTimer = true;
    runtime.RefereeEnhancedRemainingSec = {0U, 15U, 15U, 15U};
    BehaviorTree::PostureRefereeTimer referee_timer;
    referee_timer.HasInfo3 = true;
    referee_timer.Fresh = true;
    referee_timer.EnhancedRemainingSec = {0U, 15U, 15U, 15U};

    const auto enhanced = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::ProtectHeroEnhancedDefense,
        BehaviorTree::SentryPosture::Move,
        runtime,
        20,
        referee_timer);

    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, enhanced.Mode.Base);
    EXPECT_TRUE(enhanced.Mode.Enhanced);
    EXPECT_FALSE(enhanced.Policy.AllowEarlyRotate);

    referee_timer.EnhancedRemainingSec[2] = 0U;
    const auto exhausted = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::ProtectHeroEnhancedDefense,
        BehaviorTree::SentryPosture::Move,
        runtime,
        20,
        referee_timer);

    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, exhausted.Mode.Base);
    EXPECT_FALSE(exhausted.Mode.Enhanced);
    EXPECT_TRUE(exhausted.Policy.AllowOptimisticAck);

    referee_timer.EnhancedRemainingSec[2] = 15U;
    referee_timer.Fresh = false;
    const auto stale = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::ProtectHeroEnhancedDefense,
        BehaviorTree::SentryPosture::Move,
        runtime,
        20,
        referee_timer);

    EXPECT_EQ(BehaviorTree::SentryPosture::Defense, stale.Mode.Base);
    EXPECT_FALSE(stale.Mode.Enhanced);
    EXPECT_TRUE(stale.Policy.AllowOptimisticAck);
}

TEST(TaskPostureIntentTest, ProtectHeroHoldUsesNormalDefenseUntilDamageBurst) {
    EXPECT_EQ(
        BehaviorTree::TaskPostureIntent::ProtectHeroDefenseHold,
        BehaviorTree::ResolveProtectHeroHoldIntent(true, false, false));
    EXPECT_EQ(
        BehaviorTree::TaskPostureIntent::ProtectHeroEnhancedDefense,
        BehaviorTree::ResolveProtectHeroHoldIntent(true, true, false));
    EXPECT_EQ(
        BehaviorTree::TaskPostureIntent::ProtectHeroDefenseHold,
        BehaviorTree::ResolveProtectHeroHoldIntent(false, true, false));
    EXPECT_EQ(
        BehaviorTree::TaskPostureIntent::ProtectHeroDefenseHold,
        BehaviorTree::ResolveProtectHeroHoldIntent(true, true, true));
}

TEST(TaskPostureIntentTest, OnlyConfirmedProtectHeroEnhancedDefenseDefersRecovery) {
    BehaviorTree::PostureRuntime runtime;
    runtime.Current = {BehaviorTree::SentryPosture::Defense, true};

    EXPECT_TRUE(BehaviorTree::ShouldDeferRecoveryForProtectHeroEnhancedDefense(true, false, runtime));
    EXPECT_FALSE(BehaviorTree::ShouldDeferRecoveryForProtectHeroEnhancedDefense(true, true, runtime));
    EXPECT_FALSE(BehaviorTree::ShouldDeferRecoveryForProtectHeroEnhancedDefense(false, false, runtime));

    runtime.FeedbackStale = true;
    EXPECT_FALSE(BehaviorTree::ShouldDeferRecoveryForProtectHeroEnhancedDefense(true, false, runtime));

    runtime.FeedbackStale = false;
    runtime.Current = {BehaviorTree::SentryPosture::Defense, false};
    EXPECT_FALSE(BehaviorTree::ShouldDeferRecoveryForProtectHeroEnhancedDefense(true, false, runtime));
    EXPECT_FALSE(BehaviorTree::ShouldDeferRecoveryForProtectHeroEnhancedDefense(true, false, runtime));
}

TEST(TaskPostureIntentTest, EnhancedMoveNeedsFreshPositiveBudget) {
    BehaviorTree::PostureRuntime runtime;
    BehaviorTree::PostureRefereeTimer referee_timer;
    referee_timer.HasInfo3 = true;
    referee_timer.Fresh = true;
    referee_timer.EnhancedRemainingSec = {0U, 15U, 15U, 12U};

    const auto available = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::RecoveryEnhancedMove,
        BehaviorTree::SentryPosture::Defense,
        runtime,
        20,
        referee_timer);
    EXPECT_EQ(BehaviorTree::SentryPosture::Move, available.Mode.Base);
    EXPECT_TRUE(available.Mode.Enhanced);

    referee_timer.EnhancedRemainingSec[3] = 0U;
    const auto exhausted = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::RecoveryEnhancedMove,
        BehaviorTree::SentryPosture::Defense,
        runtime,
        20,
        referee_timer);
    EXPECT_EQ(BehaviorTree::SentryPosture::Move, exhausted.Mode.Base);
    EXPECT_FALSE(exhausted.Mode.Enhanced);

    referee_timer.EnhancedRemainingSec[3] = 12U;
    referee_timer.Fresh = false;
    const auto stale = BehaviorTree::ResolveTaskPostureRequest(
        BehaviorTree::TaskPostureIntent::RecoveryEnhancedMove,
        BehaviorTree::SentryPosture::Defense,
        runtime,
        20,
        referee_timer);
    EXPECT_FALSE(stale.Mode.Enhanced);
}

TEST(TaskPostureIntentTest, RecoveryEnhancedMoveSkipsNormalRespawnAndArrivedState) {
    BehaviorTree::PostureRuntime runtime;
    BehaviorTree::PostureRefereeTimer referee_timer;
    referee_timer.HasInfo3 = true;
    referee_timer.Fresh = true;
    referee_timer.EnhancedRemainingSec = {0U, 10U, 10U, 10U};

    EXPECT_TRUE(BehaviorTree::ShouldRequestRecoveryEnhancedMove(
        true, true, false, false, true, true, 40U, 80, runtime, referee_timer));
    EXPECT_FALSE(BehaviorTree::ShouldRequestRecoveryEnhancedMove(
        true, true, true, false, true, true, 40U, 80, runtime, referee_timer));
    EXPECT_FALSE(BehaviorTree::ShouldRequestRecoveryEnhancedMove(
        true, true, false, false, false, true, 40U, 80, runtime, referee_timer));
    EXPECT_FALSE(BehaviorTree::ShouldRequestRecoveryEnhancedMove(
        true, true, false, true, true, true, 40U, 80, runtime, referee_timer));
    EXPECT_FALSE(BehaviorTree::ShouldRequestRecoveryEnhancedMove(
        true, true, false, false, true, true, 0U, 80, runtime, referee_timer));
    EXPECT_FALSE(BehaviorTree::ShouldRequestRecoveryEnhancedMove(
        true, true, false, false, true, false, 40U, 80, runtime, referee_timer));
}

TEST(TaskPostureIntentTest, NormalRespawnIsOnlyZeroToPositiveHealthTransition) {
    EXPECT_TRUE(BehaviorTree::IsNormalRespawnHealthTransition(true, 0U, 40U));
    EXPECT_FALSE(BehaviorTree::IsNormalRespawnHealthTransition(false, 0U, 40U));
    EXPECT_FALSE(BehaviorTree::IsNormalRespawnHealthTransition(true, 40U, 80U));
    EXPECT_FALSE(BehaviorTree::IsNormalRespawnHealthTransition(true, 0U, 0U));
}

TEST(PostureManagerTest, EnhancedFeedbackContradictionNeedsGraceBeforeQuarantine) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Defense);

    BehaviorTree::PostureRefereeTimer timer;
    timer.HasInfo3 = true;
    timer.Fresh = true;
    timer.Enhanced = true;
    timer.EnhancedContradictionGraceMs = 500;
    timer.EnhancedRemainingSec = {0U, 12U, 0U, 12U};

    manager.Tick(
        now + 1ms,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, true, true, true, now + 1ms},
        timer);
    EXPECT_FALSE(manager.Runtime().EnhancedFeedbackQuarantined);

    manager.Tick(
        now + 500ms,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, true, true, true, now + 500ms},
        timer);
    EXPECT_FALSE(manager.Runtime().EnhancedFeedbackQuarantined);

    manager.Tick(
        now + 501ms,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, true, true, true, now + 501ms},
        timer);
    EXPECT_TRUE(manager.Runtime().EnhancedFeedbackQuarantined);

    const auto convergence = manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Defense, false},
        {2U, true, true, true, now + 5s},
        timer);
    EXPECT_EQ(2U, convergence.Command);
}
