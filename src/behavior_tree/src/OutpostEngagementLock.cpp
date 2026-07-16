#include "../include/OutpostEngagementLock.hpp"

namespace BehaviorTree {

namespace {

constexpr PostureMode kNormalAttack{SentryPosture::Attack, false};
constexpr PostureMode kEnhancedAttack{SentryPosture::Attack, true};

bool IsEnhancedAttackPending(const PostureRuntime& posture) noexcept {
    return posture.HasPending && posture.Pending == kEnhancedAttack;
}

bool IsEnhancedAttackActive(const PostureRuntime& posture) noexcept {
    return posture.Current == kEnhancedAttack;
}

}  // namespace

void OutpostEngagementLock::Configure(const OutpostEngagementSetting& setting) noexcept {
    setting_ = setting;
}

void OutpostEngagementLock::Reset() noexcept {
    active_ = false;
    have_enemy_hp_ = false;
    last_enemy_hp_ = 0;
    enhanced_armed_ = false;
    enhanced_attempted_ = false;
    enhanced_unavailable_ = false;
}

void OutpostEngagementLock::MarkEnhancedUnavailable() noexcept {
    if (active_ && enhanced_attempted_) {
        enhanced_armed_ = false;
        enhanced_unavailable_ = true;
    }
}

OutpostEngagementDecision OutpostEngagementLock::Exit(const OutpostEngagementExitReason reason) noexcept {
    Reset();
    return {
        .Active = false,
        .HoldTarget = false,
        .CancelPending = true,
        .ExitReason = reason,
    };
}

OutpostEngagementDecision OutpostEngagementLock::Tick(
    const TimePoint,
    const OutpostEngagementInput& input) {
    const bool eligible = setting_.Enable && input.Target7Fresh && input.SelectedTarget7 &&
        input.EnemyHpFresh && input.EnemyHp > 0;
    if (!active_) {
        if (!eligible) {
            return {};
        }
        active_ = true;
        have_enemy_hp_ = true;
        last_enemy_hp_ = input.EnemyHp;
    }

    if (!input.EnemyHpFresh) {
        return Exit(OutpostEngagementExitReason::EnemyHpStale);
    }
    if (input.EnemyHp == 0) {
        return Exit(OutpostEngagementExitReason::EnemyHpZero);
    }
    if (!input.NavigationReachable) {
        return Exit(OutpostEngagementExitReason::NavigationUnreachable);
    }

    const bool enhanced_pending = IsEnhancedAttackPending(input.Posture);
    const bool enhanced_active = IsEnhancedAttackActive(input.Posture);
    if (enhanced_active) {
        enhanced_armed_ = false;
    }
    const auto health_threshold = (enhanced_pending || enhanced_active)
        ? setting_.EnhancedAttackLockExitHp
        : setting_.NormalAttackLockExitHp;
    if (input.SelfHpFresh && input.SelfHp <= health_threshold) {
        return Exit((enhanced_pending || enhanced_active)
            ? OutpostEngagementExitReason::EnhancedHealthThreshold
            : OutpostEngagementExitReason::NormalHealthThreshold);
    }
    if (!input.Target7Fresh || !input.SelectedTarget7) {
        return Exit(OutpostEngagementExitReason::TargetLost);
    }

    if (have_enemy_hp_ && input.EnemyHp < last_enemy_hp_ &&
        setting_.EnhancedAttackOnEnemyHpDrop && !enhanced_attempted_ &&
        input.EnhancedAttackRemainingFresh && input.EnhancedAttackRemainingSec > 0) {
        enhanced_armed_ = true;
        enhanced_attempted_ = true;
    }
    have_enemy_hp_ = true;
    last_enemy_hp_ = input.EnemyHp;

    OutpostEngagementDecision decision{
        .Active = true,
        .HoldTarget = true,
        .EnhancedArmed = enhanced_armed_,
        .EnhancedUnavailable = enhanced_unavailable_,
        .EnhancedPending = enhanced_pending,
        .EnhancedActive = enhanced_active,
    };

    if (enhanced_pending || enhanced_active || enhanced_unavailable_) {
        return decision;
    }
    if (enhanced_armed_ && input.Posture.Current == kNormalAttack &&
        !input.Posture.HasPending && input.PostureCooldownReady) {
        decision.Intent = kEnhancedAttack;
        return decision;
    }
    if (input.Posture.Current != kNormalAttack && !input.Posture.HasPending) {
        decision.Intent = kNormalAttack;
    }
    return decision;
}

}  // namespace BehaviorTree
