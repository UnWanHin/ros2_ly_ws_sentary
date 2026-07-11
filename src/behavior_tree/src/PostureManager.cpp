// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/PostureManager.hpp"

#include <algorithm>
#include <limits>

namespace BehaviorTree {

void PostureManager::Configure(const LangYa::PostureSetting& setting) {
    config_ = setting;
}

void PostureManager::Reset(TimePoint now, SentryPosture initial_posture) {
    runtime_ = {};
    runtime_.Current = IsValidPosture(initial_posture) ? initial_posture : SentryPosture::Move;
    runtime_.Desired = runtime_.Current;

    initialized_ = true;
    has_feedback_ = false;
    last_tick_ = now;
    last_switch_ = now;
    pending_since_ = now;
    last_command_ = now;
    last_feedback_ = now;
}

void PostureManager::accumulate_time(const double dt_seconds) {
    const auto idx = ToPostureValue(runtime_.Current);
    if (idx == 0U) return;

    runtime_.AccumSec[idx] += dt_seconds;
    runtime_.LocalDegraded[idx] = runtime_.AccumSec[idx] >= static_cast<double>(config_.MaxSinglePostureSec);
}

void PostureManager::update_feedback(const TimePoint now, const std::uint8_t feedback_posture_value) {
    const auto feedback = ToPosture(feedback_posture_value);
    if (IsValidPosture(feedback)) {
        has_feedback_ = true;
        last_feedback_ = now;
        runtime_.FeedbackStale = false;
        runtime_.Current = feedback;
        if (runtime_.HasPending && feedback == runtime_.Pending) {
            runtime_.HasPending = false;
            runtime_.Pending = SentryPosture::Unknown;
            runtime_.RetryCount = 0;
            last_switch_ = now;
        }
        return;
    }

    if (has_feedback_ && now - last_feedback_ > std::chrono::seconds(2)) {
        runtime_.FeedbackStale = true;
    }
}

void PostureManager::update_referee_timer(const PostureRefereeTimer& referee_timer) {
    runtime_.RefereeTimerFresh = referee_timer.Fresh;
    runtime_.RefereeEnhancedPosture = referee_timer.Enhanced;
    runtime_.RefereeRemainingSec = referee_timer.RemainingSec;
    runtime_.RefereeEnhancedRemainingSec = referee_timer.EnhancedRemainingSec;
    runtime_.UsingRefereeTimer = referee_timer.Fresh;
}

double PostureManager::effective_accum_sec(const SentryPosture posture) const {
    const auto idx = ToPostureValue(posture);
    if (idx == 0U) return 0.0;
    if (!runtime_.UsingRefereeTimer) return runtime_.AccumSec[idx];

    const auto remaining = runtime_.RefereeEnhancedPosture
        ? runtime_.RefereeEnhancedRemainingSec[idx]
        : runtime_.RefereeRemainingSec[idx];
    return std::max(0.0, static_cast<double>(config_.MaxSinglePostureSec) - remaining);
}

bool PostureManager::effective_degraded(const SentryPosture posture) const {
    const auto idx = ToPostureValue(posture);
    if (idx == 0U) return false;
    if (!runtime_.UsingRefereeTimer) return runtime_.LocalDegraded[idx];
    const auto remaining = runtime_.RefereeEnhancedPosture
        ? runtime_.RefereeEnhancedRemainingSec[idx]
        : runtime_.RefereeRemainingSec[idx];
    return remaining == 0U;
}

bool PostureManager::effective_early_rotate(const SentryPosture posture) const {
    if (!runtime_.UsingRefereeTimer) {
        return effective_accum_sec(posture) >= static_cast<double>(config_.EarlyRotateSec);
    }
    const auto idx = ToPostureValue(posture);
    if (idx == 0U) return false;
    const auto remaining = runtime_.RefereeEnhancedPosture
        ? runtime_.RefereeEnhancedRemainingSec[idx]
        : runtime_.RefereeRemainingSec[idx];
    return remaining <= std::max(0, config_.MaxSinglePostureSec - config_.EarlyRotateSec);
}

SentryPosture PostureManager::choose_alternative_posture(const SentryPosture avoid) const {
    constexpr SentryPosture candidates[] = {
        SentryPosture::Attack,
        SentryPosture::Defense,
        SentryPosture::Move};

    SentryPosture best = avoid;
    double best_accum = std::numeric_limits<double>::max();
    for (const auto posture : candidates) {
        if (posture == avoid) continue;
        const auto idx = ToPostureValue(posture);
        if (idx == 0U) continue;
        const double accum = effective_accum_sec(posture);
        if (accum < best_accum) {
            best_accum = accum;
            best = posture;
        }
    }
    return best;
}

PostureDecision PostureManager::Tick(
    const TimePoint now,
    const SentryPosture desired_posture,
    const std::uint8_t feedback_posture_value,
    const PostureRefereeTimer& referee_timer) {

    if (!initialized_) {
        const auto initial = IsValidPosture(ToPosture(feedback_posture_value))
            ? ToPosture(feedback_posture_value)
            : SentryPosture::Move;
        Reset(now, initial);
    }

    PostureDecision decision{};

    const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_tick_).count();
    if (dt > 0.0 && dt < 1.0) {
        accumulate_time(dt);
    }
    last_tick_ = now;

    update_feedback(now, feedback_posture_value);
    update_referee_timer(referee_timer);
    for (const auto posture : {SentryPosture::Attack, SentryPosture::Defense, SentryPosture::Move}) {
        const auto idx = ToPostureValue(posture);
        runtime_.Degraded[idx] = effective_degraded(posture);
    }

    if (!config_.Enable) {
        runtime_.Desired = runtime_.Current;
        decision.Reason = "disabled";
        return decision;
    }

    runtime_.Desired = IsValidPosture(desired_posture) ? desired_posture : runtime_.Current;

    const auto current_idx = ToPostureValue(runtime_.Current);
    if (current_idx > 0U &&
        runtime_.Desired == runtime_.Current &&
        effective_early_rotate(runtime_.Current)) {
        const auto alternative = choose_alternative_posture(runtime_.Current);
        if (IsValidPosture(alternative) && alternative != runtime_.Current) {
            runtime_.Desired = alternative;
        }
    }

    if (runtime_.HasPending) {
        if (runtime_.Current == runtime_.Pending) {
            runtime_.HasPending = false;
            runtime_.Pending = SentryPosture::Unknown;
            runtime_.RetryCount = 0;
            last_switch_ = now;
            decision.Reason = "pending_confirmed";
            return decision;
        }

        const auto pending_elapsed = now - pending_since_;
        if (pending_elapsed < std::chrono::milliseconds(config_.PendingAckTimeoutMs)) {
            decision.Reason = "pending_wait";
            return decision;
        }

        if (config_.OptimisticAck && !has_feedback_) {
            runtime_.Current = runtime_.Pending;
            runtime_.HasPending = false;
            runtime_.Pending = SentryPosture::Unknown;
            runtime_.RetryCount = 0;
            last_switch_ = now;
            decision.Reason = "optimistic_ack";
            return decision;
        }

        const auto retry_elapsed = now - last_command_;
        if (runtime_.RetryCount < config_.MaxRetryCount &&
            retry_elapsed >= std::chrono::milliseconds(config_.RetryIntervalMs)) {
            runtime_.RetryCount++;
            last_command_ = now;
            decision.Command = ToPostureValue(runtime_.Pending);
            decision.Sent = decision.Command != 0U;
            decision.Reason = "retry_pending";
            return decision;
        }

        if (runtime_.RetryCount >= config_.MaxRetryCount) {
            // 防死锁：长时间切换失败时，放弃 pending 并尝试切到负担更低的姿态。
            runtime_.HasPending = false;
            runtime_.Pending = SentryPosture::Unknown;
            runtime_.RetryCount = 0;
            runtime_.Desired = choose_alternative_posture(runtime_.Current);
            decision.Reason = "pending_dropped";
        } else {
            decision.Reason = "pending_retry_wait";
            return decision;
        }
    }

    if (!IsValidPosture(runtime_.Desired) || runtime_.Desired == runtime_.Current) {
        decision.Reason = "hold";
        return decision;
    }

    const auto switch_elapsed = now - last_switch_;
    if (switch_elapsed < std::chrono::seconds(config_.SwitchCooldownSec)) {
        decision.Reason = "cooldown";
        return decision;
    }

    const bool current_degraded = current_idx > 0U && runtime_.Degraded[current_idx];
    if (!current_degraded && switch_elapsed < std::chrono::seconds(config_.MinHoldSec)) {
        decision.Reason = "min_hold";
        return decision;
    }

    runtime_.Pending = runtime_.Desired;
    runtime_.HasPending = true;
    runtime_.RetryCount = 1;
    pending_since_ = now;
    last_command_ = now;

    decision.Command = ToPostureValue(runtime_.Pending);
    decision.Sent = decision.Command != 0U;
    decision.Reason = "switch_request";
    return decision;
}

}  // namespace BehaviorTree
