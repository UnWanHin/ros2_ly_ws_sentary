// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>

namespace BehaviorTree {

enum class SentryPosture : std::uint8_t {
    Unknown = 0,
    Attack = 1,
    Defense = 2,
    Move = 3
};

inline constexpr bool IsValidPosture(const SentryPosture posture) noexcept {
    return posture == SentryPosture::Attack ||
           posture == SentryPosture::Defense ||
           posture == SentryPosture::Move;
}

inline constexpr bool IsValidPostureValue(const std::uint8_t posture) noexcept {
    return posture >= static_cast<std::uint8_t>(SentryPosture::Attack) &&
           posture <= static_cast<std::uint8_t>(SentryPosture::Move);
}

inline constexpr SentryPosture ToPosture(const std::uint8_t posture) noexcept {
    return IsValidPostureValue(posture) ? static_cast<SentryPosture>(posture) : SentryPosture::Unknown;
}

inline constexpr std::uint8_t ToPostureValue(const SentryPosture posture) noexcept {
    return IsValidPosture(posture) ? static_cast<std::uint8_t>(posture) : 0U;
}

inline bool IsPostureFeedbackFresh(
    const bool has_received,
    const std::chrono::steady_clock::time_point received_at,
    const int fresh_ms,
    const std::chrono::steady_clock::time_point now) noexcept {
    return has_received && fresh_ms > 0 && now >= received_at &&
        now - received_at <= std::chrono::milliseconds(fresh_ms);
}

struct PostureMode {
    SentryPosture Base{SentryPosture::Unknown};
    bool Enhanced{false};

    constexpr bool operator==(const PostureMode&) const noexcept = default;
};

inline constexpr bool IsValidPostureMode(const PostureMode mode) noexcept {
    return IsValidPosture(mode.Base);
}

inline constexpr std::uint8_t ToPostureCommandValue(const PostureMode mode) noexcept {
    const auto base = ToPostureValue(mode.Base);
    return base == 0U ? 0U : static_cast<std::uint8_t>(base + (mode.Enhanced ? 3U : 0U));
}

inline constexpr const char* PostureToString(const SentryPosture posture) noexcept {
    switch (posture) {
        case SentryPosture::Attack: return "Attack";
        case SentryPosture::Defense: return "Defense";
        case SentryPosture::Move: return "Move";
        default: return "Unknown";
    }
}

struct PostureDecision {
    std::uint8_t Command{0}; // 0 = this tick does not send command
    bool Sent{false};
    const char* Reason{"hold"};
};

struct PostureFeedback {
    std::uint8_t Base{0};
    bool Enhanced{false};
    bool Fresh{false};
    bool EnhancedFresh{false};
    std::chrono::steady_clock::time_point ReceivedAt{};
};

struct PostureRequestPolicy {
    bool AllowOptimisticAck{true};
    bool PreserveCurrentOnRetryExhausted{false};
    bool AllowEarlyRotate{true};

    static constexpr PostureRequestPolicy OutpostLock() noexcept {
        return {false, true, false};
    }

    static constexpr PostureRequestPolicy RequiredPosture() noexcept {
        return {true, false, false};
    }
};

struct PostureRefereeTimer {
    bool HasInfo3{false};
    bool Fresh{false};
    std::uint32_t AgeMs{std::numeric_limits<std::uint32_t>::max()};
    std::chrono::steady_clock::time_point AgeMeasuredAt{};
    bool Enhanced{false};
    std::array<std::uint8_t, 4> RemainingSec{};
    std::array<std::uint8_t, 4> EnhancedRemainingSec{};
};

struct PostureRuntime {
    PostureMode Current{SentryPosture::Move, false};
    PostureMode Desired{SentryPosture::Move, false};
    PostureMode Pending{SentryPosture::Unknown, false};
    std::array<double, 4> AccumSec{};  // index = posture value(1..3)
    std::array<bool, 4> Degraded{};    // effective state: referee timer when fresh, local timer otherwise
    std::array<bool, 4> LocalDegraded{};
    std::array<std::uint8_t, 4> RefereeRemainingSec{};
    std::array<std::uint8_t, 4> RefereeEnhancedRemainingSec{};
    bool RefereeTimerFresh{false};
    bool RefereeEnhancedPosture{false};
    bool UsingRefereeTimer{false};
    bool HasPending{false};
    bool FeedbackStale{false};
    int RetryCount{0};
};

enum class TaskPostureIntent : std::uint8_t {
    None = 0,
    SoftTransit = 1,
    SoftArrived = 2,
    HardMove = 3,
    HardAttack = 4,
    HardDefense = 5,
};

inline constexpr const char* TaskPostureIntentToString(const TaskPostureIntent intent) noexcept {
    switch (intent) {
        case TaskPostureIntent::SoftTransit: return "soft_transit";
        case TaskPostureIntent::SoftArrived: return "soft_arrived";
        case TaskPostureIntent::HardMove: return "hard_move";
        case TaskPostureIntent::HardAttack: return "hard_attack";
        case TaskPostureIntent::HardDefense: return "hard_defense";
        case TaskPostureIntent::None: return "none";
    }
    return "none";
}

struct TaskPostureIntentState {
    TaskPostureIntent Intent{TaskPostureIntent::None};
    const char* Source{"none"};
    bool OwnsCurrentGoal{false};
};

inline SentryPosture SelectTransitPosture(
    const PostureRuntime& runtime,
    const int reserve_sec) noexcept {
    if (!runtime.UsingRefereeTimer) {
        return SentryPosture::Move;
    }

    const auto& remaining = runtime.RefereeEnhancedPosture
        ? runtime.RefereeEnhancedRemainingSec
        : runtime.RefereeRemainingSec;
    const auto move_index = ToPostureValue(SentryPosture::Move);
    const auto reserve = static_cast<std::uint8_t>(std::clamp(reserve_sec, 0, 255));
    // Reserve a positive Move budget before it is exhausted. Once the referee
    // reports zero, keep Move for travel instead of trading chassis mobility
    // for another posture's remaining time.
    if (remaining[move_index] == 0 || remaining[move_index] > reserve) {
        return SentryPosture::Move;
    }

    SentryPosture selected = SentryPosture::Unknown;
    std::uint8_t selected_remaining = 0;
    // Defense is visited first so an equal remaining time chooses the safer transit posture.
    for (const auto posture : {SentryPosture::Defense, SentryPosture::Attack}) {
        const auto index = ToPostureValue(posture);
        if (remaining[index] > selected_remaining) {
            selected = posture;
            selected_remaining = remaining[index];
        }
    }
    return selected_remaining > 0 ? selected : SentryPosture::Move;
}

struct TaskPostureRequest {
    PostureMode Mode{};
    PostureRequestPolicy Policy{};
};

inline TaskPostureRequest ResolveTaskPostureRequest(
    const TaskPostureIntent intent,
    const SentryPosture scored_posture,
    const PostureRuntime& runtime,
    const int reserve_sec) noexcept {
    switch (intent) {
        case TaskPostureIntent::SoftTransit:
            if (scored_posture == SentryPosture::Defense) {
                return {{SentryPosture::Defense, false}, PostureRequestPolicy::RequiredPosture()};
            }
            return {{SelectTransitPosture(runtime, reserve_sec), false},
                    PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::HardMove:
            return {{SentryPosture::Move, false}, PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::HardAttack:
            return {{SentryPosture::Attack, false}, PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::HardDefense:
            return {{SentryPosture::Defense, false}, PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::SoftArrived:
        case TaskPostureIntent::None:
            return {{scored_posture, false}, {}};
    }
    return {{scored_posture, false}, {}};
}

}  // namespace BehaviorTree
