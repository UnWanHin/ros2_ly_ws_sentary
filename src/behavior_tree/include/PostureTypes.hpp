// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

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
};

struct PostureRequestPolicy {
    bool AllowOptimisticAck{true};
    bool PreserveCurrentOnRetryExhausted{false};

    static constexpr PostureRequestPolicy OutpostLock() noexcept {
        return {false, true};
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

}  // namespace BehaviorTree
