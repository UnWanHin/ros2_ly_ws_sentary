// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
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

enum class PostureRequestPriority : std::uint8_t {
    Scored = 0,
    Required = 1,
    Safety = 2,
};

inline constexpr const char* PostureRequestPriorityToString(
    const PostureRequestPriority priority) noexcept {
    switch (priority) {
        case PostureRequestPriority::Safety: return "safety";
        case PostureRequestPriority::Required: return "required";
        case PostureRequestPriority::Scored: return "scored";
    }
    return "scored";
}

struct PostureRequestPolicy {
    bool AllowOptimisticAck{true};
    bool PreserveCurrentOnRetryExhausted{false};
    bool AllowEarlyRotate{true};
    PostureRequestPriority Priority{PostureRequestPriority::Scored};
    const char* Source{"scored"};

    static constexpr PostureRequestPolicy OutpostLock() noexcept {
        return {false, true, false, PostureRequestPriority::Required, "outpost_lock"};
    }

    static constexpr PostureRequestPolicy RequiredPosture() noexcept {
        return {true, false, false, PostureRequestPriority::Required, "required"};
    }

    static constexpr PostureRequestPolicy HardMove() noexcept {
        return {true, false, false, PostureRequestPriority::Safety, "hard_move"};
    }

    static constexpr PostureRequestPolicy EnhancedDefenseHold() noexcept {
        return {false, true, false, PostureRequestPriority::Required, "enhanced_defense_hold"};
    }

    static constexpr PostureRequestPolicy RecoveryMove() noexcept {
        return {false, true, false, PostureRequestPriority::Safety, "recovery_move"};
    }
};

struct PostureRefereeTimer {
    bool HasInfo3{false};
    bool Fresh{false};
    std::uint32_t AgeMs{std::numeric_limits<std::uint32_t>::max()};
    std::chrono::steady_clock::time_point AgeMeasuredAt{};
    bool Enhanced{false};
    int EnhancedContradictionGraceMs{500};
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
    bool EnhancedFeedbackQuarantined{false};
    bool UsingRefereeTimer{false};
    bool HasPending{false};
    bool FeedbackStale{false};
    int RetryCount{0};
    PostureRequestPriority PendingPriority{PostureRequestPriority::Scored};
    const char* PendingSource{"none"};
};

enum class TaskPostureIntent : std::uint8_t {
    None = 0,
    SoftTransit = 1,
    SoftArrived = 2,
    HardMove = 3,
    HardAttack = 4,
    HardDefense = 5,
    ProtectHeroDefenseHold = 6,
    ProtectHeroEnhancedDefense = 7,
    RecoveryEnhancedMove = 8,
};

inline constexpr const char* TaskPostureIntentToString(const TaskPostureIntent intent) noexcept {
    switch (intent) {
        case TaskPostureIntent::SoftTransit: return "soft_transit";
        case TaskPostureIntent::SoftArrived: return "soft_arrived";
        case TaskPostureIntent::HardMove: return "hard_move";
        case TaskPostureIntent::HardAttack: return "hard_attack";
        case TaskPostureIntent::HardDefense: return "hard_defense";
        case TaskPostureIntent::ProtectHeroDefenseHold: return "protect_hero_defense_hold";
        case TaskPostureIntent::ProtectHeroEnhancedDefense: return "protect_hero_enhanced_defense";
        case TaskPostureIntent::RecoveryEnhancedMove: return "recovery_enhanced_move";
        case TaskPostureIntent::None: return "none";
    }
    return "none";
}

inline constexpr TaskPostureIntent ResolveProtectHeroHoldIntent(
    const bool enhanced_defense_enabled,
    const bool damage_burst,
    const bool enhanced_defense_unavailable) noexcept {
    return enhanced_defense_enabled && damage_burst && !enhanced_defense_unavailable
        ? TaskPostureIntent::ProtectHeroEnhancedDefense
        : TaskPostureIntent::ProtectHeroDefenseHold;
}

inline constexpr bool ShouldForceDamageBurstDefense(
    const bool recovery_goal,
    const bool damage_burst) noexcept {
    return damage_burst && !recovery_goal;
}

struct TaskPostureIntentState {
    TaskPostureIntent Intent{TaskPostureIntent::None};
    const char* Source{"none"};
    bool OwnsCurrentGoal{false};
};

struct TransitPostureContext {
    bool Enabled{true};
    bool HasFreshDistance{false};
    bool HasFreshVelocity{false};
    bool AllowAttackDuringTransit{false};
    double DistanceCm{0.0};
    double VelocityMps{0.0};
    double NominalSpeedMps{0.8};
    double SafetyFactor{1.5};
    int ArrivalBufferSec{8};
    int MinReserveSec{30};
    int MaxReserveSec{120};
    int FallbackReserveSec{45};
};

inline int ComputeTransitMoveReserveSec(const TransitPostureContext& context) noexcept {
    const int fallback = std::max(0, context.FallbackReserveSec);
    const int minimum = std::max(0, context.MinReserveSec);
    const int maximum = std::max(minimum, context.MaxReserveSec);
    int reserve = std::max(fallback, minimum);

    if (context.Enabled && context.HasFreshDistance && std::isfinite(context.DistanceCm) &&
        context.DistanceCm > 0.0) {
        const double observed_speed = context.HasFreshVelocity &&
            std::isfinite(context.VelocityMps) && context.VelocityMps > 0.05
            ? context.VelocityMps
            : context.NominalSpeedMps;
        const double safety_factor = std::max(1.0, context.SafetyFactor);
        if (std::isfinite(observed_speed) && observed_speed > 0.05) {
            const double eta_sec = (context.DistanceCm / 100.0) / observed_speed;
            const double required_sec = std::ceil(
                eta_sec * safety_factor + static_cast<double>(std::max(0, context.ArrivalBufferSec)));
            if (std::isfinite(required_sec)) {
                reserve = std::max(reserve, static_cast<int>(required_sec));
            }
        }
    }

    return std::clamp(reserve, minimum, maximum);
}

inline SentryPosture SelectTransitPosture(
    const PostureRuntime& runtime,
    const TransitPostureContext& context) noexcept {
    if (!runtime.UsingRefereeTimer) {
        return SentryPosture::Move;
    }

    const auto& remaining = runtime.RefereeEnhancedPosture
        ? runtime.RefereeEnhancedRemainingSec
        : runtime.RefereeRemainingSec;
    const auto move_index = ToPostureValue(SentryPosture::Move);
    const auto reserve = static_cast<std::uint8_t>(std::clamp(
        ComputeTransitMoveReserveSec(context), 0, 255));
    // Zero keeps the existing weakened-but-still-mobile semantics. A positive
    // budget is reserved dynamically from the current travel ETA.
    if (remaining[move_index] == 0 || remaining[move_index] > reserve) {
        return SentryPosture::Move;
    }

    SentryPosture selected = SentryPosture::Unknown;
    std::uint8_t selected_remaining = 0;
    for (const auto posture : {SentryPosture::Defense, SentryPosture::Attack}) {
        const auto index = ToPostureValue(posture);
        if (remaining[index] > selected_remaining) {
            selected = posture;
            selected_remaining = remaining[index];
        }
    }
    return selected_remaining > 0 ? selected : SentryPosture::Move;
}

inline SentryPosture SelectTransitPosture(
    const PostureRuntime& runtime,
    const int reserve_sec) noexcept {
    TransitPostureContext context;
    context.Enabled = false;
    context.MinReserveSec = 0;
    context.MaxReserveSec = 255;
    context.FallbackReserveSec = reserve_sec;
    return SelectTransitPosture(runtime, context);
}

struct TaskPostureRequest {
    PostureMode Mode{};
    PostureRequestPolicy Policy{};
};

inline bool CanRequestEnhancedPosture(
    const PostureRuntime& runtime,
    const PostureRefereeTimer& referee_timer,
    const SentryPosture posture) noexcept {
    const auto posture_index = ToPostureValue(posture);
    return !runtime.EnhancedFeedbackQuarantined &&
        posture_index > 0U &&
        referee_timer.HasInfo3 &&
        referee_timer.Fresh &&
        referee_timer.EnhancedRemainingSec[posture_index] > 0U;
}

inline bool CanRequestEnhancedDefense(
    const PostureRuntime& runtime,
    const PostureRefereeTimer& referee_timer) noexcept {
    return CanRequestEnhancedPosture(runtime, referee_timer, SentryPosture::Defense);
}

inline bool ShouldRequestRecoveryEnhancedMove(
    const bool regional_profile,
    const bool enabled,
    const bool respawn_suppressed,
    const bool unavailable_for_current_recovery,
    const bool recovery_traveling,
    const bool self_health_fresh,
    const std::uint16_t self_health,
    const int health_threshold_hp,
    const PostureRuntime& runtime,
    const PostureRefereeTimer& referee_timer) noexcept {
    return regional_profile &&
        enabled &&
        !respawn_suppressed &&
        !unavailable_for_current_recovery &&
        recovery_traveling &&
        self_health_fresh &&
        self_health > 0U &&
        self_health <= static_cast<std::uint16_t>(std::max(1, health_threshold_hp)) &&
        CanRequestEnhancedPosture(runtime, referee_timer, SentryPosture::Move);
}

inline constexpr bool IsNormalRespawnHealthTransition(
    const bool has_previous_health,
    const std::uint16_t previous_health,
    const std::uint16_t current_health) noexcept {
    return has_previous_health && previous_health == 0U && current_health > 0U;
}

inline bool ShouldDeferRecoveryForProtectHeroEnhancedDefense(
    const bool bt_owns_protect_hero_enhanced_defense,
    const bool switch_cooldown_ready,
    const PostureRuntime& runtime) noexcept {
    return bt_owns_protect_hero_enhanced_defense &&
        !switch_cooldown_ready &&
        runtime.Current == PostureMode{SentryPosture::Defense, true} &&
        !runtime.EnhancedFeedbackQuarantined &&
        !runtime.FeedbackStale;
}

inline TaskPostureRequest ResolveTaskPostureRequest(
    const TaskPostureIntent intent,
    const SentryPosture scored_posture,
    const PostureRuntime& runtime,
    const TransitPostureContext& transit_context,
    const PostureRefereeTimer& referee_timer = {}) noexcept {
    switch (intent) {
        case TaskPostureIntent::SoftTransit:
            if (scored_posture == SentryPosture::Defense ||
                (scored_posture == SentryPosture::Attack &&
                 transit_context.AllowAttackDuringTransit)) {
                return {{scored_posture, false}, PostureRequestPolicy::RequiredPosture()};
            }
            return {{SelectTransitPosture(runtime, transit_context), false},
                    PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::HardMove:
            return {{SentryPosture::Move, false}, PostureRequestPolicy::HardMove()};
        case TaskPostureIntent::HardAttack:
            return {{SentryPosture::Attack, false}, PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::HardDefense:
            return {{SentryPosture::Defense, false}, PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::ProtectHeroDefenseHold:
            return {{SentryPosture::Defense, false}, PostureRequestPolicy::RequiredPosture()};
        case TaskPostureIntent::ProtectHeroEnhancedDefense: {
            const bool enhanced_available = CanRequestEnhancedDefense(runtime, referee_timer);
            return {{SentryPosture::Defense, enhanced_available},
                    enhanced_available
                        ? PostureRequestPolicy::EnhancedDefenseHold()
                        : PostureRequestPolicy::RequiredPosture()};
        }
        case TaskPostureIntent::RecoveryEnhancedMove: {
            const bool enhanced_available = CanRequestEnhancedPosture(
                runtime, referee_timer, SentryPosture::Move);
            return {{SentryPosture::Move, enhanced_available},
                    PostureRequestPolicy::RecoveryMove()};
        }
        case TaskPostureIntent::SoftArrived:
        case TaskPostureIntent::None:
            return {{scored_posture, false}, {}};
    }
    return {{scored_posture, false}, {}};
}

inline TaskPostureRequest ResolveTaskPostureRequest(
    const TaskPostureIntent intent,
    const SentryPosture scored_posture,
    const PostureRuntime& runtime,
    const int reserve_sec,
    const PostureRefereeTimer& referee_timer = {}) noexcept {
    TransitPostureContext context;
    context.Enabled = false;
    context.MinReserveSec = 0;
    context.MaxReserveSec = 255;
    context.FallbackReserveSec = reserve_sec;
    return ResolveTaskPostureRequest(intent, scored_posture, runtime, context, referee_timer);
}

}  // namespace BehaviorTree
