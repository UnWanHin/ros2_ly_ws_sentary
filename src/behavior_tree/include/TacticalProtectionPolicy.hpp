#pragma once

#include <chrono>
#include <cstdint>

namespace BehaviorTree {

// A Tactical YAML value is authoritative only when the parameter was provided.
inline bool ResolveTacticalFeatureEnable(
    const bool profile_baseline,
    const bool yaml_provided,
    const bool yaml_enable) noexcept {
    return yaml_provided ? yaml_enable : profile_baseline;
}

inline bool IsProtectCastleRfidEventEnabled(
    const bool protect_castle_enable,
    const bool rfid_enable,
    const bool event_active) noexcept {
    return protect_castle_enable && rfid_enable && event_active;
}

inline bool IsProtectCastleRfidStayEnabled(
    const bool protect_castle_enable,
    const bool rfid_enable,
    const bool stay_when_rfid,
    const bool event_active) noexcept {
    return protect_castle_enable && rfid_enable && stay_when_rfid && event_active;
}

inline bool IsProtectCastleEnemyPositionEnabled(
    const bool protect_castle_enable,
    const bool enemy_position_enable) noexcept {
    return protect_castle_enable && enemy_position_enable;
}

enum class ProtectOutpostPhase : std::uint8_t {
    Idle = 0,
    Travel = 1,
    SearchHold = 2,
    Cooldown = 3,
    Complete = 4,
};

inline const char* ProtectOutpostPhaseToString(const ProtectOutpostPhase phase) noexcept {
    switch (phase) {
        case ProtectOutpostPhase::Idle: return "idle";
        case ProtectOutpostPhase::Travel: return "travel";
        case ProtectOutpostPhase::SearchHold: return "search_hold";
        case ProtectOutpostPhase::Cooldown: return "cooldown";
        case ProtectOutpostPhase::Complete: return "complete";
        default: return "idle";
    }
}

// Pure lifecycle state for one own-outpost damage event. Runtime code owns the
// target point and navigation command; this policy only decides event lifetime.
struct ProtectOutpostState {
    bool HasSample{false};
    std::uint16_t LastHealth{0};
    std::uint64_t ActiveEventGeneration{0};
    ProtectOutpostPhase Phase{ProtectOutpostPhase::Idle};
    std::chrono::steady_clock::time_point HoldStartedAt{};
    std::chrono::steady_clock::time_point CooldownUntil{};
    bool PendingDamageAfterCooldown{false};
};

inline bool ObserveProtectOutpostHealth(
    ProtectOutpostState& state,
    const std::uint16_t health,
    const bool fresh,
    const std::chrono::steady_clock::time_point now) noexcept {
    if (!fresh) {
        return false;
    }
    if (!state.HasSample) {
        state.HasSample = true;
        state.LastHealth = health;
        return false;
    }

    const bool strict_nonzero_decrease =
        health > 0 && state.LastHealth > 0 && health < state.LastHealth;
    state.LastHealth = health;
    if (!strict_nonzero_decrease) {
        return false;
    }

    ++state.ActiveEventGeneration;
    if (state.Phase == ProtectOutpostPhase::Cooldown) {
        // Do not bypass an unreachable cooldown. Resume this newer event only
        // after the cooldown deadline has elapsed.
        state.PendingDamageAfterCooldown = true;
        return true;
    }

    state.CooldownUntil = {};
    state.PendingDamageAfterCooldown = false;
    if (state.Phase == ProtectOutpostPhase::SearchHold) {
        // Keep the selected outpost target and restart its search interval.
        state.HoldStartedAt = now;
    } else {
        state.Phase = ProtectOutpostPhase::Travel;
        state.HoldStartedAt = {};
    }
    return true;
}

inline ProtectOutpostState& TickProtectOutpost(
    ProtectOutpostState& state,
    const bool fresh,
    const bool reached,
    const bool unreachable,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds search_hold,
    const std::chrono::milliseconds unreachable_cooldown = std::chrono::seconds(10)) noexcept {
    // A created event remains actionable even if a later health sample expires.
    // Freshness gates observation above, rather than cancelling an in-flight task.
    static_cast<void>(fresh);

    if ((state.Phase == ProtectOutpostPhase::Travel ||
         state.Phase == ProtectOutpostPhase::SearchHold) && unreachable) {
        state.Phase = ProtectOutpostPhase::Cooldown;
        state.CooldownUntil = now + unreachable_cooldown;
        state.PendingDamageAfterCooldown = false;
        return state;
    }

    if (state.Phase == ProtectOutpostPhase::Travel && reached) {
        state.Phase = ProtectOutpostPhase::SearchHold;
        state.HoldStartedAt = now;
        return state;
    }

    if (state.Phase == ProtectOutpostPhase::SearchHold &&
        now >= state.HoldStartedAt + search_hold) {
        state.Phase = ProtectOutpostPhase::Complete;
        return state;
    }

    if (state.Phase == ProtectOutpostPhase::Cooldown && now >= state.CooldownUntil) {
        state.Phase = state.PendingDamageAfterCooldown
            ? ProtectOutpostPhase::Travel
            : ProtectOutpostPhase::Complete;
        state.PendingDamageAfterCooldown = false;
    }
    return state;
}

struct BaseDamageWindowState {
    bool HasSample{false};
    std::uint16_t LastHealth{0};
    std::chrono::steady_clock::time_point ActiveUntil{};
};

inline bool ObserveBaseHealthForProtection(
    BaseDamageWindowState& state,
    const std::uint16_t health,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds protection_window = std::chrono::seconds(8)) noexcept {
    if (!state.HasSample) {
        state.HasSample = true;
        state.LastHealth = health;
        return false;
    }

    const bool strict_nonzero_decrease =
        health > 0 && state.LastHealth > 0 && health < state.LastHealth;
    state.LastHealth = health;
    if (strict_nonzero_decrease) {
        state.ActiveUntil = now + protection_window;
    }
    return strict_nonzero_decrease;
}

inline bool IsProtectCastleBaseDamageActive(
    const bool protect_castle_enable,
    const bool base_enable,
    const bool base_health_fresh,
    const BaseDamageWindowState& state,
    const std::chrono::steady_clock::time_point now) noexcept {
    return protect_castle_enable && base_enable && base_health_fresh &&
        state.LastHealth > 0 &&
        state.ActiveUntil.time_since_epoch().count() != 0 &&
        now < state.ActiveUntil;
}

enum class CastleOccupancyAction : std::uint8_t {
    ApproachCastle = 0,
    HoldCastle = 1,
    PerimeterDefense = 2,
};

inline const char* CastleOccupancyActionToString(
    const CastleOccupancyAction action) noexcept {
    switch (action) {
        case CastleOccupancyAction::ApproachCastle: return "approach_castle";
        case CastleOccupancyAction::HoldCastle: return "hold_castle";
        case CastleOccupancyAction::PerimeterDefense: return "perimeter_defense";
        default: return "perimeter_defense";
    }
}

struct CastleOccupancyInput {
    bool RefereeFresh{false};
    std::uint8_t RefereeStatus{0};
    bool SelfRfidAtCastle{false};
    bool SelfPositionAtCastle{false};
    bool ReachedCastleGrace{false};
    bool TeammatePositionAtCastle{false};
};

struct CastleOccupancyResolution {
    CastleOccupancyAction Action{CastleOccupancyAction::PerimeterDefense};
    bool SelfLikelyAtCastle{false};
    bool TeammateLikelyAtCastle{false};
    bool TeamOrAmbiguousOccupant{false};
};

inline CastleOccupancyResolution ResolveCastleOccupancy(
    const CastleOccupancyInput& input) noexcept {
    CastleOccupancyResolution result;
    result.TeammateLikelyAtCastle = input.TeammatePositionAtCastle;
    const bool direct_self_presence = input.SelfRfidAtCastle || input.SelfPositionAtCastle;
    result.SelfLikelyAtCastle = direct_self_presence && !input.TeammatePositionAtCastle;
    result.TeamOrAmbiguousOccupant =
        input.TeammatePositionAtCastle || (direct_self_presence && input.TeammatePositionAtCastle) ||
        (input.ReachedCastleGrace && !direct_self_presence);

    if (!input.RefereeFresh) {
        return result;
    }
    if (input.RefereeStatus == 0U || input.RefereeStatus == 2U) {
        result.Action = CastleOccupancyAction::ApproachCastle;
        return result;
    }
    if ((input.RefereeStatus == 1U || input.RefereeStatus == 3U) && direct_self_presence) {
        result.Action = CastleOccupancyAction::HoldCastle;
        return result;
    }
    result.Action = CastleOccupancyAction::PerimeterDefense;
    return result;
}

}  // namespace BehaviorTree
