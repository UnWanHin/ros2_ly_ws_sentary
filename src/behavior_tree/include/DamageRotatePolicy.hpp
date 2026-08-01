#pragma once

#include <algorithm>
#include <cstdint>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

enum class RotateSuppressionSource : std::uint8_t {
    None = 0,
    StopRotate,
    HighlandCompatibility,
    NavigationShouldRotateFalse,
    FollowMode,
};

inline constexpr const char* RotateSuppressionSourceToString(
    const RotateSuppressionSource source) noexcept {
    switch (source) {
        case RotateSuppressionSource::StopRotate: return "stop_rotate";
        case RotateSuppressionSource::HighlandCompatibility: return "highland_compatibility";
        case RotateSuppressionSource::NavigationShouldRotateFalse:
            return "navi_should_rotate_false";
        case RotateSuppressionSource::FollowMode: return "follow_mode";
        case RotateSuppressionSource::None: return "none";
    }
    return "unknown";
}

struct RotateResolution {
    std::uint8_t Gear{0};
    RotateSuppressionSource SuppressedBy{RotateSuppressionSource::None};
};

// The order mirrors the final firecode arbitration in PublishTogether().
// Castle/ProtectCastle is intentionally not an input: arriving there does not
// suppress damage rotation. Only an explicit control source may do so.
inline RotateResolution ResolveFinalRotateGear(
    const std::uint8_t requested_gear,
    const bool stop_rotate,
    const bool highland_compat_disable,
    const bool navigation_should_rotate_false,
    const bool follow_mode) noexcept {
    if (stop_rotate) {
        return {0U, RotateSuppressionSource::StopRotate};
    }
    if (highland_compat_disable) {
        return {0U, RotateSuppressionSource::HighlandCompatibility};
    }
    if (navigation_should_rotate_false) {
        return {0U, RotateSuppressionSource::NavigationShouldRotateFalse};
    }
    if (follow_mode) {
        return {0U, RotateSuppressionSource::FollowMode};
    }
    return {requested_gear, RotateSuppressionSource::None};
}

inline std::uint8_t ResolveRotateGearWithFollowPriority(
    const std::uint8_t rotate_gear,
    const bool follow_mode) noexcept {
    return ResolveFinalRotateGear(rotate_gear, false, false, false, follow_mode).Gear;
}

inline std::uint8_t ResolveDamageRotateGear(
    const LangYa::DamageRotateSetting& setting,
    const std::uint8_t base_gear,
    const int elapsed_ms) noexcept {
    const int elapsed = std::max(0, elapsed_ms);
    std::uint8_t damage_gear = 0;
    if (elapsed >= setting.Gear0HoldMs + setting.Gear1HoldMs + setting.Gear2HoldMs) {
        damage_gear = 3;
    } else if (elapsed >= setting.Gear0HoldMs + setting.Gear1HoldMs) {
        damage_gear = 2;
    } else if (elapsed >= setting.Gear0HoldMs) {
        damage_gear = 1;
    }
    return std::max(base_gear, damage_gear);
}

}  // namespace BehaviorTree
