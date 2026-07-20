#pragma once

#include <algorithm>
#include <cstdint>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

inline std::uint8_t ResolveRotateGearWithFollowPriority(
    const std::uint8_t rotate_gear,
    const bool follow_mode) noexcept {
    return follow_mode ? 0U : rotate_gear;
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
