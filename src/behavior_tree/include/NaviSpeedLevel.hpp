#pragma once

#include <cstdint>

namespace BehaviorTree {

inline constexpr std::uint8_t kNaviSpeedStop = 0;
inline constexpr std::uint8_t kNaviSpeedNormal = 1;
inline constexpr std::uint8_t kNaviSpeedFast = 2;

inline constexpr std::uint8_t NormalizeNaviSpeedLevel(const std::uint8_t level) noexcept {
    return level <= kNaviSpeedFast ? level : kNaviSpeedNormal;
}

}  // namespace BehaviorTree
