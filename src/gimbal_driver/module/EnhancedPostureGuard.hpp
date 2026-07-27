#pragma once

#include <cstdint>

namespace LangYa {

inline constexpr bool IsEnhancedPostureCommand(const std::uint8_t posture) noexcept {
    return posture >= 4U && posture <= 6U;
}

inline constexpr std::uint8_t EnhancedPostureRemainingSec(
    const std::uint8_t posture,
    const std::uint64_t sentry_info_3) noexcept {
    switch (posture) {
        case 4U: return static_cast<std::uint8_t>((sentry_info_3 >> 32U) & 0xFFU);
        case 5U: return static_cast<std::uint8_t>((sentry_info_3 >> 40U) & 0xFFU);
        case 6U: return static_cast<std::uint8_t>((sentry_info_3 >> 48U) & 0xFFU);
        default: return 0U;
    }
}

inline constexpr bool IsEnhancedPostureCommandAllowed(
    const std::uint8_t posture,
    const bool guard_enabled,
    const bool sentry_info_3_fresh,
    const std::uint64_t sentry_info_3) noexcept {
    return !IsEnhancedPostureCommand(posture) ||
        !guard_enabled ||
        (sentry_info_3_fresh && EnhancedPostureRemainingSec(posture, sentry_info_3) > 0U);
}

// This is deliberately a pure last-write-boundary helper.  Semantic callers
// retain their requested posture, while the serial writer cannot re-send an
// enhanced posture after its corresponding TypeID 10 budget became stale.
inline constexpr std::uint8_t PostureCommandForTransmission(
    const std::uint8_t posture,
    const bool guard_enabled,
    const bool sentry_info_3_fresh,
    const std::uint64_t sentry_info_3) noexcept {
    return IsEnhancedPostureCommandAllowed(
        posture, guard_enabled, sentry_info_3_fresh, sentry_info_3)
        ? posture
        : 0U;
}

}  // namespace LangYa
