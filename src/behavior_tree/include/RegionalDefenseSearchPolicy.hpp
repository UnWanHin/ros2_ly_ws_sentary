#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace BehaviorTree {

// Existing navigation IDs 17 and 24 bracket the dedicated CommonCentral
// tactical search points 29 and 30. IDs 26/27 remain CentralLeft.A/B.
inline constexpr std::array<std::uint8_t, 4> kCommonCentralSearchGoals{
    17U, 29U, 30U, 24U};

inline constexpr std::size_t NextCommonCentralSearchIndex(
    const std::size_t current,
    const int direction) noexcept {
    if (direction < 0) {
        return current == 0U ? 0U : current - 1U;
    }
    return current + 1U >= kCommonCentralSearchGoals.size()
        ? kCommonCentralSearchGoals.size() - 1U
        : current + 1U;
}

inline constexpr int ReverseCommonCentralSearchDirection(
    const std::size_t index,
    const int direction) noexcept {
    if (index == 0U) {
        return 1;
    }
    if (index + 1U == kCommonCentralSearchGoals.size()) {
        return -1;
    }
    return direction;
}

// Tactical CommonCentral advances only after an arrived hold, an explicit
// external unreachable result, or the shared navigation progress watchdog.
struct CommonCentralSearchAdvanceInput {
    bool GoalReached{false};
    bool GoalUnreachable{false};
    bool NoProgressTimeout{false};
    bool HoldElapsed{false};
    bool VisualTargetRecentlySeen{false};
};

constexpr bool ShouldAdvanceCommonCentralSearch(
    const CommonCentralSearchAdvanceInput& input) noexcept {
    if (input.GoalUnreachable || input.NoProgressTimeout) {
        return true;
    }
    return input.GoalReached && input.HoldElapsed && !input.VisualTargetRecentlySeen;
}

}  // namespace BehaviorTree
