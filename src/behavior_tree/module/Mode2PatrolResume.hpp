#pragma once

#include <cmath>

namespace BehaviorTree {

struct Mode2PatrolResumeDecision {
    bool ContinueSmoothing{false};
    bool RebaseFromFeedback{false};
};

// Keeps the saved Mode 2 trajectory only while the gimbal is converging to it.
// A bounded wait prevents a stale/unreachable target yaw from freezing patrol.
inline Mode2PatrolResumeDecision EvaluateMode2PatrolResume(
    const float yaw_error_deg,
    const int elapsed_ms,
    const float yaw_tolerance_deg,
    const int max_wait_ms) noexcept {
    if (std::abs(yaw_error_deg) <= yaw_tolerance_deg) {
        return {};
    }
    if (elapsed_ms >= max_wait_ms) {
        return {.ContinueSmoothing = false, .RebaseFromFeedback = true};
    }
    return {.ContinueSmoothing = true, .RebaseFromFeedback = false};
}

}  // namespace BehaviorTree
