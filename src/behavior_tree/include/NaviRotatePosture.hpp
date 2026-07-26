#pragma once

#include <algorithm>
#include <chrono>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

inline bool ShouldRequestMovePostureWhenNaviFalse(
    const LangYa::NaviControlSetting& setting,
    const bool has_received,
    const std::chrono::steady_clock::time_point received_at,
    const std::chrono::steady_clock::time_point now,
    const bool should_rotate) noexcept {
    if (!setting.Enable ||
        !setting.SetPostureToMoveWhenFalse ||
        !has_received ||
        should_rotate ||
        received_at.time_since_epoch().count() == 0 ||
        now < received_at) {
        return false;
    }

    const auto fresh_timeout = std::chrono::milliseconds(
        std::max(0, setting.FreshTimeoutMs));
    return now - received_at <= fresh_timeout;
}

}  // namespace BehaviorTree
