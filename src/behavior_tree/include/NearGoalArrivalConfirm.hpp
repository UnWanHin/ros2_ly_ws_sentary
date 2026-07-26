#pragma once

#include <chrono>
#include <cstdint>

#include "../module/Area.hpp"

namespace BehaviorTree {

class NearGoalArrivalConfirm {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    bool Observe(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position,
        const bool inside_arrival_radius,
        const TimePoint now,
        const int wait_ms) noexcept {
        if (wait_ms <= 0) {
            Reset();
            return inside_arrival_radius;
        }
        if (!inside_arrival_radius) {
            Reset();
            return false;
        }
        if (!active_ || goal_id_ != goal_id ||
            goal_position_.x != goal_position.x || goal_position_.y != goal_position.y) {
            active_ = true;
            goal_id_ = goal_id;
            goal_position_ = goal_position;
            entered_at_ = now;
            return false;
        }
        return now - entered_at_ >= std::chrono::milliseconds(wait_ms);
    }

    void Reset() noexcept {
        active_ = false;
        goal_id_ = 0;
        goal_position_ = {};
        entered_at_ = {};
    }

    bool Pending() const noexcept { return active_; }

    int ElapsedMs(const TimePoint now) const noexcept {
        if (!active_ || now < entered_at_) {
            return 0;
        }
        return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            now - entered_at_).count());
    }

private:
    bool active_{false};
    std::uint8_t goal_id_{0};
    Area::Point<std::uint16_t> goal_position_{};
    TimePoint entered_at_{};
};

}  // namespace BehaviorTree
