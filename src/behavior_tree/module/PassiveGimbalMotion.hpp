#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include "BasicTypes.hpp"

namespace BehaviorTree {

// Applies only to passive fixed-target and patrol output. Visual aiming keeps
// its direct response path.
class PassiveGimbalMotion {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    LangYa::GimbalAnglesType Step(
        const LangYa::GimbalAnglesType& measured,
        const LangYa::GimbalAnglesType& desired,
        const TimePoint now,
        const double max_yaw_rate_deg_per_sec,
        const double max_pitch_rate_deg_per_sec,
        const int max_interval_ms = 25) noexcept {
        if (!initialized_) {
            initialized_ = true;
            last_command_ = measured;
            last_update_ = now;
            return last_command_;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now >= last_update_ ? now - last_update_ : Clock::duration::zero());
        const auto bounded_elapsed = std::min(
            elapsed,
            std::chrono::milliseconds(std::max(1, max_interval_ms)));
        const float seconds = static_cast<float>(bounded_elapsed.count()) / 1000.0f;
        const float yaw_limit = static_cast<float>(
            std::max(0.0, max_yaw_rate_deg_per_sec)) * seconds;
        const float pitch_limit = static_cast<float>(
            std::max(0.0, max_pitch_rate_deg_per_sec)) * seconds;

        const float desired_yaw = NormalizeNear(desired.Yaw, last_command_.Yaw);
        last_command_.Yaw += std::clamp(desired_yaw - last_command_.Yaw, -yaw_limit, yaw_limit);
        last_command_.Pitch += std::clamp(desired.Pitch - last_command_.Pitch, -pitch_limit, pitch_limit);
        last_update_ = now;
        return last_command_;
    }

    void Reset() noexcept { initialized_ = false; }

private:
    static float NormalizeNear(const float angle, const float reference) noexcept {
        return reference + static_cast<float>(std::remainder(angle - reference, 360.0f));
    }

    bool initialized_{false};
    LangYa::GimbalAnglesType last_command_{};
    TimePoint last_update_{};
};

}  // namespace BehaviorTree
