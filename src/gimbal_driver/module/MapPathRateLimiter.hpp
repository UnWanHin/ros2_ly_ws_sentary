#pragma once

#include <chrono>

namespace LangYa {

class MapPathRateLimiter {
public:
    using Clock = std::chrono::steady_clock;

    explicit MapPathRateLimiter(const std::chrono::milliseconds interval) noexcept
        : interval_(interval) {}

    bool TryAcquire(const Clock::time_point now) noexcept {
        if (now < next_allowed_time_) {
            return false;
        }
        next_allowed_time_ = now + interval_;
        return true;
    }

private:
    std::chrono::milliseconds interval_;
    Clock::time_point next_allowed_time_{Clock::time_point::min()};
};

}  // namespace LangYa
