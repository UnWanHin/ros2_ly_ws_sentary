#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

struct MapCommandInput {
    bool HasTargetPosition{false};
    float XMeter{0.0F};
    float YMeter{0.0F};
};

struct MapCommandRawGoal {
    std::uint16_t XCentimeter{0};
    std::uint16_t YCentimeter{0};
};

class MapCommandTask {
public:
    bool Observe(
        const MapCommandInput& input,
        const LangYa::MapCommandSetting& setting,
        const std::chrono::steady_clock::time_point now) noexcept {
        if (!setting.Enable || !IsValidCoordinate(input)) {
            return false;
        }

        const auto goal = ToRawGoal(input);
        if (!goal.has_value()) {
            return false;
        }

        if (last_command_.has_value() &&
            WithinDedupDistance(*last_command_, *goal, setting.DedupDistanceCm)) {
            return false;
        }

        last_command_ = *goal;
        active_goal_ = *goal;
        hold_until_ = now + std::chrono::seconds(std::max(1, setting.HoldSec));
        return true;
    }

    bool Active(const std::chrono::steady_clock::time_point now) const noexcept {
        return active_goal_.has_value() && now < hold_until_;
    }

    std::optional<MapCommandRawGoal> ActiveGoal(
        const std::chrono::steady_clock::time_point now) const noexcept {
        return Active(now) ? active_goal_ : std::nullopt;
    }

    void Cancel() noexcept {
        active_goal_.reset();
        hold_until_ = {};
    }

private:
    static bool IsValidCoordinate(const MapCommandInput& input) noexcept {
        return input.HasTargetPosition &&
            std::isfinite(input.XMeter) &&
            std::isfinite(input.YMeter) &&
            (input.XMeter != 0.0F || input.YMeter != 0.0F);
    }

    static std::optional<MapCommandRawGoal> ToRawGoal(const MapCommandInput& input) noexcept {
        constexpr double kCentimetersPerMeter = 100.0;
        const double x_cm = static_cast<double>(input.XMeter) * kCentimetersPerMeter;
        const double y_cm = static_cast<double>(input.YMeter) * kCentimetersPerMeter;
        if (x_cm < 0.0 || y_cm < 0.0 ||
            x_cm > static_cast<double>(std::numeric_limits<std::uint16_t>::max()) ||
            y_cm > static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
            return std::nullopt;
        }
        return MapCommandRawGoal{
            static_cast<std::uint16_t>(std::lround(x_cm)),
            static_cast<std::uint16_t>(std::lround(y_cm))};
    }

    static bool WithinDedupDistance(
        const MapCommandRawGoal& lhs,
        const MapCommandRawGoal& rhs,
        const int dedup_distance_cm) noexcept {
        const long dx = static_cast<long>(lhs.XCentimeter) - static_cast<long>(rhs.XCentimeter);
        const long dy = static_cast<long>(lhs.YCentimeter) - static_cast<long>(rhs.YCentimeter);
        const long distance = std::max(0, dedup_distance_cm);
        return dx * dx + dy * dy <= distance * distance;
    }

    std::optional<MapCommandRawGoal> last_command_{};
    std::optional<MapCommandRawGoal> active_goal_{};
    std::chrono::steady_clock::time_point hold_until_{};
};

}  // namespace BehaviorTree
