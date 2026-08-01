#pragma once

#include <optional>
#include <utility>
#include <vector>

namespace BehaviorTree {

// A temporary override is intentionally opt-in so the profile JSON remains the
// source of truth for normal competition launches.
struct AimConfigOverride {
    bool Enabled{false};
    std::optional<std::vector<int>> TargetPriority;
    std::optional<std::vector<int>> TargetIgnore;
};

inline void ApplyAimConfigOverride(
    std::vector<int>& target_priority,
    std::vector<int>& target_ignore,
    const AimConfigOverride& override_config) {
    if (!override_config.Enabled) {
        return;
    }
    if (override_config.TargetPriority.has_value()) {
        target_priority = *override_config.TargetPriority;
    }
    if (override_config.TargetIgnore.has_value()) {
        target_ignore = *override_config.TargetIgnore;
    }
}

}  // namespace BehaviorTree
