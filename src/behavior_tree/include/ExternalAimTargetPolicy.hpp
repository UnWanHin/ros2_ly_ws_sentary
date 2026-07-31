#pragma once

namespace BehaviorTree {

inline bool ShouldSelectFreshOutpostAimTarget(
    const bool outpost_target_fresh,
    const bool outpost_ignored) {
    return outpost_target_fresh && !outpost_ignored;
}

}  // namespace BehaviorTree
