#pragma once

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

inline bool IsOutpostOpeningHoldActive(
    const bool outpost_enabled,
    const LangYa::OutpostConfirmSetting& setting,
    const int elapsed_sec) noexcept {
    return outpost_enabled &&
        setting.OpeningHoldUntilWindowEnd &&
        setting.OpeningHoldSec > 0 &&
        elapsed_sec >= 0 &&
        elapsed_sec < setting.OpeningHoldSec;
}

inline bool IsOutpostOpeningPriorityActive(
    const bool outpost_enabled,
    const LangYa::OutpostConfirmSetting& setting,
    const int elapsed_sec) noexcept {
    const bool normal_opening_priority =
        setting.OpeningHighPriority &&
        setting.MaxGameTimeSec > 0 &&
        elapsed_sec >= 0 &&
        elapsed_sec < setting.MaxGameTimeSec;
    return outpost_enabled &&
        (normal_opening_priority ||
         IsOutpostOpeningHoldActive(outpost_enabled, setting, elapsed_sec));
}

}  // namespace BehaviorTree
