#pragma once

#include "BasicTypes.hpp"

namespace BehaviorTree {

inline bool ShouldAcceptNaviTargetOfficialFallback(
    const LangYa::ChaseSetting& setting,
    const bool has_fresh_position_data) {
    return setting.EnableNaviTargetOfficialFallback && !has_fresh_position_data;
}

}  // namespace BehaviorTree
