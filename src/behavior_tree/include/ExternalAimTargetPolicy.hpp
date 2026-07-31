#pragma once

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

inline bool IsSelectableExternalAimArmor(
    const LangYa::ArmorType armor_type) noexcept {
    return armor_type >= LangYa::ArmorType::Base &&
        armor_type <= LangYa::ArmorType::Outpost;
}

inline bool ShouldSelectFreshExternalAimTarget(
    const LangYa::ArmorType armor_type,
    const bool target_fresh,
    const bool target_ignored) noexcept {
    return IsSelectableExternalAimArmor(armor_type) && target_fresh && !target_ignored;
}

inline bool ShouldSelectFreshOutpostAimTarget(
    const bool outpost_target_fresh,
    const bool outpost_ignored) noexcept {
    return ShouldSelectFreshExternalAimTarget(
        LangYa::ArmorType::Outpost, outpost_target_fresh, outpost_ignored);
}

inline bool ShouldUseVisibleExternalAimForAttackPosture(
    const bool selected_target_fresh) noexcept {
    return selected_target_fresh;
}

}  // namespace BehaviorTree
