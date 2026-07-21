#pragma once

namespace BehaviorTree {

// A Tactical YAML value is authoritative only when the parameter was provided.
inline bool ResolveTacticalFeatureEnable(
    const bool profile_baseline,
    const bool yaml_provided,
    const bool yaml_enable) noexcept {
    return yaml_provided ? yaml_enable : profile_baseline;
}

inline bool IsProtectCastleRfidEventEnabled(
    const bool protect_castle_enable,
    const bool rfid_enable,
    const bool event_active) noexcept {
    return protect_castle_enable && rfid_enable && event_active;
}

inline bool IsProtectCastleRfidStayEnabled(
    const bool protect_castle_enable,
    const bool rfid_enable,
    const bool stay_when_rfid,
    const bool event_active) noexcept {
    return protect_castle_enable && rfid_enable && stay_when_rfid && event_active;
}

inline bool IsProtectCastleEnemyPositionEnabled(
    const bool protect_castle_enable,
    const bool enemy_position_enable) noexcept {
    return protect_castle_enable && enemy_position_enable;
}

}  // namespace BehaviorTree
