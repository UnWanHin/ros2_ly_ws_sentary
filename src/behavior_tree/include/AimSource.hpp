#pragma once

#include <chrono>
#include <optional>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

struct AimSourceView {
    const LangYa::AimData* Active{nullptr};
    const LangYa::AimData* AutoAim{nullptr};
    const LangYa::AimData* Buff{nullptr};
    const LangYa::AimData* Outpost{nullptr};
    bool ExternalAimActive{false};
};

inline const LangYa::AimData& SelectActiveAimData(
    const bool external_aim_active,
    const LangYa::AimMode aim_mode,
    const LangYa::AimData& auto_aim,
    const LangYa::AimData& external_aim,
    const LangYa::AimData& buff_aim,
    const LangYa::AimData& outpost_aim) noexcept {
    if (external_aim_active) {
        return external_aim;
    }
    if (aim_mode == LangYa::AimMode::Buff) {
        return buff_aim;
    }
    if (aim_mode == LangYa::AimMode::Outpost) {
        return outpost_aim;
    }
    return auto_aim;
}

inline AimSourceView MakeAimSourceView(
    const bool external_aim_active,
    const LangYa::AimMode aim_mode,
    const LangYa::AimData& auto_aim,
    const LangYa::AimData& external_aim,
    const LangYa::AimData& buff_aim,
    const LangYa::AimData& outpost_aim) noexcept {
    const LangYa::AimData& active = SelectActiveAimData(
        external_aim_active,
        aim_mode,
        auto_aim,
        external_aim,
        buff_aim,
        outpost_aim);
    return AimSourceView{
        .Active = &active,
        .AutoAim = external_aim_active ? &external_aim : &auto_aim,
        .Buff = external_aim_active ? &external_aim : &buff_aim,
        .Outpost = external_aim_active ? &external_aim : &outpost_aim,
        .ExternalAimActive = external_aim_active,
    };
}

inline bool AimFreshAndValid(const LangYa::AimData& data) noexcept {
    return data.Fresh && data.Valid;
}

inline bool AimBuffTargetLocked(
    const LangYa::AimData& data,
    const bool external_aim_active) noexcept {
    return AimFreshAndValid(data) && (external_aim_active || data.BuffFollow);
}

inline bool AimBuffFireReady(
    const LangYa::AimData& data,
    const bool external_aim_active) noexcept {
    return AimBuffTargetLocked(data, external_aim_active) && data.FireStatus;
}

inline bool AimLatchedRecently(
    const LangYa::AimData& data,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds hold) noexcept {
    if (!data.HasLatchedAngles ||
        data.LastValidTime.time_since_epoch().count() == 0 ||
        hold.count() <= 0) {
        return false;
    }
    return now - data.LastValidTime <= hold;
}

inline bool AimFreshOrLatchedRecently(
    const LangYa::AimData& data,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds hold) noexcept {
    return AimFreshAndValid(data) || AimLatchedRecently(data, now, hold);
}

inline bool AimTargetForAngles(
    const LangYa::AimData& data,
    const bool callback_seen,
    const bool reuse_latched,
    const std::chrono::steady_clock::time_point now,
    const std::chrono::milliseconds hold,
    bool* fresh_target = nullptr,
    bool* latched_target = nullptr) noexcept {
    const bool fresh = callback_seen && AimFreshAndValid(data);
    const bool latched = !fresh && reuse_latched && AimLatchedRecently(data, now, hold);
    if (fresh_target != nullptr) {
        *fresh_target = fresh;
    }
    if (latched_target != nullptr) {
        *latched_target = latched;
    }
    return fresh || latched;
}

}  // namespace BehaviorTree
