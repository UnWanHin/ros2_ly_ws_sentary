// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "AreaManager.hpp"
#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

class FaceModeManager {
public:
    using TargetPublisher = rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>;

    struct ControlState {
        bool Active{false};
        bool UseFaceMode{false};
        RegionalAreaTaskPhase Phase{RegionalAreaTaskPhase::Idle};
    };

    void ResetControl() noexcept;
    void SetControl(bool active, bool use_face_mode, RegionalAreaTaskPhase phase) noexcept;
    const ControlState& Control() const noexcept { return control_; }

    bool Requested(const LangYa::FaceModeSetting& setting) const noexcept;
    bool Active(
        const LangYa::FaceModeSetting& setting,
        bool visual_target_has_priority) const noexcept;

    void CacheAngles(
        LangYa::AimData& data,
        float yaw_deg,
        float pitch_deg,
        AreaTimePoint now) const noexcept;

    std::optional<LangYa::GimbalAnglesType> SelectAngles(
        const LangYa::AimData& data,
        const LangYa::FaceModeSetting& setting,
        AreaTimePoint now) const noexcept;

    bool PublishAimTarget(
        LangYa::AimMode aim_mode,
        LangYa::UnitTeam target_team,
        const TargetPublisher::SharedPtr& publisher,
        const std::optional<Area::Point3<double>>& outpost_manual_target = std::nullopt);

    bool ApplyRegionalTaskResult(
        const RegionalAreaTaskTickResult& result,
        const TargetPublisher::SharedPtr& publisher);

private:
    static std_msgs::msg::UInt16MultiArray BuildTargetMessage(const Area::Point3<double>& point);
    static std_msgs::msg::UInt16MultiArray BuildTargetMessage(
        const Area::Point<std::uint16_t>& point,
        int z_cm);
    static std::uint16_t ClampCmToU16(double value) noexcept;
    static std::uint16_t ClampCmToU16(int value) noexcept;

    ControlState control_{};
};

}  // namespace BehaviorTree
