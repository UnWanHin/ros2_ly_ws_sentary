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

    enum class Source : std::uint8_t {
        None = 0,
        Regional = 1,
        Buff = 2,
        Outpost = 3,
        StartGate = 4,
    };

    struct ControlState {
        bool Active{false};
        bool UseFaceMode{false};
        RegionalAreaTaskPhase Phase{RegionalAreaTaskPhase::Idle};
        Source RequestSource{Source::None};
        std::uint8_t Priority{0};
    };

    // PublishTogether() consumes this single resolved result. Tasks may submit
    // requests, but never directly decide final gimbal-angle/firecode output.
    struct Decision {
        bool Requested{false};
        bool Active{false};
        bool UsePatrolFallback{false};
        bool SuppressFire{false};
        Source RequestSource{Source::None};
        RegionalAreaTaskPhase Phase{RegionalAreaTaskPhase::Idle};
        std::optional<LangYa::GimbalAnglesType> Angles{};
    };

    // Start a request collection cycle before evaluating BT tasks.
    void BeginCycle() noexcept;
    const ControlState& Control() const noexcept { return control_; }

    void CacheAngles(
        LangYa::AimData& data,
        float yaw_deg,
        float pitch_deg,
        AreaTimePoint now) const noexcept;

    Decision Resolve(
        const LangYa::AimData& data,
        const LangYa::FaceModeSetting& setting,
        const LangYa::PatrolScanSetting& patrol_scan,
        bool visual_target_has_priority,
        bool navigation_release_request,
        bool clear_regional_when_navigation_releases,
        AreaTimePoint now) const noexcept;

    bool RequestAimTarget(
        LangYa::AimMode aim_mode,
        LangYa::UnitTeam target_team,
        const TargetPublisher::SharedPtr& publisher);

    // Opening-only fixed target request. It shares the normal FaceMode output
    // arbitration and stale-angle fallback instead of publishing gimbal control directly.
    bool RequestStartGateOutpost(
        LangYa::UnitTeam enemy_team,
        const TargetPublisher::SharedPtr& publisher);

    static bool StartGateOutpostSolutionReady(
        bool status_fresh,
        bool solver_function,
        bool manual_target,
        std::uint32_t target_update_count,
        std::uint32_t target_update_count_floor,
        bool face_mode_angles_fresh) noexcept;

    bool RequestRegionalTask(
        const RegionalAreaTaskTickResult& result,
        const TargetPublisher::SharedPtr& publisher);

private:
    static constexpr std::uint8_t kRegionalPriority = 10;
    static constexpr std::uint8_t kAimTaskPriority = 20;
    static constexpr std::uint8_t kStartGatePriority = 5;

    bool RegisterRequest(
        Source source,
        bool active,
        bool use_face_mode,
        RegionalAreaTaskPhase phase,
        std::uint8_t priority) noexcept;
    bool Requested(const LangYa::FaceModeSetting& setting) const noexcept;
    std::optional<LangYa::GimbalAnglesType> SelectAngles(
        const LangYa::AimData& data,
        const LangYa::FaceModeSetting& setting,
        AreaTimePoint now) const noexcept;

    static std_msgs::msg::UInt16MultiArray BuildTargetMessage(const Area::Point3<double>& point);
    static std_msgs::msg::UInt16MultiArray BuildTargetMessage(
        const Area::Point<std::uint16_t>& point,
        int z_cm);
    static std::uint16_t ClampCmToU16(double value) noexcept;
    static std::uint16_t ClampCmToU16(int value) noexcept;

    ControlState control_{};
};

}  // namespace BehaviorTree
