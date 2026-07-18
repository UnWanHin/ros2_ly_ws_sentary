// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <algorithm>

namespace {
    constexpr float kVelocityRawToMps = 0.025f;
    constexpr std::uint8_t kVisionModeDisabled = 0;
    constexpr std::uint8_t kVisionModeArmor = 1;
    constexpr std::uint8_t kVisionModeBuff = 2;
    constexpr std::uint8_t kVisionModeOutpost = 3;
    constexpr auto kUnitInfoFreshTimeout = std::chrono::seconds(30);

    const char* UnitAreaKindName(const BehaviorTree::Area::MainAreaKind kind) {
        switch (kind) {
            case BehaviorTree::Area::MainAreaKind::Base: return "base";
            case BehaviorTree::Area::MainAreaKind::Highland: return "highland";
            case BehaviorTree::Area::MainAreaKind::PreRoadland: return "pre_roadland";
            case BehaviorTree::Area::MainAreaKind::ReadyRoadland: return "ready_roadland";
            case BehaviorTree::Area::MainAreaKind::Central: return "central";
            default: return "unknown";
        }
    }

    std::uint8_t UnitAreaId(const BehaviorTree::AreaKey& key) {
        using Info = gimbal_driver::msg::UnitInfo;
        if (key.Side == BehaviorTree::AreaSide::Common &&
            key.Kind == BehaviorTree::Area::MainAreaKind::Central) {
            return Info::AREA_CENTRAL;
        }
        if (key.Side == BehaviorTree::AreaSide::My) {
            switch (key.Kind) {
                case BehaviorTree::Area::MainAreaKind::Base: return Info::AREA_MY_BASE;
                case BehaviorTree::Area::MainAreaKind::Highland: return Info::AREA_MY_HIGHLAND;
                case BehaviorTree::Area::MainAreaKind::PreRoadland: return Info::AREA_MY_PRE_ROADLAND;
                case BehaviorTree::Area::MainAreaKind::ReadyRoadland: return Info::AREA_MY_ROADLAND;
                default: return Info::AREA_UNKNOWN;
            }
        }
        if (key.Side == BehaviorTree::AreaSide::Enemy) {
            switch (key.Kind) {
                case BehaviorTree::Area::MainAreaKind::Base: return Info::AREA_ENEMY_BASE;
                case BehaviorTree::Area::MainAreaKind::Highland: return Info::AREA_ENEMY_HIGHLAND;
                case BehaviorTree::Area::MainAreaKind::PreRoadland: return Info::AREA_ENEMY_PRE_ROADLAND;
                case BehaviorTree::Area::MainAreaKind::ReadyRoadland: return Info::AREA_ENEMY_ROADLAND;
                default: return Info::AREA_UNKNOWN;
            }
        }
        return Info::AREA_UNKNOWN;
    }

    std::string UnitAreaName(const BehaviorTree::AreaKey& key) {
        if (key.Side == BehaviorTree::AreaSide::Common) {
            return "central";
        }
        std::string prefix;
        if (key.Side == BehaviorTree::AreaSide::My) {
            prefix = "my_";
        } else if (key.Side == BehaviorTree::AreaSide::Enemy) {
            prefix = "enemy_";
        } else {
            return "unknown";
        }
        return prefix + UnitAreaKindName(key.Kind);
    }

    std::uint8_t GoalBaseIdFromResolvedGoal(const std::uint8_t goal_id) noexcept {
        return goal_id >= LangYa::TeamedLocation::LocationCount
            ? static_cast<std::uint8_t>(goal_id - LangYa::TeamedLocation::LocationCount)
            : goal_id;
    }

    gimbal_driver::msg::FireCode MakeFireCodeMsg(const LangYa::FireCodeType& firecode, const rclcpp::Time& stamp) {
        gimbal_driver::msg::FireCode msg;
        msg.header.stamp = stamp;
        msg.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
        msg.fire_status = firecode.FireStatus;
        msg.cap_state = firecode.CapState;
        msg.follow_mode = firecode.FollowMode != 0;
        msg.aim_mode = firecode.AimMode != 0;
        msg.rotate = firecode.Rotate;
        msg.raw = *reinterpret_cast<const std::uint8_t*>(&firecode);
        return msg;
    }
}

namespace BehaviorTree {

    void Application::PublishMessageAll() {
        // 发布顺序固定：
        // 先模式/云台/姿态/目标，再发导航速度和导航目标，减少下游状态抖动。
        PubAimModeEnableData();
        PubGimbalControlData();
        PubPostureControlData();
        const auto now = std::chrono::steady_clock::now();
        const int referee_fresh_ms =
            std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs);
        const bool sentry_info_fresh =
            hasReceivedSentryInfo_ &&
            lastSentryInfoRxTime_.time_since_epoch().count() != 0 &&
            now - lastSentryInfoRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
        const bool event_data_fresh =
            hasReceivedEventData_ &&
            lastEventDataRxTime_.time_since_epoch().count() != 0 &&
            now - lastEventDataRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
        const bool energy_done_without_next =
            event_data_fresh &&
            (eventSelfSmallEnergyStatus_ == 1 || eventSelfLargeEnergyStatus_ == 1) &&
            !sentryCanActivateEnergyMechanism_;
        const bool should_confirm_energy_activate =
            aimMode == AimMode::Buff &&
            sentry_info_fresh &&
            sentryCanActivateEnergyMechanism_ &&
            !energy_done_without_next &&
            IsBaseGoalArrived(LangYa::BuffOutpost.ID, team, true) &&
            BuffAimFreshAndFireReady();
        UpdateEnergyActivateConfirmCommand(should_confirm_energy_activate);
        PubAimTargetData();
        PubNaviControlData();
        PubNaviReachState();
        PubSentryPosition();
        const bool enable_chase_to_navi =
            chaseTacticalAllowed_ &&
            config.ChaseSettings.Enable &&
            config.ChaseSettings.ToNavi;
        const bool chase_official_target_active =
            enable_chase_to_navi &&
            naviChaseOfficialTargetValid;
        const bool chase_relative_target_publish_active =
            enable_chase_to_navi &&
            !chase_official_target_active &&
            (naviRelativeTargetValid || config.ChaseSettings.StopWhenNoTarget);
        if (chase_relative_target_publish_active) {
            PubNaviRelativeTarget();
        }
        if(publishNaviGoal_ && naviCommandRateClock.trigger()) {
            naviCommandRateClock.tick();
            if (!naviGoalPublishAllowed_) {
                return;
            }
            const bool chase_bridge_active =
                config.NaviSettings.UseXY &&
                config.NaviSettings.ToNavi &&
                chase_relative_target_publish_active;
            // 导航目标按模式二选一：
            // - UseXY=true 时固定点位发布坐标；ToNavi=true 则走 /ly/navi/goal_pos_raw -> /goal_pose。
            // - Tactical 授权 Chase 且 target_rel 有效，或 StopWhenNoTarget 要求失靶停车时，
            //   /ly/navi/target_rel 交给 bridge 输出 /goal_pose，避免固定点位覆盖追击。
            // - 官方坐标追击源有效时，发布 /ly/navi/goal_pos_raw，避免和 target_rel 同 tick 双写 /goal_pose。
            if(chase_official_target_active) PubNaviGoalPos();
            else if(config.NaviSettings.UseXY && !chase_bridge_active) PubNaviGoalPos();
            else PubNaviGoal();
        }
    }

    void Application::PubNaviReachState() {
        if (!pub_navi_reach_state_) {
            return;
        }

        const auto base_goal_id = GoalBaseIdFromResolvedGoal(naviCommandGoal);
        const auto state = EvaluateNaviGoalReach(
            naviCommandGoal,
            naviGoalPosition,
            std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm),
            0,
            base_goal_id,
            GoalReachTimeoutSecForBaseGoal(base_goal_id));

        auto_aim_common::msg::GoalReach msg;
        msg.header.stamp = node_ ? node_->now() : rclcpp::Time{};
        msg.header.frame_id = "map";
        msg.status = static_cast<std::uint8_t>(state.Status);
        msg.reason = static_cast<std::uint8_t>(state.Reason);
        msg.goal_id = state.GoalId;
        msg.base_goal_id = state.BaseGoalId;
        msg.goal_x_cm = state.GoalPosition.x;
        msg.goal_y_cm = state.GoalPosition.y;
        msg.goal_start_stamp = state.GoalStartStamp;
        msg.goal_age_ms = state.GoalAgeMs;
        msg.external_reach_fresh = state.ExternalReach.has_value();
        msg.external_reach = state.ExternalReach.value_or(false);
        msg.external_reachable_fresh = state.ExternalReachable.has_value();
        msg.external_reachable = state.ExternalReachable.value_or(true);
        msg.position_fresh = state.PositionFresh;
        msg.has_position = state.HasPosition;
        msg.self_x_cm = state.SelfX;
        msg.self_y_cm = state.SelfY;
        msg.distance_cm = static_cast<float>(state.DistanceCm);
        msg.arrive_distance_cm = static_cast<std::uint16_t>(
            std::clamp(state.ArriveDistanceCm, 0, 65535));
        msg.face_distance_cm = static_cast<std::uint16_t>(
            std::clamp(state.FaceDistanceCm, 0, 65535));
        msg.distance_fallback_allowed = state.DistanceFallbackAllowed;
        msg.within_arrive_distance = state.WithinArriveDistance;
        msg.within_face_distance = state.WithinFaceDistance;
        msg.timeout = state.Timeout;
        pub_navi_reach_state_->publish(msg);
    }

    gimbal_driver::msg::UnitInfoArray Application::MakeFriendInfoMsg() {
        const auto now = std::chrono::steady_clock::now();
        const auto enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const int info_position_fresh_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                kUnitInfoFreshTimeout).count());
        gimbal_driver::msg::UnitInfoArray msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "official_map";
        msg.units.reserve(RobotLists.size());
        for (const auto unit_type : RobotLists) {
            const auto index = static_cast<std::size_t>(unit_type);
            if (index >= lastFriendPositionRxTime_.size() ||
                index >= lastFriendHealthRxTime_.size()) {
                continue;
            }
            const auto& robot = friendRobots[unit_type];
            const bool has_hp =
                lastFriendHealthRxTime_[index].time_since_epoch().count() != 0;
            const auto position = GetFriendPositionState(
                unit_type,
                info_position_fresh_ms,
                now);

            gimbal_driver::msg::UnitInfo unit;
            unit.car_id = static_cast<std::uint8_t>(unit_type);
            unit.hp = robot.currentHealth_;
            unit.has_hp = has_hp;
            unit.hp_fresh = has_hp && now - lastFriendHealthRxTime_[index] <= kUnitInfoFreshTimeout;
            unit.hp_stamp = lastFriendHealthStamp_[index].Stamp;
            unit.position_x = static_cast<std::int16_t>(position.X);
            unit.position_y = static_cast<std::int16_t>(position.Y);
            unit.has_position = position.HasPosition;
            unit.position_fresh = position.Fresh;
            unit.position_stamp = position.Stamp;
            unit.position_source = position.Source;
            unit.area_id = gimbal_driver::msg::UnitInfo::AREA_UNKNOWN;
            unit.area_name = "unknown";
            unit.area_used_nearest_fallback = false;
            if (position.HasPosition) {
                const auto area = AreaManager::ResolveAreaKeyForPointWithNearest(
                    team,
                    enemy_team,
                    unit.position_x,
                    unit.position_y);
                if (area.has_value()) {
                    unit.area_id = UnitAreaId(area->Key);
                    unit.area_name = UnitAreaName(area->Key);
                    unit.area_used_nearest_fallback = area->UsedNearestFallback;
                }
            }
            msg.units.push_back(std::move(unit));
        }
        return msg;
    }

    void Application::PubFriendInfo() {
        if (!pub_friend_info_) {
            return;
        }
        pub_friend_info_->publish(MakeFriendInfoMsg());
    }

    gimbal_driver::msg::UnitInfoArray Application::MakeEnemyInfoMsg() {
        const auto now = std::chrono::steady_clock::now();
        const auto enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const int info_position_fresh_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                kUnitInfoFreshTimeout).count());
        gimbal_driver::msg::UnitInfoArray msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "official_map";
        msg.units.reserve(RobotLists.size());
        for (const auto unit_type : RobotLists) {
            const auto index = static_cast<std::size_t>(unit_type);
            if (index >= lastEnemyPositionRxTime_.size() ||
                index >= lastEnemyHealthRxTime_.size()) {
                continue;
            }
            const auto& robot = enemyRobots[unit_type];
            const bool has_hp =
                lastEnemyHealthRxTime_[index].time_since_epoch().count() != 0;
            const auto position = GetEnemyPositionState(
                unit_type,
                info_position_fresh_ms,
                now);

            gimbal_driver::msg::UnitInfo unit;
            unit.car_id = static_cast<std::uint8_t>(unit_type);
            unit.hp = robot.currentHealth_;
            unit.has_hp = has_hp;
            unit.hp_fresh = has_hp && now - lastEnemyHealthRxTime_[index] <= kUnitInfoFreshTimeout;
            unit.hp_stamp = lastEnemyHealthStamp_[index].Stamp;
            unit.position_x = static_cast<std::int16_t>(position.X);
            unit.position_y = static_cast<std::int16_t>(position.Y);
            unit.has_position = position.HasPosition;
            unit.position_fresh = position.Fresh;
            unit.position_stamp = position.Stamp;
            unit.position_source = position.Source;
            unit.area_id = gimbal_driver::msg::UnitInfo::AREA_UNKNOWN;
            unit.area_name = "unknown";
            unit.area_used_nearest_fallback = false;
            if (position.HasPosition) {
                const auto area = AreaManager::ResolveAreaKeyForPointWithNearest(
                    team,
                    enemy_team,
                    unit.position_x,
                    unit.position_y);
                if (area.has_value()) {
                    unit.area_id = UnitAreaId(area->Key);
                    unit.area_name = UnitAreaName(area->Key);
                    unit.area_used_nearest_fallback = area->UsedNearestFallback;
                }
            }
            msg.units.push_back(std::move(unit));
        }
        return msg;
    }

    void Application::PubEnemyInfo() {
        if (!pub_enemy_info_) {
            return;
        }
        pub_enemy_info_->publish(MakeEnemyInfoMsg());
    }

    void Application::PubAimModeEnableData() {
        const auto vision_mode = [&]() -> std::uint8_t {
            switch (aimMode) {
                case AimMode::AutoAim:
                case AimMode::RotateScan:
                    return kVisionModeArmor;
                case AimMode::Buff:
                    return kVisionModeBuff;
                case AimMode::Outpost:
                    return kVisionModeOutpost;
                default:
                    return kVisionModeDisabled;
            }
        }();
        {
            std_msgs::msg::UInt8 msg;
            msg.data = vision_mode;
            pub_vision_mode_->publish(msg);
        }
    }

    /**
     * @brief 发布云台角度控制数据和火控数据 \n
     * @brief 云台数据可以随便修改，修改完之后发布即可，火控数据只有在接收到辐瞄的目标数据之后才会自动翻转
     * @param gimbalControlData 云台控制数据
     */
    void Application::PubGimbalControlData() {
        {
            gimbal_driver::msg::GimbalAngles msg;
            msg.yaw   = gimbalControlData.GimbalAngles.Yaw;
            msg.pitch = gimbalControlData.GimbalAngles.Pitch;
            msg.header.stamp = node_->now();
            pub_gimbal_control_->publish(msg);
        }
        {
            auto msg = MakeFireCodeMsg(gimbalControlData.FireCode, node_->now());
            pub_gimbal_firecode_->publish(msg);
        }
    }

    void Application::PubPostureControlData() {
        // 0 作为“当前决策层不下发姿态”的保留值，避免影响现有链路。
        if (postureCommand < 1 || postureCommand > 6) {
            return;
        }
        gimbal_driver::msg::SentryCmd msg;
        msg.header.stamp = node_->now();
        msg.field_mask = gimbal_driver::msg::SentryCmd::FIELD_POSTURE;
        msg.posture = postureCommand;
        msg.raw = static_cast<std::uint32_t>(postureCommand) << 21;
        pub_control_posture_->publish(msg);
    }

    void Application::PubEnergyActivateConfirmData(const bool confirm) {
        if (!pub_control_sentry_cmd_) {
            return;
        }
        gimbal_driver::msg::SentryCmd msg;
        msg.header.stamp = node_->now();
        msg.field_mask = gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_ENERGY_ACTIVATE;
        msg.confirm_energy_activate = confirm;
        msg.raw = confirm ? (1u << 23) : 0u;
        pub_control_sentry_cmd_->publish(msg);
    }

    void Application::UpdateEnergyActivateConfirmCommand(const bool should_confirm) {
        const auto now = std::chrono::steady_clock::now();
        if (energyActivateConfirmPulseActive_ && now >= energyActivateConfirmPulseUntil_) {
            PubEnergyActivateConfirmData(false);
            energyActivateConfirmPulseActive_ = false;
        }

        if (!should_confirm || energyActivateConfirmPulseActive_) {
            return;
        }
        if (nextEnergyActivateConfirmTime_.time_since_epoch().count() != 0 &&
            now < nextEnergyActivateConfirmTime_) {
            return;
        }

        const int pulse_ms = std::max(0, config.TaskSettings.BuffConfirm.PulseMs);
        const int retry_ms = std::max(
            pulse_ms,
            std::max(0, config.TaskSettings.BuffConfirm.RetryIntervalMs));
        PubEnergyActivateConfirmData(true);
        lastEnergyActivateConfirmTime_ = now;
        energyActivateConfirmPulseActive_ = true;
        energyActivateConfirmPulseUntil_ = now + std::chrono::milliseconds(pulse_ms);
        nextEnergyActivateConfirmTime_ = now + std::chrono::milliseconds(retry_ms);
        LoggerPtr->Info(
            "SentryCmd confirm_energy_activate pulse: pulse_ms={} retry_ms={} buff_goal_arrived=1 visual_locked=1",
            pulse_ms,
            retry_ms);
    }

    /**
     * @brief 发布自瞄应该击打的目标
     */
    void Application::PubAimTargetData() {
        {
            std_msgs::msg::UInt8 msg;
            msg.data = static_cast<uint8_t>(targetArmor.Type);
            pub_bt_target_->publish(msg);
        }
        PubExternalAimTargetData();
    }

    void Application::PubExternalAimTargetData() {
#ifdef LY_ENABLE_SENTRY_MSGS
        if (!config.ExternalAimSettings.Enable ||
            !config.ExternalAimSettings.PublishSelectTarget ||
            !pub_external_aim_select_target_) {
            return;
        }
        const auto target_id = static_cast<std::uint8_t>(targetArmor.Type);
        sentry_msgs::msg::AimTarget msg;
        msg.header.stamp = node_->now();
        msg.id = target_id;

        const auto target_index = static_cast<std::size_t>(target_id);
        const auto now = std::chrono::steady_clock::now();
        const int fresh_ms = std::max(1, config.ExternalAimSettings.TargetFreshTimeoutMs);
        if (target_index < externalAimTargets_.size()) {
            const auto& cached = externalAimTargets_[target_index];
            const bool fresh =
                cached.Valid &&
                cached.LastSeen.time_since_epoch().count() != 0 &&
                now - cached.LastSeen <= std::chrono::milliseconds(fresh_ms);
            if (fresh) {
                msg.header.frame_id = cached.FrameId;
                msg.position.x = cached.X;
                msg.position.y = cached.Y;
                msg.position.z = cached.Z;
            }
        }
        pub_external_aim_select_target_->publish(msg);
#endif
    }

    /**
     * @brief 发布导航的底盘速度控制数据
     * @param naviVelocity 导航速度X, Y
     */
    void Application::PubNaviControlData() {
        {
            gimbal_driver::msg::ControlVelocity msg;
            msg.header.stamp = node_->now();
            msg.x_mps = static_cast<float>(naviVelocity.X) * kVelocityRawToMps;
            msg.y_mps = static_cast<float>(naviVelocity.Y) * kVelocityRawToMps;
            msg.raw_x = naviVelocity.X;
            msg.raw_y = naviVelocity.Y;
            msg.use_raw = true;
            // 桥接到 gimbal_driver 控制口，恢复 navi->BT->control_vel 老链路。
            pub_gimbal_vel_->publish(msg);
            // 兼容保留：继续发布到 /ly/navi/vel，避免影响外部联调工具。
            //pub_navi_vel_->publish(msg);
        }
    }

    void Application::PubNaviRelativeTarget() {
        if (!pub_navi_target_rel_) {
            return;
        }
        auto_aim_common::msg::RelativeTarget msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = naviRelativeTargetFrameId;
        msg.valid = naviRelativeTargetValid;
        msg.x = naviRelativeTargetX;
        msg.y = naviRelativeTargetY;
        msg.z = naviRelativeTargetZ;
        msg.distance_m = naviRelativeTargetDistance;
        msg.yaw_error_deg = naviRelativeTargetYawErrorDeg;
        msg.pitch_error_deg = naviRelativeTargetPitchErrorDeg;
        msg.armor_type = naviRelativeTargetArmorType;
        msg.aim_mode = naviRelativeTargetAimMode;
        pub_navi_target_rel_->publish(msg);
    }

    /**
     * @brief 发布给导航的目标点
     */
    void Application::PubNaviGoal() {
        {
            std_msgs::msg::UInt8 msg;
            msg.data = naviCommandGoal;
            // 语义：导航目标点 ID（不是坐标）
            pub_navi_goal_->publish(msg);
            UpdateNaviExternalStatusGoal(naviCommandGoal, naviGoalPosition);
        }
        {
            std_msgs::msg::UInt8 msg;
            msg.data = speedLevel;
            pub_navi_speed_level_->publish(msg);
        }
    }

    void Application::PubNaviGoalPos() {
        if (config.TaskSettings.OutpostConfirm.ManualGoalEnable &&
            naviCommandGoal == ResolveGoalId(LangYa::BuffOutpost.ID, team, true) &&
            PubManualOutpostGoalPose("manual_outpost_goal")) {
            return;
        }
        std_msgs::msg::UInt16MultiArray msg;
        std::vector<uint16_t> data = {
            static_cast<uint16_t>(naviGoalPosition.x),
            static_cast<uint16_t>(naviGoalPosition.y)
        };
        msg.data = data;
        if (config.NaviSettings.ToNavi && pub_navi_goal_pos_raw_) {
            // 统一由 navi_tf_bridge 输出 /goal_pose，BT 仅发布 raw 点位输入。
            pub_navi_goal_pos_raw_->publish(msg);
            UpdateNaviExternalStatusGoal(naviCommandGoal, naviGoalPosition);
            return;
        }
        pub_navi_goal_pos_->publish(msg);
        UpdateNaviExternalStatusGoal(naviCommandGoal, naviGoalPosition);
    }

    bool Application::PubManualOutpostGoalPose(const char* reason) {
        if (!pub_navi_goal_pose_ || !config.TaskSettings.OutpostConfirm.ManualGoalEnable) {
            return false;
        }

        const auto goal_id = ResolveGoalId(LangYa::BuffOutpost.ID, team, true);
        naviCommandGoal = goal_id;
        naviGoalPosition = AreaManager::GoalPointByBaseId(LangYa::BuffOutpost.ID, team);

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = config.TaskSettings.OutpostConfirm.ManualGoalMapXM;
        msg.pose.position.y = config.TaskSettings.OutpostConfirm.ManualGoalMapYM;
        msg.pose.position.z = config.TaskSettings.OutpostConfirm.ManualGoalMapZM;
        msg.pose.orientation.w = 1.0;
        pub_navi_goal_pose_->publish(msg);
        UpdateNaviExternalStatusGoal(naviCommandGoal, naviGoalPosition);

        static auto last_manual_goal_log = std::chrono::steady_clock::time_point{};
        const auto now = std::chrono::steady_clock::now();
        if (now - last_manual_goal_log > std::chrono::seconds(2)) {
            LoggerPtr->Info(
                "Manual Outpost /goal_pose: ({:.3f}, {:.3f}, {:.3f})m frame=map reason={}",
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                reason ? reason : "");
            last_manual_goal_log = now;
        }
        return true;
    }

    void Application::PubSentryPosition() {
        if (!pub_sentry_position_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto self_position = GetSentryPositionState(now);
        if (!self_position.HasPosition || !self_position.Fresh) {
            return;
        }

        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = node_ ? node_->now() : rclcpp::Time{};
        msg.header.frame_id = "map";
        msg.point.x = static_cast<double>(self_position.X) / 100.0;
        msg.point.y = static_cast<double>(self_position.Y) / 100.0;
        msg.point.z = 0.0;
        pub_sentry_position_->publish(msg);
    }
}
