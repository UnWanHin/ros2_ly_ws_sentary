// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <optional>

using namespace LangYa;

namespace BehaviorTree {


    float normalize_angle_0_360(float angle) {
        float normalized = fmod(angle, 360.0f);
        if (normalized < 0)
            normalized += 360.0f;
        return normalized;
    }

    float normalize_angle_near(float angle, float reference) {
        return reference + static_cast<float>(std::remainder(angle - reference, 360.0f));
    }

    namespace {
    constexpr std::uint8_t kLeagueRouteCompatViaGoalBaseId = LangYa::LeftHighLand.ID;  // base goal id=4
    constexpr int kLeagueRouteCompatViaHoldSec = 5;
    constexpr int kOfficialFieldWidthCm = 2800;
    constexpr int kOfficialFieldHeightCm = 1500;
    constexpr auto kRfidFreshTimeout = std::chrono::milliseconds(1000);
    // 丢 1~2 帧时保留锁角，避免抖动；时间过长会让云台“粘住旧目标”。
    constexpr auto kLostTargetHold = std::chrono::milliseconds(200);

    std::optional<ArmorType> ArmorTypeFromPriorityId(const int armor_type_id) {
        switch (static_cast<ArmorType>(armor_type_id)) {
            case ArmorType::Hero:
            case ArmorType::Engineer:
            case ArmorType::Infantry1:
            case ArmorType::Infantry2:
            case ArmorType::Sentry:
                return static_cast<ArmorType>(armor_type_id);
            default:
                return std::nullopt;
        }
    }

    std::optional<UnitType> UnitTypeFromArmorType(const ArmorType armor_type) {
        switch (armor_type) {
            case ArmorType::Hero:
                return UnitType::Hero;
            case ArmorType::Engineer:
                return UnitType::Engineer;
            case ArmorType::Infantry1:
                return UnitType::Infantry1;
            case ArmorType::Infantry2:
                return UnitType::Infantry2;
            case ArmorType::Sentry:
                return UnitType::Sentry;
            default:
                return std::nullopt;
        }
    }

    std::optional<ArmorType> ArmorTypeFromUnitType(const UnitType unit_type) {
        switch (unit_type) {
            case UnitType::Hero:
                return ArmorType::Hero;
            case UnitType::Engineer:
                return ArmorType::Engineer;
            case UnitType::Infantry1:
                return ArmorType::Infantry1;
            case UnitType::Infantry2:
                return ArmorType::Infantry2;
            case UnitType::Sentry:
                return ArmorType::Sentry;
            default:
                return std::nullopt;
        }
    }

    bool IsIgnoredArmorType(const std::vector<int>& ignore_list, const ArmorType armor_type) {
        return std::find(ignore_list.begin(),
                         ignore_list.end(),
                         static_cast<int>(armor_type)) != ignore_list.end();
    }

    bool IsIgnoredUnitType(const std::vector<int>& ignore_list, const UnitType unit_type) {
        const auto armor_type = ArmorTypeFromUnitType(unit_type);
        return armor_type.has_value() && IsIgnoredArmorType(ignore_list, *armor_type);
    }

    int ClampToInt8(const int value) {
        return std::clamp(value, -128, 127);
    }

    bool IsOfficialFieldPointValid(const int x, const int y) {
        return x > 0 && y > 0 && x <= kOfficialFieldWidthCm && y <= kOfficialFieldHeightCm;
    }

    Area::Point<std::uint16_t> BuildOfficialChaseGoal(
        const int self_x,
        const int self_y,
        const int target_x,
        const int target_y,
        const int preferred_distance_cm,
        const int distance_deadband_cm) {
        int goal_x = self_x;
        int goal_y = self_y;
        const double dx = static_cast<double>(target_x - self_x);
        const double dy = static_cast<double>(target_y - self_y);
        const double distance = std::hypot(dx, dy);
        const double preferred = static_cast<double>(std::max(0, preferred_distance_cm));
        const double deadband = static_cast<double>(std::max(0, distance_deadband_cm));

        if (distance > 1e-6 && std::abs(distance - preferred) > deadband && distance > preferred) {
            const double scale = (distance - preferred) / distance;
            goal_x = static_cast<int>(std::lround(static_cast<double>(self_x) + dx * scale));
            goal_y = static_cast<int>(std::lround(static_cast<double>(self_y) + dy * scale));
        }

        goal_x = std::clamp(goal_x, 0, kOfficialFieldWidthCm);
        goal_y = std::clamp(goal_y, 0, kOfficialFieldHeightCm);
        return Area::Point<std::uint16_t>{
            static_cast<std::uint16_t>(goal_x),
            static_cast<std::uint16_t>(goal_y)};
    }

    std::string NormalizeDecisionModule(std::string_view module) {
        std::string normalized(module);
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
            [](unsigned char c) {
                if (c == '-' || c == ' ') {
                    return '_';
                }
                return static_cast<char>(std::tolower(c));
            });
        return normalized;
    }

    bool HasAutonomyToken(const std::vector<std::string>& tokens, std::string_view expected) {
        const auto normalized_expected = NormalizeDecisionModule(expected);
        return std::find(tokens.begin(), tokens.end(), normalized_expected) != tokens.end();
    }

    const char* RegionalDefenseSearchKindToString(const RegionalDefenseSearchKind kind) {
        switch (kind) {
            case RegionalDefenseSearchKind::OwnBase: return "own_base";
            case RegionalDefenseSearchKind::OwnHighland: return "own_highland";
            case RegionalDefenseSearchKind::OwnRoadland: return "own_roadland";
            case RegionalDefenseSearchKind::OwnHighlandRoadland: return "own_highland_roadland";
            case RegionalDefenseSearchKind::CommonCentral: return "common_central";
            case RegionalDefenseSearchKind::EnemySideSoft: return "enemy_side_soft";
            default: return "none";
        }
    }

    }  // namespace

     /**
     * @brief 更新黑板数据 \n
     * @brief  更新数据从上到下依次是：我方颜色，敌方哨站血量，我方哨站血量，剩余弹药，比赛剩余时间 \n
     * @brief  自身血量，己方英雄血量，己方3号步兵血量，视野中的装甲板序列，是否找到目标 \n
     */
    void Application::UpdateBlackBoard() {

        std::uint16_t SelfHealth = myselfHealth;
        ResetRegionalAreaControlOverride();
        // 三路目标源统一折叠成一个 IsFindTarget，供 BT 和姿态模块复用。
        // 注意这里是“本拍是否有新鲜目标”，不是长期跟踪状态。
        const bool has_auto_target = autoAimData.Fresh && autoAimData.Valid;
        const bool has_buff_target = buffAimData.Fresh && buffAimData.Valid && buffAimData.BuffFollow;
        const bool has_outpost_target = outpostAimData.Fresh && outpostAimData.Valid;
        const bool IsFindTarget = has_auto_target || has_buff_target || has_outpost_target;
        const auto now = std::chrono::steady_clock::now();
        rfidMatchState.Fresh =
            hasReceivedRfidStatus_ &&
            lastRfidStatusRxTime_.time_since_epoch().count() != 0 &&
            now - lastRfidStatusRxTime_ <= kRfidFreshTimeout;
        const auto enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const bool self_position_fresh =
            hasReceivedSentryPosition_ &&
            lastSentryPositionRxTime_.time_since_epoch().count() != 0 &&
            now - lastSentryPositionRxTime_ <= std::chrono::seconds(2);
        areaManager_.TickSelfArea(
            now,
            self_position_fresh,
            static_cast<int>(friendRobots[UnitType::Sentry].position_.X),
            static_cast<int>(friendRobots[UnitType::Sentry].position_.Y),
            team,
            enemy_team);

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }

        // 将数据写入黑板
        GlobalBlackboard_->set<UnitTeam>("MyTeam", team);
        GlobalBlackboard_->set<std::uint16_t>("TimeLeft", timeLeft);
        GlobalBlackboard_->set<std::uint16_t>("SelfHealth", SelfHealth);
        GlobalBlackboard_->set<std::uint16_t>("AmmoLeft", ammoLeft);
        GlobalBlackboard_->set<Robots>("FriendRobots", friendRobots);
        GlobalBlackboard_->set<Robots>("EnemyRobots", enemyRobots);
        GlobalBlackboard_->set<std::uint16_t>("EnemyOutpostHealth", enemyOutpostHealth);
        GlobalBlackboard_->set<std::uint16_t>("SelfOutpostHealth", selfOutpostHealth);
        GlobalBlackboard_->set<std::uint16_t>("SelfBaseHealth", selfBaseHealth);
        GlobalBlackboard_->set<std::uint16_t>("EnemyBaseHealth", enemyBaseHealth);
        GlobalBlackboard_->set<std::uint32_t>("RfidStatus", rfidStatus);
        GlobalBlackboard_->set<bool>("HasRfidStatus2", hasRfidStatus2);
        GlobalBlackboard_->set<std::uint8_t>("RfidStatus2", rfidStatus2);
        GlobalBlackboard_->set<RfidMatchState>("RfidMatch", rfidMatchState);
        GlobalBlackboard_->set<bool>("RfidFresh", rfidMatchState.Fresh);
        GlobalBlackboard_->set<bool>("RfidAny", rfidMatchState.Fresh && rfidMatchState.Any);
        GlobalBlackboard_->set<bool>("RfidSelfSupply", rfidMatchState.Fresh && rfidMatchState.SelfSupply);
        GlobalBlackboard_->set<bool>("RfidSelfBaseGainPoint", rfidMatchState.Fresh && rfidMatchState.SelfBaseGainPoint);
        GlobalBlackboard_->set<bool>("RfidSelfHighlandGainPoint", rfidMatchState.Fresh && rfidMatchState.SelfHighlandGainPoint);
        GlobalBlackboard_->set<bool>("RfidEnemyHighlandGainPoint", rfidMatchState.Fresh && rfidMatchState.EnemyHighlandGainPoint);
        GlobalBlackboard_->set<bool>("RfidSelfRoadCrossing", rfidMatchState.Fresh && rfidMatchState.SelfRoadCrossing);
        GlobalBlackboard_->set<bool>("RfidEnemyRoadCrossing", rfidMatchState.Fresh && rfidMatchState.EnemyRoadCrossing);
        GlobalBlackboard_->set<bool>("RfidSelfTunnel", rfidMatchState.Fresh && rfidMatchState.SelfTunnel);
        GlobalBlackboard_->set<bool>("RfidEnemyTunnel", rfidMatchState.Fresh && rfidMatchState.EnemyTunnel);
        GlobalBlackboard_->set<bool>("RfidTunnel", rfidMatchState.Fresh && rfidMatchState.Tunnel);
        GlobalBlackboard_->set<bool>("RfidCenterGainPoint", rfidMatchState.Fresh && rfidMatchState.CenterGainPoint);
        GlobalBlackboard_->set<bool>("RfidOnSelfSide", rfidMatchState.Fresh && rfidMatchState.OnSelfSideRfid);
        GlobalBlackboard_->set<bool>("RfidOnEnemySide", rfidMatchState.Fresh && rfidMatchState.OnEnemySideRfid);
        GlobalBlackboard_->set<std::uint32_t>("ExtEventData", extEventData);
        GlobalBlackboard_->set<bool>("HasEventData", hasReceivedEventData_);
        GlobalBlackboard_->set<std::uint8_t>("EventSelfSmallEnergyStatus", eventSelfSmallEnergyStatus_);
        GlobalBlackboard_->set<std::uint8_t>("EventSelfLargeEnergyStatus", eventSelfLargeEnergyStatus_);
        GlobalBlackboard_->set("ArmorList", armorList);
        GlobalBlackboard_->set("TeamBuff", teamBuff);
        GlobalBlackboard_->set<std::uint8_t>("AimMode", static_cast<std::uint8_t>(aimMode));
        GlobalBlackboard_->set<std::uint8_t>("NaviGoal", naviCommandGoal);
        GlobalBlackboard_->set<int>("BuffShootCount", buff_shoot_count);
        GlobalBlackboard_->set<std::uint8_t>("StrategyMode", static_cast<std::uint8_t>(strategyMode_));
        GlobalBlackboard_->set<std::chrono::steady_clock::time_point>("GameStartTime", gameStartTime);
        GlobalBlackboard_->set<bool>("IsFindTarget", IsFindTarget);
        GlobalBlackboard_->set<std::uint8_t>("PostureState", postureState);
        GlobalBlackboard_->set<std::uint8_t>("PostureCommand", postureCommand);
        GlobalBlackboard_->set<bool>("PostureUnderFireRecent", IsUnderFireRecent());
        GlobalBlackboard_->set<bool>("PostureUnderFireBurst", IsUnderFireBurst());
        GlobalBlackboard_->set<std::int16_t>("GimbalYawVelRaw", gimbalYawVelRaw);
        GlobalBlackboard_->set<float>("GimbalYawVelDegPerSec", gimbalYawVelDegPerSec);
        GlobalBlackboard_->set<std::int16_t>("GimbalYawAngleRaw", gimbalYawAngleRaw);
        GlobalBlackboard_->set<float>("GimbalYawAngleDeg", gimbalYawAngleDeg);

        if (now - lastUpdateBlackboardLogTime_ > std::chrono::seconds(2)) {
            LoggerPtr->Debug("Blackboard updated: TimeLeft={}, SelfHealth={}, AmmoLeft={}, EnemyOutpostHealth={}, SelfOutpostHealth={}",
                timeLeft, SelfHealth, ammoLeft, enemyOutpostHealth, selfOutpostHealth);
            lastUpdateBlackboardLogTime_ = now;
        }
    }

    /**
     * @brief 从黑板获取数据,处理ros队列的消息并发布 \n
     * @brief 从黑板获取的有辐瞄击打目标，导航目的地
     */
    void Application::TransportData() {
        // targetArmor.Type = GetInfoFromBlackBoard<ArmorType>("AimTarget");
        // naviCommandGoal = GetInfoFromBlackBoard<std::uint8_t>("naviCommandGoal");
        // LoggerPtr->Info("AimTarget: {}, NaviGoal: {}", static_cast<int>(targetArmor.Type), static_cast<int>(naviCommandGoal));

        
        // targetArmor = ArmorType::Infantry1;
        // 设置数据内容
        PublishTogether();
        const auto now = std::chrono::steady_clock::now();
        if (now - lastTransportLogTime_ > std::chrono::seconds(1)) {
            PrintMessageAll();
            LoggerPtr->Debug("TransportData: published control data.");
            lastTransportLogTime_ = now;
        }
    }

     /**
     * @brief 实现PublishTogether \n
     * @brief 判断是否找到目标， 找到目标就发送目标数据 \n
     * @brief 否则经过一定时间之后， 将gimbalControlData的GimbalAngles均匀变化
     */
    void Application::PublishTogether() {

        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        static constexpr auto delta_yaw = 1.0f; //bt每tick單位
        static constexpr auto buff_yaw = -50.0f + 360.0f;
        static constexpr auto kPatrolScanYawStep = 9.0f * delta_yaw; //單向巡航每tick度數
        static constexpr auto kPatrolScanYawBoostStep = 10.0f * delta_yaw; //受擊加速
        static constexpr auto kPatrolScanPitchCenterDeg = 0.0f; //巡航pitch中心
        static constexpr auto kPatrolScanPitchHalfRangeDeg = 12.0f; //巡航pitch上下半幅
        static constexpr auto kPatrolScanPitchPeriodMs = 500.0f; //巡航pitch完整波形週期

        static constexpr auto kPatrolSwingYawStep = 1.0f * delta_yaw; //雙向巡航每tick度數
        static constexpr auto kPatrolSwingYawBoostStep = 1.1f * delta_yaw; //受擊加速
        static constexpr auto kPatrolSwingHalfRangeDeg = 30.0f; // mode2: 左右擺頭半幅
        static constexpr auto kPatrolSwingCenterDriftPerCycleDeg = -70.0f; // mode2: 每完整左右掃一圈，中心點右偏角度
        static constexpr auto kTwoPi = 6.2831853071795864769f;
        static constexpr int kDamageScanBoostWindowMs = 1300;
        static constexpr int kDamageScanYawPhaseMs = 160;
        const bool follow_mode_active = gimbalControlData.FireCode.FollowMode != 0;

        
        // 小陀螺策略（老设计）：
        // 1) 受击后按 1 -> 2 -> 3 递进换档；
        // 2) 到 3 档后持续保持；
        // 3) 仅在一段时间未受击后，回落到 1 档。
        const auto rotate_now = std::chrono::steady_clock::now();
        static auto last_damage_rotate_time = std::chrono::steady_clock::time_point{};
        static auto rotate_ramp_start_time = std::chrono::steady_clock::time_point{};
        static bool rotate_under_fire = false;

        constexpr int kRotateNoHitTimeoutMs = 1800;
        constexpr int kRotateGear1HoldMs = 220;
        constexpr int kRotateGear2HoldMs = 220;

        if (healthDecreaseDetector.trigger(myselfHealth)) { // 血量减少
            last_damage_rotate_time = rotate_now;
            if (!rotate_under_fire) {
                rotate_under_fire = true;
                rotate_ramp_start_time = rotate_now;
            }
            rotateTimerClock.tick();
        }

        bool in_damage_rotate_window = false;
        int damage_rotate_elapsed_ms = -1;
        std::uint8_t rotate_gear = 1;

        if (last_damage_rotate_time.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                rotate_now - last_damage_rotate_time).count();
            damage_rotate_elapsed_ms = static_cast<int>(elapsed_ms);
            in_damage_rotate_window = elapsed_ms <= kDamageScanBoostWindowMs;
        }

        if (rotate_under_fire) {
            const auto no_hit_ms = (last_damage_rotate_time.time_since_epoch().count() == 0)
                ? kRotateNoHitTimeoutMs + 1
                : static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    rotate_now - last_damage_rotate_time).count());

            if (no_hit_ms > kRotateNoHitTimeoutMs) {
                rotate_under_fire = false;
                rotate_gear = 1;
            } else {
                const auto ramp_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    rotate_now - rotate_ramp_start_time).count());
                if (ramp_ms < kRotateGear1HoldMs) {
                    rotate_gear = 1;
                } else if (ramp_ms < (kRotateGear1HoldMs + kRotateGear2HoldMs)) {
                    rotate_gear = 2;
                } else {
                    rotate_gear = 3;
                }
            }
        }

        gimbalControlData.FireCode.Rotate = rotate_gear;
        if (config.AimDebugSettings.StopRotate) {
            // StopRotate=true means disable chassis spin output.
            gimbalControlData.FireCode.Rotate = 0;
        }
        if (areaManager_.HighlandTransitionActive() &&
            config.DecisionAutonomySettings.NaviGoal.HighlandCompatDisableRotate) {
            gimbalControlData.FireCode.Rotate = 0;
        }
        if (follow_mode_active) {
            gimbalControlData.FireCode.Rotate = 0;
        }

        static auto last_rotate_log = std::chrono::steady_clock::time_point{};
        if (now_time >= 0) {
            const auto log_now = std::chrono::steady_clock::now();
            if (log_now - last_rotate_log > std::chrono::seconds(2)) {
                LoggerPtr->Debug(
                    "Rotate Gear: {} (under_fire={} damage_elapsed_ms={} no_hit_timeout_ms={})",
                    gimbalControlData.FireCode.Rotate,
                    rotate_under_fire ? 1 : 0,
                    damage_rotate_elapsed_ms,
                    kRotateNoHitTimeoutMs);
                last_rotate_log = log_now;
            }
        }

        /*----------云台----------*/
        auto now = std::chrono::steady_clock::now();
        const AimData* activeAimData = &autoAimData;
        if (aimMode == AimMode::Buff) {
            activeAimData = &buffAimData;
        } else if (aimMode == AimMode::Outpost) {
            activeAimData = &outpostAimData;
        }
        GimbalAnglesType nextAngles = gimbalAngles;
        VelocityType nextVelocity = naviVelocityInput;
        const bool find_target = isFindTargetAtomic.load(std::memory_order_relaxed);
        const bool has_recent_latched_target = [&]() {
            if (find_target || !config.AimDebugSettings.ReuseLatchedAnglesOnNoTarget ||
                !activeAimData->HasLatchedAngles ||
                activeAimData->LastValidTime.time_since_epoch().count() == 0) {
                return false;
            }
            const int hold_ms = std::max(0, config.AimDebugSettings.LatchedTargetHoldMs);
            return hold_ms > 0 &&
                   (now - activeAimData->LastValidTime) <= std::chrono::milliseconds(hold_ms);
        }();
        const bool has_target_for_angles = find_target || has_recent_latched_target;
        const bool visual_target_has_face_priority =
            has_target_for_angles && (aimMode == AimMode::Buff || aimMode == AimMode::Outpost);
        const bool face_mode_active =
            faceModeManager_.Active(config.FaceModeSettings, visual_target_has_face_priority);
        const auto chase_mode_enabled = [&]() -> bool {
            if (!config.ChaseSettings.Enable || !config.ChaseSettings.FollowAimTarget) {
                return false;
            }
            switch (aimMode) {
                case AimMode::AutoAim:
                    return config.ChaseSettings.EnableInAutoAim;
                case AimMode::RotateScan:
                    return config.ChaseSettings.EnableInRotateScan;
                case AimMode::Outpost:
                    return config.ChaseSettings.EnableInOutpostMode;
                case AimMode::Buff:
                    return config.ChaseSettings.EnableInBuffMode;
                default:
                    return false;
            }
        }();
        naviRelativeTargetValid = false;
        naviRelativeTargetX = 0.0F;
        naviRelativeTargetY = 0.0F;
        naviRelativeTargetZ = 0.0F;
        naviRelativeTargetDistance = 0.0F;
        naviRelativeTargetYawErrorDeg = 0.0F;
        naviRelativeTargetPitchErrorDeg = 0.0F;
        naviRelativeTargetArmorType = 0U;
        naviRelativeTargetAimMode = static_cast<std::uint8_t>(aimMode);
        naviChaseOfficialTargetValid = false;
        naviChaseOfficialTargetArmorType = 0U;
        auto reset_patrol_scan_state = [this]() {
            patrolScanDirection_ = 1;
            patrolScanCenterYaw_ = 0.0f;
            patrolScanOffsetYaw_ = 0.0f;
            patrolScanPhaseRad_ = 0.0f;
            patrolScanCenterInitialized_ = false;
            patrolScanActiveMode_ = 0;
        };
        if (face_mode_active) {
            reset_patrol_scan_state();
            gimbalControlData.FireCode.AimMode = 0;
            if (config.FaceModeSettings.SuppressFire) {
                gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                buffAimData.FireStatus = false;
            }
            const auto face_angles =
                faceModeManager_.SelectAngles(faceModeData, config.FaceModeSettings, now);
            nextAngles = face_angles.value_or(gimbalAngles);

            static auto last_face_mode_log = std::chrono::steady_clock::time_point{};
            if (now - last_face_mode_log > std::chrono::seconds(2)) {
                LoggerPtr->Debug(
                    "FaceMode active: {}, stop patrol scan, {} gimbal angles, suppress_fire={}",
                    follow_mode_active ? "follow mode controls rotate" : "keep rotate",
                    face_angles.has_value() ? "use FaceMode" : "hold current",
                    config.FaceModeSettings.SuppressFire ? 1 : 0);
                last_face_mode_log = now;
            }
        } else if (follow_mode_active) {
            reset_patrol_scan_state();
            gimbalControlData.FireCode.AimMode = 0;
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
            buffAimData.FireStatus = false;
            nextAngles = gimbalAngles;

            static auto last_follow_mode_log = std::chrono::steady_clock::time_point{};
            if (now - last_follow_mode_log > std::chrono::seconds(2)) {
                LoggerPtr->Debug(
                    "FollowMode active: stop rotate, stop patrol scan, hold current gimbal angles, suppress fire");
                last_follow_mode_log = now;
            }
        } else if (has_target_for_angles) {
            reset_patrol_scan_state();
            LoggerPtr->Debug(
                "Aim target active, AimMode={}, fresh={}, held={}",
                static_cast<int>(aimMode),
                find_target ? 1 : 0,
                has_recent_latched_target ? 1 : 0);
            if (find_target && !config.AimDebugSettings.StopFire){
                if(aimMode == AimMode::Buff) { // 打符模式
                    const int referee_fresh_ms =
                        std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs);
                    const bool event_data_fresh =
                        hasReceivedEventData_ &&
                        lastEventDataRxTime_.time_since_epoch().count() != 0 &&
                        now - lastEventDataRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
                    const bool energy_activating =
                        event_data_fresh &&
                        (eventSelfSmallEnergyStatus_ == 2 || eventSelfLargeEnergyStatus_ == 2);
                    const int post_confirm_grace_ms =
                        std::max(0, config.TaskSettings.BuffConfirm.PostConfirmGraceMs);
                    const bool confirm_grace_active =
                        lastEnergyActivateConfirmTime_.time_since_epoch().count() != 0 &&
                        now - lastEnergyActivateConfirmTime_ <= std::chrono::milliseconds(post_confirm_grace_ms);
                    const bool buff_fire_allowed =
                        config.TaskSettings.BuffTimer.Enable || energy_activating || confirm_grace_active;
                    if(buff_fire_allowed && activeAimData->FireStatus){
                        /// 立刻响应不需要tick
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                        buffAimData.FireStatus = false;
                        buff_shoot_count++;
                    } else {
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                } else { // 非打符模式（打前哨和打车），沿用老代码：收到回调就按频率开火
                    if(fireRateClock.trigger()){
                        fireRateClock.tick();
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                }
            } else {
                gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
            }
            gimbalControlData.FireCode.AimMode = 1;
            if (find_target) {
                lastFoundEnemyTime = now;
            }
            
            nextAngles = activeAimData->Angles;
            if (aimMode != AimMode::Buff && aimMode != AimMode::Outpost) {
                LoggerPtr->Debug(
                    "AutoAim Angles -> Pitch: {}, Yaw: {}",
                    autoAimData.Angles.Pitch,
                    autoAimData.Angles.Yaw);
            }
        }
        else { // 未识别到目标
            gimbalControlData.FireCode.AimMode = 0;
            
            if(aimMode != AimMode::Buff) {
                if (!config.AimDebugSettings.StopScan && now - lastFoundEnemyTime > std::chrono::milliseconds(2000)) {
                    static auto last_searching_log = std::chrono::steady_clock::time_point{};
                    const int patrol_mode = config.PatrolScanSettings.Mode;
                    const bool boost_patrol_scan =
                        aimMode == AimMode::RotateScan &&
                        damage_rotate_elapsed_ms >= 0 &&
                        damage_rotate_elapsed_ms <= kDamageScanBoostWindowMs;
                    float yaw_scan_step = boost_patrol_scan
                        ? kPatrolScanYawBoostStep
                        : kPatrolScanYawStep;
                    int yaw_scan_direction = 1;

                    if (patrol_mode == 2) {
                        yaw_scan_step = boost_patrol_scan ? kPatrolSwingYawBoostStep : kPatrolSwingYawStep;

                        if (!patrolScanCenterInitialized_ || patrolScanActiveMode_ != patrol_mode) {
                            patrolScanCenterInitialized_ = true;
                            patrolScanActiveMode_ = patrol_mode;
                            patrolScanCenterYaw_ = gimbalAngles.Yaw;
                            patrolScanOffsetYaw_ = 0.0f;
                            patrolScanPhaseRad_ = 0.0f;
                            patrolScanDirection_ = 1; // 新一轮巡逻默认先向右
                        }

                        const float half_range = kPatrolSwingHalfRangeDeg;
                        const float phase_step = yaw_scan_step / std::max(half_range, 1.0f);
                        const float center_drift_step =
                            kPatrolSwingCenterDriftPerCycleDeg * phase_step / kTwoPi;

                        // mode2: 讓中心點每完成一個正弦掃描週期固定右偏同樣角度。
                        patrolScanCenterYaw_ = normalize_angle_near(
                            patrolScanCenterYaw_ + center_drift_step,
                            gimbalAngles.Yaw);

                        patrolScanPhaseRad_ = std::fmod(patrolScanPhaseRad_ + phase_step, kTwoPi);
                        patrolScanOffsetYaw_ = half_range * std::sin(patrolScanPhaseRad_);
                        patrolScanDirection_ = (std::cos(patrolScanPhaseRad_) >= 0.0f) ? 1 : -1;
                        if (patrolScanPhaseRad_ < 0.0f) {
                            patrolScanPhaseRad_ += kTwoPi;
                        }
                        yaw_scan_direction = patrolScanDirection_;
                    } else {
                        if (patrolScanActiveMode_ != patrol_mode || patrolScanCenterInitialized_) {
                            reset_patrol_scan_state();
                            patrolScanActiveMode_ = patrol_mode;
                        }
                        yaw_scan_direction = boost_patrol_scan
                            ? (((damage_rotate_elapsed_ms / kDamageScanYawPhaseMs) % 2 == 0) ? 1 : -1)
                            : 1;
                    }
                    if (now - last_searching_log > std::chrono::seconds(2)) {
                        LoggerPtr->Debug(
                            "Searching Target... patrol_mode={} yaw_step={} dir={} (damage_boost={} elapsed_ms={})",
                            patrol_mode,
                            yaw_scan_step,
                            yaw_scan_direction,
                            boost_patrol_scan ? 1 : 0,
                            damage_rotate_elapsed_ms);
                        last_searching_log = now;
                        gimbalControlData.FireCode.AimMode = 0;
                    }
                    const auto current_time = std::chrono::steady_clock::now();
                    const float next_scan_yaw = (patrol_mode == 2 && patrolScanCenterInitialized_)
                        ? normalize_angle_near(patrolScanCenterYaw_ + patrolScanOffsetYaw_, gimbalAngles.Yaw)
                        : static_cast<float>(gimbalAngles.Yaw + yaw_scan_direction * yaw_scan_step);
                    const auto pitch_elapsed_ms = static_cast<float>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            current_time - gameStartTime).count());
                    const float next_scan_pitch = kPatrolScanPitchCenterDeg +
                        kPatrolScanPitchHalfRangeDeg *
                            std::sin(pitch_elapsed_ms * kTwoPi / std::max(kPatrolScanPitchPeriodMs, 1.0f));
                    nextAngles = GimbalAnglesType{
                        static_cast<AngleType>(next_scan_yaw),
                        static_cast<AngleType>(next_scan_pitch)
                    };

                    if (aimMode == AimMode::Outpost) {
                        nextAngles.Pitch += 15.0f;
                    }
                } else {
                    reset_patrol_scan_state();
                    nextAngles = gimbalAngles;
                    LoggerPtr->Debug(
                        "Hold current gimbal angles (latched reuse disabled or unavailable) -> Pitch: {}, Yaw: {}",
                        gimbalAngles.Pitch,
                        gimbalAngles.Yaw);
                }
            }else { // 打符模式
                reset_patrol_scan_state();
                if (now_time < 5) {
                    LoggerPtr->Info("Set Angles, Buff Mode, 10 min!");
                    // Yaw
                    float current_yaw = normalize_angle_0_360(gimbalAngles.Yaw);
                    float delta = buff_yaw - current_yaw;
                    if (delta > 180.0f) delta -= 360.0f; // 角度差大于180，反向旋转
                    if (delta < -180.0f) delta += 360.0f;
                    int opt = delta > 0 ? 1 : -1;
                    int target_yaw = gimbalAngles.Yaw + delta;
                    nextAngles = gimbalAngles;

                    if (std::abs(delta) > 10 * delta_yaw) {
                        nextAngles.Yaw = static_cast<AngleType>(gimbalAngles.Yaw + delta_yaw * opt);
                    } else {
                        nextAngles.Yaw = static_cast<AngleType>(target_yaw);
                    }
                    nextAngles.Pitch = 19.0f;
                }else {
                    nextAngles = (buffAimData.Fresh && buffAimData.Valid && buffAimData.BuffFollow)
                        ? buffAimData.Angles
                        : gimbalAngles;
                }
            }
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
        }

        if (!follow_mode_active && chase_mode_enabled) {
            const bool chase_to_navi = config.ChaseSettings.ToNavi;
            const bool navi_to_navi =
                config.NaviSettings.UseXY && config.NaviSettings.ToNavi;
            bool has_chase_target = has_target_for_angles;
            if (!has_chase_target &&
                config.ChaseSettings.LostTargetHoldMs > 0 &&
                activeAimData->HasLatchedAngles &&
                lastTargetSeenTime.time_since_epoch().count() != 0) {
                has_chase_target = (now - lastTargetSeenTime) <=
                    std::chrono::milliseconds(config.ChaseSettings.LostTargetHoldMs);
            }

            bool chase_distance_valid = false;
            float distance_cm = 0.0f;
            if (std::isfinite(targetArmor.Distance) && targetArmor.Distance > 0.0f) {
                distance_cm = targetArmor.Distance * 100.0f;
                chase_distance_valid =
                    distance_cm >= static_cast<float>(config.ChaseSettings.MinValidDistanceCm) &&
                    distance_cm <= static_cast<float>(config.ChaseSettings.MaxValidDistanceCm);
            }

            if (has_chase_target &&
                chase_distance_valid &&
                targetArmor.Type != ArmorType::UnKnown) {
                const auto yaw_error_deg = static_cast<double>(
                    std::remainder(nextAngles.Yaw - gimbalAngles.Yaw, 360.0f));
                const auto pitch_error_deg = static_cast<double>(
                    std::remainder(nextAngles.Pitch - gimbalAngles.Pitch, 360.0f));
                const double distance_m = static_cast<double>(distance_cm) * 0.01;
                constexpr double kDegToRad = 0.017453292519943295;
                const double yaw_rad = yaw_error_deg * kDegToRad;
                const double pitch_rad = pitch_error_deg * kDegToRad;
                const double cos_pitch = std::cos(pitch_rad);

                naviRelativeTargetValid = true;
                naviRelativeTargetX = static_cast<float>(distance_m * cos_pitch * std::cos(yaw_rad));
                naviRelativeTargetY = static_cast<float>(distance_m * cos_pitch * std::sin(yaw_rad));
                naviRelativeTargetZ = static_cast<float>(distance_m * std::sin(pitch_rad));
                naviRelativeTargetDistance = static_cast<float>(distance_m);
                naviRelativeTargetYawErrorDeg = static_cast<float>(yaw_error_deg);
                naviRelativeTargetPitchErrorDeg = static_cast<float>(pitch_error_deg);
                naviRelativeTargetArmorType = static_cast<std::uint8_t>(targetArmor.Type);

                if (chase_to_navi) {
                    // 直发地图坐标模式：UseXY=true 且关闭 tf bridge。
                    if (config.NaviSettings.UseXY && !navi_to_navi) {
                        const auto maybe_target_unit = UnitTypeFromArmorType(targetArmor.Type);
                        if (maybe_target_unit.has_value()) {
                            const int enemy_x = static_cast<int>(enemyRobots[*maybe_target_unit].position_.X);
                            const int enemy_y = static_cast<int>(enemyRobots[*maybe_target_unit].position_.Y);
                            if (enemy_x >= 0 && enemy_y >= 0) {
                                naviGoalPosition.x = static_cast<std::uint16_t>(std::clamp(enemy_x, 0, 65535));
                                naviGoalPosition.y = static_cast<std::uint16_t>(std::clamp(enemy_y, 0, 65535));
                            }
                        }
                    }
                } else {
                    const double distance_error_cm =
                        static_cast<double>(distance_cm) -
                        static_cast<double>(config.ChaseSettings.PreferredDistanceCm);

                    int chase_vx = 0;
                    if (std::abs(distance_error_cm) >
                        static_cast<double>(config.ChaseSettings.DistanceDeadbandCm)) {
                        chase_vx = static_cast<int>(std::lround(config.ChaseSettings.DistanceKp * distance_error_cm));
                        chase_vx = std::clamp(
                            chase_vx,
                            -config.ChaseSettings.MaxBackwardSpeed,
                            config.ChaseSettings.MaxForwardSpeed);
                    }

                    int chase_vy = 0;
                    if (config.ChaseSettings.UseYawStrafe) {
                        if (std::abs(yaw_error_deg) > static_cast<double>(config.ChaseSettings.YawDeadbandDeg)) {
                            chase_vy = static_cast<int>(std::lround(config.ChaseSettings.YawKp * yaw_error_deg));
                            if (config.ChaseSettings.InvertStrafeDirection) {
                                chase_vy = -chase_vy;
                            }
                            chase_vy = std::clamp(
                                chase_vy,
                                -config.ChaseSettings.MaxStrafeSpeed,
                                config.ChaseSettings.MaxStrafeSpeed);
                        }
                    }

                    nextVelocity.X = static_cast<std::int8_t>(ClampToInt8(chase_vx));
                    nextVelocity.Y = static_cast<std::int8_t>(ClampToInt8(chase_vy));
                }
            } else if (!chase_to_navi &&
                       config.ChaseSettings.StopWhenNoTarget) {
                nextVelocity = VelocityType{0, 0};
            } else if (chase_to_navi &&
                       config.NaviSettings.UseXY &&
                       !navi_to_navi &&
                       config.ChaseSettings.StopWhenNoTarget) {
                const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
                const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
                if (self_x >= 0 && self_y >= 0) {
                    naviGoalPosition.x = static_cast<std::uint16_t>(std::clamp(self_x, 0, 65535));
                    naviGoalPosition.y = static_cast<std::uint16_t>(std::clamp(self_y, 0, 65535));
                }
            }

            if (chase_to_navi && config.ChaseSettings.UseOfficialPositionSource) {
                const bool should_try_official =
                    config.ChaseSettings.PreferOfficialPositionSource || !naviRelativeTargetValid;
                const auto maybe_target_unit = UnitTypeFromArmorType(targetArmor.Type);
                const auto sentry_index = static_cast<std::size_t>(UnitType::Sentry);
                const bool self_position_fresh =
                    hasReceivedSentryPosition_ &&
                    sentry_index < lastFriendPositionRxTime_.size() &&
                    lastFriendPositionRxTime_[sentry_index].time_since_epoch().count() != 0 &&
                    now - lastFriendPositionRxTime_[sentry_index] <=
                        std::chrono::milliseconds(std::max(1, config.ChaseSettings.OfficialPositionFreshMs));

                if (should_try_official &&
                    maybe_target_unit.has_value() &&
                    IsEnemyPositionFresh(*maybe_target_unit, config.ChaseSettings.OfficialPositionFreshMs)) {
                    const int target_x = static_cast<int>(enemyRobots[*maybe_target_unit].position_.X);
                    const int target_y = static_cast<int>(enemyRobots[*maybe_target_unit].position_.Y);
                    const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
                    const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
                    if (IsOfficialFieldPointValid(target_x, target_y) &&
                        self_position_fresh &&
                        IsOfficialFieldPointValid(self_x, self_y)) {
                        naviGoalPosition = BuildOfficialChaseGoal(
                            self_x,
                            self_y,
                            target_x,
                            target_y,
                            config.ChaseSettings.PreferredDistanceCm,
                            config.ChaseSettings.DistanceDeadbandCm);
                        naviChaseOfficialTargetValid = true;
                        naviChaseOfficialTargetArmorType =
                            static_cast<std::uint8_t>(targetArmor.Type);
                    }
                }
            }
        }

        // lower_head 只在未锁目标时生效，并且整对角一起切换，避免混用旧 yaw/new pitch。
        if(!follow_mode_active && naviLowerHead && !has_target_for_angles) {
            nextAngles = GimbalAnglesType{gimbalAngles.Yaw, -15.0f}; //-22.5 - 26.0
        }
        gimbalControlData.GimbalAngles = nextAngles;
        naviVelocity = nextVelocity;

        PublishMessageAll();
        autoAimData.Fresh = false;
        buffAimData.Fresh = false;
        outpostAimData.Fresh = false;
        faceModeData.Fresh = false;
        isFindTargetAtomic = false;
    }

    /**
     * @brief 决策进程主循环 \n
     * @brief 1. 设置黑板数据 \n
     * @brief 2. 休眠 \n
     * @brief 3. 更新黑板数据 \n
     * @brief 4. 处理行为树 \n
     */
    void Application::GameLoop() {

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        ResetTickBlackboard();

        GlobalBlackboard_->set<UnitTeam>("MyTeam", team); // 队伍颜色
        GlobalBlackboard_->set<std::chrono::steady_clock::time_point>(
            "LastCommandTime", std::chrono::steady_clock::now()); // 上次发送命令的时间
        GlobalBlackboard_->set<std::chrono::seconds>("CommandInterval", std::chrono::seconds{0}); // 命令间隔
        GlobalBlackboard_->set<ArmorType>("AimTarget", ArmorType::Hero); // 辅瞄击打目标
        GlobalBlackboard_->set<std::uint8_t>("naviCommandGoal", Home(team)); // 导航目的地
        GlobalBlackboard_->set<std::shared_ptr<Logger>>("LoggerPtr", LoggerPtr);
        GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);
        UpdateBlackBoard();

        while (rclcpp::ok()) {
            MarkLoopBeat();
            rclcpp::spin_some(node_); // 处理回调函数

            if (TryHandleSoftRecovery()) {
                treeTickRateClock.sleep();
                continue;
            }

            if (IsCriticalInputStale()) {
                static auto last_stale_log = std::chrono::steady_clock::time_point{};
                const auto now = std::chrono::steady_clock::now();
                if (LoggerPtr && (now - last_stale_log > std::chrono::seconds(2))) {
                    LoggerPtr->Warning("Critical input stale: gimbal angles > {} ms, publish safe-control.",
                                       kRuntimeGimbalStaleMs);
                    last_stale_log = now;
                }
                PublishSafeControl("gimbal_stale");
                treeTickRateClock.sleep();
                continue;
            }

            if (runtimeRearmStartGate_ && !is_game_begin) {
                const auto now = std::chrono::steady_clock::now();
                if (!runtimeStartGateActive_) {
                    runtimeStartGateActive_ = true;
                    runtimeStartGateLastLogTime_ = now;
                    LoggerPtr->Info("Runtime start gate armed: waiting /ly/game/is_start=true.");
                } else if (LoggerPtr && (now - runtimeStartGateLastLogTime_ > std::chrono::seconds(2))) {
                    LoggerPtr->Debug("Runtime start gate waiting for /ly/game/is_start=true...");
                    runtimeStartGateLastLogTime_ = now;
                }

                SET_POSITION(Home, team);
                if (publishNaviGoal_ && naviCommandRateClock.trigger()) {
                    naviCommandRateClock.tick();
                    const bool navi_to_navi =
                        config.NaviSettings.UseXY &&
                        config.NaviSettings.ToNavi &&
                        config.ChaseSettings.Enable &&
                        config.ChaseSettings.ToNavi;
                    if (config.NaviSettings.UseXY && !navi_to_navi) {
                        PubNaviGoalPos();
                    } else {
                        PubNaviGoal();
                    }
                }

                naviVelocityInput = VelocityType{0, 0};
                naviVelocity = VelocityType{0, 0};
                postureCommand = 0;
                PublishSafeControl("runtime_start_gate");
                UpdateBlackBoard();
                if (decisionTraceEnabled_) {
                    WriteDecisionTrace("tick");
                }
                treeTickRateClock.sleep();
                continue;
            }
            if (runtimeRearmStartGate_ && runtimeStartGateActive_ && is_game_begin) {
                runtimeStartGateActive_ = false;
                gameStartTime = std::chrono::steady_clock::now();
                if (LoggerPtr) {
                    LoggerPtr->Info("Runtime start gate opened: resume decision loop.");
                }
                if (decisionTraceEnabled_) {
                    WriteDecisionTrace("game_start");
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastTreeTickLogTime_ > std::chrono::seconds(2)) {
                LoggerPtr->Debug("BehaviorTree Root Tick...");
                lastTreeTickLogTime_ = now;
            }
            TreeTickGuarded();
            if (decisionTraceEnabled_) {
                WriteDecisionTrace("tick");
            }
            treeTickRateClock.sleep();
        }
    }

    void Application::TreeTickGuarded() {
        MarkTickStart();
        try {
            TreeTick();
        } catch (const std::exception& ex) {
            if (LoggerPtr) {
                LoggerPtr->Error("BehaviorTree tick exception: {}", ex.what());
            }
            RequestSoftRecovery(RuntimeFaultCode::TreeException);
        } catch (...) {
            if (LoggerPtr) {
                LoggerPtr->Error("BehaviorTree tick exception: <unknown>");
            }
            RequestSoftRecovery(RuntimeFaultCode::TreeException);
        }
        MarkTickEnd();
    }

    void Application::TreeTick() {
        if (BTree.subtrees.empty()) {
            LoggerPtr->Error("BehaviorTree is empty, skip tick.");
            RequestSoftRecovery(RuntimeFaultCode::TreeEmpty);
            return;
        }

        const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
        if (status == BT::NodeStatus::FAILURE) {
            LoggerPtr->Warning("BehaviorTree tick returned FAILURE.");
        }
    }

    bool Application::IsDecisionAutonomyModuleEnabled(std::string_view module) const {
        const auto& autonomy = config.DecisionAutonomySettings;
        if (!autonomy.Enable) {
            return false;
        }
        const auto normalized_module = NormalizeDecisionModule(module);
        if (HasAutonomyToken(autonomy.HardRuleModules, normalized_module)) {
            return false;
        }
        if (HasAutonomyToken(autonomy.EnabledModules, "all")) {
            return true;
        }
        return HasAutonomyToken(autonomy.EnabledModules, normalized_module);
    }

    void Application::SelectStrategyMode() {
        if (GetCompetitionProfile() == CompetitionProfile::League) {
            SetStrategyMode(StrategyMode::LeagueSimple);
            return;
        }

        // Regional uses AreaManager/default-task policy only; old point-table
        // strategies stay out of the live regional chain.
        SetStrategyMode(StrategyMode::Regional);
    }

    void Application::SetAimMode() {
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        LoggerPtr->Info("SetAimMode - now_time: {}", now_time);
        if(config.TaskSettings.Buff) { // 打符
            const auto now = std::chrono::steady_clock::now();
            const int referee_fresh_ms =
                std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs);
            const int post_confirm_grace_ms =
                std::max(0, config.TaskSettings.BuffConfirm.PostConfirmGraceMs);
            const int task_hold_timeout_ms =
                std::max(0, config.TaskSettings.BuffConfirm.TaskHoldTimeoutMs);
            const int damage_abort_threshold =
                std::max(0, config.TaskSettings.BuffConfirm.DamageAbortThreshold);
            const int damage_abort_window_ms =
                std::max(0, config.TaskSettings.BuffConfirm.DamageAbortWindowMs);
            const int damage_abort_hold_ms =
                std::max(0, config.TaskSettings.BuffConfirm.DamageAbortHoldMs);
            const bool event_data_fresh =
                hasReceivedEventData_ &&
                lastEventDataRxTime_.time_since_epoch().count() != 0 &&
                now - lastEventDataRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
            const bool sentry_info_fresh =
                hasReceivedSentryInfo_ &&
                lastSentryInfoRxTime_.time_since_epoch().count() != 0 &&
                now - lastSentryInfoRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
            const bool energy_activated =
                event_data_fresh &&
                (eventSelfSmallEnergyStatus_ == 1 || eventSelfLargeEnergyStatus_ == 1);
            const bool energy_activating =
                event_data_fresh &&
                (eventSelfSmallEnergyStatus_ == 2 || eventSelfLargeEnergyStatus_ == 2);
            const bool can_activate_energy =
                sentry_info_fresh && sentryCanActivateEnergyMechanism_;
            const bool energy_done_without_next =
                energy_activated && !energy_activating && !can_activate_energy;
            const bool confirm_grace_active =
                lastEnergyActivateConfirmTime_.time_since_epoch().count() != 0 &&
                now - lastEnergyActivateConfirmTime_ <= std::chrono::milliseconds(post_confirm_grace_ms);
            const bool damage_abort_active =
                buffTaskDamageAbortUntil_.time_since_epoch().count() != 0 &&
                now < buffTaskDamageAbortUntil_;
            const bool recent_damage_abort = [&]() {
                if (damage_abort_threshold <= 0 || damage_abort_window_ms <= 0) {
                    return false;
                }
                for (auto it = postureRecentDamageSamples_.rbegin();
                     it != postureRecentDamageSamples_.rend();
                     ++it) {
                    if ((now - it->Time) > std::chrono::milliseconds(damage_abort_window_ms)) {
                        break;
                    }
                    if (it->Delta > static_cast<std::uint16_t>(damage_abort_threshold)) {
                        return true;
                    }
                }
                return false;
            }();

            if (recent_damage_abort) {
                buffTaskLocked_ = false;
                buffTaskStartTime_ = {};
                buffTaskDamageAbortUntil_ = now + std::chrono::milliseconds(damage_abort_hold_ms);
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Buff interrupted by damage > {}. Hold armor mode for {} ms.",
                    damage_abort_threshold,
                    damage_abort_hold_ms);
            } else if (damage_abort_active) {
                aimMode = AimMode::RotateScan;
            } else if (energy_done_without_next) {
                buffTaskLocked_ = false;
                buffTaskStartTime_ = {};
                LoggerPtr->Info(
                    "Buff energy active and no next activation window. event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={}",
                    event_data_fresh ? 1 : 0,
                    sentry_info_fresh ? 1 : 0,
                    static_cast<int>(eventSelfSmallEnergyStatus_),
                    static_cast<int>(eventSelfLargeEnergyStatus_),
                    can_activate_energy ? 1 : 0);
                aimMode = AimMode::RotateScan;
            } else if (config.TaskSettings.BuffTimer.Enable) {
                const int timer_start = config.TaskSettings.BuffTimer.StartSec;
                const int timer_end = config.TaskSettings.BuffTimer.EndSec;
                const int max_shoot_count = config.TaskSettings.BuffTimer.MaxShootCount;
                const bool in_timer_window = now_time >= timer_start && now_time < timer_end;
                const bool shoot_count_allowed =
                    max_shoot_count < 0 || buff_shoot_count <= max_shoot_count;
                if (in_timer_window && shoot_count_allowed) {
                    if (!buffTaskLocked_) {
                        buffTaskStartTime_ = now;
                    }
                    buffTaskLocked_ = true;
                    aimMode = AimMode::Buff;
                } else {
                    buffTaskLocked_ = false;
                    buffTaskStartTime_ = {};
                    LoggerPtr->Info(
                        "Buff timer gate closed: now={} window=[{}, {}) shoot_count={} max={}",
                        now_time,
                        timer_start,
                        timer_end,
                        buff_shoot_count,
                        max_shoot_count);
                    aimMode = AimMode::RotateScan;
                }
            } else {
                const bool task_hold_timeout =
                    buffTaskLocked_ &&
                    task_hold_timeout_ms > 0 &&
                    buffTaskStartTime_.time_since_epoch().count() != 0 &&
                    now - buffTaskStartTime_ > std::chrono::milliseconds(task_hold_timeout_ms);
                const bool official_state_keeps_task =
                    energy_activating || can_activate_energy || confirm_grace_active;
                if (task_hold_timeout && !official_state_keeps_task) {
                    buffTaskLocked_ = false;
                    buffTaskStartTime_ = {};
                    LoggerPtr->Warning(
                        "Buff task timeout without fresh active state: timeout_ms={} event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={}",
                        task_hold_timeout_ms,
                        event_data_fresh ? 1 : 0,
                        sentry_info_fresh ? 1 : 0,
                        static_cast<int>(eventSelfSmallEnergyStatus_),
                        static_cast<int>(eventSelfLargeEnergyStatus_),
                        can_activate_energy ? 1 : 0);
                    aimMode = AimMode::RotateScan;
                } else if (official_state_keeps_task || buffTaskLocked_) {
                    if (!buffTaskLocked_) {
                        buffTaskStartTime_ = now;
                    }
                    buffTaskLocked_ = true;
                    LoggerPtr->Info(
                        "Buff state gate open: event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={} confirm_grace={} locked={}",
                        event_data_fresh ? 1 : 0,
                        sentry_info_fresh ? 1 : 0,
                        static_cast<int>(eventSelfSmallEnergyStatus_),
                        static_cast<int>(eventSelfLargeEnergyStatus_),
                        can_activate_energy ? 1 : 0,
                        confirm_grace_active ? 1 : 0,
                        buffTaskLocked_ ? 1 : 0);
                    aimMode = AimMode::Buff;
                } else {
                    LoggerPtr->Info(
                        "Buff state gate closed: event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={} confirm_grace={}",
                        event_data_fresh ? 1 : 0,
                        sentry_info_fresh ? 1 : 0,
                        static_cast<int>(eventSelfSmallEnergyStatus_),
                        static_cast<int>(eventSelfLargeEnergyStatus_),
                        can_activate_energy ? 1 : 0,
                        confirm_grace_active ? 1 : 0);
                    aimMode = AimMode::RotateScan;
                }
            }
        }else if(config.TaskSettings.Outpost) { // 打前哨站
            if (IsRegionalDefenseAimSuppressActive()) {
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info("Regional defense active: suppress Outpost aim mode.");
                return;
            }
            if(enemyOutpostHealth > 0) {
                LoggerPtr->Info("Enemy Outpost Health: {}", enemyOutpostHealth);;
                if(now_time < 90) {
                    aimMode = AimMode::Outpost;
                }else {
                    LoggerPtr->Info("Time out 1.5 min, stop hit outpost!");
                    aimMode = AimMode::RotateScan;
                }
            }else {
                LoggerPtr->Info("Enemy Outpost has been destroyed!");
                aimMode = AimMode::RotateScan;
            }
        }else { // 普通模式
            if (IsRegionalDefenseAimSuppressActive()) {
                LoggerPtr->Info("Regional defense active: keep armor vision mode.");
            } else {
                LoggerPtr->Info("AimMode: RotateScan!");
            }
            aimMode = AimMode::RotateScan;
        }

    }
    // 提前处理坐标等数据
    void Application::ProcessData() {
        int now_time = 420 - timeLeft;
        // 处理坐标数据
        reliableEnemyPosuition.clear();
        for(auto robot : RobotLists) {
           if(enemyRobots[robot].position_.X > 100 && enemyRobots[robot].position_.Y > 100) {
                reliableEnemyPosuition.push_back(robot);
            }
        }
        LoggerPtr->Info("> reliableEnemyPosuition <");
        for(auto robot : reliableEnemyPosuition) {
            LoggerPtr->Info("ID: {}, X: {}, Y:{}", static_cast<int>(robot), enemyRobots[robot].position_.X, enemyRobots[robot].position_.Y);
        }

        // 处理距离和无敌状态的数据
        hitableTargets.clear();
        for (auto Armor : armorList) {
            if (Armor.Type == ArmorType::UnKnown) continue;
            if(Armor.Type == ArmorType::Hero) {
                enemyRobots[UnitType::Hero].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Hero].isInvulnerable()) hitableTargets.push_back(UnitType::Hero);
            }else if(Armor.Type == ArmorType::Engineer) {
                enemyRobots[UnitType::Engineer].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Engineer].isInvulnerable() && now_time > 60) hitableTargets.push_back(UnitType::Engineer);
            }else if(Armor.Type == ArmorType::Infantry1) {
                enemyRobots[UnitType::Infantry1].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Infantry1].isInvulnerable()) hitableTargets.push_back(UnitType::Infantry1);
            }else if(Armor.Type == ArmorType::Infantry2) {
                enemyRobots[UnitType::Infantry2].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Infantry2].isInvulnerable()) hitableTargets.push_back(UnitType::Infantry2);
            }else if(Armor.Type == ArmorType::Sentry) {
                enemyRobots[UnitType::Sentry].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Sentry].isInvulnerable()) hitableTargets.push_back(UnitType::Sentry);
            }
        }
        for(auto robot : hitableTargets) {
            LoggerPtr->Info("ID{}", static_cast<int>(robot));
        }
    }

    void Application::SetAimTarget() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        std::uint16_t nowx = friendRobots[UnitType::Sentry].position_.X, nowy = friendRobots[UnitType::Sentry].position_.Y;
        const auto is_ignored_armor = [this](const ArmorType armor_type) -> bool {
            return IsIgnoredArmorType(config.AimTargetIgnore, armor_type);
        };
        if(aimMode == AimMode::Buff) { // 打符，修改为默认值
            if(BehaviorTree::Area::BuffOutpost.near(nowx, nowy, 100, MyTeam) &&
               !is_ignored_armor(ArmorType::Hero)) {
                targetArmor.Type = ArmorType::Hero;
            } else {
                SetAimTargetNormal();
            }
        }else if(aimMode == AimMode::Outpost) { // 打前哨站
            if(BehaviorTree::Area::BuffOutpost.near(nowx, nowy, 100, MyTeam) &&
               !is_ignored_armor(ArmorType::Outpost)) {
                targetArmor.Type = ArmorType::Outpost;
            } else {
                SetAimTargetNormal();
            }
        }else { // 普通模式
            if(naviCommandGoal == LangYa::HoleRoad(EnemyTeam)) { // 英雄点位1
                if(BehaviorTree::Area::HoleRoad.near(nowx, nowy, 100, MyTeam) &&
                   !is_ignored_armor(ArmorType::Hero)) {
                    targetArmor.Type = ArmorType::Hero;
                    targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
                } else {
                    SetAimTargetNormal();
                }
            } else {
                SetAimTargetNormal();
            }
            LoggerPtr->Info("Target: {}", static_cast<int>(targetArmor.Type));
        }

    }

    bool Application::TrySetAimTargetByAutonomy() {
        if (!IsDecisionAutonomyModuleEnabled("aim_target")) {
            return false;
        }

        struct AimCandidate {
            ArmorType Armor{ArmorType::UnKnown};
            UnitType Unit{UnitType::Unknown};
            float Distance{0.0f};
            std::uint16_t Health{0U};
        };

        std::vector<AimCandidate> candidates;
        candidates.reserve(hitableTargets.size());
        std::uint16_t max_health = 1U;
        for (const auto unit_type : hitableTargets) {
            if (IsIgnoredUnitType(config.AimTargetIgnore, unit_type)) {
                continue;
            }
            const auto armor_type = ArmorTypeFromUnitType(unit_type);
            if (!armor_type.has_value()) {
                continue;
            }
            const auto& robot = enemyRobots[unit_type];
            candidates.push_back(AimCandidate{
                .Armor = *armor_type,
                .Unit = unit_type,
                .Distance = robot.distance_,
                .Health = robot.currentHealth_
            });
            max_health = std::max(max_health, robot.currentHealth_);
        }
        if (candidates.empty()) {
            return false;
        }

        const auto& autonomy = config.DecisionAutonomySettings.AimTarget;
        const auto get_priority_rank = [this](const ArmorType armor_type) {
            const int armor_id = static_cast<int>(armor_type);
            const auto it = std::find(config.AimTargetPriority.begin(), config.AimTargetPriority.end(), armor_id);
            if (it == config.AimTargetPriority.end()) {
                return static_cast<int>(config.AimTargetPriority.size());
            }
            return static_cast<int>(std::distance(config.AimTargetPriority.begin(), it));
        };

        double best_score = -std::numeric_limits<double>::infinity();
        std::optional<AimCandidate> best_candidate;
        for (const auto& candidate : candidates) {
            const int rank = get_priority_rank(candidate.Armor);
            const double priority_score = config.AimTargetPriority.empty()
                ? 0.0
                : static_cast<double>(config.AimTargetPriority.size() - rank) /
                    static_cast<double>(config.AimTargetPriority.size());
            const double distance_score = (std::isfinite(candidate.Distance) && candidate.Distance > 0.0f)
                ? 1.0 / (0.1 + static_cast<double>(candidate.Distance))
                : 0.0;
            const double health_score = 1.0 -
                static_cast<double>(candidate.Health) / static_cast<double>(std::max<std::uint16_t>(1U, max_health));

            double score = 0.0;
            score += autonomy.PriorityWeight * priority_score;
            score += autonomy.DistanceWeight * distance_score;
            score += autonomy.LowHealthWeight * health_score;
            if (candidate.Armor == targetArmor.Type) {
                score += autonomy.CurrentTargetBonus;
            }
            if (candidate.Armor == ArmorType::Hero) {
                score += autonomy.HeroBonus;
            } else if (candidate.Armor == ArmorType::Sentry) {
                score += autonomy.SentryBonus;
            }

            if (score > best_score) {
                best_score = score;
                best_candidate = candidate;
            }
        }

        if (!best_candidate.has_value()) {
            return false;
        }
        targetArmor.Type = best_candidate->Armor;
        targetArmor.Distance = best_candidate->Distance;
        return true;
    }

    void Application::SetAimTargetNormal() {
        auto set_target = [&](const ArmorType armor_type, const UnitType unit_type) {
            targetArmor.Type = armor_type;
            targetArmor.Distance = enemyRobots[unit_type].distance_;
        };
        const auto is_armor_ignored = [this](const ArmorType armor_type) -> bool {
            return IsIgnoredArmorType(config.AimTargetIgnore, armor_type);
        };
        const auto has_target = [&](const UnitType unit_type) -> bool {
            return !IsIgnoredUnitType(config.AimTargetIgnore, unit_type) &&
                   std::find(hitableTargets.begin(), hitableTargets.end(), unit_type) != hitableTargets.end();
        };
        const bool infantry1_find = has_target(UnitType::Infantry1);
        const bool infantry2_find = has_target(UnitType::Infantry2);

        if (TrySetAimTargetByAutonomy()) {
            return;
        }

        if (!hitableTargets.empty()) {
            for (const auto armor_id : config.AimTargetPriority) {
                const auto armor_type = ArmorTypeFromPriorityId(armor_id);
                if (!armor_type.has_value()) {
                    continue;
                }
                if (is_armor_ignored(*armor_type)) {
                    continue;
                }
                if ((*armor_type == ArmorType::Infantry1 || *armor_type == ArmorType::Infantry2) &&
                    infantry1_find && infantry2_find) {
                    // 兼容旧逻辑：步兵1/2同时可打时，优先近距离；距离接近时看血量更低者。
                    const auto distance1 = enemyRobots[UnitType::Infantry1].distance_;
                    const auto distance2 = enemyRobots[UnitType::Infantry2].distance_;
                    const auto delta_distance = distance1 - distance2;
                    if (std::fabs(delta_distance) > 1.0f) {
                        if (distance1 < distance2) {
                            set_target(ArmorType::Infantry1, UnitType::Infantry1);
                        } else {
                            set_target(ArmorType::Infantry2, UnitType::Infantry2);
                        }
                    } else {
                        const auto health1 = enemyRobots[UnitType::Infantry1].currentHealth_;
                        const auto health2 = enemyRobots[UnitType::Infantry2].currentHealth_;
                        if (health1 < health2) {
                            set_target(ArmorType::Infantry1, UnitType::Infantry1);
                        } else {
                            set_target(ArmorType::Infantry2, UnitType::Infantry2);
                        }
                    }
                    return;
                }
                const auto unit_type = UnitTypeFromArmorType(*armor_type);
                if (!unit_type.has_value()) {
                    continue;
                }
                if (!has_target(*unit_type)) {
                    continue;
                }
                set_target(*armor_type, *unit_type);
                return;
            }

            // 配置优先级没有命中时，回退到最近目标
            std::optional<UnitType> nearest_unit;
            for (const auto unit_type : hitableTargets) {
                if (IsIgnoredUnitType(config.AimTargetIgnore, unit_type)) {
                    continue;
                }
                if (!nearest_unit.has_value() ||
                    enemyRobots[unit_type].distance_ < enemyRobots[*nearest_unit].distance_) {
                    nearest_unit = unit_type;
                }
            }
            if (nearest_unit.has_value()) {
                const auto armor_type = ArmorTypeFromUnitType(*nearest_unit);
                if (armor_type.has_value()) {
                    set_target(*armor_type, *nearest_unit);
                    return;
                }
            }
        }

        for (const auto armor_id : config.AimTargetPriority) {
            const auto armor_type = ArmorTypeFromPriorityId(armor_id);
            if (!armor_type.has_value() || is_armor_ignored(*armor_type)) {
                continue;
            }
            targetArmor.Type = *armor_type;
            targetArmor.Distance = 30;
            return;
        }
        targetArmor.Type = ArmorType::UnKnown;
        targetArmor.Distance = 30;
    }

    void Application::CheckDebug() {
        if (config.AimDebugSettings.ForceBuff) aimMode = AimMode::Buff;
        else if(config.AimDebugSettings.ForceOutpost) aimMode = AimMode::Outpost;
        /*------------打印日志---------*/
        if(aimMode == AimMode::AutoAim) LoggerPtr->Info("AimMode: AutoAim");
        else if(aimMode == AimMode::Buff) LoggerPtr->Info("AimMode: Buff");
        else if(aimMode == AimMode::Outpost) LoggerPtr->Info("AimMode: Outpost");
        else if(aimMode == AimMode::RotateScan) LoggerPtr->Info("AimMode: RotateScan");
    }
}


namespace BehaviorTree {

    bool Application::IsLeagueRouteCompatEnabled() const noexcept {
        return IsLeagueProfile() &&
            !config.NaviSettings.UseXY &&
            config.LeagueStrategySettings.EnableRouteCompat;
    }

    bool Application::IsLeagueGoalSwitchBetween2And3(
        const std::uint8_t from_goal_id,
        const std::uint8_t to_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const noexcept {
        const auto goal_2 = ResolveGoalId(LangYa::Recovery.ID, goal_team, apply_team_offset);
        const auto goal_3 = ResolveGoalId(LangYa::BuffShoot.ID, goal_team, apply_team_offset);
        return (from_goal_id == goal_2 && to_goal_id == goal_3) ||
            (from_goal_id == goal_3 && to_goal_id == goal_2);
    }

    bool Application::TickLeagueRouteCompat(
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        if (!IsLeagueRouteCompatEnabled() || !leagueRouteCompatActive_) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < leagueRouteCompatUntil_) {
            SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, goal_team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            return true;
        }

        leagueRouteCompatActive_ = false;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::time_point{};
        if (!leagueRouteCompatHasPendingGoal_) {
            return false;
        }

        const auto pending_base_goal = leagueRouteCompatPendingBaseGoal_;
        const int pending_hold_sec = std::max(1, leagueRouteCompatPendingHoldSec_);
        leagueRouteCompatHasPendingGoal_ = false;

        SetPositionByBaseGoal(pending_base_goal, goal_team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{pending_hold_sec});
        speedLevel = 1;
        LoggerPtr->Info(
            "League route compat finished: via goal done, continue goal={} hold={}s.",
            static_cast<int>(naviCommandGoal),
            pending_hold_sec);
        return true;
    }

    void Application::StartLeagueRouteCompat(
        const std::uint8_t pending_base_goal,
        const int pending_hold_sec,
        const UnitTeam goal_team,
        const bool apply_team_offset,
        const char* reason) {
        if (!IsLeagueRouteCompatEnabled()) {
            return;
        }
        const int safe_pending_hold_sec = std::max(1, pending_hold_sec);
        leagueRouteCompatActive_ = true;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::now() + std::chrono::seconds(kLeagueRouteCompatViaHoldSec);
        leagueRouteCompatHasPendingGoal_ = true;
        leagueRouteCompatPendingBaseGoal_ = pending_base_goal;
        leagueRouteCompatPendingHoldSec_ = safe_pending_hold_sec;

        SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, goal_team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{1});
        speedLevel = 1;
        LoggerPtr->Info(
            "League route compat {}: via goal={} for {}s, then goal={} hold={}s.",
            reason ? reason : "start",
            static_cast<int>(naviCommandGoal),
            kLeagueRouteCompatViaHoldSec,
            static_cast<int>(ResolveGoalId(pending_base_goal, goal_team, apply_team_offset)),
            safe_pending_hold_sec);
    }

    std::uint8_t Application::ResolveGoalId(
        const std::uint8_t base_goal_id,
        const UnitTeam team,
        const bool apply_team_offset) const noexcept {
        return AreaManager::ResolveGoalId(base_goal_id, team, apply_team_offset);
    }

    bool Application::IsNaviGoalAreaScopeEnabled() const noexcept {
        return areaManager_.IsGoalAreaScopeEnabled();
    }

    bool Application::IsNaviGoalAllowedByAreaScope(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        const auto result = areaManager_.CheckGoalAreaScope(
            base_goal_id,
            goal_team,
            my_team,
            enemy_team);
        if (result.Allowed) {
            return true;
        }
        if (result.ResolvedArea.has_value() &&
            result.ResolvedArea->UsedNearestFallback &&
            LoggerPtr) {
            LoggerPtr->Debug(
                "DecisionAutonomy[navi_goal_area]: goal={} team={} mapped to nearest area='{}'.",
                static_cast<int>(base_goal_id),
                result.ScopeName,
                Area::MainAreaKindName(result.ResolvedArea->Kind));
        }
        return false;
    }

    bool Application::IsHighlandCompatEnabled() const noexcept {
        return areaManager_.IsHighlandCompatEnabled();
    }

    bool Application::IsHighlandCompatTarget(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team) const {
        return areaManager_.IsHighlandCompatTarget(base_goal_id, goal_team);
    }

    bool Application::IsSelfInMainArea(
        const UnitTeam area_team,
        const Area::MainAreaKind kind) const {
        if (area_team != UnitTeam::Red && area_team != UnitTeam::Blue) {
            return false;
        }
        if (!hasReceivedSentryPosition_) {
            return false;
        }
        if (lastSentryPositionRxTime_.time_since_epoch().count() == 0 ||
            std::chrono::steady_clock::now() - lastSentryPositionRxTime_ > std::chrono::seconds(2)) {
            return false;
        }
        const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        if (self_x <= 0 || self_y <= 0) {
            return false;
        }
        return AreaManager::IsPositionInMainArea(area_team, kind, self_x, self_y);
    }

    bool Application::IsSelfInRoadlandFollowModeArea(const UnitTeam area_team) const {
        if (area_team != UnitTeam::Red && area_team != UnitTeam::Blue) {
            return false;
        }
        if (!hasReceivedSentryPosition_) {
            return false;
        }
        if (lastSentryPositionRxTime_.time_since_epoch().count() == 0 ||
            std::chrono::steady_clock::now() - lastSentryPositionRxTime_ > std::chrono::seconds(2)) {
            return false;
        }
        const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        if (self_x <= 0 || self_y <= 0) {
            return false;
        }
        return AreaManager::IsPositionInRoadlandFollowModeArea(area_team, self_x, self_y);
    }

    bool Application::IsNaviExternalStatusFreshForGoal(
        const std::chrono::steady_clock::time_point last_rx,
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        if (!naviExternalStatusGoalInitialized_ ||
            naviExternalStatusGoalId_ != goal_id ||
            naviExternalStatusGoalPosition_.x != goal_position.x ||
            naviExternalStatusGoalPosition_.y != goal_position.y ||
            naviCommandGoal != goal_id ||
            last_rx.time_since_epoch().count() == 0 ||
            last_rx < naviExternalStatusGoalStartTime_) {
            return false;
        }
        return std::chrono::steady_clock::now() - last_rx <=
            std::chrono::milliseconds(kNaviExternalStatusTimeoutMs);
    }

    std::optional<bool> Application::GetExternalNaviReachForGoal(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        if (!hasReceivedNaviReach_ ||
            !IsNaviExternalStatusFreshForGoal(lastNaviReachRxTime_, goal_id, goal_position)) {
            return std::nullopt;
        }
        return naviReach;
    }

    std::optional<bool> Application::GetExternalNaviReachableForGoal(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        if (!hasReceivedNaviReachable_ ||
            !IsNaviExternalStatusFreshForGoal(lastNaviReachableRxTime_, goal_id, goal_position)) {
            return std::nullopt;
        }
        return naviReachable;
    }

    void Application::UpdateNaviExternalStatusGoal(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) {
        const bool same_goal =
            naviExternalStatusGoalInitialized_ &&
            naviExternalStatusGoalId_ == goal_id &&
            naviExternalStatusGoalPosition_.x == goal_position.x &&
            naviExternalStatusGoalPosition_.y == goal_position.y;
        if (same_goal) {
            return;
        }
        naviExternalStatusGoalInitialized_ = true;
        naviExternalStatusGoalId_ = goal_id;
        naviExternalStatusGoalPosition_ = goal_position;
        naviExternalStatusGoalStartTime_ = std::chrono::steady_clock::now();
    }

    bool Application::IsBaseGoalArrived(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const {
        if (!AreaManager::IsValidBaseGoalId(base_goal_id)) {
            return false;
        }
        const auto goal_id = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        const auto goal_point = AreaManager::GoalPointByBaseId(base_goal_id, goal_team);
        const auto external_reachable = GetExternalNaviReachableForGoal(goal_id, goal_point);
        if (external_reachable.has_value() && !*external_reachable) {
            return false;
        }
        const auto external_reach = GetExternalNaviReachForGoal(goal_id, goal_point);
        if (external_reach.has_value()) {
            return *external_reach;
        }
        if (!hasReceivedSentryPosition_) {
            return false;
        }
        if (lastSentryPositionRxTime_.time_since_epoch().count() == 0 ||
            std::chrono::steady_clock::now() - lastSentryPositionRxTime_ > std::chrono::seconds(2)) {
            return false;
        }
        const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        if (self_x <= 0 || self_y <= 0) {
            return false;
        }
        const auto arrive_distance = static_cast<std::uint16_t>(
            std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm));
        return AreaManager::DistanceSq(
            self_x,
            self_y,
            static_cast<int>(goal_point.x),
            static_cast<int>(goal_point.y)) <=
            static_cast<double>(arrive_distance) * static_cast<double>(arrive_distance);
    }

    bool Application::IsBaseGoalExternallyUnreachable(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const {
        if (!AreaManager::IsValidBaseGoalId(base_goal_id)) {
            return false;
        }
        const auto goal_id = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        const auto goal_point = AreaManager::GoalPointByBaseId(base_goal_id, goal_team);
        const auto external_reachable = GetExternalNaviReachableForGoal(goal_id, goal_point);
        return external_reachable.has_value() && !*external_reachable;
    }

    bool Application::IsHighlandCompatArrived(const UnitTeam goal_team) const {
        return IsBaseGoalArrived(LangYa::Highland.ID, goal_team);
    }

    void Application::ResetRegionalAreaControlOverride() noexcept {
        faceModeManager_.ResetControl();
    }

    void Application::ApplyAimModeFaceTarget(const UnitTeam target_team) {
        (void)faceModeManager_.PublishAimTarget(
            aimMode,
            target_team,
            pub_face_mode_target_raw_);
    }

    bool Application::TrySetAimModeTaskGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const char* reason) {
        if (aimMode != AimMode::Buff && aimMode != AimMode::Outpost) {
            return false;
        }

        TrySetScopedPositionByBaseGoal(
            LangYa::BuffOutpost.ID,
            my_team,
            my_team,
            enemy_team,
            true,
            reason);
        const auto face_target_team = aimMode == AimMode::Outpost ? enemy_team : my_team;
        ApplyAimModeFaceTarget(face_target_team);
        naviCommandIntervalClock.reset(Seconds{2});
        speedLevel = 1;
        return true;
    }

    void Application::ApplyRegionalAreaTaskControl(const RegionalAreaTaskTickResult& result) {
        faceModeManager_.ApplyRegionalTaskResult(result, pub_face_mode_target_raw_);
        gimbalControlData.FireCode.FollowMode = result.FollowMode ? 1 : 0;
        if (result.SuppressFire) {
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
            buffAimData.FireStatus = false;
        }
    }

    bool Application::RequestRoadlandSafeReturn(const char* reason) {
        if (!areaManager_.RegionalAreaTaskActive() ||
            areaManager_.RegionalAreaTask().Type != RegionalAreaTaskType::MyRoadland) {
            return false;
        }
        const auto before_phase = areaManager_.RegionalAreaTask().Phase;
        areaManager_.RequestRoadlandReturnToBase(std::chrono::steady_clock::now());
        const auto after_phase = areaManager_.RegionalAreaTask().Phase;
        const bool changed = before_phase != after_phase;
        if (changed && LoggerPtr) {
            LoggerPtr->Info(
                "RegionalAreaTask[MyRoadland] safe return requested: {}.",
                reason ? reason : "higher priority");
        }
        return changed;
    }

    bool Application::TickRegionalAreaTask(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        if (!areaManager_.RegionalAreaTaskActive()) {
            return false;
        }

        const auto active_task_type = areaManager_.RegionalAreaTask().Type;
        const auto now = std::chrono::steady_clock::now();
        const bool active_low_priority_task =
            active_task_type == RegionalAreaTaskType::MyBase ||
            active_task_type == RegionalAreaTaskType::MyRoadland ||
            active_task_type == RegionalAreaTaskType::CommonCentral;
        const bool active_roadland_task = active_task_type == RegionalAreaTaskType::MyRoadland;
        const bool can_yield_to_higher_priority = areaManager_.RegionalAreaTaskCanYieldToHigherPriority();
        if (active_low_priority_task && (aimMode == AimMode::Buff || aimMode == AimMode::Outpost)) {
            if (!can_yield_to_higher_priority) {
                // Roadland crossing is a bound control segment; do not release FollowMode/FaceMode
                // until the far endpoint or timeout protection completes it.
            } else if (active_roadland_task) {
                RequestRoadlandSafeReturn("aim mode has higher priority");
            } else {
                areaManager_.ClearRegionalAreaTask();
                defaultStrategyManager_.RecordRegionalAreaResult(
                    active_task_type,
                    "canceled",
                    now,
                    config.RegionalAreaTaskSettings.DefaultPolicy);
                ResetRegionalAreaControlOverride();
                gimbalControlData.FireCode.FollowMode = 0;
                if (LoggerPtr) {
                    LoggerPtr->Info("RegionalAreaTask canceled: aim mode has higher priority.");
                }
                return false;
            }
        }
        if (active_low_priority_task) {
            if (active_roadland_task) {
                const auto threat = EvaluateRegionalDefenseThreat(my_team, enemy_team);
                if (threat.has_value() && can_yield_to_higher_priority) {
                    const bool soft_threat_blocked =
                        !threat->HardThreat &&
                        (aimMode == AimMode::Buff || aimMode == AimMode::Outpost ||
                         !naviCommandIntervalClock.trigger());
                    if (!soft_threat_blocked) {
                        RequestRoadlandSafeReturn("regional defense has higher priority");
                    }
                }
            } else if (TrySetRegionalDefenseGoal(my_team, enemy_team)) {
                areaManager_.ClearRegionalAreaTask();
                defaultStrategyManager_.RecordRegionalAreaResult(
                    active_task_type,
                    "canceled",
                    now,
                    config.RegionalAreaTaskSettings.DefaultPolicy);
                ResetRegionalAreaControlOverride();
                gimbalControlData.FireCode.FollowMode = 0;
                if (LoggerPtr) {
                    LoggerPtr->Info("RegionalAreaTask canceled: regional defense has higher priority.");
                }
                return true;
            }
        }

        const bool apply_team_offset = areaManager_.RegionalAreaTask().ApplyTeamOffset;
        const auto goal_team = areaManager_.RegionalAreaTask().GoalTeam;
        const auto current_base_goal = areaManager_.RegionalAreaTask().CurrentBaseGoal;
        auto referee_value_fresh = [&](const bool received, const std::chrono::steady_clock::time_point last_rx) {
            return received &&
                last_rx.time_since_epoch().count() != 0 &&
                now - last_rx <= std::chrono::seconds(2);
        };
        const auto& roadland_setting = config.RegionalAreaTaskSettings.MyRoadland;
        const auto& central_setting = config.RegionalAreaTaskSettings.CommonCentral;
        const bool roadland_health_known =
            referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
        const bool roadland_ammo_known =
            referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime);
        const bool roadland_data_unhealthy =
            roadland_health_known &&
            roadland_ammo_known &&
            (myselfHealth < static_cast<std::uint16_t>(std::max(0, roadland_setting.HealthyHpMin)) ||
             ammoLeft < static_cast<std::uint16_t>(std::max(0, roadland_setting.HealthyAmmoMin)));
        const bool central_health_known =
            referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
        const bool central_ammo_known =
            referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime);
        const bool central_data_unhealthy =
            central_health_known &&
            central_ammo_known &&
            (myselfHealth < static_cast<std::uint16_t>(std::max(0, central_setting.HealthyHpMin)) ||
             ammoLeft < static_cast<std::uint16_t>(std::max(0, central_setting.HealthyAmmoMin)));
        const auto result = areaManager_.TickRegionalAreaTask(
            RegionalAreaTaskTickInput{
                .Setting = config.RegionalAreaTaskSettings,
                .Now = now,
                .HighlandArrived = IsBaseGoalArrived(LangYa::Highland.ID, goal_team, apply_team_offset),
                .HighlandUnreachable = IsBaseGoalExternallyUnreachable(LangYa::Highland.ID, goal_team, apply_team_offset),
                .BuffShootArrived = IsBaseGoalArrived(LangYa::BuffShoot.ID, goal_team, apply_team_offset),
                .BuffShootUnreachable = IsBaseGoalExternallyUnreachable(LangYa::BuffShoot.ID, goal_team, apply_team_offset),
                .HoleRoadArrived = IsBaseGoalArrived(LangYa::HoleRoad.ID, goal_team, apply_team_offset),
                .HoleRoadUnreachable = IsBaseGoalExternallyUnreachable(LangYa::HoleRoad.ID, goal_team, apply_team_offset),
                .CurrentBaseGoalArrived = IsBaseGoalArrived(current_base_goal, goal_team, apply_team_offset),
                .CurrentBaseGoalUnreachable = IsBaseGoalExternallyUnreachable(current_base_goal, goal_team, apply_team_offset),
                .RoadlandCentralToBaseArrived = IsBaseGoalArrived(LangYa::CentralToBase.ID, goal_team, apply_team_offset),
                .RoadlandCentralToBaseUnreachable = IsBaseGoalExternallyUnreachable(LangYa::CentralToBase.ID, goal_team, apply_team_offset),
                .RoadlandBaseToCentralArrived = IsBaseGoalArrived(LangYa::BaseToCentral.ID, goal_team, apply_team_offset),
                .RoadlandBaseToCentralUnreachable = IsBaseGoalExternallyUnreachable(LangYa::BaseToCentral.ID, goal_team, apply_team_offset),
                .RoadlandShouldLeave = active_roadland_task && roadland_data_unhealthy,
                .CentralShouldLeave =
                    active_task_type == RegionalAreaTaskType::CommonCentral && central_data_unhealthy
            });

        if (result.Completed) {
            ApplyRegionalAreaTaskControl(result);
            defaultStrategyManager_.RecordRegionalAreaResult(
                result.Type,
                result.Reason.empty() ? std::string_view{"done"} : std::string_view{result.Reason},
                now,
                config.RegionalAreaTaskSettings.DefaultPolicy);
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "RegionalAreaTask completed type={} phase={} reason={}, follow_mode off.",
                    RegionalAreaTaskTypeToString(result.Type),
                    RegionalAreaTaskPhaseToString(result.Phase),
                    result.Reason.empty() ? "done" : result.Reason.c_str());
            }
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            return true;
        }

        if (!result.Active || !result.SetGoal) {
            return false;
        }

        ApplyRegionalAreaTaskControl(result);
        SetPositionByBaseGoal(result.BaseGoalId, result.GoalTeam, result.ApplyTeamOffset);
        if (result.ResetNaviHold) {
            naviCommandIntervalClock.reset(Seconds{std::max(1, result.NaviHoldSec)});
        }
        speedLevel = 1;
        return true;
    }

    bool Application::TryStartRegionalAreaTaskForGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const bool apply_team_offset,
        const char* reason) {
        if (!config.RegionalAreaTaskSettings.Enable ||
            (!config.RegionalAreaTaskSettings.MyHighland.Enable &&
             !config.RegionalAreaTaskSettings.MyBase.Enable &&
             !config.RegionalAreaTaskSettings.MyRoadland.Enable &&
             !config.RegionalAreaTaskSettings.CommonCentral.Enable) ||
            areaManager_.HighlandTransitionActive() ||
            areaManager_.RegionalAreaTaskActive()) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool self_position_fresh =
            hasReceivedSentryPosition_ &&
            lastSentryPositionRxTime_.time_since_epoch().count() != 0 &&
            now - lastSentryPositionRxTime_ <= std::chrono::seconds(2);
        const int self_x = self_position_fresh
            ? static_cast<int>(friendRobots[UnitType::Sentry].position_.X)
            : 0;
        const int self_y = self_position_fresh
            ? static_cast<int>(friendRobots[UnitType::Sentry].position_.Y)
            : 0;
        const auto plan = areaManager_.PlanRegionalAreaTaskForGoal(
            base_goal_id,
            goal_team,
            my_team,
            apply_team_offset,
            self_position_fresh && self_x > 0 && self_y > 0,
            self_x,
            self_y,
            IsSelfInMainArea(my_team, Area::MainAreaKind::Highland));
        if (!plan.has_value()) {
            return false;
        }
        if (plan->Type == RegionalAreaTaskType::MyRoadland ||
            plan->Type == RegionalAreaTaskType::CommonCentral) {
            auto referee_value_fresh = [&](const bool received, const std::chrono::steady_clock::time_point last_rx) {
                return received &&
                    last_rx.time_since_epoch().count() != 0 &&
                    now - last_rx <= std::chrono::seconds(2);
            };
            const int healthy_hp_min = plan->Type == RegionalAreaTaskType::MyRoadland
                ? config.RegionalAreaTaskSettings.MyRoadland.HealthyHpMin
                : config.RegionalAreaTaskSettings.CommonCentral.HealthyHpMin;
            const int healthy_ammo_min = plan->Type == RegionalAreaTaskType::MyRoadland
                ? config.RegionalAreaTaskSettings.MyRoadland.HealthyAmmoMin
                : config.RegionalAreaTaskSettings.CommonCentral.HealthyAmmoMin;
            const bool data_healthy =
                referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime) &&
                referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime) &&
                myselfHealth >= static_cast<std::uint16_t>(std::max(0, healthy_hp_min)) &&
                ammoLeft >= static_cast<std::uint16_t>(std::max(0, healthy_ammo_min));
            if (!data_healthy) {
                return false;
            }
        }
        if ((plan->Type == RegionalAreaTaskType::MyHighland &&
             !config.RegionalAreaTaskSettings.MyHighland.Enable) ||
            (plan->Type == RegionalAreaTaskType::MyBase &&
             !config.RegionalAreaTaskSettings.MyBase.Enable) ||
            (plan->Type == RegionalAreaTaskType::MyRoadland &&
             !config.RegionalAreaTaskSettings.MyRoadland.Enable) ||
            (plan->Type == RegionalAreaTaskType::CommonCentral &&
             !config.RegionalAreaTaskSettings.CommonCentral.Enable)) {
            return false;
        }

        areaManager_.StartRegionalAreaTask(*plan, now);
        if (LoggerPtr) {
            if (plan->Type == RegionalAreaTaskType::MyBase) {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyBase] start from goal={} reason={}: start_base_goal={} route=CastleLeft1->CastleLeft2->CastleRight2->CastleRight1.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task",
                    static_cast<int>(plan->InitialBaseGoal));
            } else if (plan->Type == RegionalAreaTaskType::MyRoadland) {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyRoadland] start from goal={} reason={}: CentralToBase -> BaseToCentral guarded crossing.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task");
            } else if (plan->Type == RegionalAreaTaskType::CommonCentral) {
                LoggerPtr->Info(
                    "RegionalAreaTask[CommonCentral] start from goal={} reason={}: start_base_goal={} team={} patrol=own OutpostArea->RightShoot->BuffAround2->LeftShoot->OutpostShoot->enemy RightShoot->OccupyArea->OutpostShoot.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task",
                    static_cast<int>(ResolveGoalId(plan->InitialBaseGoal, plan->InitialGoalTeam, apply_team_offset)),
                    plan->InitialGoalTeam == my_team ? "my" : "enemy");
            } else {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyHighland] start from goal={} reason={}: Highland -> BuffShoot hold={}s -> HoleRoad.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task",
                    config.RegionalAreaTaskSettings.MyHighland.BuffShootHoldSec);
            }
        }
        return TickRegionalAreaTask(my_team, UnitTeam::Unknown);
    }

    bool Application::TickNaviAreaTransition() {
        if (!areaManager_.HighlandTransitionActive()) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto& runtime = areaManager_.TransitionRuntime();
        const auto via_base_goal = AreaManager::IsValidBaseGoalId(runtime.ViaBaseGoal)
            ? runtime.ViaBaseGoal
            : LangYa::Highland.ID;
        const auto via_goal_id = ResolveGoalId(
            via_base_goal,
            runtime.GoalTeam,
            runtime.ApplyTeamOffset);
        const auto via_goal_position = AreaManager::GoalPointByBaseId(via_base_goal, runtime.GoalTeam);
        const auto external_reachable =
            GetExternalNaviReachableForGoal(via_goal_id, via_goal_position);
        const bool route_unreachable = external_reachable.has_value() && !*external_reachable;
        const bool arrived = !route_unreachable &&
            IsBaseGoalArrived(
                via_base_goal,
                runtime.GoalTeam,
                runtime.ApplyTeamOffset);
        const auto tick = areaManager_.TickHighlandTransition(now, route_unreachable, arrived);
        gimbalControlData.FireCode.FollowMode = tick.FollowMode ? 1 : 0;

        if (tick.Action == NaviAreaTransitionTickAction::ContinueVia) {
            SetPositionByBaseGoal(
                tick.ViaBaseGoal,
                tick.GoalTeam,
                tick.ApplyTeamOffset);
            return true;
        }

        if (tick.Action == NaviAreaTransitionTickAction::FinishNoPending) {
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Area transition {} {}: via goal={} done, follow_mode off.",
                    NaviAreaTransitionKindToString(tick.Kind),
                    tick.CompletionReason,
                    static_cast<int>(tick.ViaBaseGoal));
            }
            return false;
        }

        if (tick.Action == NaviAreaTransitionTickAction::FinishWithPending) {
            SetPositionByBaseGoal(tick.PendingBaseGoal, tick.GoalTeam, tick.ApplyTeamOffset);
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Area transition {} {}: via goal={} done, continue goal={}, follow_mode off.",
                    NaviAreaTransitionKindToString(tick.Kind),
                    tick.CompletionReason,
                    static_cast<int>(tick.ViaBaseGoal),
                    static_cast<int>(naviCommandGoal));
            }
            return true;
        }
        return false;
    }

    bool Application::TryStartNaviAreaTransition(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const bool apply_team_offset,
        const char* reason) {
        if (areaManager_.HighlandTransitionActive() ||
            !areaManager_.IsHighlandCompatEnabled() ||
            !AreaManager::IsValidBaseGoalId(base_goal_id) ||
            goal_team == UnitTeam::Unknown ||
            my_team == UnitTeam::Unknown) {
            return false;
        }
        const auto plan = areaManager_.PlanHighlandTransition(
            base_goal_id,
            goal_team,
            my_team,
            apply_team_offset,
            naviCommandGoal,
            IsHighlandCompatArrived(goal_team),
            IsSelfInMainArea(my_team, Area::MainAreaKind::Highland));
        if (!plan.has_value()) {
            return false;
        }
        if (plan->CheckViaAlreadyArrived &&
            IsBaseGoalArrived(plan->ViaBaseGoal, plan->GoalTeam, plan->ApplyTeamOffset)) {
            return false;
        }

        areaManager_.StartHighlandTransition(*plan, std::chrono::steady_clock::now());
        gimbalControlData.FireCode.FollowMode = 1;

        SetPositionByBaseGoal(plan->ViaBaseGoal, plan->GoalTeam, plan->ApplyTeamOffset);
        if (LoggerPtr) {
            LoggerPtr->Info(
                "Area transition {} {}: via goal={} then goal={} follow_mode on.",
                NaviAreaTransitionKindToString(plan->Kind),
                reason ? reason : "start",
                static_cast<int>(naviCommandGoal),
                plan->HasPendingGoal
                    ? static_cast<int>(ResolveGoalId(plan->PendingBaseGoal, plan->GoalTeam, plan->ApplyTeamOffset))
                    : -1);
        }
        return true;
    }

    bool Application::IsRegionalDefenseAimSuppressActive() const noexcept {
        return areaManager_.IsRegionalDefenseAimSuppressActive(std::chrono::steady_clock::now());
    }

    bool Application::IsEnemyPositionFresh(
        const UnitType unit_type,
        const int fresh_ms) const {
        const auto index = static_cast<std::size_t>(unit_type);
        if (index >= lastEnemyPositionRxTime_.size()) {
            return false;
        }
        const auto& last_rx = lastEnemyPositionRxTime_[index];
        if (last_rx.time_since_epoch().count() == 0) {
            return false;
        }
        return std::chrono::steady_clock::now() - last_rx <=
            std::chrono::milliseconds(std::max(1, fresh_ms));
    }

    std::optional<RegionalDefenseThreat> Application::EvaluateRegionalDefenseThreat(
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        const auto& defense = config.RegionalDefenseSettings;
        if (!defense.Enable || IsLeagueProfile() || IsShowcasePatrolEnabled()) {
            return std::nullopt;
        }

        std::vector<RegionalDefenseEnemyPosition> fresh_enemies;
        for (const auto unit_type : RobotLists) {
            if (!IsEnemyPositionFresh(unit_type, defense.EnemyPositionFreshMs)) {
                continue;
            }
            const int enemy_x = static_cast<int>(enemyRobots[unit_type].position_.X);
            const int enemy_y = static_cast<int>(enemyRobots[unit_type].position_.Y);
            if (enemy_x <= 0 || enemy_y <= 0) {
                continue;
            }
            // /ly/position/data is normalized into official-field centimeters in SubscribeMessage.cpp.
            // Keep regional defense area tests in that frame; do not mix map/odom coordinates here.
            fresh_enemies.push_back(RegionalDefenseEnemyPosition{.X = enemy_x, .Y = enemy_y});
        }

        const auto threat = areaManager_.AnalyzeRegionalDefenseThreat(
            my_team,
            enemy_team,
            defense.EnableSoftEnemySideThreat,
            fresh_enemies);

        if (!threat.HardThreat && !threat.SoftEnemySideThreat) {
            return std::nullopt;
        }
        return threat;
    }

    bool Application::TrySetRegionalDefenseGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& defense = config.RegionalDefenseSettings;
        const auto maybe_threat = EvaluateRegionalDefenseThreat(my_team, enemy_team);
        if (!maybe_threat.has_value()) {
            regionalDefenseSearchKind_ = RegionalDefenseSearchKind::None;
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchBaseGoal_ = LangYa::Home.ID;
            regionalDefenseSearchStartTime_ = {};
            return false;
        }
        const auto threat = *maybe_threat;

        if (!threat.HardThreat) {
            if (aimMode == AimMode::Buff || aimMode == AimMode::Outpost ||
                !naviCommandIntervalClock.trigger()) {
                return false;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const bool strong_resource =
            myselfHealth >= defense.StrongHealthMin &&
            ammoLeft >= defense.StrongAmmoMin;

        RegionalDefenseSearchKind search_kind = RegionalDefenseSearchKind::None;
        const char* reason = "regional_defense";
        int hold_sec = defense.HardHoldSec;
        std::vector<std::uint8_t> candidates;

        if (threat.HardThreat) {
            if (threat.OwnBaseCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnBase;
                reason = threat.OwnBaseCount >= defense.MultiEnemyBaseCount
                    ? "own_base_multi"
                    : (strong_resource ? "own_base_chase" : "own_base_guard");
                candidates = {
                    LangYa::Castle.ID,
                    LangYa::CastleRight2.ID,
                    LangYa::CastleLeft2.ID,
                    LangYa::CastleRight1.ID,
                    LangYa::CastleLeft1.ID
                };
            } else if (threat.OwnHighlandCount > 0 && threat.OwnRoadlandCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnHighlandRoadland;
                reason = "own_highland_roadland";
                candidates = {LangYa::Castle.ID, LangYa::HoleRoad.ID, LangYa::CastleRight2.ID};
            } else if (threat.OwnRoadlandCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnRoadland;
                reason = "own_roadland";
                candidates = {LangYa::CastleRight2.ID, LangYa::CastleRight1.ID, LangYa::Castle.ID};
            } else if (threat.OwnHighlandCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnHighland;
                reason = "own_highland";
                candidates = {LangYa::HoleRoad.ID, LangYa::Highland.ID, LangYa::Castle.ID};
            } else if (threat.CommonCentralCount > 0) {
                search_kind = RegionalDefenseSearchKind::CommonCentral;
                reason = "common_central";
                candidates = {LangYa::HoleRoad.ID, LangYa::Castle.ID};
            }
        } else {
            search_kind = RegionalDefenseSearchKind::EnemySideSoft;
            reason = threat.EnemyRoadlandCount > 0 ? "enemy_roadland_soft" : "enemy_highland_soft";
            hold_sec = defense.SoftHoldSec;
            candidates = {LangYa::HoleRoad.ID, LangYa::Highland.ID, LangYa::Castle.ID};
        }

        if (candidates.empty()) {
            regionalDefenseSearchKind_ = RegionalDefenseSearchKind::None;
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchBaseGoal_ = LangYa::Home.ID;
            regionalDefenseSearchStartTime_ = {};
            return false;
        }

        if (threat.HardThreat) {
            areaManager_.StartRegionalDefenseSuppress(now, defense.HardHoldSec);
            aimMode = AimMode::RotateScan;
        }

        const bool same_search = regionalDefenseSearchKind_ == search_kind;
        if (!same_search) {
            regionalDefenseSearchKind_ = search_kind;
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchBaseGoal_ = candidates.front();
            regionalDefenseSearchStartTime_ = now;
        } else if (regionalDefenseSearchIndex_ >= candidates.size()) {
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchBaseGoal_ = candidates.front();
            regionalDefenseSearchStartTime_ = now;
        } else {
            regionalDefenseSearchBaseGoal_ = candidates[regionalDefenseSearchIndex_];
        }

        auto autoaim_recently_seen = [&]() {
            if (!autoAimData.HasLatchedAngles ||
                autoAimData.LastValidTime.time_since_epoch().count() == 0) {
                return false;
            }
            return now - autoAimData.LastValidTime <=
                std::chrono::seconds(std::max(1, defense.SearchNoTargetSec));
        };

        auto current_goal_done = [&](const std::uint8_t base_goal_id) {
            return IsBaseGoalArrived(base_goal_id, my_team, true) ||
                IsBaseGoalExternallyUnreachable(base_goal_id, my_team, true);
        };

        const bool search_held_long_enough =
            regionalDefenseSearchStartTime_.time_since_epoch().count() != 0 &&
            now - regionalDefenseSearchStartTime_ >=
                std::chrono::seconds(std::max(1, defense.SearchHoldSec));
        const bool current_search_done =
            current_goal_done(regionalDefenseSearchBaseGoal_);
        const bool should_advance_search =
            same_search &&
            candidates.size() > 1U &&
            (current_search_done || search_held_long_enough) &&
            !autoaim_recently_seen();

        if (should_advance_search) {
            regionalDefenseSearchIndex_ =
                (regionalDefenseSearchIndex_ + 1U) % candidates.size();
            regionalDefenseSearchBaseGoal_ = candidates[regionalDefenseSearchIndex_];
            regionalDefenseSearchStartTime_ = now;
        }

        const auto current_goal_id =
            ResolveGoalId(regionalDefenseSearchBaseGoal_, my_team, true);
        if (same_search &&
            !should_advance_search &&
            regionalDefenseSearchBaseGoal_ != LangYa::Home.ID &&
            naviCommandGoal == current_goal_id) {
            return true;
        }

        auto set_defense_goal = [&](const std::uint8_t goal_id) {
            if (!AreaManager::IsValidBaseGoalId(goal_id)) {
                return false;
            }
            if (!IsNaviGoalAllowedByAreaScope(goal_id, my_team, my_team, enemy_team)) {
                naviGoalPublishAllowed_ = false;
                return false;
            }
            if (!TryStartNaviAreaTransition(goal_id, my_team, my_team, true, reason)) {
                SetPositionByBaseGoal(goal_id, my_team, true);
            }
            naviCommandIntervalClock.reset(Seconds{std::max(1, hold_sec)});
            speedLevel = 1;
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Regional defense {} kind={} search_index={} goal={} own_base={} own_highland={} own_roadland={} common_central={} enemy_highland={} enemy_roadland={}",
                    reason,
                    RegionalDefenseSearchKindToString(search_kind),
                    regionalDefenseSearchIndex_,
                    static_cast<int>(naviCommandGoal),
                    threat.OwnBaseCount,
                    threat.OwnHighlandCount,
                    threat.OwnRoadlandCount,
                    threat.CommonCentralCount,
                    threat.EnemyHighlandCount,
                    threat.EnemyRoadlandCount);
            }
            return true;
        };

        for (std::size_t attempt = 0; attempt < candidates.size(); ++attempt) {
            const auto index = (regionalDefenseSearchIndex_ + attempt) % candidates.size();
            regionalDefenseSearchIndex_ = index;
            regionalDefenseSearchBaseGoal_ = candidates[index];
            if (set_defense_goal(regionalDefenseSearchBaseGoal_)) {
                regionalDefenseSearchStartTime_ = now;
                return true;
            }
        }

        return false;
    }

    void Application::UpdateNaviProgressWatchdogGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        const std::uint8_t goal_id = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        areaManager_.UpdateProgressWatchdogGoal(
            goal_id,
            base_goal_id,
            goal_team,
            apply_team_offset,
            naviGoalPosition,
            self_x,
            self_y,
            std::chrono::steady_clock::now());
    }

    bool Application::TickNaviProgressWatchdog(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& watchdog = config.NaviProgressWatchdogSettings;
        const auto now = std::chrono::steady_clock::now();
        const auto& runtime = areaManager_.ProgressWatchdogRuntime();
        const bool self_position_fresh =
            hasReceivedSentryPosition_ &&
            lastSentryPositionRxTime_.time_since_epoch().count() != 0 &&
            now - lastSentryPositionRxTime_ <= std::chrono::seconds(2);
        const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        const bool has_self_position = self_position_fresh && self_x > 0 && self_y > 0;
        const auto external_reach =
            GetExternalNaviReachForGoal(runtime.GoalId, runtime.GoalPosition);
        const auto external_reachable =
            GetExternalNaviReachableForGoal(runtime.GoalId, runtime.GoalPosition);

        const auto decision = areaManager_.TickProgressWatchdog(
            NaviProgressWatchdogInput{
                .Enabled = watchdog.Enable,
                .BlockedByAreaTransition = areaManager_.HighlandTransitionActive(),
                .HasSelfPosition = has_self_position,
                .SelfX = self_x,
                .SelfY = self_y,
                .ExternalReach = external_reach,
                .ExternalReachable = external_reachable,
                .Setting = watchdog,
                .Now = now
            });
        if (!decision.NeedFallback) {
            return false;
        }

        for (const auto fallback_goal : decision.FallbackCandidates) {
            if (TrySetScopedPositionByBaseGoal(
                    fallback_goal,
                    decision.OriginalGoalTeam,
                    my_team,
                    enemy_team,
                    decision.OriginalApplyTeamOffset,
                    "navi_progress_watchdog")) {
                naviCommandIntervalClock.reset(Seconds{std::max(1, watchdog.FallbackHoldSec)});
                speedLevel = 1;
                areaManager_.CommitProgressWatchdogFallback(
                    now,
                    watchdog.FallbackCooldownSec,
                    decision.OriginalBaseGoal,
                    decision.OriginalGoalTeam);
                if (LoggerPtr) {
                    LoggerPtr->Warning(
                        "Navi progress watchdog: goal={} {}, fallback goal={}.",
                        static_cast<int>(decision.OriginalGoalId),
                        decision.ExternalUnreachable ? "unreachable" : "no movement",
                        static_cast<int>(naviCommandGoal));
                }
                return true;
            }
        }

        areaManager_.MarkProgressWatchdogFallbackFailed(now, has_self_position, self_x, self_y);
        return false;
    }

    bool Application::TrySetRegionalIdlePatrolGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& patrol = config.RegionalIdlePatrolSettings;
        if (!patrol.Enable || IsLeagueProfile() || IsShowcasePatrolEnabled()) {
            return false;
        }
        if (aimMode == AimMode::Buff || aimMode == AimMode::Outpost) {
            return false;
        }
        if (patrol.Goals.empty()) {
            naviCommandIntervalClock.reset(Seconds{std::max(1, patrol.GoalHoldSec)});
            if (LoggerPtr) {
                LoggerPtr->Warning("Regional idle patrol enabled but no goals are enabled.");
            }
            return true;
        }

        constexpr bool apply_team_offset = true;
        const int hold_sec = std::max(1, patrol.GoalHoldSec);
        const auto candidates = areaManager_.BuildRegionalIdlePatrolCandidates(
            patrol.Goals,
            naviCommandGoal,
            my_team);
        for (const auto& candidate : candidates) {
            const auto base_goal_id = candidate.BaseGoalId;
            if (!AreaManager::IsValidBaseGoalId(base_goal_id)) {
                continue;
            }
            if (TrySetScopedPositionByBaseGoal(
                    base_goal_id,
                    my_team,
                    my_team,
                    enemy_team,
                    apply_team_offset,
                    "regional_idle_patrol")) {
                areaManager_.CommitRegionalIdlePatrolCandidate(candidate.Index);
                naviCommandIntervalClock.reset(Seconds{hold_sec});
                speedLevel = 1;
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "Regional idle patrol: base_goal={} goal={} hold={}s.",
                        static_cast<int>(base_goal_id),
                        static_cast<int>(naviCommandGoal),
                        hold_sec);
                }
                return true;
            }
        }

        areaManager_.ResetRegionalIdlePatrol();
        naviCommandIntervalClock.reset(Seconds{hold_sec});
        if (LoggerPtr) {
            LoggerPtr->Warning("Regional idle patrol: all configured goals blocked by area scope.");
        }
        return true;
    }

    bool Application::IsDefaultRegionalDecisionReady(
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        return !IsLeagueProfile() &&
            !IsShowcasePatrolEnabled() &&
            GetStrategyMode() == StrategyMode::Regional &&
            aimMode != AimMode::Buff &&
            aimMode != AimMode::Outpost &&
            !areaManager_.RegionalAreaTaskActive() &&
            !areaManager_.HighlandTransitionActive() &&
            !EvaluateRegionalDefenseThreat(my_team, enemy_team).has_value();
    }

    bool Application::TrySetDefaultRegionalAreaTaskGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& task = config.RegionalAreaTaskSettings;
        if (!task.Enable ||
            IsLeagueProfile() ||
            IsShowcasePatrolEnabled() ||
            aimMode == AimMode::Buff ||
            aimMode == AimMode::Outpost) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        auto referee_value_fresh = [&](const bool received, const std::chrono::steady_clock::time_point last_rx) {
            return received &&
                last_rx.time_since_epoch().count() != 0 &&
                now - last_rx <= std::chrono::seconds(2);
        };
        const bool self_position_fresh =
            hasReceivedSentryPosition_ &&
            lastSentryPositionRxTime_.time_since_epoch().count() != 0 &&
            now - lastSentryPositionRxTime_ <= std::chrono::seconds(2);
        const auto candidates =
            defaultStrategyManager_.BuildRegionalAreaCandidates(
                DefaultRegionalPolicyInput{
                    .Config = &config,
                    .MyTeam = my_team,
                    .EnemyTeam = enemy_team,
                    .HealthFresh = referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime),
                    .AmmoFresh = referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime),
                    .Health = myselfHealth,
                    .Ammo = ammoLeft,
                    .HasSelfPosition = self_position_fresh,
                    .SelfX = self_position_fresh
                        ? static_cast<int>(friendRobots[UnitType::Sentry].position_.X)
                        : 0,
                    .SelfY = self_position_fresh
                        ? static_cast<int>(friendRobots[UnitType::Sentry].position_.Y)
                        : 0,
                    .SelfArea = areaManager_.SelfAreaRuntime(),
                    .Now = now
                });
        if (candidates.empty()) {
            defaultStrategyManager_.ResetRegionalPolicy();
            return false;
        }

        for (const auto& candidate : candidates) {
            if (!TrySetScopedPositionByBaseGoal(
                    candidate.BaseGoalId,
                    candidate.GoalTeam,
                    my_team,
                    enemy_team,
                    true,
                    "default_area_policy")) {
                continue;
            }

            defaultStrategyManager_.CommitRegionalAreaSelection(candidate, now);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Default regional policy: area={} score={:.2f} base_goal={} goal={}.",
                    candidate.Name,
                    candidate.Score,
                    static_cast<int>(candidate.BaseGoalId),
                    static_cast<int>(naviCommandGoal));
            }
            return true;
        }

        return false;
    }

    bool Application::TrySetDefaultRegionalGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        if (TrySetDefaultRegionalAreaTaskGoal(my_team, enemy_team)) {
            speedLevel = 1;
            return true;
        }
        return false;
    }

    bool Application::TrySetScopedPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const bool apply_team_offset,
        const char* reason) {
        if (!IsNaviGoalAllowedByAreaScope(base_goal_id, goal_team, my_team, enemy_team)) {
            naviGoalPublishAllowed_ = false;
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "DecisionAutonomy[navi_goal_area]: block goal={} team={} reason={}",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    goal_team == my_team ? "my" : "enemy",
                    reason ? reason : "area_scope");
            }
            return false;
        }

        if (TryStartRegionalAreaTaskForGoal(base_goal_id, goal_team, my_team, apply_team_offset, reason)) {
            return true;
        }

        if (TryStartNaviAreaTransition(base_goal_id, goal_team, my_team, apply_team_offset, reason)) {
            return true;
        }

        SetPositionByBaseGoal(base_goal_id, goal_team, apply_team_offset);
        return true;
    }

    bool Application::TrySetRandomScopedPositionByBaseGoal(
        const std::vector<std::pair<std::uint8_t, UnitTeam>>& goals,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const char* reason) {
        std::vector<std::pair<std::uint8_t, UnitTeam>> allowed_goals;
        allowed_goals.reserve(goals.size());
        for (const auto& goal : goals) {
            if (!AreaManager::IsValidBaseGoalId(goal.first)) {
                continue;
            }
            if (IsNaviGoalAllowedByAreaScope(goal.first, goal.second, my_team, enemy_team)) {
                allowed_goals.push_back(goal);
            }
        }
        if (allowed_goals.empty()) {
            if (IsNaviGoalAreaScopeEnabled()) {
                naviGoalPublishAllowed_ = false;
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "DecisionAutonomy[navi_goal_area]: block all random goals reason={}",
                        reason ? reason : "area_scope");
                }
            }
            return false;
        }

        Random random;
        const auto index = random.Get(0, static_cast<int>(allowed_goals.size()) - 1);
        const auto& selected = allowed_goals[static_cast<std::size_t>(index)];
        return TrySetScopedPositionByBaseGoal(
            selected.first,
            selected.second,
            my_team,
            enemy_team,
            true,
            reason);
    }

    void Application::SetPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        auto assign_position = [&](const auto& goal_location, const auto& area_location) {
            naviCommandGoal = apply_team_offset ? goal_location(goal_team) : goal_location.ID;
            naviGoalPosition = area_location(goal_team);
            naviGoalPublishAllowed_ = true;
        };
        std::uint8_t effective_base_goal_id = base_goal_id;

        switch (base_goal_id) {
            case LangYa::Home.ID: assign_position(LangYa::Home, BehaviorTree::Area::Home); break;
            case LangYa::Base.ID: assign_position(LangYa::Base, BehaviorTree::Area::Base); break;
            case LangYa::Recovery.ID: assign_position(LangYa::Recovery, BehaviorTree::Area::Recovery); break;
            case LangYa::BuffShoot.ID: assign_position(LangYa::BuffShoot, BehaviorTree::Area::BuffShoot); break;
            case LangYa::LeftHighLand.ID: assign_position(LangYa::LeftHighLand, BehaviorTree::Area::LeftHighLand); break;
            case LangYa::CastleLeft1.ID: assign_position(LangYa::CastleLeft1, BehaviorTree::Area::CastleLeft1); break;
            case LangYa::CastleLeft2.ID: assign_position(LangYa::CastleLeft2, BehaviorTree::Area::CastleLeft2); break;
            case LangYa::Castle.ID: assign_position(LangYa::Castle, BehaviorTree::Area::Castle); break;
            case LangYa::CastleRight1.ID: assign_position(LangYa::CastleRight1, BehaviorTree::Area::CastleRight1); break;
            case LangYa::CastleRight2.ID: assign_position(LangYa::CastleRight2, BehaviorTree::Area::CastleRight2); break;
            case LangYa::FlyRoad.ID: assign_position(LangYa::FlyRoad, BehaviorTree::Area::FlyRoad); break;
            case LangYa::OutpostArea.ID: assign_position(LangYa::OutpostArea, BehaviorTree::Area::OutpostArea); break;
            case LangYa::MidShoot.ID: assign_position(LangYa::MidShoot, BehaviorTree::Area::MidShoot); break;
            case LangYa::LeftShoot.ID: assign_position(LangYa::LeftShoot, BehaviorTree::Area::LeftShoot); break;
            case LangYa::OutpostShoot.ID: assign_position(LangYa::OutpostShoot, BehaviorTree::Area::OutpostShoot); break;
            case LangYa::BuffAround1.ID: assign_position(LangYa::BuffAround1, BehaviorTree::Area::BuffAround1); break;
            case LangYa::BuffAround2.ID: assign_position(LangYa::BuffAround2, BehaviorTree::Area::BuffAround2); break;
            case LangYa::RightShoot.ID: assign_position(LangYa::RightShoot, BehaviorTree::Area::RightShoot); break;
            case LangYa::HoleRoad.ID: assign_position(LangYa::HoleRoad, BehaviorTree::Area::HoleRoad); break;
            case LangYa::OccupyArea.ID: assign_position(LangYa::OccupyArea, BehaviorTree::Area::OccupyArea); break;
            case LangYa::Highland.ID: assign_position(LangYa::Highland, BehaviorTree::Area::Highland); break;
            case LangYa::BaseToCentral.ID: assign_position(LangYa::BaseToCentral, BehaviorTree::Area::BaseToCentral); break;
            case LangYa::CentralToBase.ID: assign_position(LangYa::CentralToBase, BehaviorTree::Area::CentralToBase); break;
            case LangYa::BuffOutpost.ID: assign_position(LangYa::BuffOutpost, BehaviorTree::Area::BuffOutpost); break;
            case LangYa::OutpostGuard.ID: assign_position(LangYa::OutpostGuard, BehaviorTree::Area::OutpostGuard); break;
            default:
                LoggerPtr->Warning("Unknown base goal id={}, fallback to Home.", static_cast<int>(base_goal_id));
                effective_base_goal_id = LangYa::Home.ID;
                assign_position(LangYa::Home, BehaviorTree::Area::Home);
                break;
        }
        UpdateNaviProgressWatchdogGoal(effective_base_goal_id, goal_team, apply_team_offset);
    }

    void Application::SetPositionRepeat() {
        if (IsLeagueProfile()) {
            SetPositionLeagueSimple();
            return;
        }
        if (IsShowcasePatrolEnabled()) {
            SetPositionShowcasePatrol();
            return;
        }
        const UnitTeam my_team = team;
        const UnitTeam enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        (void)TrySetDefaultRegionalGoal(my_team, enemy_team);
    }

    void Application::SetPositionLeagueSimple() {
        const auto& league = config.LeagueStrategySettings;
        const int hold_sec = std::max(1, league.GoalHoldSec);
        constexpr bool apply_team_offset = true;
        if (IsLeagueRouteCompatEnabled() && leagueRouteCompatAfterGatePending_) {
            leagueRouteCompatAfterGatePending_ = false;
            leagueRouteCompatActive_ = true;
            leagueRouteCompatUntil_ =
                std::chrono::steady_clock::now() + std::chrono::seconds(kLeagueRouteCompatViaHoldSec);
            leagueRouteCompatHasPendingGoal_ = false;
            leagueRouteCompatPendingBaseGoal_ = LangYa::Home.ID;
            leagueRouteCompatPendingHoldSec_ = 1;
            SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            LoggerPtr->Info(
                "League route compat after_gate: via goal={} for {}s.",
                static_cast<int>(naviCommandGoal),
                kLeagueRouteCompatViaHoldSec);
            return;
        }

        if (TickLeagueRouteCompat(team, apply_team_offset)) {
            return;
        }

        // 联赛模式优先做回补判定；命中后直接返回，不再切换巡航点。
        if (CheckPositionRecovery()) {
            LoggerPtr->Info("League profile recovery: health={} ammo={}", myselfHealth, ammoLeft);
            leaguePatrolGoalIndex_ = 0;
            leaguePatrolGoalInitialized_ = false;
            return;
        }

        std::vector<std::uint8_t> plan;
        plan.reserve(1 + league.PatrolGoals.size());
        // 计划路径由 MainGoal + PatrolGoals 去重组成。
        // 这里保证非法点位不会进入运行态。
        auto append_goal = [&](const std::uint8_t goal_id) {
            if (!AreaManager::IsValidBaseGoalId(goal_id)) {
                LoggerPtr->Warning("Skip invalid league goal id={}.", static_cast<int>(goal_id));
                return;
            }
            if (std::find(plan.begin(), plan.end(), goal_id) == plan.end()) {
                plan.push_back(goal_id);
            }
        };
        append_goal(league.MainGoal);
        for (const auto goal_id : league.PatrolGoals) {
            append_goal(goal_id);
        }
        if (plan.empty()) {
            append_goal(LangYa::OccupyArea.ID);
        }

        if (!leaguePatrolGoalInitialized_) {
            leaguePatrolGoalIndex_ = 0;
            const auto init_goal_id = ResolveGoalId(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            if (IsLeagueRouteCompatEnabled()) {
                if (IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        init_goal_id,
                        team,
                        apply_team_offset)) {
                    leaguePatrolGoalInitialized_ = true;
                    StartLeagueRouteCompat(
                        plan[leaguePatrolGoalIndex_],
                        hold_sec,
                        team,
                        apply_team_offset,
                        "init_2_3_switch");
                    return;
                }
            }
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            leaguePatrolGoalInitialized_ = true;
            LoggerPtr->Info("League profile init goal={}", static_cast<int>(naviCommandGoal));
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [this](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team);
            });
        if (current_it == plan.end()) {
            leaguePatrolGoalIndex_ = 0;
            const auto reset_goal_id = ResolveGoalId(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            if (IsLeagueRouteCompatEnabled() &&
                IsLeagueGoalSwitchBetween2And3(
                    naviCommandGoal,
                    reset_goal_id,
                    team,
                    apply_team_offset)) {
                StartLeagueRouteCompat(
                    plan[leaguePatrolGoalIndex_],
                    hold_sec,
                    team,
                    apply_team_offset,
                    "reset_2_3_switch");
                return;
            }
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Info("League profile reset goal={}", static_cast<int>(naviCommandGoal));
            return;
        }
        leaguePatrolGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = 1;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return;
        }

        if (plan.size() > 1U) {
            leaguePatrolGoalIndex_ = (leaguePatrolGoalIndex_ + 1U) % plan.size();
        }
        const auto target_base_goal = plan[leaguePatrolGoalIndex_];
        const auto target_goal_id = ResolveGoalId(target_base_goal, team, apply_team_offset);
        if (IsLeagueRouteCompatEnabled() &&
            IsLeagueGoalSwitchBetween2And3(
                naviCommandGoal,
                target_goal_id,
                team,
                apply_team_offset)) {
            StartLeagueRouteCompat(
                target_base_goal,
                hold_sec,
                team,
                apply_team_offset,
                "switch_2_3");
            return;
        }
        SetPositionByBaseGoal(target_base_goal, team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{hold_sec});
        speedLevel = 1;
        LoggerPtr->Info("League profile switch goal={}", static_cast<int>(naviCommandGoal));
    }

    void Application::SetPositionShowcasePatrol() {
        const auto& showcase = config.ShowcasePatrolSettings;
        const bool apply_team_offset = !showcase.DisableTeamOffset;
        const int hold_sec = std::max(1, showcase.GoalHoldSec);
        const auto& plan = showcase.Goals;

        auto choose_index = [&](const bool initialize) -> std::size_t {
            if (plan.empty()) {
                return 0U;
            }
            if (!showcase.Random || plan.size() == 1U) {
                return initialize ? 0U : (showcasePatrolGoalIndex_ + 1U) % plan.size();
            }
            Random random;
            const auto upper_bound = static_cast<int>(plan.size()) - 1;
            std::size_t next_index = initialize
                ? static_cast<std::size_t>(random.Get(0, upper_bound))
                : showcasePatrolGoalIndex_;
            while (!initialize && next_index == showcasePatrolGoalIndex_) {
                next_index = static_cast<std::size_t>(random.Get(0, upper_bound));
            }
            return next_index;
        };

        auto apply_goal = [&](const std::size_t goal_index, const char* reason) {
            showcasePatrolGoalIndex_ = goal_index;
            SetPositionByBaseGoal(plan[goal_index], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Info(
                "Showcase patrol {} goal_id={} raw_base_goal={}",
                reason,
                static_cast<int>(naviCommandGoal),
                static_cast<int>(plan[goal_index]));
        };

        if (!showcase.IgnoreRecovery && CheckPositionRecovery()) {
            LoggerPtr->Info("Showcase patrol recovery: health={} ammo={}", myselfHealth, ammoLeft);
            showcasePatrolGoalIndex_ = 0;
            showcasePatrolGoalInitialized_ = false;
            return;
        }

        if (plan.empty()) {
            SetPositionByBaseGoal(LangYa::OccupyArea.ID, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Warning("Showcase patrol has empty goal plan, fallback to OccupyArea.");
            return;
        }

        if (!showcasePatrolGoalInitialized_) {
            apply_goal(choose_index(true), "init");
            showcasePatrolGoalInitialized_ = true;
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [&](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team, apply_team_offset);
            });
        if (current_it == plan.end()) {
            apply_goal(choose_index(true), "reset");
            showcasePatrolGoalInitialized_ = true;
            return;
        }
        showcasePatrolGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = 1;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return;
        }

        apply_goal(choose_index(false), "switch");
    }

    void Application::SetPositionNaviDebugPlan() {
        const auto& navi_debug = config.NaviDebugSettings;
        const bool apply_team_offset = !navi_debug.DisableTeamOffset;
        const int hold_sec = std::max(1, navi_debug.GoalHoldSec);
        const auto& plan = navi_debug.Goals;

        auto choose_index = [&](const bool initialize) -> std::size_t {
            if (plan.empty()) {
                return 0U;
            }
            if (!navi_debug.Random || plan.size() == 1U) {
                return initialize ? 0U : (naviDebugGoalIndex_ + 1U) % plan.size();
            }
            Random random;
            const auto upper_bound = static_cast<int>(plan.size()) - 1;
            std::size_t next_index = initialize
                ? static_cast<std::size_t>(random.Get(0, upper_bound))
                : naviDebugGoalIndex_;
            while (!initialize && next_index == naviDebugGoalIndex_) {
                next_index = static_cast<std::size_t>(random.Get(0, upper_bound));
            }
            return next_index;
        };

        auto apply_goal = [&](const std::size_t goal_index, const char* reason) {
            naviDebugGoalIndex_ = goal_index;
            SetPositionByBaseGoal(plan[goal_index], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = navi_debug.SpeedLevel;
            LoggerPtr->Info(
                "NaviDebug {} goal_id={} raw_base_goal={} speed_level={}",
                reason,
                static_cast<int>(naviCommandGoal),
                static_cast<int>(plan[goal_index]),
                static_cast<int>(speedLevel));
        };

        if (!navi_debug.IgnoreRecovery && CheckPositionRecovery()) {
            LoggerPtr->Info("NaviDebug recovery: health={} ammo={}", myselfHealth, ammoLeft);
            naviDebugGoalIndex_ = 0;
            naviDebugGoalInitialized_ = false;
            return;
        }

        if (plan.empty()) {
            SetPositionByBaseGoal(LangYa::OccupyArea.ID, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = navi_debug.SpeedLevel;
            LoggerPtr->Warning("NaviDebug has empty goal plan, fallback to OccupyArea.");
            return;
        }

        if (!naviDebugGoalInitialized_) {
            apply_goal(choose_index(true), "init");
            naviDebugGoalInitialized_ = true;
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [&](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team, apply_team_offset);
            });
        if (current_it == plan.end()) {
            apply_goal(choose_index(true), "reset");
            naviDebugGoalInitialized_ = true;
            return;
        }
        naviDebugGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = navi_debug.SpeedLevel;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = navi_debug.SpeedLevel;
            return;
        }

        apply_goal(choose_index(false), "switch");
    }

    bool Application::CheckPositionRecovery() {
        if (config.RegionalAreaTaskSettings.IgnoreRecovery) {
            return false;
        }

        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        int now_time = 420 - timeLeft;
        auto cancel_regional_area_task_for_recovery = [&]() -> bool {
            if (areaManager_.RegionalAreaTaskActive()) {
                if (areaManager_.RegionalAreaTask().Type == RegionalAreaTaskType::MyRoadland) {
                    RequestRoadlandSafeReturn("recovery has higher priority");
                    return TickRegionalAreaTask(MyTeam, EnemyTeam);
                }
                const auto canceled_task_type = areaManager_.RegionalAreaTask().Type;
                areaManager_.ClearRegionalAreaTask();
                defaultStrategyManager_.RecordRegionalAreaResult(
                    canceled_task_type,
                    "canceled",
                    std::chrono::steady_clock::now(),
                    config.RegionalAreaTaskSettings.DefaultPolicy);
                ResetRegionalAreaControlOverride();
                gimbalControlData.FireCode.FollowMode = 0;
                if (LoggerPtr) {
                    LoggerPtr->Info("RegionalAreaTask canceled: recovery has higher priority.");
                }
            }
            return false;
        };
        const bool disable_team_offset_for_debug =
            (config.ShowcasePatrolSettings.Enable && config.ShowcasePatrolSettings.DisableTeamOffset) ||
            (config.NaviDebugSettings.Enable && config.NaviDebugSettings.DisableTeamOffset);
        const bool apply_team_offset = IsLeagueProfile()
            ? true
            : !disable_team_offset_for_debug;
        const auto recovery_goal_id = ResolveGoalId(LangYa::Recovery.ID, MyTeam, apply_team_offset);
        if (IsLeagueProfile()) {
            // 联赛回补策略核心：
            // - 以裁判输入（自身血量/弹药）作为唯一触发源
            // - 带 stale 检查，避免旧数据误触发回补
            // - 回补失败后带 cooldown，防止频繁抖动
            const auto& league = config.LeagueStrategySettings;
            const auto now = std::chrono::steady_clock::now();
            if (TickLeagueRouteCompat(MyTeam, apply_team_offset)) {
                return true;
            }
            const std::uint16_t recovery_exit_min = league.HealthRecoveryExitMin;
            const std::uint16_t recovery_exit_preferred = league.HealthRecoveryExitPreferred;
            const auto recovery_plateau = std::chrono::seconds(league.HealthRecoveryPlateauSec);
            const auto recovery_max_hold = std::chrono::seconds(league.HealthRecoveryMaxHoldSec);
            const auto recovery_cooldown = std::chrono::seconds(league.HealthRecoveryCooldownSec);
            auto reset_league_recovery_state = [&]() {
                leagueRecoveryActive_ = false;
                leagueRecoveryStartTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryLastIncreaseTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryEntryHealth_ = 0;
                leagueRecoveryPeakHealth_ = 0;
            };
            auto is_referee_value_ready = [&](const bool has_received,
                                              const std::chrono::steady_clock::time_point& last_rx_time) {
                if (!has_received) {
                    return false;
                }
                if (leagueRefereeStaleTimeoutMs_ <= 0) {
                    return true;
                }
                if (last_rx_time.time_since_epoch().count() == 0) {
                    return false;
                }
                const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_rx_time).count();
                return age_ms <= leagueRefereeStaleTimeoutMs_;
            };

            const bool health_ready = is_referee_value_ready(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
            const bool health_low = league.UseHealthRecovery && health_ready &&
                myselfHealth < league.HealthRecoveryThreshold;
            const bool missing_health_input = league.UseHealthRecovery && !health_ready;
            const bool health_recovery_cooldown_active =
                leagueRecoveryCooldownUntil_.time_since_epoch().count() != 0 &&
                now < leagueRecoveryCooldownUntil_;
            const bool effective_health_low = health_low && !health_recovery_cooldown_active;

            if (missing_health_input &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                LoggerPtr->Warning(
                    "League recovery guard: skip invalid referee inputs (health_ready={} stale_timeout_ms={})",
                    health_ready ? 1 : 0,
                    leagueRefereeStaleTimeoutMs_);
                lastLeagueRecoveryGuardLogTime_ = now;
            }
            if (health_recovery_cooldown_active &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                const auto cooldown_left_sec = std::chrono::duration_cast<std::chrono::seconds>(
                    leagueRecoveryCooldownUntil_ - now).count();
                LoggerPtr->Warning(
                    "League recovery cooldown active: skip health-triggered Recovery for {}s.",
                    cooldown_left_sec > 0 ? cooldown_left_sec : 0);
                lastLeagueRecoveryGuardLogTime_ = now;
            }

            if (effective_health_low && !leagueRecoveryActive_) {
                // 进入“血量回补状态机”
                leagueRecoveryActive_ = true;
                leagueRecoveryStartTime_ = now;
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryLastIncreaseTime_ = now;
                leagueRecoveryEntryHealth_ = myselfHealth;
                leagueRecoveryPeakHealth_ = myselfHealth;
                LoggerPtr->Info(
                    "League health recovery activated: hp={} < threshold={}",
                    myselfHealth,
                    league.HealthRecoveryThreshold);
            }

            if (leagueRecoveryActive_) {
                if (health_ready && myselfHealth > leagueRecoveryPeakHealth_) {
                    leagueRecoveryPeakHealth_ = myselfHealth;
                    leagueRecoveryLastIncreaseTime_ = now;
                }

                const bool reach_preferred =
                    health_ready && myselfHealth >= recovery_exit_preferred;
                const bool reach_min_plateau =
                    health_ready &&
                    myselfHealth >= recovery_exit_min &&
                    leagueRecoveryLastIncreaseTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryLastIncreaseTime_) >= recovery_plateau;
                const bool recovery_timeout =
                    leagueRecoveryStartTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryStartTime_) >= recovery_max_hold;

                if (reach_preferred || reach_min_plateau) {
                    LoggerPtr->Info(
                        "League health recovery completed: hp={} (peak={}), return to normal strategy.",
                        myselfHealth,
                        leagueRecoveryPeakHealth_);
                    reset_league_recovery_state();
                    return false;
                }

                if (recovery_timeout) {
                    LoggerPtr->Warning(
                        "League health recovery timeout: entry_hp={} peak_hp={} current_hp={}, fallback to normal strategy.",
                        leagueRecoveryEntryHealth_,
                        leagueRecoveryPeakHealth_,
                        myselfHealth);
                    reset_league_recovery_state();
                    leagueRecoveryCooldownUntil_ = now + recovery_cooldown;
                    return false;
                }

                // 回补进行中：持续锁定 Recovery 点位，缩短导航重发间隔。
                if (IsLeagueRouteCompatEnabled() &&
                    IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        recovery_goal_id,
                        MyTeam,
                        apply_team_offset)) {
                    StartLeagueRouteCompat(
                        LangYa::Recovery.ID,
                        1,
                        MyTeam,
                        apply_team_offset,
                        "recovery_2_3_switch");
                    return true;
                }
                SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (naviCommandGoal == recovery_goal_id &&
                (effective_health_low || missing_health_input)) {
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (effective_health_low) {
                if (IsLeagueRouteCompatEnabled() &&
                    IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        recovery_goal_id,
                        MyTeam,
                        apply_team_offset)) {
                    StartLeagueRouteCompat(
                        LangYa::Recovery.ID,
                        1,
                        MyTeam,
                        apply_team_offset,
                        "recovery_2_3_switch");
                    return true;
                }
                SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }
            return false;
        }
        // 复活
        if(naviCommandGoal == recovery_goal_id) {
            if(myselfHealth < 380) {
                if (cancel_regional_area_task_for_recovery()) {
                    return true;
                }
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }
        }
        // 回家
        // 条件为：血量低于150 或者 弹药为0且距离上一次回家已经过去90秒
        if(myselfHealth < 150 || (ammoLeft <= 30 && recoveryClock.trigger())) {
            if (cancel_regional_area_task_for_recovery()) {
                return true;
            }
            SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
            recoveryClock.tick();
            naviCommandIntervalClock.reset(Seconds{1});
            return true;
        }
        return false;
    }


}
