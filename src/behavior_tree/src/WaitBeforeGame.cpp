// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <algorithm>
#include <cmath>

using namespace LangYa;

namespace BehaviorTree {
    namespace {
    constexpr float kGatePatrolTwoPi = 6.2831853071795864769f;
    constexpr float kGatePatrolScanYawStepDeg = 9.0f;
    constexpr float kGatePatrolSwingYawStepDeg = 1.0f;
    constexpr float kGatePatrolSwingHalfRangeDeg = 30.0f;
    constexpr float kGatePatrolOutpostYawStepDeg = 1.0f;
    constexpr float kGatePatrolOutpostPitchDeg = 17.0f;
    constexpr float kGatePatrolPitchCenterDeg = 0.0f;
    constexpr float kGatePatrolPitchHalfRangeDeg = 12.0f;
    constexpr float kGatePatrolPitchPeriodMs = 500.0f;

    float NormalizeGatePatrolAngleNear(const float angle, const float reference) {
        return reference + static_cast<float>(std::remainder(angle - reference, 360.0f));
    }
    }  // namespace

    /**
     * @brief 等待比赛开始前的预操作 \n
     * @brief 1. 等待云台数据 \n
     * @brief 2. 等待比赛开始 \n
     * @brief 3. 设置云台角度 \n
     * @brief 4. 设置导航目标 \n
     * @brief 5. 设置底盘速度 \n
     */
    void Application::WaitForGameStart() {
        /// 设置云台角度
        gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw;
        gimbalControlData.GimbalAngles.Pitch = AngleType{0};
        gimbalControlData.FireCode.FireStatus = 0;
        gimbalControlData.FireCode.Rotate = 0;
        gimbalControlData.FireCode.FollowMode = 0;
        gimbalControlData.FireCode.AimMode = 0;
        naviVelocityInput = VelocityType{0, 0};
        naviVelocity = VelocityType{0, 0};
        postureCommand = 0;

        LoggerPtr->Info("Waiting For Game Start!");

        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        auto last_damage_gate_log = wait_begin;
        auto last_gate_patrol_log = wait_begin;
        bool gate_patrol_center_initialized = false;
        float gate_patrol_center_yaw = 0.0f;
        float gate_patrol_last_yaw = 0.0f;
        float gate_patrol_phase_rad = 0.0f;
        bool bypass_logged = false;
        const bool gate_gimbal_patrol_enabled =
            config.StartGateSettings.AllowGimbalPatrolBeforeStart &&
            !config.AimDebugSettings.StopScan;
        const bool damage_open_gate_enabled = config.DamageOpenGateSettings.Enable;
        const std::uint16_t damage_open_gate_threshold =
            std::max<std::uint16_t>(1, config.DamageOpenGateSettings.HealthDropThreshold);
        bool damage_health_baseline_initialized = false;
        std::uint16_t damage_health_peak = 0;

        if (damage_open_gate_enabled) {
            LoggerPtr->Info(
                "Damage start gate enabled: open by health drop >= {}.",
                static_cast<int>(damage_open_gate_threshold));
        }
        if (gate_gimbal_patrol_enabled) {
            LoggerPtr->Info(
                "Start gate gimbal patrol enabled: chassis velocity/rotate stay zero before game start.");
        }

        // [ROS 2] 不再依賴文件系統判斷，直接等待 is_game_begin 標誌
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            const auto now_steady = std::chrono::steady_clock::now();

            SET_POSITION(Home, team);

            if(publishNaviGoal_ && naviCommandRateClock.trigger()) {
                naviCommandRateClock.tick();
                const bool navi_to_navi =
                    config.NaviSettings.UseXY &&
                    config.NaviSettings.ToNavi &&
                    config.ChaseSettings.Enable &&
                    config.ChaseSettings.ToNavi;
                if(config.NaviSettings.UseXY && !navi_to_navi) PubNaviGoalPos();
                else PubNaviGoal();
            }

            // 开赛门控期间持续压零速度，避免下位机沿用上一拍底盘控制量。
            naviVelocityInput.X = 0;
            naviVelocityInput.Y = 0;
            naviVelocity.X = 0;
            naviVelocity.Y = 0;
            gimbalControlData.FireCode.FireStatus = 0;
            gimbalControlData.FireCode.Rotate = 0;
            gimbalControlData.FireCode.FollowMode = 0;
            gimbalControlData.FireCode.AimMode = 0;
            if (gate_gimbal_patrol_enabled && !debugBypassGameStart_) {
                if (!gate_patrol_center_initialized) {
                    gate_patrol_center_initialized = true;
                    gate_patrol_center_yaw = gimbalAngles.Yaw;
                    gate_patrol_last_yaw = gimbalAngles.Yaw;
                }

                const int patrol_mode = config.PatrolScanSettings.Mode;
                float next_yaw = gate_patrol_last_yaw;
                if (patrol_mode == 2) {
                    const float phase_step =
                        kGatePatrolSwingYawStepDeg / std::max(kGatePatrolSwingHalfRangeDeg, 1.0f);
                    gate_patrol_phase_rad = std::fmod(gate_patrol_phase_rad + phase_step, kGatePatrolTwoPi);
                    if (gate_patrol_phase_rad < 0.0f) {
                        gate_patrol_phase_rad += kGatePatrolTwoPi;
                    }
                    next_yaw = NormalizeGatePatrolAngleNear(
                        gate_patrol_center_yaw +
                            kGatePatrolSwingHalfRangeDeg * std::sin(gate_patrol_phase_rad),
                        gimbalAngles.Yaw);
                } else {
                    const float yaw_step =
                        patrol_mode == 3 ? kGatePatrolOutpostYawStepDeg : kGatePatrolScanYawStepDeg;
                    next_yaw = NormalizeGatePatrolAngleNear(
                        gate_patrol_last_yaw + yaw_step,
                        gimbalAngles.Yaw);
                }
                gate_patrol_last_yaw = next_yaw;

                const float pitch_elapsed_ms = static_cast<float>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now_steady - wait_begin).count());
                const float next_pitch = patrol_mode == 3
                    ? kGatePatrolOutpostPitchDeg
                    : kGatePatrolPitchCenterDeg +
                        kGatePatrolPitchHalfRangeDeg *
                            std::sin(
                                pitch_elapsed_ms * kGatePatrolTwoPi /
                                std::max(kGatePatrolPitchPeriodMs, 1.0f));
                gimbalControlData.GimbalAngles = GimbalAnglesType{
                    static_cast<AngleType>(next_yaw),
                    static_cast<AngleType>(next_pitch)};

                if (now_steady - last_gate_patrol_log > std::chrono::seconds(2)) {
                    LoggerPtr->Debug(
                        "Start gate gimbal patrol: mode={} yaw={} pitch={}.",
                        patrol_mode,
                        gimbalControlData.GimbalAngles.Yaw,
                        gimbalControlData.GimbalAngles.Pitch);
                    last_gate_patrol_log = now_steady;
                }
            } else {
                gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw;
                gimbalControlData.GimbalAngles.Pitch = AngleType{0};
            }
            PubNaviControlData();
            PubGimbalControlData();

            if (debugBypassGameStart_) {
                if (!bypass_logged) {
                    LoggerPtr->Warning(
                        "debug_bypass_is_start=true, skip waiting for /ly/game/is_start.");
                    bypass_logged = true;
                }
                break;
            }

            if (waitForGameStartTimeoutSec_ > 0 &&
                (now_steady - wait_begin) > std::chrono::seconds(waitForGameStartTimeoutSec_)) {
                LoggerPtr->Warning(
                    "WaitForGameStart timeout after {}s, continue without is_start gate.",
                    waitForGameStartTimeoutSec_);
                break;
            }

            if (now_steady - last_wait_log > std::chrono::seconds(2)) {
                if (!hasReceivedGameStartFlag_) {
                    LoggerPtr->Warning("Waiting /ly/game/is_start message...");
                } else {
                    LoggerPtr->Debug("Waiting /ly/game/is_start=true...");
                }
                last_wait_log = now_steady;
            }

            if (is_game_begin) {
                LoggerPtr->Info("!!!!Game !! start!!!!");
                break;
            }

            if (damage_open_gate_enabled) {
                const auto is_health_input_ready = [&]() -> bool {
                    if (!hasReceivedMyselfHealth_) {
                        return false;
                    }
                    if (lastMyselfHealthRxTime.time_since_epoch().count() == 0) {
                        return false;
                    }
                    if (leagueRefereeStaleTimeoutMs_ <= 0) {
                        return true;
                    }
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now_steady - lastMyselfHealthRxTime).count();
                    return age_ms <= leagueRefereeStaleTimeoutMs_;
                };

                if (is_health_input_ready()) {
                    if (!damage_health_baseline_initialized) {
                        damage_health_peak = myselfHealth;
                        damage_health_baseline_initialized = true;
                        LoggerPtr->Info(
                            "Damage gate baseline health captured: {}.",
                            static_cast<int>(damage_health_peak));
                    } else {
                        if (myselfHealth > damage_health_peak) {
                            damage_health_peak = myselfHealth;
                        }
                        const std::uint16_t health_drop = damage_health_peak > myselfHealth
                            ? static_cast<std::uint16_t>(damage_health_peak - myselfHealth)
                            : 0;
                        if (health_drop >= damage_open_gate_threshold) {
                            LoggerPtr->Warning(
                                "Damage gate opened by health drop: peak={} current={} drop={} threshold={}.",
                                static_cast<int>(damage_health_peak),
                                static_cast<int>(myselfHealth),
                                static_cast<int>(health_drop),
                                static_cast<int>(damage_open_gate_threshold));
                            break;
                        }
                    }
                } else if (now_steady - last_damage_gate_log > std::chrono::seconds(2)) {
                    LoggerPtr->Warning(
                        "Damage gate waiting for fresh /ly/game/all.selfhealth (stale_timeout_ms={}).",
                        leagueRefereeStaleTimeoutMs_);
                    last_damage_gate_log = now_steady;
                }
            }
        }
        LoggerPtr->Info("Stop Waiting For Game");
    }

    void Application::WaitBeforeGame() {
        LoggerPtr->Info("Waiting Before Game");
        PublishSafeControl("wait_before_game_reset");

        /// 取得第一个云台数据包（不能用 yaw/pitch 非 0 判定，0 也是合法角度）
        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        constexpr auto kMaxWait = std::chrono::seconds(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);

            if (hasReceivedGimbalAngles_.load()) {
                LoggerPtr->Info("First gimbal message received.");
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - wait_begin > kMaxWait) {
                LoggerPtr->Warning("No gimbal message within {}s, continue with current angles (Yaw={}, Pitch={}).",
                    std::chrono::duration_cast<std::chrono::seconds>(kMaxWait).count(),
                    gimbalAngles.Yaw, gimbalAngles.Pitch);
                break;
            }
            if (now - last_wait_log > std::chrono::seconds(2)) {
                LoggerPtr->Warning("Waiting for first gimbal message...");
                last_wait_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        LoggerPtr->Info("Stop waiting for first gimbal data.");
        
        WaitForGameStart();

        LoggerPtr->Info("Stop Waiting Before Game");
    }
}
