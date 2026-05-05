// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <cmath>

using namespace LangYa;

namespace BehaviorTree{

    void Application::PrintMessageAll() {
        LoggerPtr->Debug("-------------PrintMessageAll-------------");
        LoggerPtr->Debug("|  Myself Hreo Position: {}, {}", friendRobots[UnitType::Hero].position_.X, friendRobots[UnitType::Hero].position_.Y);
        LoggerPtr->Debug("|  Myself Engineer Position: {}, {}", friendRobots[UnitType::Engineer].position_.X, friendRobots[UnitType::Engineer].position_.Y);
        LoggerPtr->Debug("|  Myself Infantry1 Position: {}, {}", friendRobots[UnitType::Infantry1].position_.X, friendRobots[UnitType::Infantry1].position_.Y);
        LoggerPtr->Debug("|  Myself Infantry2 Position: {}, {}", friendRobots[UnitType::Infantry2].position_.X, friendRobots[UnitType::Infantry2].position_.Y);
        LoggerPtr->Debug("|  Myself Sentry Position: {}, {}", friendRobots[UnitType::Sentry].position_.X, friendRobots[UnitType::Sentry].position_.Y);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Enemy Hero Position: {}, {}", enemyRobots[UnitType::Hero].position_.X, enemyRobots[UnitType::Hero].position_.Y);
        LoggerPtr->Debug("|  Enemy Engineer Position: {}, {}", enemyRobots[UnitType::Engineer].position_.X, enemyRobots[UnitType::Engineer].position_.Y);
        LoggerPtr->Debug("|  Enemy Infantry1 Position: {}, {}", enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y);
        LoggerPtr->Debug("|  Enemy Infantry2 Position: {}, {}", enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y);
        LoggerPtr->Debug("|  Enemy Sentry Position: {}, {}", enemyRobots[UnitType::Sentry].position_.X, enemyRobots[UnitType::Sentry].position_.Y);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Myself Hero Health: {}", friendRobots[UnitType::Hero].currentHealth_);
        LoggerPtr->Debug("|  Myself Engineer Health: {}", friendRobots[UnitType::Engineer].currentHealth_);
        LoggerPtr->Debug("|  Myself Infantry1 Health: {}", friendRobots[UnitType::Infantry1].currentHealth_);
        LoggerPtr->Debug("|  Myself Infantry2 Health: {}", friendRobots[UnitType::Infantry2].currentHealth_);
        LoggerPtr->Debug("|  Myself Sentry Health: {}", friendRobots[UnitType::Sentry].currentHealth_);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Enemy Hero Health: {}", enemyRobots[UnitType::Hero].currentHealth_);
        LoggerPtr->Debug("|  Enemy Engineer Health: {}", enemyRobots[UnitType::Engineer].currentHealth_);
        LoggerPtr->Debug("|  Enemy Infantry1 Health: {}", enemyRobots[UnitType::Infantry1].currentHealth_);
        LoggerPtr->Debug("|  Enemy Infantry2 Health: {}", enemyRobots[UnitType::Infantry2].currentHealth_);
        LoggerPtr->Debug("|  Enemy Sentry Health: {}", enemyRobots[UnitType::Sentry].currentHealth_);
        LoggerPtr->Debug("|------------------------------------");
        LoggerPtr->Debug("|  DefenceBuff: {}", int(teamBuff.DefenceBuff));
        LoggerPtr->Debug("|  RecoveryBuff: {}", int(teamBuff.RecoveryBuff));
        LoggerPtr->Debug("|  RemainingEnergy: {}", int(teamBuff.RemainingEnergy));
        LoggerPtr->Debug("|  VulnerabilityBuff: {}", int(teamBuff.VulnerabilityBuff));
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  CapV: {}", int(capV));
        LoggerPtr->Debug("-----------End PrintMessageAll-----------");
    }
    void Application::SubscribeMessageAll() {
        // 输入分组说明：
        // 1) 云台与火控回读
        // 2) 裁判/比赛态数据（血量、弹药、时间、开赛标志）
        // 3) 感知与预测结果（装甲板、predictor/buff/outpost 目标）
        // 4) 导航与定位（速度、位置、低头标志）
        auto cache_event_data_raw = [](Application& app, const std::uint32_t raw) {
            app.extEventData = raw;
            app.eventSelfSmallEnergyStatus_ = static_cast<std::uint8_t>((raw >> 3U) & 0x03U);
            app.eventSelfLargeEnergyStatus_ = static_cast<std::uint8_t>((raw >> 5U) & 0x03U);
            app.hasReceivedEventData_ = true;
            app.lastEventDataRxTime_ = std::chrono::steady_clock::now();
        };

        // ly_gimbal_angles
        GenSub<ly_gimbal_angles>([](Application& app, auto msg) {
            app.gimbalAngles = GimbalAnglesType{
                static_cast<AngleType>(msg->yaw),
                static_cast<AngleType>(msg->pitch)
            }; // ROS2改成小寫
            app.hasReceivedGimbalAngles_ = true;
            app.lastGimbalAnglesRxTime = std::chrono::steady_clock::now();
        });

        // ly_gimbal_firecode
        GenSub<ly_gimbal_firecode>([](Application& app, auto msg) {
            app.RecFireCode.FireStatus = msg->fire_status & 0b11;
            app.RecFireCode.CapState = msg->cap_state & 0b11;
            app.RecFireCode.FollowMode = msg->follow_mode ? 1 : 0;
            app.RecFireCode.AimMode = msg->aim_mode ? 1 : 0;
            app.RecFireCode.Rotate = msg->rotate & 0b11;
        });

        // ly_gimbal_chassis
        GenSub<ly_gimbal_chassis>([](Application& app, auto msg) {
            app.gimbalYawVelDegPerSec = static_cast<float>(msg->angular_velocity);
            app.gimbalYawAngleDeg = static_cast<float>(msg->steer_angle);
            app.gimbalYawVelRaw = static_cast<std::int16_t>(std::lround(msg->angular_velocity * 100.0f));
            app.gimbalYawAngleRaw = static_cast<std::int16_t>(std::lround(msg->steer_angle * 100.0f));
        });

        // ly_gimbal_posture
        GenSub<ly_gimbal_posture>([](Application& app, auto msg) {
            app.postureState = msg->data;
        });

        // ly_gimbal_capV
        GenSub<ly_gimbal_capV>([](Application& app, auto msg) {
            app.capV = msg->data;
        });

        // ly_game_eventdata
        GenSub<ly_game_eventdata>([cache_event_data_raw](Application& app, auto msg) {
            cache_event_data_raw(app, msg->data);
        });

        // ly_game_event_data
        GenSub<ly_game_event_data>([](Application& app, auto msg) {
            app.extEventData = msg->raw;
            app.eventSelfSmallEnergyStatus_ = msg->self_small_energy_status;
            app.eventSelfLargeEnergyStatus_ = msg->self_large_energy_status;
            app.hasReceivedEventData_ = true;
            app.lastEventDataRxTime_ = std::chrono::steady_clock::now();
        });

        // 兼容历史 topic（无前导 '/'）:
        // gimbal_driver 旧版本可能发布到 "ly/gimbal/eventdata"。
        // 为避免链路断开，这里额外订阅一次，统一写入同一变量。
        auto legacy_game_event_sub = node_->create_subscription<std_msgs::msg::UInt32>(
            "ly/gimbal/eventdata",
            rclcpp::QoS(10),
            [this, cache_event_data_raw](const std_msgs::msg::UInt32::SharedPtr msg) {
                cache_event_data_raw(*this, msg->data);
            }
        );
        subscribers_.push_back(legacy_game_event_sub);

        // ly_me_is_team_red
        GenSub<ly_me_is_team_red>([](Application& app, auto msg) {
            app.team = msg->data ? UnitTeam::Red : UnitTeam::Blue;
        });

        // ly_game_all
        GenSub<ly_game_all>([](Application& app, auto msg) {
            // 联赛回补逻辑依赖这组“最近接收时间”，用于 stale 防护。
            app.myselfHealth = msg->selfhealth;
            app.hasReceivedMyselfHealth_ = true;
            app.lastMyselfHealthRxTime = std::chrono::steady_clock::now();
        });

        // ly_enemy_op_hp
        GenSub<ly_enemy_op_hp>([](Application& app, auto msg) {
            app.enemyOutpostHealth = msg->data;
        });

        // ly_me_op_hp
        GenSub<ly_me_op_hp>([](Application& app, auto msg) {
            app.selfOutpostHealth = msg->data;
        });

        // ly_me_base_hp
        GenSub<ly_me_base_hp>([](Application& app, auto msg) {
            app.selfBaseHealth = msg->data;
        });

        // ly_enemy_base_hp
        GenSub<ly_enemy_base_hp>([](Application& app, auto msg) {
            app.enemyBaseHealth = msg->data;
        });

        // ly_me_ammo_left
        GenSub<ly_me_ammo_left>([](Application& app, auto msg) {
            app.ammoLeft = msg->data;
            app.hasReceivedAmmoLeft_ = true;
            app.lastAmmoLeftRxTime = std::chrono::steady_clock::now();
        });

        // ly_game_time_left
        GenSub<ly_game_time_left>([](Application& app, auto msg) {
            app.timeLeft = msg->data;
        });

        // ly_game_is_start
        GenSub<ly_game_is_start>([](Application& app, auto msg) {
            app.is_game_begin = msg->data;
            app.hasReceivedGameStartFlag_ = true;
            app.lastGameStartRxTime = std::chrono::steady_clock::now();
        });

        // ly_navi_vel
        GenSub<ly_navi_vel>([](Application& app, auto msg) {
            app.naviVelocityInput.X = msg->x;
            app.naviVelocityInput.Y = msg->y;
            // 兼容旧语义：保留原始转发链路变量
            app.naviVelocity.X = msg->x;
            app.naviVelocity.Y = msg->y;
        });

        // ly_navi_lower_head
        GenSub<ly_navi_lower_head>([](Application& app, auto msg) {
            app.naviLowerHead = msg->data;
        });

        // ly_navi_reached
        GenSub<ly_navi_reached>([](Application& app, auto msg) {
            app.naviReach = msg->data;
            app.hasReceivedNaviReach_ = true;
            app.lastNaviReachRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_navi_reachable
        GenSub<ly_navi_reachable>([](Application& app, auto msg) {
            app.naviReachable = msg->data;
            app.hasReceivedNaviReachable_ = true;
            app.lastNaviReachableRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_team_buff
        GenSub<ly_team_buff>([](Application& app, auto msg) {
            app.teamBuff.RecoveryBuff = msg->recoverybuff;
            app.teamBuff.CoolingBuff = msg->coolingbuff;
            app.teamBuff.DefenceBuff = msg->defencebuff;
            app.teamBuff.VulnerabilityBuff = msg->vulnerabilitybuff;
            app.teamBuff.AttackBuff = msg->attackbuff;
            app.teamBuff.RemainingEnergy = msg->remainingenergy;
        });

        // ly_me_rfid
        GenSub<ly_me_rfid>([](Application& app, auto msg) {
            app.rfidStatus = msg->raw;
            app.hasRfidStatus2 = msg->has_rfid_status_2;
            app.rfidStatus2 = msg->rfid_status_2_raw;
        });

        // ly_navi_position
        // Navigation/TF-derived self position in official-map centimeters: [x, y].
        GenSub<ly_navi_position>([](Application& app, auto msg) {
            if (msg->data.size() < 2) {
                const auto now = std::chrono::steady_clock::now();
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    if (app.LoggerPtr) {
                        app.LoggerPtr->Warning(
                            "Ignore invalid /ly/navi/position: data size={} (need >=2)",
                            msg->data.size());
                    }
                    app.lastPositionDataGuardLogTime_ = now;
                }
                return;
            }
            const auto sentry_index = static_cast<std::size_t>(UnitType::Sentry);
            app.friendRobots[UnitType::Sentry].position_.X = msg->data[0];
            app.friendRobots[UnitType::Sentry].position_.Y = msg->data[1];
            app.hasReceivedSentryPosition_ = true;
            app.lastSentryPositionRxTime_ = std::chrono::steady_clock::now();
            app.lastFriendPositionRxTime_[sentry_index] = app.lastSentryPositionRxTime_;
        });

        // ly_position_data
        GenSub<ly_position_data>([](Application& app, auto msg) {
            int FriendCarId = msg->friendcarid;
            auto in_range = [](const int idx) { return idx >= 0 && idx < 10; };
            auto maybe_warn_invalid_id = [&](const char* side, const int raw_id) {
                const auto now = std::chrono::steady_clock::now();
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    app.LoggerPtr->Warning(
                        "Ignore invalid {} car id from /ly/position/data: {}",
                        side, raw_id);
                    app.lastPositionDataGuardLogTime_ = now;
                }
            };
            if (in_range(FriendCarId)) {
                app.friendRobots[FriendCarId].position_.X = msg->friendx;
                app.friendRobots[FriendCarId].position_.Y = 1500 - msg->friendy;
                app.lastFriendPositionRxTime_[static_cast<std::size_t>(FriendCarId)] =
                    std::chrono::steady_clock::now();
                if (FriendCarId == static_cast<int>(UnitType::Sentry)) {
                    app.hasReceivedSentryPosition_ = true;
                    app.lastSentryPositionRxTime_ =
                        app.lastFriendPositionRxTime_[static_cast<std::size_t>(FriendCarId)];
                }
            } else {
                maybe_warn_invalid_id("friend", FriendCarId);
            }
            int EnemyCarId = msg->enemycarid;
            EnemyCarId = EnemyCarId % 100;
            if (in_range(EnemyCarId)) {
                app.enemyRobots[EnemyCarId].position_.X = msg->enemyx;
                app.enemyRobots[EnemyCarId].position_.Y = 1500 - msg->enemyy;
                app.lastEnemyPositionRxTime_[static_cast<std::size_t>(EnemyCarId)] =
                    std::chrono::steady_clock::now();
            } else {
                maybe_warn_invalid_id("enemy", EnemyCarId);
            }
        });

        // ly_detector_armors
        GenSub<ly_detector_armors>([](Application& app, auto msg) {
            auto &armorList = app.armorList;
            std::fill(armorList.begin(), armorList.end(), ArmorData{ArmorType::UnKnown, 30});
            const auto &armors = msg->armors;
            int count = std::min(10, static_cast<int>(armors.size()));
            for (int i = 0; i < count; ++i) {
                armorList[i] = ArmorData{
                    static_cast<ArmorType>(armors[i].type),
                    armors[i].distance
                };
            }
        });

        // ly_predictor_target
        GenSub<ly_predictor_target>([](Application& app, auto msg) {
            auto &obj = app;
            const bool target_valid = msg->status;
            obj.autoAimData.Angles = GimbalAnglesType{
                static_cast<AngleType>(msg->yaw),
                static_cast<AngleType>(msg->pitch)
            };
            obj.autoAimData.BuffFollow = false;
            obj.autoAimData.FireStatus = target_valid;
            obj.autoAimData.Valid = target_valid;
            obj.autoAimData.Fresh = target_valid;
            const auto now = std::chrono::steady_clock::now();
            if (target_valid) {
                obj.autoAimData.HasLatchedAngles = true;
                obj.autoAimData.LastValidTime = now;
                obj.isFindTargetAtomic = true;
                obj.lastTargetSeenTime = now;
                obj.LoggerPtr->Debug("Predictor callback latched valid auto-aim angles.");
            } else {
                obj.autoAimData.HasLatchedAngles = false;
                obj.LoggerPtr->Debug("Predictor callback ignored invalid auto-aim target.");
            }
        });

        // ly_buff_target
        GenSub<ly_buff_target>([](Application& app, auto msg) { 
            auto  &obj = app;
            obj.buffAimData.Angles = GimbalAnglesType{
                static_cast<AngleType>(msg->yaw),
                static_cast<AngleType>(msg->pitch)
            };
            obj.buffAimData.FireStatus = msg->status;
            obj.buffAimData.BuffFollow = true;
            obj.buffAimData.Valid = true;
            obj.buffAimData.Fresh = true;
            const auto now = std::chrono::steady_clock::now();
            obj.buffAimData.HasLatchedAngles = true;
            obj.buffAimData.LastValidTime = now;
            obj.isFindTargetAtomic = true;
            obj.lastTargetSeenTime = now;
        });

        // ly_outpost_target
        GenSub<ly_outpost_target>([](Application& app, auto msg) {
            auto &obj = app;
            obj.outpostAimData.Angles = GimbalAnglesType{
                static_cast<AngleType>(msg->yaw),
                static_cast<AngleType>(msg->pitch)
            };
            obj.outpostAimData.FireStatus = true;
            obj.outpostAimData.BuffFollow = false;
            obj.outpostAimData.Valid = true;
            obj.outpostAimData.Fresh = true;
            const auto now = std::chrono::steady_clock::now();
            obj.outpostAimData.HasLatchedAngles = true;
            obj.outpostAimData.LastValidTime = now;
            obj.isFindTargetAtomic = true;
            obj.lastTargetSeenTime = now;
        });

        // ly_face_mode_angles
        GenSub<ly_face_mode_angles>([](Application& app, auto msg) {
            app.faceModeManager_.CacheAngles(
                app.faceModeData,
                msg->yaw,
                msg->pitch,
                std::chrono::steady_clock::now());
        });

        // ly_enemy_hp
        GenSub<ly_enemy_hp>([](Application& app, auto msg) {
            app.enemyRobots[UnitType::Hero].setCurrentHealth(static_cast<std::uint16_t>(msg->hero));
            app.enemyRobots[UnitType::Engineer].setCurrentHealth(static_cast<std::uint16_t>(msg->engineer));
            app.enemyRobots[UnitType::Infantry1].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry1));
            app.enemyRobots[UnitType::Infantry2].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry2));
            app.enemyRobots[UnitType::Sentry].setCurrentHealth(static_cast<std::uint16_t>(msg->sentry));
        });

        // ly_me_hp
        GenSub<ly_me_hp>([](Application& app, auto msg) {
            app.friendRobots[UnitType::Hero].setCurrentHealth(static_cast<std::uint16_t>(msg->hero));
            app.friendRobots[UnitType::Engineer].setCurrentHealth(static_cast<std::uint16_t>(msg->engineer));
            app.friendRobots[UnitType::Infantry1].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry1));
            app.friendRobots[UnitType::Infantry2].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry2));
            app.friendRobots[UnitType::Sentry].setCurrentHealth(static_cast<std::uint16_t>(msg->sentry));
        });
    }    

}
