// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <cmath>

using namespace LangYa;

namespace BehaviorTree{

    namespace {
    RfidMatchState BuildRfidMatchState(const gimbal_driver::msg::RfidStatus& msg) {
        RfidMatchState state;
        state.Fresh = true;
        state.Raw = msg.raw;
        state.HasRfidStatus2 = msg.has_rfid_status_2;
        state.RfidStatus2Raw = msg.rfid_status_2_raw;

        state.SelfBaseGainPoint = msg.friend_base;
        state.SelfNonResourceSupply = msg.friend_supply_noremix;
        state.SelfResourceSupply = msg.friend_supply_remix;
        state.SelfSupply = state.SelfNonResourceSupply || state.SelfResourceSupply;
        state.SelfHighlandGainPoint =
            msg.friend_central || msg.friend_highland;
        state.EnemyHighlandGainPoint =
            msg.enemy_central || msg.enemy_highland;
        state.SelfRoadCrossing = msg.friend_roadland_under || msg.friend_roadland_high;
        state.EnemyRoadCrossing = msg.enemy_roadland_under || msg.enemy_roadland_high;
        state.SelfCentralHighlandCrossing =
            msg.friend_central_under || msg.friend_central_high;
        state.EnemyCentralHighlandCrossing =
            msg.enemy_central_under || msg.enemy_central_high;
        state.SelfTunnel =
            msg.friend_tunnel_roadland_down || msg.friend_tunnel_roadland_mid ||
            msg.friend_tunnel_roadland_up || msg.friend_tunnel_highland_low ||
            msg.friend_tunnel_highland_mid || msg.friend_tunnel_highland_high;
        state.EnemyTunnel =
            msg.enemy_tunnel_roadland_down || msg.enemy_tunnel_roadland_mid ||
            msg.enemy_tunnel_roadland_up || msg.enemy_tunnel_highland_low ||
            msg.enemy_tunnel_highland_mid || msg.enemy_tunnel_highland_high;
        state.Tunnel = state.SelfTunnel || state.EnemyTunnel;
        state.CenterGainPoint = msg.central_rmul;
        state.SelfFortressGainPoint = msg.friend_bastion;
        state.EnemyFortressGainPoint = msg.enemy_bastion;
        state.SelfOutpostGainPoint = msg.friend_outpost;
        state.EnemyOutpostGainPoint = msg.enemy_outpost;
        state.SelfAssemblyGainPoint = msg.friend_armor;
        state.EnemyAssemblyGainPoint = msg.enemy_armor;
        state.SelfFlyRamp = msg.friend_flyroad_front || msg.friend_flyroad_back;
        state.EnemyFlyRamp = msg.enemy_flyroad_front || msg.enemy_flyroad_back;

        state.OnSelfSideRfid =
            state.SelfBaseGainPoint || state.SelfSupply || state.SelfHighlandGainPoint ||
            state.SelfRoadCrossing || state.SelfCentralHighlandCrossing || state.SelfTunnel ||
            state.SelfFortressGainPoint || state.SelfOutpostGainPoint || state.SelfAssemblyGainPoint ||
            state.SelfFlyRamp;
        state.OnEnemySideRfid =
            state.EnemyHighlandGainPoint || state.EnemyRoadCrossing ||
            state.EnemyCentralHighlandCrossing || state.EnemyTunnel || state.EnemyFortressGainPoint ||
            state.EnemyOutpostGainPoint || state.EnemyAssemblyGainPoint || state.EnemyFlyRamp;
        state.Any = state.OnSelfSideRfid || state.OnEnemySideRfid || state.CenterGainPoint;
        return state;
    }
    }  // namespace

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

        // ly_game_event_data: semantic referee 0x0101 event_data topic.
        GenSub<ly_game_event_data>([](Application& app, auto msg) {
            app.extEventData = msg->raw;
            app.eventSelfSmallEnergyStatus_ = msg->self_small_energy_status;
            app.eventSelfLargeEnergyStatus_ = msg->self_large_energy_status;
            app.eventSelfFortressGainPointStatus_ = msg->self_fortress_gain_point_status;
            app.eventSelfOutpostGainPointStatus_ = msg->self_outpost_gain_point_status;
            app.eventSelfBaseGainPointStatus_ = msg->self_base_gain_point_status;
            app.hasReceivedEventData_ = true;
            app.lastEventDataRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_game_sentry_info: referee 0x020D sentry_info/sentry_info_2 semantic topic.
        GenSub<ly_game_sentry_info>([](Application& app, auto msg) {
            app.sentryCanActivateEnergyMechanism_ = msg->can_activate_energy_mechanism;
            app.hasReceivedSentryInfo_ = true;
            app.lastSentryInfoRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_friend_is_team_red
        GenSub<ly_friend_is_team_red>([](Application& app, auto msg) {
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
            app.hasReceivedEnemyOutpostHealth_ = true;
            app.lastEnemyOutpostHealthRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_friend_op_hp
        GenSub<ly_friend_op_hp>([](Application& app, auto msg) {
            app.selfOutpostHealth = msg->data;
        });

        // ly_friend_base_hp
        GenSub<ly_friend_base_hp>([](Application& app, auto msg) {
            app.selfBaseHealth = msg->data;
        });

        // ly_enemy_base_hp
        GenSub<ly_enemy_base_hp>([](Application& app, auto msg) {
            app.enemyBaseHealth = msg->data;
        });

        // ly_friend_ammo_left
        GenSub<ly_friend_ammo_left>([](Application& app, auto msg) {
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

        // ly_navi_is_rotate
        GenSub<ly_navi_is_rotate>([](Application& app, auto msg) {
            app.naviIsRotate = msg->data;
            app.hasReceivedNaviIsRotate_ = true;
            app.lastNaviIsRotateRxTime_ = std::chrono::steady_clock::now();
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

        // ly_game_rfid
        GenSub<ly_game_rfid>([](Application& app, auto msg) {
            app.rfidStatus = msg->raw;
            app.hasRfidStatus2 = msg->has_rfid_status_2;
            app.rfidStatus2 = msg->rfid_status_2_raw;
            app.rfidMatchState = BuildRfidMatchState(*msg);
            app.hasReceivedRfidStatus_ = true;
            app.lastRfidStatusRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_game_bullet
        // 先缓存 TypeID 7/8 合并后的弹速/发射事件/允许发弹量状态；当前决策仍使用旧 ammo/speed 输入。
        GenSub<ly_game_bullet>([](Application& app, auto msg) {
            app.bulletInfo = *msg;
            app.hasReceivedBulletInfo_ = true;
            app.lastBulletInfoRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_navi_position
        // Navigation/TF-derived self position in official-map centimeters: [x, y].
        GenSub<ly_navi_position>([](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            if (msg->data.size() < 2) {
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
            if (app.lastSentryRadarPositionRxTime_.time_since_epoch().count() != 0 &&
                now - app.lastSentryRadarPositionRxTime_ <= std::chrono::seconds(2)) {
                return;
            }
            app.friendRobots[UnitType::Sentry].position_.X = msg->data[0];
            app.friendRobots[UnitType::Sentry].position_.Y = msg->data[1];
            app.hasReceivedSentryPosition_ = true;
            app.lastSentryPositionRxTime_ = now;
            app.lastFriendPositionRxTime_[sentry_index] = app.lastSentryPositionRxTime_;
        });

        // ly_friend_uwb_pos
        // Dedicated radar/UWB self position. Prefer it over the generic PositionData friend slot.
        GenSub<ly_friend_uwb_pos>([](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            if (msg->data.size() < 2) {
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    if (app.LoggerPtr) {
                        app.LoggerPtr->Warning(
                            "Ignore invalid /ly/friend/uwb_pos: data size={} (need >=2)",
                            msg->data.size());
                    }
                    app.lastPositionDataGuardLogTime_ = now;
                }
                return;
            }

            const auto sentry_index = static_cast<std::size_t>(UnitType::Sentry);
            app.friendRobots[UnitType::Sentry].position_.X =
                static_cast<std::int16_t>(msg->data[0]);
            app.friendRobots[UnitType::Sentry].position_.Y =
                static_cast<std::int16_t>(1500 - static_cast<int>(msg->data[1]));
            app.hasReceivedSentryPosition_ = true;
            app.lastSentryPositionRxTime_ = now;
            app.lastSentryRadarPositionRxTime_ = now;
            app.lastFriendPositionRxTime_[sentry_index] = now;
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
                const auto now = std::chrono::steady_clock::now();
                const auto friend_index = static_cast<std::size_t>(FriendCarId);
                const bool friend_is_sentry = FriendCarId == static_cast<int>(UnitType::Sentry);
                const bool radar_sentry_position_fresh =
                    app.lastSentryRadarPositionRxTime_.time_since_epoch().count() != 0 &&
                    now - app.lastSentryRadarPositionRxTime_ <= std::chrono::seconds(2);
                if (!friend_is_sentry || !radar_sentry_position_fresh) {
                    app.friendRobots[FriendCarId].position_.X = msg->friendx;
                    app.friendRobots[FriendCarId].position_.Y = 1500 - msg->friendy;
                    app.lastFriendPositionRxTime_[friend_index] = now;
                }
                if (friend_is_sentry && !radar_sentry_position_fresh) {
                    app.hasReceivedSentryPosition_ = true;
                    app.lastSentryPositionRxTime_ = app.lastFriendPositionRxTime_[friend_index];
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
            if (app.config.ExternalAimSettings.Enable) {
                return;
            }
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

#ifdef LY_ENABLE_SENTRY_MSGS
        // ly_aim_armor_targets: external sentry.aim target candidates.
        GenSubWithQoS<ly_aim_armor_targets>(rclcpp::SensorDataQoS(), [](Application& app, auto msg) {
            if (!app.config.ExternalAimSettings.Enable) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            std::fill(app.externalAimTargets_.begin(),
                      app.externalAimTargets_.end(),
                      ExternalAimTargetCache{});
            if (app.config.ExternalAimSettings.UseTargetArrayAsArmorList) {
                std::fill(app.armorList.begin(), app.armorList.end(), ArmorData{ArmorType::UnKnown, 30});
            }

            std::size_t armor_index = 0;
            for (const auto& target : msg->aim_targets) {
                const auto target_id = static_cast<std::size_t>(target.id);
                if (target_id >= app.externalAimTargets_.size()) {
                    continue;
                }
                const auto x = static_cast<float>(target.position.x);
                const auto y = static_cast<float>(target.position.y);
                const auto z = static_cast<float>(target.position.z);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }
                const float distance = std::hypot(x, y, z);
                app.externalAimTargets_[target_id] = ExternalAimTargetCache{
                    .Valid = true,
                    .X = x,
                    .Y = y,
                    .Z = z,
                    .Distance = std::isfinite(distance) && distance > 0.0f ? distance : 30.0f,
                    .LastSeen = now
                };
                if (app.config.ExternalAimSettings.UseTargetArrayAsArmorList &&
                    armor_index < app.armorList.size()) {
                    app.armorList[armor_index++] = ArmorData{
                        static_cast<ArmorType>(target.id),
                        app.externalAimTargets_[target_id].Distance
                    };
                }
            }
            app.hasExternalAimTargets_ = true;
            app.lastExternalAimTargetsRxTime_ = now;
        });

        // ly_aim_result: external sentry.aim final yaw/pitch plus fire gate.
        GenSub<ly_aim_result>([](Application& app, auto msg) {
            if (!app.config.ExternalAimSettings.Enable) {
                return;
            }
            const auto yaw = static_cast<AngleType>(msg->yaw);
            const auto pitch = static_cast<AngleType>(msg->pitch);
            const bool finite_angles = std::isfinite(yaw) && std::isfinite(pitch);
            const auto now = std::chrono::steady_clock::now();
            bool target_context_valid = true;
            if (app.config.ExternalAimSettings.UseTargetArrayAsArmorList) {
                target_context_valid = false;
                const auto target_id = static_cast<std::size_t>(app.targetArmor.Type);
                const int fresh_ms = std::max(1, app.config.ExternalAimSettings.TargetFreshTimeoutMs);
                if (target_id < app.externalAimTargets_.size()) {
                    const auto& cached = app.externalAimTargets_[target_id];
                    target_context_valid =
                        cached.Valid &&
                        cached.LastSeen.time_since_epoch().count() != 0 &&
                        now - cached.LastSeen <= std::chrono::milliseconds(fresh_ms);
                }
            }
            const bool result_valid = finite_angles && target_context_valid;
            app.externalAimData.Angles = GimbalAnglesType{yaw, pitch};
            app.externalAimData.BuffFollow = false;
            app.externalAimData.FireStatus = result_valid && msg->fire;
            app.externalAimData.Valid = result_valid;
            app.externalAimData.Fresh = result_valid;
            app.lastExternalAimResultRxTime_ = now;
            if (result_valid) {
                app.externalAimData.HasLatchedAngles = true;
                app.externalAimData.LastValidTime = now;
                app.isFindTargetAtomic = true;
                app.lastTargetSeenTime = now;
            } else {
                app.externalAimData.FireStatus = false;
                app.externalAimData.HasLatchedAngles = false;
            }
        });
#endif

        // ly_predictor_target
        GenSub<ly_predictor_target>([](Application& app, auto msg) {
            if (app.config.ExternalAimSettings.Enable) {
                return;
            }
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
            if (app.config.ExternalAimSettings.Enable) {
                return;
            }
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
            if (app.config.ExternalAimSettings.Enable) {
                return;
            }
            auto &obj = app;
            const bool target_valid = msg->status;
            obj.outpostAimData.Angles = GimbalAnglesType{
                static_cast<AngleType>(msg->yaw),
                static_cast<AngleType>(msg->pitch)
            };
            obj.outpostAimData.FireStatus = target_valid;
            obj.outpostAimData.BuffFollow = false;
            obj.outpostAimData.Valid = target_valid;
            obj.outpostAimData.Fresh = target_valid;
            const auto now = std::chrono::steady_clock::now();
            if (target_valid) {
                obj.outpostAimData.HasLatchedAngles = true;
                obj.outpostAimData.LastValidTime = now;
                obj.isFindTargetAtomic = true;
                obj.lastTargetSeenTime = now;
            } else {
                obj.outpostAimData.HasLatchedAngles = false;
            }
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
            const auto now = std::chrono::steady_clock::now();
            const auto& aim_target = app.config.DecisionAutonomySettings.AimTarget;
            const int dead_confirm_ms = std::max(0, aim_target.DeadHealthConfirmMs);
            const int respawn_transition_timeout_ms =
                std::max(0, aim_target.RespawnTransitionTimeoutMs);
            const auto update_health = [&](const UnitType unit, const std::uint16_t health) {
                const auto index = static_cast<std::size_t>(unit);
                app.lastEnemyHealthRxTime_[index] = now;
                if (health == 0) {
                    if (!app.enemyZeroHealthObserved_[index]) {
                        app.enemyZeroHealthObserved_[index] = true;
                        app.enemyZeroHealthSince_[index] = now;
                    }
                    const auto zero_since = app.enemyZeroHealthSince_[index];
                    const bool zero_confirmed =
                        dead_confirm_ms == 0 ||
                        (zero_since.time_since_epoch().count() != 0 &&
                         now - zero_since >= std::chrono::milliseconds(dead_confirm_ms));
                    if (zero_confirmed) {
                        app.enemyHealthConfirmedDead_[index] = true;
                        app.lastEnemyConfirmedDeadTime_[index] = now;
                        app.enemyRobots[unit].setCurrentHealth(0, false);
                    }
                    return;
                }

                const auto last_confirmed_dead = app.lastEnemyConfirmedDeadTime_[index];
                const bool confirmed_dead_recent =
                    app.enemyHealthConfirmedDead_[index] &&
                    last_confirmed_dead.time_since_epoch().count() != 0 &&
                    (respawn_transition_timeout_ms == 0 ||
                     now - last_confirmed_dead <=
                         std::chrono::milliseconds(respawn_transition_timeout_ms));
                app.enemyZeroHealthObserved_[index] = false;
                app.enemyZeroHealthSince_[index] = {};
                app.enemyHealthConfirmedDead_[index] = false;
                if (confirmed_dead_recent) {
                    app.enemyRobots[unit].markInvulnerableStartNow();
                }
                app.enemyRobots[unit].setCurrentHealth(health, false);
            };
            app.hasReceivedEnemyHealth_ = true;
            update_health(UnitType::Hero, static_cast<std::uint16_t>(msg->hero));
            update_health(UnitType::Engineer, static_cast<std::uint16_t>(msg->engineer));
            update_health(UnitType::Infantry1, static_cast<std::uint16_t>(msg->infantry1));
            update_health(UnitType::Infantry2, static_cast<std::uint16_t>(msg->infantry2));
            update_health(UnitType::Sentry, static_cast<std::uint16_t>(msg->sentry));
        });

        // ly_friend_hp
        GenSub<ly_friend_hp>([](Application& app, auto msg) {
            app.friendRobots[UnitType::Hero].setCurrentHealth(static_cast<std::uint16_t>(msg->hero));
            app.friendRobots[UnitType::Engineer].setCurrentHealth(static_cast<std::uint16_t>(msg->engineer));
            app.friendRobots[UnitType::Infantry1].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry1));
            app.friendRobots[UnitType::Infantry2].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry2));
            app.friendRobots[UnitType::Sentry].setCurrentHealth(static_cast<std::uint16_t>(msg->sentry));
        });
    }    

}
