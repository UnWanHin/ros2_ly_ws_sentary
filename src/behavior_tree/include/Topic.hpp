// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
/*
 * behavior_tree 统一话题定义
 *
 * 作用：
 * - 以 LY_DEF_ROS_TOPIC 统一 topic 名称和消息类型
 * - 作为 behavior_tree 与其他模块的通信契约中心
 *
 * 维护建议：
 * - 新增或变更话题时，同时更新 docs/architecture/2026-05-03_message_and_link_flow.md
 */

// #include <ros/ros.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/UInt32.h>
// #include <std_msgs/UInt8.h>
// #include <std_msgs/UInt16MultiArray.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


// #include "auto_aim_common/Armor.h"
// #include "auto_aim_common/Armors.h"
// #include "auto_aim_common/Target.h"
// #include "gimbal_driver/GameData.h"
// #include "gimbal_driver/GimbalAngles.h"
// #include "gimbal_driver/Health.h"
// #include "gimbal_driver/UWBPos.h"
// #include "gimbal_driver/Vel.h"
// #include "gimbal_driver/BuffData.h"
// #include "gimbal_driver/PositionData.h"

#include "auto_aim_common/msg/armor.hpp"
#include "auto_aim_common/msg/armors.hpp"
#include "auto_aim_common/msg/target.hpp"
#include "auto_aim_common/msg/relative_target.hpp"
#include "auto_aim_common/msg/goal_reach.hpp"
#include "gimbal_driver/msg/game_data.hpp"
#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "gimbal_driver/msg/chassis.hpp"
#include "gimbal_driver/msg/control_velocity.hpp"
#include "gimbal_driver/msg/event_data.hpp"
#include "gimbal_driver/msg/fire_code.hpp"
#include "gimbal_driver/msg/health.hpp"
#include "gimbal_driver/msg/map_command.hpp"
#include "gimbal_driver/msg/rfid_status.hpp"
#include "gimbal_driver/msg/sentry_cmd.hpp"
#include "gimbal_driver/msg/sentry_info.hpp"
#include "gimbal_driver/msg/stamped_u_int16_multi_array.hpp"
#include "gimbal_driver/msg/uwb_pos.hpp"
#include "gimbal_driver/msg/vel.hpp"
#include "gimbal_driver/msg/buff_data.hpp"
#include "gimbal_driver/msg/bullet_info.hpp"
#include "gimbal_driver/msg/position_data.hpp"
#include "gimbal_driver/msg/unit_info_array.hpp"

#ifdef LY_ENABLE_SENTRY_MSGS
#include "sentry_msgs/msg/aim_result.hpp"
#include "sentry_msgs/msg/aim_target.hpp"
#include "sentry_msgs/msg/aim_target_array.hpp"
#endif

#include "../module/ROSTools.hpp"


namespace BehaviorTree { 
    LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", gimbal_driver::msg::FireCode);
    LY_DEF_ROS_TOPIC(ly_control_vel, "/ly/control/vel", gimbal_driver::msg::ControlVelocity);
    LY_DEF_ROS_TOPIC(ly_control_posture, "/ly/control/posture", gimbal_driver::msg::SentryCmd);
    LY_DEF_ROS_TOPIC(ly_control_sentry_cmd, "/ly/control/sentry_cmd", gimbal_driver::msg::SentryCmd);
    LY_DEF_ROS_TOPIC(ly_game_all, "/ly/game/all", gimbal_driver::msg::GameData);
    
    LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", gimbal_driver::msg::FireCode);
    LY_DEF_ROS_TOPIC(ly_gimbal_vel, "/ly/gimbal/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_gimbal_chassis, "/ly/gimbal/chassis", gimbal_driver::msg::Chassis);
    LY_DEF_ROS_TOPIC(ly_gimbal_posture, "/ly/gimbal/posture", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_capV, "/ly/gimbal/capV", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_game_event_data, "/ly/game/event_data", gimbal_driver::msg::EventData);
    LY_DEF_ROS_TOPIC(ly_game_sentry_info, "/ly/game/sentry/info", gimbal_driver::msg::SentryInfo);
    LY_DEF_ROS_TOPIC(ly_game_map_command, "/ly/game/map_command", gimbal_driver::msg::MapCommand);

    LY_DEF_ROS_TOPIC(ly_friend_is_precaution, "/ly/friend/is_precaution", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_is_at_home, "/ly/friend/is_at_home", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_is_team_red, "/ly/friend/is_team_red", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_hp, "/ly/friend/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_friend_op_hp, "/ly/friend/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_friend_base_hp, "/ly/friend/base_hp", std_msgs::msg::UInt16);
    
    LY_DEF_ROS_TOPIC(ly_friend_ammo_left, "/ly/friend/ammo_left", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_friend_uwb_pos, "/ly/friend/uwb_pos", gimbal_driver::msg::StampedUInt16MultiArray);
    
    LY_DEF_ROS_TOPIC(ly_game_is_start, "/ly/game/is_start", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_game_time_left, "/ly/game/time_left", std_msgs::msg::UInt16);
    
    LY_DEF_ROS_TOPIC(ly_enemy_hp, "/ly/enemy/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_enemy_op_hp, "/ly/enemy/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_enemy_base_hp, "/ly/enemy/base_hp", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_vision_mode, "/ly/vision/mode", std_msgs::msg::UInt8);
    
    LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_bt_sentry_position, "/ly/bt/sentry_position", geometry_msgs::msg::PointStamped);
    LY_DEF_ROS_TOPIC(ly_face_mode_angles, "/ly/face_mode/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_face_mode_target_raw, "/ly/face_mode/target_raw", std_msgs::msg::UInt16MultiArray);

#ifdef LY_ENABLE_SENTRY_MSGS
    LY_DEF_ROS_TOPIC(ly_aim_armor_targets, "/ly/aim/armor_targets", sentry_msgs::msg::AimTargetArray);
    LY_DEF_ROS_TOPIC(ly_aim_select_target, "/ly/aim/select_target", sentry_msgs::msg::AimTarget);
    LY_DEF_ROS_TOPIC(ly_aim_result, "/ly/aim/result", sentry_msgs::msg::AimResult);
#endif
    
    LY_DEF_ROS_TOPIC(ly_navi_vel, "/ly/navi/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_navi_target_rel, "/ly/navi/target_rel", auto_aim_common::msg::RelativeTarget);
    LY_DEF_ROS_TOPIC(ly_navi_goal, "/ly/navi/goal", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_navi_goal_pos_raw, "/ly/navi/goal_pos_raw", std_msgs::msg::UInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_navi_goal_pos, "/ly/navi/goal_pos", std_msgs::msg::UInt16MultiArray);
    LY_DEF_ROS_TOPIC(navi_goal_pose, "/goal_pose", geometry_msgs::msg::PoseStamped);
    LY_DEF_ROS_TOPIC(ly_navi_position, "/ly/navi/position", gimbal_driver::msg::StampedUInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_navi_target_official, "/ly/navi/target_official", gimbal_driver::msg::StampedUInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_navi_speed_level, "/ly/navi/speed_level", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_navi_lower_head, "/ly/navi/lower_head", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_navi_reached, "/ly/navi/reached", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_navi_reachable, "/ly/navi/reachable", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_navi_reach_state, "/ly/navi/reach_state", auto_aim_common::msg::GoalReach);
    LY_DEF_ROS_TOPIC(ly_navi_should_rotate, "/ly/navi/should_rotate", std_msgs::msg::Bool);

    LY_DEF_ROS_TOPIC(ly_team_buff, "/ly/team/buff", gimbal_driver::msg::BuffData);
    LY_DEF_ROS_TOPIC(ly_game_rfid, "/ly/game/rfid", gimbal_driver::msg::RfidStatus);
    LY_DEF_ROS_TOPIC(ly_game_bullet, "/ly/game/bullet", gimbal_driver::msg::BulletInfo);
    LY_DEF_ROS_TOPIC(ly_position_data, "/ly/position/data", gimbal_driver::msg::PositionData);
    LY_DEF_ROS_TOPIC(ly_friend_info, "/ly/friend/info", gimbal_driver::msg::UnitInfoArray);
    LY_DEF_ROS_TOPIC(ly_enemy_info, "/ly/enemy/info", gimbal_driver::msg::UnitInfoArray);

}
