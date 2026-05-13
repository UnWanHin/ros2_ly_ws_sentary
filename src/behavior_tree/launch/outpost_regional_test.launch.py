#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    behavior_tree_share = get_package_share_directory("behavior_tree")
    sentry_all_launch = os.path.join(behavior_tree_share, "launch", "sentry_all.launch.py")
    config_root = os.path.join(behavior_tree_share, "config")

    default_base_config_file = os.path.join(config_root, "base_config.yaml")
    default_override_config_file = os.path.join(config_root, "override_config.yaml")
    default_area_manager_config_file = os.path.join(config_root, "AreaManager.yaml")
    default_base_strategy_config_file = os.path.join(config_root, "Base.yaml")
    default_task_config_file = os.path.join(config_root, "OutpostRegionalTest.yaml")
    default_navi_rotate_config_file = os.path.join(config_root, "NaviRotateControl.yaml")
    default_point_manager_config_file = os.path.join(config_root, "PointManager.yaml")
    default_special_config_file = os.path.join(config_root, "Special.yaml")

    launch_args = [
        DeclareLaunchArgument("mode", default_value="regional"),
        DeclareLaunchArgument("config_file", default_value=default_override_config_file),
        DeclareLaunchArgument("base_config_file", default_value=default_base_config_file),
        DeclareLaunchArgument("area_manager_config_file", default_value=default_area_manager_config_file),
        DeclareLaunchArgument("base_strategy_config_file", default_value=default_base_strategy_config_file),
        DeclareLaunchArgument("task_config_file", default_value=default_task_config_file),
        DeclareLaunchArgument("navi_rotate_config_file", default_value=default_navi_rotate_config_file),
        DeclareLaunchArgument("point_manager_config_file", default_value=default_point_manager_config_file),
        DeclareLaunchArgument("special_config_file", default_value=default_special_config_file),
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("offline", default_value="false"),
        DeclareLaunchArgument("debug_bypass_is_start", default_value="true"),
        DeclareLaunchArgument("runtime_rearm_start_gate", default_value="false"),
        DeclareLaunchArgument("publish_navi_goal", default_value="true"),
        DeclareLaunchArgument("navi_publish_goal_pose", default_value="true"),
        DeclareLaunchArgument("wait_for_game_start_timeout_sec", default_value="0"),
        DeclareLaunchArgument("league_referee_stale_timeout_ms", default_value="0"),
        DeclareLaunchArgument("decision_trace_enabled", default_value="false"),
        DeclareLaunchArgument("decision_trace_file", default_value=""),
        DeclareLaunchArgument("decision_trace_every_n_ticks", default_value="5"),
        DeclareLaunchArgument("bt_tree_file", default_value=""),
        DeclareLaunchArgument(
            "bt_config_file",
            default_value="Scripts/ConfigJson/regional/debug/outpost_regional_test.json",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument("use_face_mode_solver", default_value="true"),
        DeclareLaunchArgument("use_navi_tf_bridge", default_value=""),
        DeclareLaunchArgument("use_tf_tree", default_value="false"),
        DeclareLaunchArgument("face_mode_target_frame", default_value="official_map"),
        DeclareLaunchArgument("face_mode_use_raw_goal_static_calibration", default_value="true"),
        DeclareLaunchArgument("face_mode_raw_goal_target_frame", default_value="map"),
        DeclareLaunchArgument("face_mode_outpost_manual_target_enable", default_value="false"),
        DeclareLaunchArgument("face_mode_outpost_manual_target_map_x_cm", default_value="0"),
        DeclareLaunchArgument("face_mode_outpost_manual_target_map_y_cm", default_value="0"),
        DeclareLaunchArgument("face_mode_outpost_manual_target_map_z_cm", default_value="0"),
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sentry_all_launch),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
            "config_file": LaunchConfiguration("config_file"),
            "base_config_file": LaunchConfiguration("base_config_file"),
            "area_manager_config_file": LaunchConfiguration("area_manager_config_file"),
            "base_strategy_config_file": LaunchConfiguration("base_strategy_config_file"),
            "task_config_file": LaunchConfiguration("task_config_file"),
            "navi_rotate_config_file": LaunchConfiguration("navi_rotate_config_file"),
            "point_manager_config_file": LaunchConfiguration("point_manager_config_file"),
            "special_config_file": LaunchConfiguration("special_config_file"),
            "output": LaunchConfiguration("output"),
            "offline": LaunchConfiguration("offline"),
            "debug_bypass_is_start": LaunchConfiguration("debug_bypass_is_start"),
            "runtime_rearm_start_gate": LaunchConfiguration("runtime_rearm_start_gate"),
            "publish_navi_goal": LaunchConfiguration("publish_navi_goal"),
            "navi_publish_goal_pose": LaunchConfiguration("navi_publish_goal_pose"),
            "wait_for_game_start_timeout_sec": LaunchConfiguration("wait_for_game_start_timeout_sec"),
            "league_referee_stale_timeout_ms": LaunchConfiguration("league_referee_stale_timeout_ms"),
            "decision_trace_enabled": LaunchConfiguration("decision_trace_enabled"),
            "decision_trace_file": LaunchConfiguration("decision_trace_file"),
            "decision_trace_every_n_ticks": LaunchConfiguration("decision_trace_every_n_ticks"),
            "bt_tree_file": LaunchConfiguration("bt_tree_file"),
            "bt_config_file": LaunchConfiguration("bt_config_file"),
            "use_gimbal": LaunchConfiguration("use_gimbal"),
            "use_behavior_tree": LaunchConfiguration("use_behavior_tree"),
            "use_face_mode_solver": LaunchConfiguration("use_face_mode_solver"),
            "use_navi_tf_bridge": LaunchConfiguration("use_navi_tf_bridge"),
            "use_tf_tree": LaunchConfiguration("use_tf_tree"),
            "face_mode_target_frame": LaunchConfiguration("face_mode_target_frame"),
            "face_mode_use_raw_goal_static_calibration": LaunchConfiguration(
                "face_mode_use_raw_goal_static_calibration"
            ),
            "face_mode_raw_goal_target_frame": LaunchConfiguration("face_mode_raw_goal_target_frame"),
            "face_mode_outpost_manual_target_enable": LaunchConfiguration(
                "face_mode_outpost_manual_target_enable"
            ),
            "face_mode_outpost_manual_target_map_x_cm": LaunchConfiguration(
                "face_mode_outpost_manual_target_map_x_cm"
            ),
            "face_mode_outpost_manual_target_map_y_cm": LaunchConfiguration(
                "face_mode_outpost_manual_target_map_y_cm"
            ),
            "face_mode_outpost_manual_target_map_z_cm": LaunchConfiguration(
                "face_mode_outpost_manual_target_map_z_cm"
            ),
        }.items(),
    )

    return LaunchDescription(launch_args + [include])
