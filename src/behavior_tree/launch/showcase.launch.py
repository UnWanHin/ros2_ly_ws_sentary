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
    default_aim_config_file = os.path.join(config_root, "Aim.yaml")

    launch_args = [
        DeclareLaunchArgument("config_file", default_value=default_override_config_file),
        DeclareLaunchArgument("base_config_file", default_value=default_base_config_file),
        DeclareLaunchArgument("aim_config_file", default_value=default_aim_config_file),
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("offline", default_value="false"),
        DeclareLaunchArgument("debug_bypass_is_start", default_value="true"),
        DeclareLaunchArgument("wait_for_game_start_timeout_sec", default_value="0"),
        DeclareLaunchArgument("league_referee_stale_timeout_ms", default_value="0"),
        DeclareLaunchArgument("decision_trace_enabled", default_value="false"),
        DeclareLaunchArgument("decision_trace_file", default_value=""),
        DeclareLaunchArgument("decision_trace_every_n_ticks", default_value="5"),
        DeclareLaunchArgument("rosbag", default_value="false"),
        DeclareLaunchArgument("rosbag_path", default_value="~/Log/rosbag"),
        DeclareLaunchArgument("bt_tree_file", default_value=""),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sentry_all_launch),
        launch_arguments={
            "mode": "showcase",
            "config_file": LaunchConfiguration("config_file"),
            "base_config_file": LaunchConfiguration("base_config_file"),
            "aim_config_file": LaunchConfiguration("aim_config_file"),
            "output": LaunchConfiguration("output"),
            "offline": LaunchConfiguration("offline"),
            "debug_bypass_is_start": LaunchConfiguration("debug_bypass_is_start"),
            "wait_for_game_start_timeout_sec": LaunchConfiguration("wait_for_game_start_timeout_sec"),
            "league_referee_stale_timeout_ms": LaunchConfiguration("league_referee_stale_timeout_ms"),
            "decision_trace_enabled": LaunchConfiguration("decision_trace_enabled"),
            "decision_trace_file": LaunchConfiguration("decision_trace_file"),
            "decision_trace_every_n_ticks": LaunchConfiguration("decision_trace_every_n_ticks"),
            "rosbag": LaunchConfiguration("rosbag"),
            "rosbag_path": LaunchConfiguration("rosbag_path"),
            "bt_tree_file": LaunchConfiguration("bt_tree_file"),
            "use_gimbal": LaunchConfiguration("use_gimbal"),
            "use_behavior_tree": LaunchConfiguration("use_behavior_tree"),
        }.items(),
    )

    return LaunchDescription(launch_args + [include])
