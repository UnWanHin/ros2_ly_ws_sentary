#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bridge_share = get_package_share_directory("navi_tf_bridge")
    bridge_launch = os.path.join(bridge_share, "launch", "target_rel_to_goal_pos.launch.py")

    launch_args = [
        DeclareLaunchArgument("input_goal_pos_raw_topic", default_value="/ly/navi/goal_pos_raw"),
        DeclareLaunchArgument("output_goal_pos_topic", default_value="/ly/navi/goal_pos"),
        DeclareLaunchArgument("output_goal_pose_topic", default_value="/goal_pose"),
        DeclareLaunchArgument("goal_pos_raw_frame", default_value="map"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("fallback_base_frame", default_value="baselink"),
        DeclareLaunchArgument("publish_goal_pos", default_value="false"),
        DeclareLaunchArgument("publish_goal_pose", default_value="true"),
        DeclareLaunchArgument("goal_pose_uniform_scale", default_value="1.0"),
        DeclareLaunchArgument("goal_pos_uint16_encode_enabled", default_value="false"),
        DeclareLaunchArgument("goal_pos_uint16_encode_x_scale", default_value="1.0"),
        DeclareLaunchArgument("goal_pos_uint16_encode_y_scale", default_value="1.0"),
        DeclareLaunchArgument("goal_pos_uint16_encode_x_offset_cm", default_value="0.0"),
        DeclareLaunchArgument("goal_pos_uint16_encode_y_offset_cm", default_value="0.0"),
        DeclareLaunchArgument("enable_goal_pos_raw_bridge", default_value="true"),
        DeclareLaunchArgument("use_raw_goal_static_calibration", default_value="false"),
        DeclareLaunchArgument("debug_export_point_pairs", default_value="false"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            "input_goal_pos_raw_topic": LaunchConfiguration("input_goal_pos_raw_topic"),
            "output_goal_pos_topic": LaunchConfiguration("output_goal_pos_topic"),
            "output_goal_pose_topic": LaunchConfiguration("output_goal_pose_topic"),
            "goal_pos_raw_frame": LaunchConfiguration("goal_pos_raw_frame"),
            "map_frame": LaunchConfiguration("map_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "fallback_base_frame": LaunchConfiguration("fallback_base_frame"),
            "publish_goal_pos": LaunchConfiguration("publish_goal_pos"),
            "publish_goal_pose": LaunchConfiguration("publish_goal_pose"),
            "goal_pose_uniform_scale": LaunchConfiguration("goal_pose_uniform_scale"),
            "goal_pos_uint16_encode_enabled": LaunchConfiguration(
                "goal_pos_uint16_encode_enabled"
            ),
            "goal_pos_uint16_encode_x_scale": LaunchConfiguration(
                "goal_pos_uint16_encode_x_scale"
            ),
            "goal_pos_uint16_encode_y_scale": LaunchConfiguration(
                "goal_pos_uint16_encode_y_scale"
            ),
            "goal_pos_uint16_encode_x_offset_cm": LaunchConfiguration(
                "goal_pos_uint16_encode_x_offset_cm"
            ),
            "goal_pos_uint16_encode_y_offset_cm": LaunchConfiguration(
                "goal_pos_uint16_encode_y_offset_cm"
            ),
            "enable_goal_pos_raw_bridge": LaunchConfiguration("enable_goal_pos_raw_bridge"),
            "use_raw_goal_static_calibration": LaunchConfiguration(
                "use_raw_goal_static_calibration"
            ),
            "publish_target_map": "false",
            "debug_export_point_pairs": LaunchConfiguration("debug_export_point_pairs"),
        }.items(),
    )

    return LaunchDescription(launch_args + [bridge])
