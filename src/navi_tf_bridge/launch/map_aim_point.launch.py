#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_share = get_package_share_directory("navi_tf_bridge")
    behavior_tree_share = get_package_share_directory("behavior_tree")
    gimbal_driver_share = get_package_share_directory("gimbal_driver")
    tf_tree_share = get_package_share_directory("tf_tree")

    default_bridge_config = os.path.join(bridge_share, "config", "tf_config.yaml")
    default_gimbal_config = os.path.join(behavior_tree_share, "config", "base_config.yaml")
    default_tf_tree_params = os.path.join(tf_tree_share, "config", "tf_tree.yaml")
    gimbal_launch = os.path.join(gimbal_driver_share, "launch", "gimbal_driver.launch.py")
    tf_tree_launch = os.path.join(tf_tree_share, "launch", "tf_tree.launch.py")

    target_x_cm = LaunchConfiguration("target_x_cm")
    target_y_cm = LaunchConfiguration("target_y_cm")
    target_z_cm = LaunchConfiguration("target_z_cm")
    target_frame = LaunchConfiguration("target_frame")
    raw_goal_target_frame = LaunchConfiguration("raw_goal_target_frame")
    aim_frame = LaunchConfiguration("aim_frame")
    output = LaunchConfiguration("output")

    return LaunchDescription([
        DeclareLaunchArgument("target_x_cm", default_value="1400.0"),
        DeclareLaunchArgument("target_y_cm", default_value="750.0"),
        DeclareLaunchArgument("target_z_cm", default_value="100.0"),
        DeclareLaunchArgument("target_frame", default_value="official_map"),
        DeclareLaunchArgument("aim_frame", default_value="gimbal_world"),
        DeclareLaunchArgument("gimbal_angles_topic", default_value="/ly/gimbal/angles"),
        DeclareLaunchArgument("control_angles_topic", default_value="/ly/control/angles"),
        DeclareLaunchArgument("bridge_config_file", default_value=default_bridge_config),
        DeclareLaunchArgument("use_raw_goal_static_calibration", default_value="true"),
        DeclareLaunchArgument("raw_goal_target_frame", default_value="odom"),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="0.05"),
        DeclareLaunchArgument("min_distance_m", default_value="0.10"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("pitch_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_bias_deg", default_value="0.0"),
        DeclareLaunchArgument("pitch_bias_deg", default_value="0.0"),
        DeclareLaunchArgument("max_yaw_step_deg", default_value="0.0"),
        DeclareLaunchArgument("max_pitch_step_deg", default_value="0.0"),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_tf_tree", default_value="true"),
        DeclareLaunchArgument("gimbal_config_file", default_value=default_gimbal_config),
        DeclareLaunchArgument("use_virtual_device", default_value="false"),
        DeclareLaunchArgument("tf_tree_params_file", default_value=default_tf_tree_params),
        DeclareLaunchArgument("output", default_value="screen"),
        LogInfo(msg=[
            "[map_aim_point] target=(",
            target_x_cm,
            ", ",
            target_y_cm,
            ", ",
            target_z_cm,
            ")cm@",
            target_frame,
            " raw_goal_target_frame=",
            raw_goal_target_frame,
            " aim_frame=",
            aim_frame,
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gimbal_launch),
            launch_arguments={
                "config_file": LaunchConfiguration("gimbal_config_file"),
                "use_virtual_device": LaunchConfiguration("use_virtual_device"),
                "output": output,
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gimbal")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_tree_launch),
            launch_arguments={
                "params_file": LaunchConfiguration("tf_tree_params_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_tf_tree")),
        ),
        Node(
            package="navi_tf_bridge",
            executable="map_aim_point_node",
            name="map_aim_point_node",
            output=output,
            parameters=[{
                "target_x_cm": ParameterValue(target_x_cm, value_type=float),
                "target_y_cm": ParameterValue(target_y_cm, value_type=float),
                "target_z_cm": ParameterValue(target_z_cm, value_type=float),
                "target_frame": ParameterValue(target_frame, value_type=str),
                "aim_frame": ParameterValue(aim_frame, value_type=str),
                "gimbal_angles_topic": ParameterValue(
                    LaunchConfiguration("gimbal_angles_topic"), value_type=str),
                "control_angles_topic": ParameterValue(
                    LaunchConfiguration("control_angles_topic"), value_type=str),
                "bridge_config_file": ParameterValue(
                    LaunchConfiguration("bridge_config_file"), value_type=str),
                "use_raw_goal_static_calibration": ParameterValue(
                    LaunchConfiguration("use_raw_goal_static_calibration"), value_type=bool),
                "raw_goal_target_frame": ParameterValue(
                    LaunchConfiguration("raw_goal_target_frame"), value_type=str),
                "publish_hz": ParameterValue(LaunchConfiguration("publish_hz"), value_type=float),
                "tf_timeout_sec": ParameterValue(
                    LaunchConfiguration("tf_timeout_sec"), value_type=float),
                "min_distance_m": ParameterValue(
                    LaunchConfiguration("min_distance_m"), value_type=float),
                "yaw_sign": ParameterValue(LaunchConfiguration("yaw_sign"), value_type=float),
                "pitch_sign": ParameterValue(LaunchConfiguration("pitch_sign"), value_type=float),
                "yaw_bias_deg": ParameterValue(
                    LaunchConfiguration("yaw_bias_deg"), value_type=float),
                "pitch_bias_deg": ParameterValue(
                    LaunchConfiguration("pitch_bias_deg"), value_type=float),
                "max_yaw_step_deg": ParameterValue(
                    LaunchConfiguration("max_yaw_step_deg"), value_type=float),
                "max_pitch_step_deg": ParameterValue(
                    LaunchConfiguration("max_pitch_step_deg"), value_type=float),
            }],
        ),
    ])
