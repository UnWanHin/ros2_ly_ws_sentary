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

    official_map_x = LaunchConfiguration("official_map_x")
    official_map_y = LaunchConfiguration("official_map_y")
    map_z = LaunchConfiguration("map_z")
    target_frame = LaunchConfiguration("target_frame")
    raw_goal_target_frame = LaunchConfiguration("raw_goal_target_frame")
    aim_frame = LaunchConfiguration("aim_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    solve_mode = LaunchConfiguration("solve_mode")
    solve_frame = LaunchConfiguration("solve_frame")
    output = LaunchConfiguration("output")
    use_mock_map_to_base = LaunchConfiguration("use_mock_map_to_base")
    use_mock_gimbal_state = LaunchConfiguration("use_mock_gimbal_state")

    return LaunchDescription([
        DeclareLaunchArgument("official_map_x"),
        DeclareLaunchArgument("official_map_y"),
        DeclareLaunchArgument("map_z"),
        DeclareLaunchArgument("target_frame", default_value="official_map"),
        DeclareLaunchArgument("aim_frame", default_value="gimbal_world"),
        DeclareLaunchArgument("camera_frame", default_value="gx_camera"),
        DeclareLaunchArgument("solve_mode", default_value="camera_projection"),
        DeclareLaunchArgument("solve_frame", default_value="base_link"),
        DeclareLaunchArgument("gimbal_angles_topic", default_value="/ly/gimbal/angles"),
        DeclareLaunchArgument("gimbal_big_yaw_topic", default_value="/ly/gimbal/big_yaw_angles"),
        DeclareLaunchArgument("control_angles_topic", default_value="/ly/control/angles"),
        DeclareLaunchArgument("control_firecode_topic", default_value="/ly/control/firecode"),
        DeclareLaunchArgument("face_target_topic", default_value="/ly/face_mode/target_raw"),
        DeclareLaunchArgument("publish_firecode", default_value="true"),
        DeclareLaunchArgument("aim_mode", default_value="true"),
        DeclareLaunchArgument("bridge_config_file", default_value=default_bridge_config),
        DeclareLaunchArgument("use_raw_goal_static_calibration", default_value="true"),
        DeclareLaunchArgument("raw_goal_target_frame", default_value="map"),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="0.05"),
        DeclareLaunchArgument("use_gimbal_stamp_for_tf", default_value="false"),
        DeclareLaunchArgument("max_gimbal_stamp_age_sec", default_value="0.50"),
        DeclareLaunchArgument("min_distance_m", default_value="0.10"),
        DeclareLaunchArgument("max_target_distance_m", default_value="100.0"),
        DeclareLaunchArgument("command_filter_alpha", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="-1.0"),
        DeclareLaunchArgument("pitch_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_bias_deg", default_value="0.0"),
        DeclareLaunchArgument("pitch_bias_deg", default_value="0.0"),
        DeclareLaunchArgument("max_yaw_step_deg", default_value="0.0"),
        DeclareLaunchArgument("max_pitch_step_deg", default_value="0.0"),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_tf_tree", default_value="false"),
        DeclareLaunchArgument("gimbal_config_file", default_value=default_gimbal_config),
        DeclareLaunchArgument("use_virtual_device", default_value="false"),
        DeclareLaunchArgument("tf_tree_params_file", default_value=default_tf_tree_params),
        DeclareLaunchArgument("use_mock_map_to_base", default_value="false"),
        DeclareLaunchArgument("mock_map_to_base_x", default_value="0.0"),
        DeclareLaunchArgument("mock_map_to_base_y", default_value="0.0"),
        DeclareLaunchArgument("mock_map_to_base_z", default_value="0.0"),
        DeclareLaunchArgument("mock_map_to_base_yaw", default_value="0.0"),
        DeclareLaunchArgument("mock_map_to_base_pitch", default_value="0.0"),
        DeclareLaunchArgument("mock_map_to_base_roll", default_value="0.0"),
        DeclareLaunchArgument("mock_map_frame", default_value="map"),
        DeclareLaunchArgument("mock_base_frame", default_value="base_link"),
        DeclareLaunchArgument("use_mock_gimbal_state", default_value="false"),
        DeclareLaunchArgument("mock_gimbal_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("mock_gimbal_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("mock_gimbal_big_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("mock_gimbal_publish_big_yaw", default_value="true"),
        DeclareLaunchArgument("mock_gimbal_publish_hz", default_value="30.0"),
        DeclareLaunchArgument("output", default_value="screen"),
        LogInfo(msg=[
            "[FaceMode] target=(",
            official_map_x,
            ", ",
            official_map_y,
            ", ",
            map_z,
            ")cm@",
            target_frame,
            " raw_goal_target_frame=",
            raw_goal_target_frame,
            " aim_frame=",
            aim_frame,
            " camera_frame=",
            camera_frame,
            " solve_mode=",
            solve_mode,
            " solve_frame=",
            solve_frame,
        ]),
        LogInfo(
            msg=[
                "[map_aim_point] using test-only static TF ",
                LaunchConfiguration("mock_map_frame"),
                " -> ",
                LaunchConfiguration("mock_base_frame"),
                " from mock_map_to_base_* launch args",
            ],
            condition=IfCondition(use_mock_map_to_base),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mock_map_to_base_tf",
            output=output,
            arguments=[
                LaunchConfiguration("mock_map_to_base_x"),
                LaunchConfiguration("mock_map_to_base_y"),
                LaunchConfiguration("mock_map_to_base_z"),
                LaunchConfiguration("mock_map_to_base_yaw"),
                LaunchConfiguration("mock_map_to_base_pitch"),
                LaunchConfiguration("mock_map_to_base_roll"),
                LaunchConfiguration("mock_map_frame"),
                LaunchConfiguration("mock_base_frame"),
            ],
            condition=IfCondition(use_mock_map_to_base),
        ),
        Node(
            package="navi_tf_bridge",
            executable="mock_gimbal_state_node",
            name="mock_gimbal_state_node",
            output=output,
            parameters=[{
                "gimbal_angles_topic": ParameterValue(
                    LaunchConfiguration("gimbal_angles_topic"), value_type=str),
                "gimbal_big_yaw_topic": ParameterValue(
                    LaunchConfiguration("gimbal_big_yaw_topic"), value_type=str),
                "yaw_deg": ParameterValue(
                    LaunchConfiguration("mock_gimbal_yaw_deg"), value_type=float),
                "pitch_deg": ParameterValue(
                    LaunchConfiguration("mock_gimbal_pitch_deg"), value_type=float),
                "big_yaw_deg": ParameterValue(
                    LaunchConfiguration("mock_gimbal_big_yaw_deg"), value_type=float),
                "publish_big_yaw": ParameterValue(
                    LaunchConfiguration("mock_gimbal_publish_big_yaw"), value_type=bool),
                "publish_hz": ParameterValue(
                    LaunchConfiguration("mock_gimbal_publish_hz"), value_type=float),
            }],
            condition=IfCondition(use_mock_gimbal_state),
        ),
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
                "official_map_x": ParameterValue(official_map_x, value_type=float),
                "official_map_y": ParameterValue(official_map_y, value_type=float),
                "map_z": ParameterValue(map_z, value_type=float),
                "target_frame": ParameterValue(target_frame, value_type=str),
                "aim_frame": ParameterValue(aim_frame, value_type=str),
                "camera_frame": ParameterValue(camera_frame, value_type=str),
                "solve_mode": ParameterValue(solve_mode, value_type=str),
                "solve_frame": ParameterValue(solve_frame, value_type=str),
                "gimbal_angles_topic": ParameterValue(
                    LaunchConfiguration("gimbal_angles_topic"), value_type=str),
                "control_angles_topic": ParameterValue(
                    LaunchConfiguration("control_angles_topic"), value_type=str),
                "control_firecode_topic": ParameterValue(
                    LaunchConfiguration("control_firecode_topic"), value_type=str),
                "face_target_topic": ParameterValue(
                    LaunchConfiguration("face_target_topic"), value_type=str),
                "publish_firecode": ParameterValue(
                    LaunchConfiguration("publish_firecode"), value_type=bool),
                "aim_mode": ParameterValue(LaunchConfiguration("aim_mode"), value_type=bool),
                "bridge_config_file": ParameterValue(
                    LaunchConfiguration("bridge_config_file"), value_type=str),
                "use_raw_goal_static_calibration": ParameterValue(
                    LaunchConfiguration("use_raw_goal_static_calibration"), value_type=bool),
                "raw_goal_target_frame": ParameterValue(
                    LaunchConfiguration("raw_goal_target_frame"), value_type=str),
                "publish_hz": ParameterValue(LaunchConfiguration("publish_hz"), value_type=float),
                "tf_timeout_sec": ParameterValue(
                    LaunchConfiguration("tf_timeout_sec"), value_type=float),
                "use_gimbal_stamp_for_tf": ParameterValue(
                    LaunchConfiguration("use_gimbal_stamp_for_tf"), value_type=bool),
                "max_gimbal_stamp_age_sec": ParameterValue(
                    LaunchConfiguration("max_gimbal_stamp_age_sec"), value_type=float),
                "min_distance_m": ParameterValue(
                    LaunchConfiguration("min_distance_m"), value_type=float),
                "max_target_distance_m": ParameterValue(
                    LaunchConfiguration("max_target_distance_m"), value_type=float),
                "command_filter_alpha": ParameterValue(
                    LaunchConfiguration("command_filter_alpha"), value_type=float),
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
