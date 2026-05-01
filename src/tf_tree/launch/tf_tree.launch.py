#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# Standalone launch for tf_tree package.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)

    with open(params_file, "r") as f:
        cfg = yaml.safe_load(f)

    # Keep backward compatibility with old key `sentry_tf_node`.
    tf_node_cfg = cfg.get("tf_tree_node", {})
    if not tf_node_cfg:
        tf_node_cfg = cfg.get("sentry_tf_node", {})
    tf_params = tf_node_cfg.get("ros__parameters", {})
    cam = cfg.get("barrel_to_camera", {})
    big_cam = cfg.get("big_yaw_to_usb_camera", {})
    if not tf_params:
        raise RuntimeError(
            "Missing tf_tree_node.ros__parameters in params_file. "
            "Compatible key sentry_tf_node.ros__parameters is also accepted."
        )
    required_cam_keys = ("x", "y", "z", "yaw", "pitch", "roll", "parent_frame", "child_frame")
    for key in required_cam_keys:
        if key not in cam:
            raise RuntimeError(
                f"Missing barrel_to_camera.{key} in params_file: {params_file}"
            )
    if "big_yaw_frame" not in tf_params:
        raise RuntimeError(
            f"Missing tf_tree_node.ros__parameters.big_yaw_frame in params_file: {params_file}"
        )

    nodes = [
        Node(
            package="tf_tree",
            executable="tf_node",
            name="tf_tree_node",
            output="screen",
            parameters=[tf_params],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="barrel_to_camera_tf",
            output="screen",
            arguments=[
                str(cam["x"]),
                str(cam["y"]),
                str(cam["z"]),
                str(cam["yaw"]),
                str(cam["pitch"]),
                str(cam["roll"]),
                cam["parent_frame"],
                cam["child_frame"],
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="baselink_to_gimbal_big_yaw_tf",
            output="screen",
            arguments=[
                "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
                "base_link",
                tf_params["big_yaw_frame"],
            ],
        ),
    ]
    if big_cam:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="big_yaw_to_usb_camera_tf",
                output="screen",
                arguments=[
                    str(big_cam["x"]),
                    str(big_cam["y"]),
                    str(big_cam["z"]),
                    str(big_cam["yaw"]),
                    str(big_cam["pitch"]),
                    str(big_cam["roll"]),
                    big_cam["parent_frame"],
                    big_cam["child_frame"],
                ],
            )
        )
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("tf_tree")
    default_params = os.path.join(pkg_share, "config", "tf_tree.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS2 参数文件路径（YAML）",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
