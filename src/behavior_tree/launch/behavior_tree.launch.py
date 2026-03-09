#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output = LaunchConfiguration("output")

    launch_args = [
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
    ]

    info_logs = [
        LogInfo(msg=["[behavior_tree] output: ", output]),
    ]

    nodes = [
        Node(
            package="behavior_tree",
            executable="behavior_tree_node",
            name="behavior_tree",
            output=output,
            on_exit=Shutdown(reason="behavior_tree exited"),
        )
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
