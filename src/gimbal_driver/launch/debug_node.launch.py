#!/usr/bin/env python3

"""Single-node gimbal_driver debug entry point.

This launch composes the formal hardware baseline with one explicit driver debug
profile. It starts no behavior-tree node, so it must not run beside a BT that
publishes the formal /ly/control/* chain.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gimbal_driver_share = get_package_share_directory("gimbal_driver")
    gimbal_driver_launch = os.path.join(
        gimbal_driver_share, "launch", "gimbal_driver.launch.py"
    )
    default_base_config_file = os.path.join(
        gimbal_driver_share, "config", "gimbal_driver_config.yaml"
    )
    default_debug_config_file = os.path.join(
        gimbal_driver_share, "config", "navigation_test.yaml"
    )

    argument_names = (
        "base_config_file",
        "debug_config_file",
        "output",
        "use_virtual_device",
        "raw_log_enable",
        "raw_log_uplink",
        "raw_log_downlink",
        "raw_log_screen",
        "raw_log_flush",
        "raw_log_dir",
        "raw_log_type_ids",
        "raw_topic_enable",
        "raw_topic_uplink",
        "raw_topic_downlink",
        "raw_topic_type_ids",
    )

    launch_arguments = [
        DeclareLaunchArgument(
            "base_config_file",
            default_value=default_base_config_file,
            description="Formal serial/lower-machine baseline for gimbal_driver.",
        ),
        DeclareLaunchArgument(
            "debug_config_file",
            default_value=default_debug_config_file,
            description="Single-node driver debug profile; navigation_test.yaml by default.",
        ),
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("use_virtual_device", default_value="false"),
        DeclareLaunchArgument("raw_log_enable", default_value=""),
        DeclareLaunchArgument("raw_log_uplink", default_value=""),
        DeclareLaunchArgument("raw_log_downlink", default_value=""),
        DeclareLaunchArgument("raw_log_screen", default_value=""),
        DeclareLaunchArgument("raw_log_flush", default_value=""),
        DeclareLaunchArgument("raw_log_dir", default_value=""),
        DeclareLaunchArgument("raw_log_type_ids", default_value=""),
        DeclareLaunchArgument("raw_topic_enable", default_value=""),
        DeclareLaunchArgument("raw_topic_uplink", default_value=""),
        DeclareLaunchArgument("raw_topic_downlink", default_value=""),
        DeclareLaunchArgument("raw_topic_type_ids", default_value=""),
    ]

    forwarded_arguments = {
        name: LaunchConfiguration(name) for name in argument_names
    }
    forwarded_arguments["config_file"] = forwarded_arguments.pop("debug_config_file")

    return LaunchDescription([
        *launch_arguments,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gimbal_driver_launch),
            launch_arguments=forwarded_arguments.items(),
        ),
    ])
