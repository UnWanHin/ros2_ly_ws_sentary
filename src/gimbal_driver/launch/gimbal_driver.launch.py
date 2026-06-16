#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
gimbal_driver 独立启动入口。

用途：
- 单独调试串口收发和 /ly/control/* -> /ly/gimbal/* 转发行为。

关键参数：
- base_config_file：共享基础参数 YAML（可提供串口设备名、波特率等）。
- config_file：可选 overlay YAML，用于覆盖少量调试参数。
- use_virtual_device：是否使用虚拟设备（离车调试建议 true）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    behavior_tree_share = get_package_share_directory("behavior_tree")
    default_base_config_file = os.path.join(behavior_tree_share, "config", "base_config.yaml")

    def build_node(context):
        base_config_file_value = LaunchConfiguration("base_config_file").perform(context).strip()
        config_file_value = LaunchConfiguration("config_file").perform(context).strip()
        output_value = LaunchConfiguration("output")
        use_virtual_device_value = LaunchConfiguration("use_virtual_device")
        raw_log_enable = LaunchConfiguration("raw_log_enable")
        raw_log_uplink = LaunchConfiguration("raw_log_uplink")
        raw_log_downlink = LaunchConfiguration("raw_log_downlink")
        raw_log_screen = LaunchConfiguration("raw_log_screen")
        raw_log_flush = LaunchConfiguration("raw_log_flush")
        raw_log_dir = LaunchConfiguration("raw_log_dir")
        raw_log_type_ids = LaunchConfiguration("raw_log_type_ids")
        raw_topic_enable = LaunchConfiguration("raw_topic_enable")
        raw_topic_uplink = LaunchConfiguration("raw_topic_uplink")
        raw_topic_downlink = LaunchConfiguration("raw_topic_downlink")
        raw_topic_type_ids = LaunchConfiguration("raw_topic_type_ids")

        parameters = []
        if base_config_file_value:
            parameters.append(base_config_file_value)
        if config_file_value:
            parameters.append(config_file_value)
        parameters.append({
            "io_config/use_virtual_device": use_virtual_device_value,
            "io_config.use_virtual_device": use_virtual_device_value,
            "io_config/raw_serial_log_enable": ParameterValue(raw_log_enable, value_type=bool),
            "io_config.raw_serial_log_enable": ParameterValue(raw_log_enable, value_type=bool),
            "io_config/raw_serial_log_uplink": ParameterValue(raw_log_uplink, value_type=bool),
            "io_config.raw_serial_log_uplink": ParameterValue(raw_log_uplink, value_type=bool),
            "io_config/raw_serial_log_downlink": ParameterValue(raw_log_downlink, value_type=bool),
            "io_config.raw_serial_log_downlink": ParameterValue(raw_log_downlink, value_type=bool),
            "io_config/raw_serial_log_screen": ParameterValue(raw_log_screen, value_type=bool),
            "io_config.raw_serial_log_screen": ParameterValue(raw_log_screen, value_type=bool),
            "io_config/raw_serial_log_flush": ParameterValue(raw_log_flush, value_type=bool),
            "io_config.raw_serial_log_flush": ParameterValue(raw_log_flush, value_type=bool),
            "io_config/raw_serial_log_dir": ParameterValue(raw_log_dir, value_type=str),
            "io_config.raw_serial_log_dir": ParameterValue(raw_log_dir, value_type=str),
            "io_config/raw_serial_log_type_ids": ParameterValue(raw_log_type_ids, value_type=str),
            "io_config.raw_serial_log_type_ids": ParameterValue(raw_log_type_ids, value_type=str),
            "io_config/raw_serial_topic_enable": ParameterValue(raw_topic_enable, value_type=bool),
            "io_config.raw_serial_topic_enable": ParameterValue(raw_topic_enable, value_type=bool),
            "io_config/raw_serial_topic_uplink": ParameterValue(raw_topic_uplink, value_type=bool),
            "io_config.raw_serial_topic_uplink": ParameterValue(raw_topic_uplink, value_type=bool),
            "io_config/raw_serial_topic_downlink": ParameterValue(raw_topic_downlink, value_type=bool),
            "io_config.raw_serial_topic_downlink": ParameterValue(raw_topic_downlink, value_type=bool),
            "io_config/raw_serial_topic_type_ids": ParameterValue(raw_topic_type_ids, value_type=str),
            "io_config.raw_serial_topic_type_ids": ParameterValue(raw_topic_type_ids, value_type=str),
        })

        return [
            Node(
                package="gimbal_driver",
                executable="gimbal_driver_node",
                name="gimbal_driver",
                output=output_value,
                parameters=parameters,
            ),
        ]

    output = LaunchConfiguration("output")
    use_virtual_device = LaunchConfiguration("use_virtual_device")

    return LaunchDescription([
        DeclareLaunchArgument(
            "base_config_file",
            default_value=default_base_config_file,
            description="Base YAML config file for gimbal_driver.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Optional overlay YAML config file for gimbal_driver.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument(
            "use_virtual_device",
            default_value="false",
            description="Whether to use virtual serial device in gimbal_driver.",
        ),
        DeclareLaunchArgument(
            "raw_log_enable",
            default_value="false",
            description="Enable raw serial rx/tx file log.",
        ),
        DeclareLaunchArgument(
            "raw_log_uplink",
            default_value="true",
            description="Log lower -> upper TypeID frames.",
        ),
        DeclareLaunchArgument(
            "raw_log_downlink",
            default_value="true",
            description="Log upper -> lower control frames.",
        ),
        DeclareLaunchArgument(
            "raw_log_screen",
            default_value="false",
            description="Also print raw log lines to ROS screen output.",
        ),
        DeclareLaunchArgument(
            "raw_log_flush",
            default_value="true",
            description="Flush raw log file after each line.",
        ),
        DeclareLaunchArgument(
            "raw_log_dir",
            default_value="~/Log/GimbalRaw",
            description="Raw serial log output directory.",
        ),
        DeclareLaunchArgument(
            "raw_log_type_ids",
            default_value="all",
            description="Comma-separated uplink TypeID list or all.",
        ),
        DeclareLaunchArgument(
            "raw_topic_enable",
            default_value="false",
            description="Enable binary raw serial ROS2 topics.",
        ),
        DeclareLaunchArgument(
            "raw_topic_uplink",
            default_value="true",
            description="Publish lower -> upper raw TypeID frames to /ly/log/gimbal_raw_rx.",
        ),
        DeclareLaunchArgument(
            "raw_topic_downlink",
            default_value="true",
            description="Publish upper -> lower raw control frames to /ly/log/gimbal_raw_tx.",
        ),
        DeclareLaunchArgument(
            "raw_topic_type_ids",
            default_value="all",
            description="Comma-separated uplink TypeID list or all.",
        ),
        OpaqueFunction(function=build_node),
    ])
