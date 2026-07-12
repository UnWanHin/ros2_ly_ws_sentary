#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
gimbal_driver 独立启动入口。

用途：
- 单独调试串口收发和 /ly/control/* -> /ly/gimbal/* 转发行为。

关键参数：
- base_config_file：gimbal_driver 基线 YAML（串口、下位机与 raw 诊断参数）。
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
    gimbal_driver_share = get_package_share_directory("gimbal_driver")
    default_base_config_file = os.path.join(
        gimbal_driver_share, "config", "gimbal_driver_config.yaml"
    )

    def build_node(context):
        base_config_file_value = LaunchConfiguration("base_config_file").perform(context).strip()
        config_file_value = LaunchConfiguration("config_file").perform(context).strip()
        output_value = LaunchConfiguration("output")
        use_virtual_device_value = LaunchConfiguration("use_virtual_device")

        runtime_overrides = {
            "io_config/use_virtual_device": use_virtual_device_value,
            "io_config.use_virtual_device": use_virtual_device_value,
        }

        def add_bool_override(arg_name, slash_key, dot_key):
            value = LaunchConfiguration(arg_name).perform(context).strip()
            if value:
                runtime_overrides[slash_key] = ParameterValue(value, value_type=bool)
                runtime_overrides[dot_key] = ParameterValue(value, value_type=bool)

        def add_str_override(arg_name, slash_key, dot_key):
            value = LaunchConfiguration(arg_name).perform(context).strip()
            if value:
                runtime_overrides[slash_key] = ParameterValue(value, value_type=str)
                runtime_overrides[dot_key] = ParameterValue(value, value_type=str)

        add_bool_override(
            "raw_log_enable",
            "io_config/raw_serial_log_enable",
            "io_config.raw_serial_log_enable")
        add_bool_override(
            "raw_log_uplink",
            "io_config/raw_serial_log_uplink",
            "io_config.raw_serial_log_uplink")
        add_bool_override(
            "raw_log_downlink",
            "io_config/raw_serial_log_downlink",
            "io_config.raw_serial_log_downlink")
        add_bool_override(
            "raw_log_screen",
            "io_config/raw_serial_log_screen",
            "io_config.raw_serial_log_screen")
        add_bool_override(
            "raw_log_flush",
            "io_config/raw_serial_log_flush",
            "io_config.raw_serial_log_flush")
        add_str_override(
            "raw_log_dir",
            "io_config/raw_serial_log_dir",
            "io_config.raw_serial_log_dir")
        add_str_override(
            "raw_log_type_ids",
            "io_config/raw_serial_log_type_ids",
            "io_config.raw_serial_log_type_ids")
        add_bool_override(
            "raw_topic_enable",
            "io_config/raw_serial_topic_enable",
            "io_config.raw_serial_topic_enable")
        add_bool_override(
            "raw_topic_uplink",
            "io_config/raw_serial_topic_uplink",
            "io_config.raw_serial_topic_uplink")
        add_bool_override(
            "raw_topic_downlink",
            "io_config/raw_serial_topic_downlink",
            "io_config.raw_serial_topic_downlink")
        add_str_override(
            "raw_topic_type_ids",
            "io_config/raw_serial_topic_type_ids",
            "io_config.raw_serial_topic_type_ids")

        parameters = []
        if base_config_file_value:
            parameters.append(base_config_file_value)
        if config_file_value:
            parameters.append(config_file_value)
        parameters.append(runtime_overrides)

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
            description="Baseline serial/lower-machine YAML for gimbal_driver.",
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
            default_value="",
            description="Enable raw serial rx/tx file log. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_log_uplink",
            default_value="",
            description="Log lower -> upper TypeID frames. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_log_downlink",
            default_value="",
            description="Log upper -> lower control frames. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_log_screen",
            default_value="",
            description="Also print raw log lines to ROS screen output. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_log_flush",
            default_value="",
            description="Flush raw log file after each line. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_log_dir",
            default_value="",
            description="Raw serial log output directory. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_log_type_ids",
            default_value="",
            description="Comma-separated uplink TypeID list or all. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_topic_enable",
            default_value="",
            description="Enable binary raw serial ROS2 topics. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_topic_uplink",
            default_value="",
            description="Publish lower -> upper raw TypeID frames to /ly/log/gimbal_raw_rx. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_topic_downlink",
            default_value="",
            description="Publish upper -> lower raw control frames to /ly/log/gimbal_raw_tx. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "raw_topic_type_ids",
            default_value="",
            description="Comma-separated uplink TypeID list or all. Empty uses YAML config.",
        ),
        OpaqueFunction(function=build_node),
    ])
