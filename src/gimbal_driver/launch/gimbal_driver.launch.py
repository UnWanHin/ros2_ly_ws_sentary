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
- config_file：driver 专用 overlay YAML，用于覆盖少量调试参数。
- legacy_base_config_file / legacy_config_file：由 sentry_all 保留的旧 root YAML
  入口；只提取安全的 gimbal IO 参数，不把全局 YAML 注入 driver。
- use_virtual_device：是否使用虚拟设备（离车调试建议 true）。
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


_FORMAL_BLOCKED_LEGACY_PREFIXES = (
    "io_config.navigation_test",
    "io_config.navigation_mode",
)


def _flatten_parameters(value: Any, prefix: str = "") -> dict[str, Any]:
    """Flatten ROS parameter mappings using the driver's dot-key convention."""
    if not isinstance(value, dict):
        return {prefix: value} if prefix else {}

    flattened: dict[str, Any] = {}
    for key, child in value.items():
        key_text = str(key)
        child_prefix = f"{prefix}.{key_text}" if prefix else key_text
        if isinstance(child, dict):
            flattened.update(_flatten_parameters(child, child_prefix))
        else:
            flattened[child_prefix] = child
    return flattened


def _ros_parameter_blocks(document: Any) -> list[dict[str, Any]]:
    """Read global and explicit gimbal_driver ROS parameter blocks from one YAML document."""
    if not isinstance(document, dict):
        return []

    blocks: list[dict[str, Any]] = []
    for node_selector, node_config in document.items():
        selector = str(node_selector).rstrip("/")
        is_global = selector == "/**"
        is_gimbal = selector in {"gimbal_driver", "/gimbal_driver"}
        if not (is_global or is_gimbal) or not isinstance(node_config, dict):
            continue
        params = node_config.get("ros__parameters")
        if isinstance(params, dict):
            blocks.append(params)
    return blocks


def load_legacy_gimbal_parameters(
    config_files: list[Path | str],
) -> tuple[dict[str, Any], set[str]]:
    """Translate legacy root YAML into safe gimbal-only dot-key overrides.

    `sentry_all` historically sent root `base_config_file` and `config_file`
    directly to every node. The retained driver settings remain compatible,
    but direct-navigation debug keys are intentionally excluded from the formal
    chain; `debug_node.launch.py` remains the explicit entry for those keys.
    """
    routed: dict[str, Any] = {}
    ignored: set[str] = set()

    for config_file in config_files:
        raw_path = str(config_file).strip()
        if not raw_path:
            continue
        path = Path(raw_path)
        if not path.is_file():
            raise RuntimeError(f"legacy gimbal config file does not exist: {path}")
        try:
            with path.open(encoding="utf-8") as handle:
                documents = list(yaml.safe_load_all(handle))
        except yaml.YAMLError as exc:
            raise RuntimeError(f"cannot parse legacy gimbal config '{path}': {exc}") from exc

        for document in documents:
            for block in _ros_parameter_blocks(document):
                for raw_key, value in _flatten_parameters(block).items():
                    key = raw_key.replace("/", ".")
                    if not key.startswith("io_config."):
                        continue
                    if key.startswith(_FORMAL_BLOCKED_LEGACY_PREFIXES):
                        ignored.add(key)
                        continue
                    routed[key] = value

    return routed, ignored


def generate_launch_description():
    gimbal_driver_share = get_package_share_directory("gimbal_driver")
    navi_tf_bridge_share = get_package_share_directory("navi_tf_bridge")
    default_base_config_file = os.path.join(
        gimbal_driver_share, "config", "gimbal_driver_config.yaml"
    )
    default_path_bridge_config_file = os.path.join(
        navi_tf_bridge_share, "config", "tf_config.yaml"
    )

    def build_node(context):
        base_config_file_value = LaunchConfiguration("base_config_file").perform(context).strip()
        config_file_value = LaunchConfiguration("config_file").perform(context).strip()
        legacy_base_config_file_value = LaunchConfiguration(
            "legacy_base_config_file"
        ).perform(context).strip()
        legacy_config_file_value = LaunchConfiguration(
            "legacy_config_file"
        ).perform(context).strip()
        output_value = LaunchConfiguration("output")
        use_virtual_device_value = LaunchConfiguration("use_virtual_device")

        legacy_parameters, ignored_legacy_keys = load_legacy_gimbal_parameters([
            legacy_base_config_file_value,
            legacy_config_file_value,
        ])

        runtime_overrides = {
            "io_config/use_virtual_device": ParameterValue(
                use_virtual_device_value, value_type=bool
            ),
            "io_config.use_virtual_device": ParameterValue(
                use_virtual_device_value, value_type=bool
            ),
        }

        def add_bool_override(arg_name, slash_key, dot_key):
            value = LaunchConfiguration(arg_name).perform(context).strip()
            if value:
                typed_value = ParameterValue(
                    LaunchConfiguration(arg_name), value_type=bool
                )
                runtime_overrides[slash_key] = typed_value
                runtime_overrides[dot_key] = typed_value

        def add_str_override(arg_name, slash_key, dot_key):
            value = LaunchConfiguration(arg_name).perform(context).strip()
            if value:
                runtime_overrides[slash_key] = ParameterValue(value, value_type=str)
                runtime_overrides[dot_key] = ParameterValue(value, value_type=str)

        def add_int_override(arg_name, slash_key, dot_key):
            value = LaunchConfiguration(arg_name).perform(context).strip()
            if value:
                typed_value = ParameterValue(
                    LaunchConfiguration(arg_name), value_type=int
                )
                runtime_overrides[slash_key] = typed_value
                runtime_overrides[dot_key] = typed_value

        def add_float_override(arg_name, slash_key, dot_key):
            value = LaunchConfiguration(arg_name).perform(context).strip()
            if value:
                typed_value = ParameterValue(
                    LaunchConfiguration(arg_name), value_type=float
                )
                runtime_overrides[slash_key] = typed_value
                runtime_overrides[dot_key] = typed_value

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
        add_bool_override(
            "raw_downlink_test_mode",
            "io_config/raw_downlink_test_mode",
            "io_config.raw_downlink_test_mode")
        add_str_override(
            "raw_topic_type_ids",
            "io_config/raw_serial_topic_type_ids",
            "io_config.raw_serial_topic_type_ids")
        add_int_override(
            "firecode_partial_hold_ms",
            "io_config/firecode_partial_hold_ms",
            "io_config.firecode_partial_hold_ms")
        add_float_override(
            "velocity_raw_to_mps",
            "io_config/velocity_raw_to_mps",
            "io_config.velocity_raw_to_mps")
        add_bool_override(
            "team_override_enable",
            "io_config/team_override/enable",
            "io_config.team_override.enable")
        add_bool_override(
            "team_override_red",
            "io_config/team_override/red",
            "io_config.team_override.red")
        add_bool_override(
            "team_override_blue",
            "io_config/team_override/blue",
            "io_config.team_override.blue")

        parameters = []
        if base_config_file_value:
            parameters.append(base_config_file_value)
        if legacy_parameters:
            parameters.append(legacy_parameters)
        if config_file_value:
            parameters.append(config_file_value)
        parameters.append(runtime_overrides)

        actions = []
        if legacy_parameters:
            actions.append(LogInfo(msg=[
                "[gimbal_driver] applied legacy root gimbal keys: ",
                ", ".join(sorted(legacy_parameters)),
            ]))
        if ignored_legacy_keys:
            actions.append(LogInfo(msg=[
                "[gimbal_driver] ignored formal-only legacy navigation debug keys: ",
                ", ".join(sorted(ignored_legacy_keys)),
            ]))
        actions.append(
            Node(
                package="navi_tf_bridge",
                executable="map_path_to_game_path_node",
                name="map_path_to_game_path_node",
                output=output_value,
                condition=IfCondition(LaunchConfiguration("enable_path_downsampled_bridge")),
                parameters=[
                    default_path_bridge_config_file,
                    {
                        "input_topic": LaunchConfiguration("path_downsampled_topic"),
                        "output_topic": "/ly/game/path",
                        "map_frame": "map",
                        "intention": 3,
                        "sentry_info_topic": "/ly/game/sentry/info",
                    },
                ],
            )
        )
        actions.append(
            Node(
                package="gimbal_driver",
                executable="gimbal_driver_node",
                name="gimbal_driver",
                output=output_value,
                parameters=parameters,
                on_exit=Shutdown(reason="gimbal_driver exited"),
            )
        )
        return actions

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
            "legacy_base_config_file",
            default_value="",
            description="Legacy root base YAML; only safe gimbal IO keys are routed.",
        ),
        DeclareLaunchArgument(
            "legacy_config_file",
            default_value="",
            description="Legacy root override YAML; only safe gimbal IO keys are routed.",
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
        DeclareLaunchArgument("enable_path_downsampled_bridge", default_value="true"),
        DeclareLaunchArgument(
            "path_downsampled_topic",
            default_value="/Path_downsampled",
            description="nav_msgs/Path source converted to /ly/game/path before serial downlink.",
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
        DeclareLaunchArgument(
            "raw_downlink_test_mode",
            default_value="false",
            description="Exclusive debug-only raw downlink injection mode. Normal control writes are suppressed.",
        ),
        DeclareLaunchArgument(
            "firecode_partial_hold_ms",
            default_value="",
            description="FireCode partial-field hold time. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "velocity_raw_to_mps",
            default_value="",
            description="Navigation raw velocity scale. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "team_override_enable",
            default_value="",
            description="Emergency team override enable. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "team_override_red",
            default_value="",
            description="Emergency team override force-red flag. Empty uses YAML config.",
        ),
        DeclareLaunchArgument(
            "team_override_blue",
            default_value="",
            description="Emergency team override force-blue flag. Empty uses YAML config.",
        ),
        OpaqueFunction(function=build_node),
    ])
