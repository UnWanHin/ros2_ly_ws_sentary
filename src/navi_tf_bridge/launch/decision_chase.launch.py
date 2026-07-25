#!/usr/bin/env python3

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

try:
    import yaml
except Exception:
    yaml = None


def _resolve_bt_config_path(behavior_tree_share: str, configured_path: str) -> str:
    if not configured_path:
        return ""
    path = Path(configured_path)
    if path.is_absolute():
        return str(path)
    return str((Path(behavior_tree_share) / path).resolve(strict=False))


def _normalize_bool(raw: str) -> str:
    value = (raw or "").strip().lower()
    if value in ("1", "true", "yes", "on"):
        return "true"
    if value in ("0", "false", "no", "off"):
        return "false"
    return ""


def _bool_default(value) -> str:
    return "true" if bool(value) else "false"


def _normalize_area_token(raw) -> str:
    return str(raw or "").strip().lower().replace("-", "_").replace(" ", "_")


def _area_scope_csv(raw) -> str:
    if isinstance(raw, dict):
        tokens = [key for key, enabled in raw.items() if bool(enabled)]
    elif isinstance(raw, list):
        tokens = raw
    else:
        tokens = []
    return ",".join(token for token in (_normalize_area_token(item) for item in tokens) if token)


def _load_bridge_defaults(param_file: str) -> dict:
    if yaml is None:
        return {}
    if not param_file or not os.path.exists(param_file):
        return {}
    try:
        with open(param_file, encoding="utf-8") as fh:
            root = yaml.safe_load(fh) or {}
        node_cfg = root.get("target_rel_to_goal_pos_node", {})
        ros_params = node_cfg.get("ros__parameters", {})
        if isinstance(ros_params, dict):
            return ros_params
    except Exception as ex:
        print(f"[decision_chase] failed to load bridge defaults from '{param_file}': {ex}")
    return {}


def _guess_workspace_root_from_share(share_dir: str) -> str:
    path = Path(share_dir).resolve()
    for parent in [path, *path.parents]:
        if parent.name == "install":
            return str(parent.parent)
    return str(path.parent)


def _resolve_bridge_chase_params(context, behavior_tree_share: str):
    resolved_preferred_distance_cm = "100"
    resolved_distance_deadband_cm = "50"
    resolved_stop_when_no_target = "true"
    resolved_to_navi = "true"
    resolved_area_limit_enable = "false"
    resolved_area_limit_boundary_margin_cm = "30.0"
    resolved_area_limit_chase_enable_cross_area = "false"
    resolved_area_limit_use_area_scope = "false"
    resolved_area_limit_my_area = ""
    resolved_area_limit_enemy_area = ""
    resolved_area_limit_common_area = ""
    resolved_area_limit_hold_when_no_intersection = "true"

    bt_config_file_value = LaunchConfiguration("bt_config_file").perform(context).strip()
    bt_config_path = _resolve_bt_config_path(behavior_tree_share, bt_config_file_value)
    if bt_config_path and os.path.exists(bt_config_path):
        try:
            with open(bt_config_path, encoding="utf-8") as fh:
                config_root = json.load(fh)
            chase_cfg = config_root.get("Chase", {})
            if isinstance(chase_cfg, dict):
                resolved_preferred_distance_cm = str(
                    int(chase_cfg.get("PreferredDistanceCm", int(resolved_preferred_distance_cm)))
                )
                resolved_distance_deadband_cm = str(
                    int(chase_cfg.get("DistanceDeadbandCm", int(resolved_distance_deadband_cm)))
                )
                resolved_stop_when_no_target = (
                    "true" if bool(chase_cfg.get("StopWhenNoTarget", True)) else "false"
                )
                area_limit_cfg = chase_cfg.get("AreaLimit", {})
                if isinstance(area_limit_cfg, dict):
                    resolved_area_limit_enable = (
                        "true" if bool(area_limit_cfg.get("Enable", False)) else "false"
                    )
                    resolved_area_limit_boundary_margin_cm = str(
                        float(area_limit_cfg.get("BoundaryMarginCm", 30.0))
                    )
                    resolved_area_limit_chase_enable_cross_area = (
                        "true"
                        if bool(area_limit_cfg.get("ChaseEnableCrossArea", False))
                        else "false"
                    )
                    resolved_area_limit_hold_when_no_intersection = (
                        "true"
                        if bool(area_limit_cfg.get("HoldWhenNoIntersection", True))
                        else "false"
                    )
            autonomy_cfg = config_root.get("DecisionAutonomy", {})
            if isinstance(autonomy_cfg, dict):
                navi_goal_cfg = autonomy_cfg.get("NaviGoal", {})
                if isinstance(navi_goal_cfg, dict):
                    resolved_area_limit_use_area_scope = (
                        "true" if bool(navi_goal_cfg.get("UseAreaScope", False)) else "false"
                    )
                    resolved_area_limit_my_area = _area_scope_csv(navi_goal_cfg.get("MyArea", {}))
                    resolved_area_limit_enemy_area = _area_scope_csv(
                        navi_goal_cfg.get("EnemyArea", {})
                    )
                    resolved_area_limit_common_area = _area_scope_csv(
                        navi_goal_cfg.get("CommonArea", {})
                    )
            navi_cfg = config_root.get("NaviSetting", {})
            if isinstance(navi_cfg, dict):
                to_navi = navi_cfg.get("ToNavi", navi_cfg.get("UseTfGoalBridge", True))
                resolved_to_navi = "true" if bool(to_navi) else "false"
        except Exception as ex:
            print(
                f"[decision_chase] failed to parse bt_config_file '{bt_config_path}': {ex}. "
                "Bridge chase parameters fall back to defaults."
            )

    preferred_override = LaunchConfiguration("preferred_distance_cm").perform(context).strip()
    if preferred_override:
        resolved_preferred_distance_cm = preferred_override

    deadband_override = LaunchConfiguration("distance_deadband_cm").perform(context).strip()
    if deadband_override:
        resolved_distance_deadband_cm = deadband_override

    stop_override = _normalize_bool(LaunchConfiguration("stop_when_no_target").perform(context))
    if stop_override:
        resolved_stop_when_no_target = stop_override

    to_navi_override = _normalize_bool(LaunchConfiguration("to_navi").perform(context))
    if to_navi_override:
        resolved_to_navi = to_navi_override
    else:
        legacy_tf_bridge_override = _normalize_bool(
            LaunchConfiguration("enable_tf_goal_bridge").perform(context)
        )
        if legacy_tf_bridge_override:
            resolved_to_navi = legacy_tf_bridge_override

    allow_reverse_goal = _normalize_bool(LaunchConfiguration("allow_reverse_goal").perform(context))
    if not allow_reverse_goal:
        allow_reverse_goal = "false"

    return [
        SetLaunchConfiguration(
            "resolved_bridge_preferred_distance_cm", resolved_preferred_distance_cm
        ),
        SetLaunchConfiguration(
            "resolved_bridge_distance_deadband_cm", resolved_distance_deadband_cm
        ),
        SetLaunchConfiguration(
            "resolved_bridge_stop_when_no_target", resolved_stop_when_no_target
        ),
        SetLaunchConfiguration("resolved_to_navi", resolved_to_navi),
        SetLaunchConfiguration("resolved_bridge_allow_reverse_goal", allow_reverse_goal),
        SetLaunchConfiguration("resolved_chase_area_limit_enable", resolved_area_limit_enable),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_boundary_margin_cm",
            resolved_area_limit_boundary_margin_cm,
        ),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_chase_enable_cross_area",
            resolved_area_limit_chase_enable_cross_area,
        ),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_use_area_scope",
            resolved_area_limit_use_area_scope,
        ),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_my_area",
            resolved_area_limit_my_area,
        ),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_enemy_area",
            resolved_area_limit_enemy_area,
        ),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_common_area",
            resolved_area_limit_common_area,
        ),
        SetLaunchConfiguration(
            "resolved_chase_area_limit_hold_when_no_intersection",
            resolved_area_limit_hold_when_no_intersection,
        ),
    ]


def generate_launch_description():
    behavior_tree_share = get_package_share_directory("behavior_tree")
    bridge_share = get_package_share_directory("navi_tf_bridge")

    chase_only_launch = os.path.join(behavior_tree_share, "launch", "chase_only.launch.py")
    config_root = os.path.join(behavior_tree_share, "config")

    default_base_config_file = os.path.join(config_root, "base_config.yaml")
    default_override_config_file = os.path.join(config_root, "override_config.yaml")
    default_bridge_param_file = os.path.join(
        bridge_share, "config", "tf_config.yaml"
    )
    workspace_root = _guess_workspace_root_from_share(bridge_share)
    default_area_header_file = os.path.join(
        workspace_root, "src", "behavior_tree", "module", "Area.hpp"
    )
    default_debug_pair_file = os.path.join(
        workspace_root, "log", "navi_tf_bridge", "tf_point_pairs.yaml"
    )
    bridge_defaults = _load_bridge_defaults(default_bridge_param_file)
    get_bridge_default = lambda key, fallback: bridge_defaults.get(key, fallback)
    chase_area_limit_defaults = bridge_defaults.get("chase_area_limit", {})
    if not isinstance(chase_area_limit_defaults, dict):
        chase_area_limit_defaults = {}
    get_chase_area_limit_default = lambda key, fallback: chase_area_limit_defaults.get(
        key, fallback
    )

    launch_args = [
        DeclareLaunchArgument("mode", default_value="league"),
        DeclareLaunchArgument("config_file", default_value=default_override_config_file),
        DeclareLaunchArgument("base_config_file", default_value=default_base_config_file),
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("offline", default_value="false"),
        DeclareLaunchArgument("debug_bypass_is_start", default_value="true"),
        DeclareLaunchArgument("publish_navi_goal", default_value="false"),
        DeclareLaunchArgument("wait_for_game_start_timeout_sec", default_value="0"),
        DeclareLaunchArgument("league_referee_stale_timeout_ms", default_value="0"),
        DeclareLaunchArgument("bt_tree_file", default_value=""),
        DeclareLaunchArgument(
            "bt_config_file", default_value="Scripts/ConfigJson/league/chase_only_competition.json"
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "input_topic",
            default_value=str(get_bridge_default("input_topic", "/ly/navi/target_rel")),
        ),
        DeclareLaunchArgument(
            "output_goal_pos_topic",
            default_value=str(get_bridge_default("output_goal_pos_topic", "/ly/navi/goal_pos")),
        ),
        DeclareLaunchArgument(
            "output_goal_pose_topic",
            default_value=str(get_bridge_default("output_goal_pose_topic", "/goal_pose")),
        ),
        DeclareLaunchArgument(
            "input_goal_pos_raw_topic",
            default_value=str(get_bridge_default("input_goal_pos_raw_topic", "/ly/navi/goal_pos_raw")),
        ),
        DeclareLaunchArgument(
            "output_target_map_topic",
            default_value=str(get_bridge_default("output_target_map_topic", "/ly/navi/target_map")),
        ),
        DeclareLaunchArgument(
            "map_frame", default_value=str(get_bridge_default("map_frame", "map"))
        ),
        DeclareLaunchArgument(
            "base_frame", default_value=str(get_bridge_default("base_frame", "base_link"))
        ),
        DeclareLaunchArgument(
            "fallback_base_frame",
            default_value=str(get_bridge_default("fallback_base_frame", "baselink")),
        ),
        DeclareLaunchArgument(
            "target_rel_default_frame",
            default_value=str(get_bridge_default("target_rel_default_frame", "gimbal_world")),
        ),
        DeclareLaunchArgument(
            "use_msg_frame_id",
            default_value=_bool_default(get_bridge_default("use_msg_frame_id", True)),
        ),
        DeclareLaunchArgument(
            "publish_target_map",
            default_value=_bool_default(get_bridge_default("publish_target_map", True)),
        ),
        DeclareLaunchArgument(
            "publish_goal_pos",
            default_value=_bool_default(get_bridge_default("publish_goal_pos", False)),
        ),
        DeclareLaunchArgument(
            "publish_goal_pose",
            default_value=_bool_default(get_bridge_default("publish_goal_pose", True)),
        ),
        DeclareLaunchArgument(
            "goal_pose_uniform_scale",
            default_value=str(float(get_bridge_default("goal_pose_uniform_scale", 1.0))),
        ),
        DeclareLaunchArgument(
            "invert_y_axis",
            default_value=_bool_default(get_bridge_default("invert_y_axis", False)),
        ),
        DeclareLaunchArgument(
            "y_axis_max_cm", default_value=str(int(get_bridge_default("y_axis_max_cm", 1500)))
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_enabled",
            default_value=_bool_default(get_bridge_default("goal_pos_uint16_encode_enabled", False)),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_x_scale",
            default_value=str(float(get_bridge_default("goal_pos_uint16_encode_x_scale", 1.0))),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_y_scale",
            default_value=str(float(get_bridge_default("goal_pos_uint16_encode_y_scale", 1.0))),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_x_offset_cm",
            default_value=str(float(get_bridge_default("goal_pos_uint16_encode_x_offset_cm", 0.0))),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_y_offset_cm",
            default_value=str(float(get_bridge_default("goal_pos_uint16_encode_y_offset_cm", 0.0))),
        ),
        DeclareLaunchArgument(
            "preferred_distance_cm",
            default_value="",
            description="Optional bridge override. Empty means load Chase.PreferredDistanceCm from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "distance_deadband_cm",
            default_value="",
            description="Optional bridge override. Empty means load Chase.DistanceDeadbandCm from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "stop_when_no_target",
            default_value="",
            description="Optional bridge override. Empty means load Chase.StopWhenNoTarget from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "to_navi",
            default_value="",
            description="Optional override. Empty means load NaviSetting.ToNavi from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "enable_tf_goal_bridge",
            default_value="",
            description="Legacy alias for to_navi.",
        ),
        DeclareLaunchArgument(
            "allow_reverse_goal",
            default_value=_bool_default(get_bridge_default("allow_reverse_goal", False)),
            description="Whether the bridge may output a reverse chase goal when already too close to the target.",
        ),
        DeclareLaunchArgument("resolved_chase_area_limit_enable", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_boundary_margin_cm", default_value="30.0"),
        DeclareLaunchArgument("resolved_chase_area_limit_chase_enable_cross_area", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_use_area_scope", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_my_area", default_value=""),
        DeclareLaunchArgument("resolved_chase_area_limit_enemy_area", default_value=""),
        DeclareLaunchArgument("resolved_chase_area_limit_common_area", default_value=""),
        DeclareLaunchArgument("resolved_chase_area_limit_hold_when_no_intersection", default_value="true"),
        DeclareLaunchArgument(
            "chase_area_limit_area_header_file",
            default_value=str(
                get_chase_area_limit_default("area_header_file", default_area_header_file)
                or default_area_header_file
            ),
        ),
        DeclareLaunchArgument(
            "enable_goal_pos_raw_bridge",
            default_value=_bool_default(get_bridge_default("enable_goal_pos_raw_bridge", True)),
        ),
        DeclareLaunchArgument(
            "goal_pos_raw_frame",
            default_value=str(get_bridge_default("goal_pos_raw_frame", "map")),
        ),
        DeclareLaunchArgument(
            "debug_export_point_pairs",
            default_value=_bool_default(get_bridge_default("debug_export_point_pairs", True)),
        ),
        DeclareLaunchArgument(
            "debug_points_reference_frame",
            default_value=str(get_bridge_default("debug_points_reference_frame", "map")),
        ),
        DeclareLaunchArgument(
            "debug_area_header_file",
            default_value=str(
                get_bridge_default("debug_area_header_file", default_area_header_file)
                or default_area_header_file
            ),
        ),
        DeclareLaunchArgument(
            "debug_point_pairs_output_file",
            default_value=str(
                get_bridge_default("debug_point_pairs_output_file", default_debug_pair_file)
                or default_debug_pair_file
            ),
        ),
        DeclareLaunchArgument("resolved_bridge_preferred_distance_cm", default_value="100"),
        DeclareLaunchArgument("resolved_bridge_distance_deadband_cm", default_value="50"),
        DeclareLaunchArgument("resolved_bridge_stop_when_no_target", default_value="true"),
        DeclareLaunchArgument("resolved_to_navi", default_value="true"),
        DeclareLaunchArgument("resolved_bridge_allow_reverse_goal", default_value="false"),
        OpaqueFunction(function=_resolve_bridge_chase_params, args=[behavior_tree_share]),
        LogInfo(msg=["[decision_chase] bridge param file: ", default_bridge_param_file]),
        LogInfo(
            msg=[
                "[decision_chase] debug point pairs -> ",
                LaunchConfiguration("debug_point_pairs_output_file"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge preferred_distance_cm: ",
                LaunchConfiguration("resolved_bridge_preferred_distance_cm"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge distance_deadband_cm: ",
                LaunchConfiguration("resolved_bridge_distance_deadband_cm"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge stop_when_no_target: ",
                LaunchConfiguration("resolved_bridge_stop_when_no_target"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] ToNavi: ",
                LaunchConfiguration("resolved_to_navi"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge publish_goal_pos: ",
                LaunchConfiguration("publish_goal_pos"),
                " -> ",
                LaunchConfiguration("output_goal_pos_topic"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge publish_goal_pose: ",
                LaunchConfiguration("publish_goal_pose"),
                " -> ",
                LaunchConfiguration("output_goal_pose_topic"),
            ]
        ),
        LogInfo(
            msg=[
                "[decision_chase] bridge allow_reverse_goal: ",
                LaunchConfiguration("resolved_bridge_allow_reverse_goal"),
            ]
        ),
    ]

    chase_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(chase_only_launch),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
            "config_file": LaunchConfiguration("config_file"),
            "base_config_file": LaunchConfiguration("base_config_file"),
            "output": LaunchConfiguration("output"),
            "offline": LaunchConfiguration("offline"),
            "debug_bypass_is_start": LaunchConfiguration("debug_bypass_is_start"),
            "publish_navi_goal": LaunchConfiguration("publish_navi_goal"),
            "wait_for_game_start_timeout_sec": LaunchConfiguration("wait_for_game_start_timeout_sec"),
            "league_referee_stale_timeout_ms": LaunchConfiguration("league_referee_stale_timeout_ms"),
            "bt_tree_file": LaunchConfiguration("bt_tree_file"),
            "bt_config_file": LaunchConfiguration("bt_config_file"),
            "use_gimbal": LaunchConfiguration("use_gimbal"),
            "use_behavior_tree": LaunchConfiguration("use_behavior_tree"),
        }.items(),
    )

    target_rel_bridge = Node(
        package="navi_tf_bridge",
        executable="target_rel_to_goal_pos_node",
        name="target_rel_to_goal_pos_node",
        output=LaunchConfiguration("output"),
        condition=IfCondition(LaunchConfiguration("resolved_to_navi")),
        parameters=[
            default_bridge_param_file,
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "input_goal_pos_raw_topic": LaunchConfiguration("input_goal_pos_raw_topic"),
                "output_goal_pos_topic": LaunchConfiguration("output_goal_pos_topic"),
                "output_goal_pose_topic": LaunchConfiguration("output_goal_pose_topic"),
                "output_target_map_topic": LaunchConfiguration("output_target_map_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "fallback_base_frame": LaunchConfiguration("fallback_base_frame"),
                "target_rel_default_frame": LaunchConfiguration("target_rel_default_frame"),
                "use_msg_frame_id": ParameterValue(
                    LaunchConfiguration("use_msg_frame_id"), value_type=bool
                ),
                "publish_target_map": ParameterValue(
                    LaunchConfiguration("publish_target_map"), value_type=bool
                ),
                "publish_goal_pos": ParameterValue(
                    LaunchConfiguration("publish_goal_pos"), value_type=bool
                ),
                "publish_goal_pose": ParameterValue(
                    LaunchConfiguration("publish_goal_pose"), value_type=bool
                ),
                "goal_pose_uniform_scale": ParameterValue(
                    LaunchConfiguration("goal_pose_uniform_scale"), value_type=float
                ),
                "invert_y_axis": ParameterValue(
                    LaunchConfiguration("invert_y_axis"), value_type=bool
                ),
                "y_axis_max_cm": ParameterValue(
                    LaunchConfiguration("y_axis_max_cm"), value_type=int
                ),
                "goal_pos_uint16_encode_enabled": ParameterValue(
                    LaunchConfiguration("goal_pos_uint16_encode_enabled"), value_type=bool
                ),
                "goal_pos_uint16_encode_x_scale": ParameterValue(
                    LaunchConfiguration("goal_pos_uint16_encode_x_scale"), value_type=float
                ),
                "goal_pos_uint16_encode_y_scale": ParameterValue(
                    LaunchConfiguration("goal_pos_uint16_encode_y_scale"), value_type=float
                ),
                "goal_pos_uint16_encode_x_offset_cm": ParameterValue(
                    LaunchConfiguration("goal_pos_uint16_encode_x_offset_cm"), value_type=float
                ),
                "goal_pos_uint16_encode_y_offset_cm": ParameterValue(
                    LaunchConfiguration("goal_pos_uint16_encode_y_offset_cm"), value_type=float
                ),
                "preferred_distance_cm": ParameterValue(
                    LaunchConfiguration("resolved_bridge_preferred_distance_cm"), value_type=int
                ),
                "distance_deadband_cm": ParameterValue(
                    LaunchConfiguration("resolved_bridge_distance_deadband_cm"), value_type=int
                ),
                "stop_when_no_target": ParameterValue(
                    LaunchConfiguration("resolved_bridge_stop_when_no_target"), value_type=bool
                ),
                "allow_reverse_goal": ParameterValue(
                    LaunchConfiguration("resolved_bridge_allow_reverse_goal"), value_type=bool
                ),
                "chase_area_limit.enable": ParameterValue(
                    LaunchConfiguration("resolved_chase_area_limit_enable"), value_type=bool
                ),
                "chase_area_limit.area_header_file": LaunchConfiguration(
                    "chase_area_limit_area_header_file"
                ),
                "chase_area_limit.boundary_margin_cm": ParameterValue(
                    LaunchConfiguration("resolved_chase_area_limit_boundary_margin_cm"),
                    value_type=float,
                ),
                "chase_area_limit.chase_enable_cross_area": ParameterValue(
                    LaunchConfiguration("resolved_chase_area_limit_chase_enable_cross_area"),
                    value_type=bool,
                ),
                "chase_area_limit.use_area_scope": ParameterValue(
                    LaunchConfiguration("resolved_chase_area_limit_use_area_scope"),
                    value_type=bool,
                ),
                "chase_area_limit.my_area": LaunchConfiguration(
                    "resolved_chase_area_limit_my_area"
                ),
                "chase_area_limit.enemy_area": LaunchConfiguration(
                    "resolved_chase_area_limit_enemy_area"
                ),
                "chase_area_limit.common_area": LaunchConfiguration(
                    "resolved_chase_area_limit_common_area"
                ),
                "chase_area_limit.hold_when_no_intersection": ParameterValue(
                    LaunchConfiguration("resolved_chase_area_limit_hold_when_no_intersection"),
                    value_type=bool,
                ),
                "enable_goal_pos_raw_bridge": ParameterValue(
                    LaunchConfiguration("enable_goal_pos_raw_bridge"), value_type=bool
                ),
                "goal_pos_raw_frame": LaunchConfiguration("goal_pos_raw_frame"),
                "debug_export_point_pairs": ParameterValue(
                    LaunchConfiguration("debug_export_point_pairs"), value_type=bool
                ),
                "debug_points_reference_frame": LaunchConfiguration(
                    "debug_points_reference_frame"
                ),
                "debug_area_header_file": LaunchConfiguration("debug_area_header_file"),
                "debug_point_pairs_output_file": LaunchConfiguration(
                    "debug_point_pairs_output_file"
                ),
            }
        ],
    )

    return LaunchDescription(launch_args + [chase_only, target_rel_bridge])
