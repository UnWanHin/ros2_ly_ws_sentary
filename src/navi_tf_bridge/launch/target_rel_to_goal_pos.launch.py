import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

try:
    import yaml
except Exception:
    yaml = None


def _bool_default(value) -> str:
    return "true" if bool(value) else "false"


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
        print(f"[target_rel_to_goal_pos] failed to load defaults from '{param_file}': {ex}")
    return {}


def _guess_workspace_root_from_share(share_dir: str) -> str:
    path = Path(share_dir).resolve()
    for parent in [path, *path.parents]:
        if parent.name == "install":
            return str(parent.parent)
    return str(path.parent)


def generate_launch_description():
    bridge_share = get_package_share_directory("navi_tf_bridge")
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
    get_default = lambda key, fallback: bridge_defaults.get(key, fallback)
    chase_area_limit_defaults = bridge_defaults.get("chase_area_limit", {})
    if not isinstance(chase_area_limit_defaults, dict):
        chase_area_limit_defaults = {}
    get_chase_area_limit_default = lambda key, fallback: chase_area_limit_defaults.get(
        key, fallback
    )

    launch_args = [
        DeclareLaunchArgument(
            "input_topic",
            default_value=str(get_default("input_topic", "/ly/navi/target_rel")),
        ),
        DeclareLaunchArgument(
            "output_goal_pos_topic",
            default_value=str(get_default("output_goal_pos_topic", "/ly/navi/goal_pos")),
        ),
        DeclareLaunchArgument(
            "output_goal_pose_topic",
            default_value=str(get_default("output_goal_pose_topic", "/goal_pose")),
        ),
        DeclareLaunchArgument(
            "input_goal_pos_raw_topic",
            default_value=str(get_default("input_goal_pos_raw_topic", "/ly/navi/goal_pos_raw")),
        ),
        DeclareLaunchArgument(
            "output_target_map_topic",
            default_value=str(get_default("output_target_map_topic", "/ly/navi/target_map")),
        ),
        DeclareLaunchArgument(
            "output_target_official_topic",
            default_value=str(
                get_default("output_target_official_topic", "/ly/navi/target_official")
            ),
        ),
        DeclareLaunchArgument(
            "output_navi_position_topic",
            default_value=str(get_default("output_navi_position_topic", "/ly/navi/position")),
        ),
        DeclareLaunchArgument("map_frame", default_value=str(get_default("map_frame", "map"))),
        DeclareLaunchArgument(
            "base_frame", default_value=str(get_default("base_frame", "base_link"))
        ),
        DeclareLaunchArgument(
            "fallback_base_frame",
            default_value=str(get_default("fallback_base_frame", "baselink")),
        ),
        DeclareLaunchArgument(
            "target_rel_default_frame",
            default_value=str(get_default("target_rel_default_frame", "gimbal_world")),
        ),
        DeclareLaunchArgument(
            "use_msg_frame_id",
            default_value=_bool_default(get_default("use_msg_frame_id", True)),
        ),
        DeclareLaunchArgument(
            "publish_target_map",
            default_value=_bool_default(get_default("publish_target_map", True)),
        ),
        DeclareLaunchArgument(
            "publish_target_official",
            default_value=_bool_default(get_default("publish_target_official", True)),
        ),
        DeclareLaunchArgument(
            "publish_navi_position",
            default_value=_bool_default(get_default("publish_navi_position", True)),
        ),
        DeclareLaunchArgument(
            "navi_position_publish_hz",
            default_value=str(float(get_default("navi_position_publish_hz", 10.0))),
        ),
        DeclareLaunchArgument(
            "publish_goal_pos",
            default_value=_bool_default(get_default("publish_goal_pos", False)),
        ),
        DeclareLaunchArgument(
            "publish_goal_pose",
            default_value=_bool_default(get_default("publish_goal_pose", True)),
        ),
        DeclareLaunchArgument(
            "invert_y_axis",
            default_value=_bool_default(get_default("invert_y_axis", False)),
        ),
        DeclareLaunchArgument(
            "y_axis_max_cm", default_value=str(int(get_default("y_axis_max_cm", 1500)))
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_enabled",
            default_value=_bool_default(get_default("goal_pos_uint16_encode_enabled", False)),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_x_scale",
            default_value=str(float(get_default("goal_pos_uint16_encode_x_scale", 1.0))),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_y_scale",
            default_value=str(float(get_default("goal_pos_uint16_encode_y_scale", 1.0))),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_x_offset_cm",
            default_value=str(float(get_default("goal_pos_uint16_encode_x_offset_cm", 0.0))),
        ),
        DeclareLaunchArgument(
            "goal_pos_uint16_encode_y_offset_cm",
            default_value=str(float(get_default("goal_pos_uint16_encode_y_offset_cm", 0.0))),
        ),
        DeclareLaunchArgument(
            "preferred_distance_cm",
            default_value=str(int(get_default("preferred_distance_cm", 100))),
        ),
        DeclareLaunchArgument(
            "distance_deadband_cm",
            default_value=str(int(get_default("distance_deadband_cm", 50))),
        ),
        DeclareLaunchArgument(
            "stop_when_no_target",
            default_value=_bool_default(get_default("stop_when_no_target", True)),
        ),
        DeclareLaunchArgument(
            "allow_reverse_goal",
            default_value=_bool_default(get_default("allow_reverse_goal", False)),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_enable",
            default_value=_bool_default(get_chase_area_limit_default("enable", False)),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_area_header_file",
            default_value=str(
                get_chase_area_limit_default("area_header_file", default_area_header_file)
                or default_area_header_file
            ),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_boundary_margin_cm",
            default_value=str(float(get_chase_area_limit_default("boundary_margin_cm", 30.0))),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_chase_enable_cross_area",
            default_value=_bool_default(
                get_chase_area_limit_default("chase_enable_cross_area", False)
            ),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_use_area_scope",
            default_value=_bool_default(get_chase_area_limit_default("use_area_scope", False)),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_my_area",
            default_value=str(get_chase_area_limit_default("my_area", "")),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_enemy_area",
            default_value=str(get_chase_area_limit_default("enemy_area", "")),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_common_area",
            default_value=str(get_chase_area_limit_default("common_area", "")),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_friend_is_team_red_topic",
            default_value=str(
                get_chase_area_limit_default("friend_is_team_red_topic", "/ly/friend/is_team_red")
            ),
        ),
        DeclareLaunchArgument(
            "chase_area_limit_hold_when_no_intersection",
            default_value=_bool_default(
                get_chase_area_limit_default("hold_when_no_intersection", True)
            ),
        ),
        DeclareLaunchArgument(
            "enable_goal_pos_raw_bridge",
            default_value=_bool_default(get_default("enable_goal_pos_raw_bridge", True)),
        ),
        DeclareLaunchArgument(
            "goal_pos_raw_frame",
            default_value=str(get_default("goal_pos_raw_frame", "map")),
        ),
        DeclareLaunchArgument(
            "use_raw_goal_static_calibration",
            default_value=_bool_default(get_default("use_raw_goal_static_calibration", False)),
        ),
        DeclareLaunchArgument(
            "raw_goal_calibration_model",
            default_value=str(get_default("raw_goal_calibration_model", "rigid")),
        ),
        DeclareLaunchArgument(
            "raw_goal_calibration_unit",
            default_value=str(get_default("raw_goal_calibration_unit", "cm")),
        ),
        DeclareLaunchArgument(
            "raw_goal_source_frame",
            default_value=str(get_default("raw_goal_source_frame", "official_map")),
        ),
        DeclareLaunchArgument(
            "raw_goal_target_frame",
            default_value=str(get_default("raw_goal_target_frame", "map")),
        ),
        DeclareLaunchArgument(
            "debug_export_point_pairs",
            default_value=_bool_default(get_default("debug_export_point_pairs", True)),
        ),
        DeclareLaunchArgument(
            "debug_points_reference_frame",
            default_value=str(get_default("debug_points_reference_frame", "map")),
        ),
        DeclareLaunchArgument(
            "debug_area_header_file",
            default_value=str(
                get_default("debug_area_header_file", default_area_header_file)
                or default_area_header_file
            ),
        ),
        DeclareLaunchArgument(
            "debug_point_pairs_output_file",
            default_value=str(
                get_default("debug_point_pairs_output_file", default_debug_pair_file)
                or default_debug_pair_file
            ),
        ),
    ]

    return LaunchDescription(
        launch_args
        + [
            Node(
                package="navi_tf_bridge",
                executable="target_rel_to_goal_pos_node",
                name="target_rel_to_goal_pos_node",
                output="screen",
                parameters=[
                    default_bridge_param_file,
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "input_goal_pos_raw_topic": LaunchConfiguration("input_goal_pos_raw_topic"),
                        "output_goal_pos_topic": LaunchConfiguration("output_goal_pos_topic"),
                        "output_goal_pose_topic": LaunchConfiguration("output_goal_pose_topic"),
                        "output_target_map_topic": LaunchConfiguration("output_target_map_topic"),
                        "output_target_official_topic": LaunchConfiguration(
                            "output_target_official_topic"
                        ),
                        "output_navi_position_topic": LaunchConfiguration("output_navi_position_topic"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "fallback_base_frame": LaunchConfiguration("fallback_base_frame"),
                        "target_rel_default_frame": LaunchConfiguration(
                            "target_rel_default_frame"
                        ),
                        "use_msg_frame_id": ParameterValue(
                            LaunchConfiguration("use_msg_frame_id"), value_type=bool
                        ),
                        "publish_target_map": ParameterValue(
                            LaunchConfiguration("publish_target_map"), value_type=bool
                        ),
                        "publish_target_official": ParameterValue(
                            LaunchConfiguration("publish_target_official"), value_type=bool
                        ),
                        "publish_navi_position": ParameterValue(
                            LaunchConfiguration("publish_navi_position"), value_type=bool
                        ),
                        "navi_position_publish_hz": ParameterValue(
                            LaunchConfiguration("navi_position_publish_hz"), value_type=float
                        ),
                        "publish_goal_pos": ParameterValue(
                            LaunchConfiguration("publish_goal_pos"), value_type=bool
                        ),
                        "publish_goal_pose": ParameterValue(
                            LaunchConfiguration("publish_goal_pose"), value_type=bool
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
                            LaunchConfiguration("goal_pos_uint16_encode_x_scale"),
                            value_type=float,
                        ),
                        "goal_pos_uint16_encode_y_scale": ParameterValue(
                            LaunchConfiguration("goal_pos_uint16_encode_y_scale"),
                            value_type=float,
                        ),
                        "goal_pos_uint16_encode_x_offset_cm": ParameterValue(
                            LaunchConfiguration("goal_pos_uint16_encode_x_offset_cm"),
                            value_type=float,
                        ),
                        "goal_pos_uint16_encode_y_offset_cm": ParameterValue(
                            LaunchConfiguration("goal_pos_uint16_encode_y_offset_cm"),
                            value_type=float,
                        ),
                        "preferred_distance_cm": ParameterValue(
                            LaunchConfiguration("preferred_distance_cm"), value_type=int
                        ),
                        "distance_deadband_cm": ParameterValue(
                            LaunchConfiguration("distance_deadband_cm"), value_type=int
                        ),
                        "stop_when_no_target": ParameterValue(
                            LaunchConfiguration("stop_when_no_target"), value_type=bool
                        ),
                        "allow_reverse_goal": ParameterValue(
                            LaunchConfiguration("allow_reverse_goal"), value_type=bool
                        ),
                        "chase_area_limit.enable": ParameterValue(
                            LaunchConfiguration("chase_area_limit_enable"), value_type=bool
                        ),
                        "chase_area_limit.area_header_file": LaunchConfiguration(
                            "chase_area_limit_area_header_file"
                        ),
                        "chase_area_limit.boundary_margin_cm": ParameterValue(
                            LaunchConfiguration("chase_area_limit_boundary_margin_cm"),
                            value_type=float,
                        ),
                        "chase_area_limit.chase_enable_cross_area": ParameterValue(
                            LaunchConfiguration("chase_area_limit_chase_enable_cross_area"),
                            value_type=bool,
                        ),
                        "chase_area_limit.use_area_scope": ParameterValue(
                            LaunchConfiguration("chase_area_limit_use_area_scope"),
                            value_type=bool,
                        ),
                        "chase_area_limit.my_area": LaunchConfiguration(
                            "chase_area_limit_my_area"
                        ),
                        "chase_area_limit.enemy_area": LaunchConfiguration(
                            "chase_area_limit_enemy_area"
                        ),
                        "chase_area_limit.common_area": LaunchConfiguration(
                            "chase_area_limit_common_area"
                        ),
                        "chase_area_limit.friend_is_team_red_topic": LaunchConfiguration(
                            "chase_area_limit_friend_is_team_red_topic"
                        ),
                        "chase_area_limit.hold_when_no_intersection": ParameterValue(
                            LaunchConfiguration("chase_area_limit_hold_when_no_intersection"),
                            value_type=bool,
                        ),
                        "enable_goal_pos_raw_bridge": ParameterValue(
                            LaunchConfiguration("enable_goal_pos_raw_bridge"), value_type=bool
                        ),
                        "goal_pos_raw_frame": LaunchConfiguration("goal_pos_raw_frame"),
                        "use_raw_goal_static_calibration": ParameterValue(
                            LaunchConfiguration("use_raw_goal_static_calibration"),
                            value_type=bool,
                        ),
                        "raw_goal_calibration_model": LaunchConfiguration(
                            "raw_goal_calibration_model"
                        ),
                        "raw_goal_calibration_unit": LaunchConfiguration(
                            "raw_goal_calibration_unit"
                        ),
                        "raw_goal_source_frame": LaunchConfiguration("raw_goal_source_frame"),
                        "raw_goal_target_frame": LaunchConfiguration("raw_goal_target_frame"),
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
        ]
    )
