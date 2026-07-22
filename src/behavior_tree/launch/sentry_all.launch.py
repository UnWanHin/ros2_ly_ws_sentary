#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
哨兵整链路启动入口（比赛/联调主入口）。

职责：
- 拉起 gimbal_driver / navi_tf_bridge / FaceMode / behavior_tree。
- Gimbal TF 由外部 sentry_tf 提供。
- 外部 aim 通过 /ly/aim/* 接入；本 launch 不再启动内部相机/辅瞄链。
- 支持通过 offline 参数统一覆盖“虚拟串口 + 视频回放”。

注意：
- behavior_tree 会接管 /ly/control/*，调试外部控制脚本时不要并行启动。
"""
import json
import os
from datetime import datetime
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _normalize_bool(raw: str) -> str:
    value = (raw or "").strip().lower()
    if value in ("1", "true", "yes", "on"):
        return "true"
    if value in ("0", "false", "no", "off"):
        return "false"
    return ""


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


def _config_bool(raw, default: bool) -> bool:
    if isinstance(raw, bool):
        return raw
    normalized = _normalize_bool(str(raw))
    if normalized:
        return normalized == "true"
    return default


def _regional_area_scope_from_yaml(config_path: Path) -> tuple[list[str], list[str]]:
    with open(config_path, encoding="utf-8") as fh:
        document = yaml.safe_load(fh)
    if not isinstance(document, dict):
        raise ValueError("root must be a mapping")

    node_config = document.get("behavior_tree", document.get("/**", {}))
    if not isinstance(node_config, dict):
        raise ValueError("behavior_tree must be a mapping")
    parameters = node_config.get("ros__parameters", {})
    if not isinstance(parameters, dict):
        raise ValueError("ros__parameters must be a mapping")
    area_manager = parameters.get("AreaManager", {})
    if not isinstance(area_manager, dict):
        raise ValueError("AreaManager must be a mapping")
    regional_task = area_manager.get("RegionalAreaTask", {})
    if not isinstance(regional_task, dict):
        raise ValueError("AreaManager.RegionalAreaTask must be a mapping")

    if "Enable" not in regional_task:
        raise ValueError("AreaManager.RegionalAreaTask.Enable is required")
    if not _config_bool(regional_task["Enable"], True):
        return [], []

    def enabled(name: str) -> bool:
        setting = regional_task.get(name, {})
        if not isinstance(setting, dict) or "Enable" not in setting:
            raise ValueError(f"AreaManager.RegionalAreaTask.{name}.Enable is required")
        return _config_bool(setting["Enable"], True)

    my_area = []
    if enabled("MyBase"):
        my_area.append("base")
    if enabled("MyHighland"):
        my_area.append("highland")
    if enabled("MyPreRoadland"):
        my_area.append("pre_roadland")
    if enabled("MyReadyRoadland"):
        my_area.append("ready_roadland")
    common_area = ["central"] if enabled("CommonCentral") else []
    return my_area, common_area


def generate_launch_description():
    def normalize_mode(raw: str) -> str:
        normalized = (raw or "").strip().lower()
        if normalized in ("1", "league"):
            return "league"
        if normalized in ("regional_simple", "regional-simple", "simple"):
            return "regional_simple"
        if normalized in ("3", "showcase", "demo"):
            return "showcase"
        return "regional"

    def resolve_mode_defaults(context):
        mode_raw = LaunchConfiguration("mode").perform(context)
        competition_profile_raw = LaunchConfiguration("competition_profile").perform(context).strip()
        bt_config_file_raw = LaunchConfiguration("bt_config_file").perform(context).strip()

        if mode_raw.strip():
            mode_kind = normalize_mode(mode_raw)
        elif bt_config_file_raw.endswith("regional_simple_competition.json"):
            mode_kind = "regional_simple"
        elif bt_config_file_raw.endswith("showcase_competition.json"):
            mode_kind = "showcase"
        elif competition_profile_raw.strip().lower() == "league":
            mode_kind = "league"
        else:
            mode_kind = "regional"

        resolved_profile = competition_profile_raw or ("league" if mode_kind == "league" else "regional")
        if bt_config_file_raw:
            resolved_bt_config = bt_config_file_raw
        elif mode_kind == "league":
            resolved_bt_config = "Scripts/ConfigJson/league_competition.json"
        elif mode_kind == "regional_simple":
            resolved_bt_config = "Scripts/ConfigJson/regional_simple_competition.json"
        elif mode_kind == "showcase":
            resolved_bt_config = "Scripts/ConfigJson/regional/debug/showcase_competition.json"
        else:
            resolved_bt_config = "Scripts/ConfigJson/regional_competition.json"

        return [
            SetLaunchConfiguration("resolved_mode_kind", mode_kind),
            SetLaunchConfiguration("resolved_competition_profile", resolved_profile),
            SetLaunchConfiguration("resolved_bt_config_file", resolved_bt_config),
        ]

    def resolve_rosbag_defaults(context):
        rosbag_path_raw = LaunchConfiguration("rosbag_path").perform(context).strip()
        rosbag_base_dir = os.path.expanduser(rosbag_path_raw or "~/Log/rosbag")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        resolved_rosbag_path = os.path.join(
            rosbag_base_dir,
            f"sentry_all_{timestamp}_{os.getpid()}",
        )
        return [
            SetLaunchConfiguration("resolved_rosbag_base_dir", rosbag_base_dir),
            SetLaunchConfiguration("resolved_rosbag_path", resolved_rosbag_path),
        ]

    def resolve_navi_tf_bridge_defaults(context):
        use_navi_tf_bridge_override = _normalize_bool(
            LaunchConfiguration("use_navi_tf_bridge").perform(context)
        )
        resolved_use_navi_tf_bridge = "true"
        resolved_chase_preferred_distance_cm = "300"
        resolved_chase_distance_deadband_cm = "50"
        resolved_chase_stop_when_no_target = "true"
        resolved_chase_area_limit_enable = "false"
        resolved_chase_area_limit_boundary_margin_cm = "30.0"
        resolved_chase_area_limit_chase_enable_cross_area = "false"
        resolved_chase_area_limit_use_area_scope = "false"
        resolved_chase_area_limit_my_area = ""
        resolved_chase_area_limit_enemy_area = ""
        resolved_chase_area_limit_common_area = ""
        resolved_chase_area_limit_hold_when_no_intersection = "true"
        bt_config_file_raw = LaunchConfiguration("resolved_bt_config_file").perform(context).strip()
        bt_config_path = Path(bt_config_file_raw)
        if bt_config_file_raw and not bt_config_path.is_absolute():
            bt_config_path = Path(behavior_tree_share) / bt_config_path
        if bt_config_file_raw and bt_config_path.exists():
            try:
                with open(bt_config_path, encoding="utf-8") as fh:
                    root = json.load(fh)
                navi_cfg = root.get("NaviSetting", {})
                if isinstance(navi_cfg, dict):
                    to_navi = navi_cfg.get("ToNavi", navi_cfg.get("UseTfGoalBridge", True))
                    resolved_use_navi_tf_bridge = (
                        "true" if bool(to_navi) else "false"
                    )
                chase_cfg = root.get("Chase", {})
                if isinstance(chase_cfg, dict):
                    resolved_chase_preferred_distance_cm = str(
                        int(chase_cfg.get("PreferredDistanceCm", int(resolved_chase_preferred_distance_cm)))
                    )
                    resolved_chase_distance_deadband_cm = str(
                        int(chase_cfg.get("DistanceDeadbandCm", int(resolved_chase_distance_deadband_cm)))
                    )
                    resolved_chase_stop_when_no_target = (
                        "true" if bool(chase_cfg.get("StopWhenNoTarget", True)) else "false"
                    )
                    area_limit_cfg = chase_cfg.get("AreaLimit", {})
                    if isinstance(area_limit_cfg, dict):
                        resolved_chase_area_limit_enable = (
                            "true" if bool(area_limit_cfg.get("Enable", False)) else "false"
                        )
                        resolved_chase_area_limit_boundary_margin_cm = str(
                            float(area_limit_cfg.get("BoundaryMarginCm", 30.0))
                        )
                        resolved_chase_area_limit_chase_enable_cross_area = (
                            "true"
                            if bool(area_limit_cfg.get("ChaseEnableCrossArea", False))
                            else "false"
                        )
                        resolved_chase_area_limit_hold_when_no_intersection = (
                            "true"
                            if bool(area_limit_cfg.get("HoldWhenNoIntersection", True))
                            else "false"
                        )
                    autonomy_cfg = root.get("DecisionAutonomy", {})
                    if isinstance(autonomy_cfg, dict):
                        navi_goal_cfg = autonomy_cfg.get("NaviGoal", {})
                        if isinstance(navi_goal_cfg, dict):
                            resolved_chase_area_limit_use_area_scope = (
                                "true" if bool(navi_goal_cfg.get("UseAreaScope", False)) else "false"
                            )
                            resolved_chase_area_limit_my_area = _area_scope_csv(
                                navi_goal_cfg.get("MyArea", {})
                            )
                            resolved_chase_area_limit_enemy_area = _area_scope_csv(
                                navi_goal_cfg.get("EnemyArea", {})
                            )
                            resolved_chase_area_limit_common_area = _area_scope_csv(
                                navi_goal_cfg.get("CommonArea", {})
                            )
            except Exception as ex:
                print(
                    f"[sentry_all] failed to parse bt_config_file '{bt_config_path}': {ex}. "
                    "navi_tf_bridge falls back to enabled."
                )
        if use_navi_tf_bridge_override:
            resolved_use_navi_tf_bridge = use_navi_tf_bridge_override

        if resolved_chase_area_limit_use_area_scope == "true":
            area_manager_config_raw = LaunchConfiguration(
                "area_manager_config_file"
            ).perform(context).strip()
            area_manager_config_path = Path(os.path.expanduser(area_manager_config_raw))
            try:
                my_area, common_area = _regional_area_scope_from_yaml(
                    area_manager_config_path
                )
            except Exception as ex:
                raise RuntimeError(
                    f"[sentry_all] failed to parse area_manager_config_file "
                    f"'{area_manager_config_path}': {ex}"
                ) from ex
            resolved_chase_area_limit_my_area = ",".join(my_area)
            resolved_chase_area_limit_common_area = ",".join(common_area)

        return [
            SetLaunchConfiguration("resolved_use_navi_tf_bridge", resolved_use_navi_tf_bridge),
            SetLaunchConfiguration(
                "resolved_chase_preferred_distance_cm",
                resolved_chase_preferred_distance_cm,
            ),
            SetLaunchConfiguration(
                "resolved_chase_distance_deadband_cm",
                resolved_chase_distance_deadband_cm,
            ),
            SetLaunchConfiguration(
                "resolved_chase_stop_when_no_target",
                resolved_chase_stop_when_no_target,
            ),
            SetLaunchConfiguration("resolved_chase_area_limit_enable", resolved_chase_area_limit_enable),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_boundary_margin_cm",
                resolved_chase_area_limit_boundary_margin_cm,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_chase_enable_cross_area",
                resolved_chase_area_limit_chase_enable_cross_area,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_use_area_scope",
                resolved_chase_area_limit_use_area_scope,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_my_area",
                resolved_chase_area_limit_my_area,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_enemy_area",
                resolved_chase_area_limit_enemy_area,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_common_area",
                resolved_chase_area_limit_common_area,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_hold_when_no_intersection",
                resolved_chase_area_limit_hold_when_no_intersection,
            ),
        ]

    # 分层配置默认入口：
    #   base + module + optional global override(config_file)
    behavior_tree_share = get_package_share_directory("behavior_tree")
    gimbal_driver_share = get_package_share_directory("gimbal_driver")
    behavior_tree_config_root = os.path.join(behavior_tree_share, "config")
    gimbal_driver_launch_file = os.path.join(
        gimbal_driver_share, "launch", "gimbal_driver.launch.py"
    )
    navi_tf_bridge_launch_file = PathJoinSubstitution([
        FindPackageShare("navi_tf_bridge"),
        "launch",
        "target_rel_to_goal_pos.launch.py",
    ])
    face_mode_solver_bridge_config_file = PathJoinSubstitution([
        FindPackageShare("navi_tf_bridge"),
        "config",
        "tf_config.yaml",
    ])
    default_base_config_file = os.path.join(behavior_tree_config_root, "base_config.yaml")
    default_gimbal_driver_config_file = os.path.join(
        gimbal_driver_share, "config", "gimbal_driver_config.yaml"
    )
    default_override_config_file = os.path.join(behavior_tree_config_root, "override_config.yaml")
    default_area_manager_config_file = os.path.join(behavior_tree_config_root, "AreaManager.yaml")
    default_task_config_file = os.path.join(behavior_tree_config_root, "Task.yaml")
    default_chase_config_file = os.path.join(behavior_tree_config_root, "Chase.yaml")
    default_navi_rotate_config_file = os.path.join(behavior_tree_config_root, "NaviRotateControl.yaml")
    default_tactical_config_file = os.path.join(behavior_tree_config_root, "Tactical.yaml")
    default_patrol_config_file = os.path.join(behavior_tree_config_root, "Patrol.yaml")
    default_special_config_file = os.path.join(behavior_tree_config_root, "Special.yaml")

    mode = LaunchConfiguration("mode")
    config_file = LaunchConfiguration("config_file")
    area_manager_config_file = LaunchConfiguration("area_manager_config_file")
    task_config_file = LaunchConfiguration("task_config_file")
    chase_config_file = LaunchConfiguration("chase_config_file")
    navi_rotate_config_file = LaunchConfiguration("navi_rotate_config_file")
    tactical_config_file = LaunchConfiguration("tactical_config_file")
    patrol_config_file = LaunchConfiguration("patrol_config_file")
    special_config_file = LaunchConfiguration("special_config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    gimbal_driver_config_file = LaunchConfiguration("gimbal_driver_config_file")
    output = LaunchConfiguration("output")
    competition_profile = LaunchConfiguration("competition_profile")
    bt_config_file = LaunchConfiguration("bt_config_file")
    bt_tree_file = LaunchConfiguration("bt_tree_file")
    debug_bypass_is_start = LaunchConfiguration("debug_bypass_is_start")
    runtime_rearm_start_gate = LaunchConfiguration("runtime_rearm_start_gate")
    publish_navi_goal = LaunchConfiguration("publish_navi_goal")
    navi_publish_goal_pose = LaunchConfiguration("navi_publish_goal_pose")
    wait_for_game_start_timeout_sec = LaunchConfiguration("wait_for_game_start_timeout_sec")
    league_referee_stale_timeout_ms = LaunchConfiguration("league_referee_stale_timeout_ms")
    start_gate_allow_gimbal_patrol_before_start = LaunchConfiguration(
        "start_gate_allow_gimbal_patrol_before_start"
    )
    firecode_partial_hold_ms = LaunchConfiguration("firecode_partial_hold_ms")
    velocity_raw_to_mps = LaunchConfiguration("velocity_raw_to_mps")
    face_mode_target_frame = LaunchConfiguration("face_mode_target_frame")
    face_mode_use_raw_goal_static_calibration = LaunchConfiguration(
        "face_mode_use_raw_goal_static_calibration"
    )
    face_mode_raw_goal_target_frame = LaunchConfiguration("face_mode_raw_goal_target_frame")
    face_mode_manual_target_enable = LaunchConfiguration("face_mode_manual_target_enable")
    face_mode_manual_target_frame = LaunchConfiguration("face_mode_manual_target_frame")
    face_mode_manual_target_x_m = LaunchConfiguration("face_mode_manual_target_x_m")
    face_mode_manual_target_y_m = LaunchConfiguration("face_mode_manual_target_y_m")
    face_mode_manual_target_z_m = LaunchConfiguration("face_mode_manual_target_z_m")
    face_mode_max_yaw_step_deg = LaunchConfiguration("face_mode_max_yaw_step_deg")
    face_mode_max_pitch_step_deg = LaunchConfiguration("face_mode_max_pitch_step_deg")
    outpost_manual_goal_enable = LaunchConfiguration("outpost_manual_goal_enable")
    outpost_manual_goal_x_m = LaunchConfiguration("outpost_manual_goal_x_m")
    outpost_manual_goal_y_m = LaunchConfiguration("outpost_manual_goal_y_m")
    outpost_manual_goal_z_m = LaunchConfiguration("outpost_manual_goal_z_m")
    gimbal_raw_log_enable = LaunchConfiguration("gimbal_raw_log_enable")
    gimbal_raw_log_uplink = LaunchConfiguration("gimbal_raw_log_uplink")
    gimbal_raw_log_downlink = LaunchConfiguration("gimbal_raw_log_downlink")
    gimbal_raw_log_screen = LaunchConfiguration("gimbal_raw_log_screen")
    gimbal_raw_log_flush = LaunchConfiguration("gimbal_raw_log_flush")
    gimbal_raw_log_dir = LaunchConfiguration("gimbal_raw_log_dir")
    gimbal_raw_log_type_ids = LaunchConfiguration("gimbal_raw_log_type_ids")
    gimbal_raw_topic_enable = LaunchConfiguration("gimbal_raw_topic_enable")
    gimbal_raw_topic_uplink = LaunchConfiguration("gimbal_raw_topic_uplink")
    gimbal_raw_topic_downlink = LaunchConfiguration("gimbal_raw_topic_downlink")
    gimbal_raw_topic_type_ids = LaunchConfiguration("gimbal_raw_topic_type_ids")
    decision_trace_enabled = LaunchConfiguration("decision_trace_enabled")
    decision_trace_file = LaunchConfiguration("decision_trace_file")
    decision_trace_every_n_ticks = LaunchConfiguration("decision_trace_every_n_ticks")
    rosbag = LaunchConfiguration("rosbag")
    rosbag_path = LaunchConfiguration("rosbag_path")
    resolved_rosbag_path = LaunchConfiguration("resolved_rosbag_path")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    use_navi_tf_bridge = LaunchConfiguration("use_navi_tf_bridge")
    use_face_mode_solver = LaunchConfiguration("use_face_mode_solver")
    resolved_use_navi_tf_bridge = LaunchConfiguration("resolved_use_navi_tf_bridge")
    resolved_chase_preferred_distance_cm = LaunchConfiguration(
        "resolved_chase_preferred_distance_cm"
    )
    resolved_chase_distance_deadband_cm = LaunchConfiguration(
        "resolved_chase_distance_deadband_cm"
    )
    resolved_chase_stop_when_no_target = LaunchConfiguration(
        "resolved_chase_stop_when_no_target"
    )
    resolved_chase_area_limit_enable = LaunchConfiguration("resolved_chase_area_limit_enable")
    resolved_chase_area_limit_boundary_margin_cm = LaunchConfiguration(
        "resolved_chase_area_limit_boundary_margin_cm"
    )
    resolved_chase_area_limit_chase_enable_cross_area = LaunchConfiguration(
        "resolved_chase_area_limit_chase_enable_cross_area"
    )
    resolved_chase_area_limit_use_area_scope = LaunchConfiguration(
        "resolved_chase_area_limit_use_area_scope"
    )
    resolved_chase_area_limit_my_area = LaunchConfiguration(
        "resolved_chase_area_limit_my_area"
    )
    resolved_chase_area_limit_enemy_area = LaunchConfiguration(
        "resolved_chase_area_limit_enemy_area"
    )
    resolved_chase_area_limit_common_area = LaunchConfiguration(
        "resolved_chase_area_limit_common_area"
    )
    resolved_chase_area_limit_hold_when_no_intersection = LaunchConfiguration(
        "resolved_chase_area_limit_hold_when_no_intersection"
    )
    offline = LaunchConfiguration("offline")
    resolved_mode_kind = LaunchConfiguration("resolved_mode_kind")
    resolved_competition_profile = LaunchConfiguration("resolved_competition_profile")
    resolved_bt_config_file = LaunchConfiguration("resolved_bt_config_file")

    launch_args = [
        DeclareLaunchArgument(
            "mode",
            default_value="",
            description="Startup mode: league/regional/regional_simple/showcase. Empty falls back to regional unless overridden.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_override_config_file,
            description="Optional global override YAML (applied last to all nodes).",
        ),
        DeclareLaunchArgument(
            "base_config_file",
            default_value=default_base_config_file,
            description="Base YAML for cross-module shared parameters.",
        ),
        DeclareLaunchArgument(
            "gimbal_driver_config_file",
            default_value=default_gimbal_driver_config_file,
            description="Baseline serial/lower-machine YAML for gimbal_driver.",
        ),
        DeclareLaunchArgument(
            "area_manager_config_file",
            default_value=default_area_manager_config_file,
            description="AreaManager/state-machine YAML for behavior_tree.",
        ),
        DeclareLaunchArgument(
            "task_config_file",
            default_value=default_task_config_file,
            description="Task YAML for behavior_tree Buff/Outpost enable switches.",
        ),
        DeclareLaunchArgument(
            "chase_config_file",
            default_value=default_chase_config_file,
            description="Regional Chase ownership YAML for behavior_tree.",
        ),
        DeclareLaunchArgument(
            "navi_rotate_config_file",
            default_value=default_navi_rotate_config_file,
            description="External navigation rotate/follow compatibility YAML for behavior_tree.",
        ),
        DeclareLaunchArgument(
            "tactical_config_file",
            default_value=default_tactical_config_file,
            description="Tactical damage Rotate policy YAML for behavior_tree.",
        ),
        DeclareLaunchArgument(
            "patrol_config_file",
            default_value=default_patrol_config_file,
            description="Gimbal patrol scan YAML for behavior_tree.",
        ),
        DeclareLaunchArgument(
            "special_config_file",
            default_value=default_special_config_file,
            description="Special strategy layer YAML for behavior_tree.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS node output mode: screen or log.",
        ),
        DeclareLaunchArgument(
            "competition_profile",
            default_value="",
            description="behavior_tree profile override: regional or league. Empty keeps config/default.",
        ),
        DeclareLaunchArgument(
            "bt_config_file",
            default_value="",
            description="Optional behavior_tree JSON config path. Relative paths resolve under behavior_tree share dir.",
        ),
        DeclareLaunchArgument(
            "bt_tree_file",
            default_value="",
            description="Optional behavior_tree XML path. Relative paths resolve under behavior_tree share dir.",
        ),
        DeclareLaunchArgument(
            "debug_bypass_is_start",
            default_value="false",
            description="Debug only: true will bypass waiting /ly/game/is_start gate.",
        ),
        DeclareLaunchArgument(
            "runtime_rearm_start_gate",
            default_value="false",
            description="Debug only: when true, game loop re-enters start gate if /ly/game/is_start becomes false.",
        ),
        DeclareLaunchArgument(
            "publish_navi_goal",
            default_value="true",
            description="Whether behavior_tree publishes navigation goal inputs; tf bridge outputs /goal_pose when enabled.",
        ),
        DeclareLaunchArgument(
            "navi_publish_goal_pose",
            default_value="true",
            description="Whether navi_tf_bridge publishes geometry_msgs/PoseStamped to /goal_pose.",
        ),
        DeclareLaunchArgument(
            "wait_for_game_start_timeout_sec",
            default_value="0",
            description="0 disables timeout. >0 continues after waiting this many seconds for /ly/game/is_start.",
        ),
        DeclareLaunchArgument(
            "league_referee_stale_timeout_ms",
            default_value="0",
            description="0 disables stale-check. >0 enables league referee freshness guard for HP/Ammo recovery.",
        ),
        DeclareLaunchArgument(
            "start_gate_allow_gimbal_patrol_before_start",
            default_value="true",
            description="Allow gimbal patrol scan while gated before /ly/game/is_start=true.",
        ),
        DeclareLaunchArgument(
            "firecode_partial_hold_ms",
            default_value="100",
            description="gimbal_driver FireCode partial-field hold time before stale fields degrade to 0.",
        ),
        DeclareLaunchArgument(
            "velocity_raw_to_mps",
            default_value="0.025",
            description="gimbal_driver scale from lower raw int8 velocity to m/s.",
        ),
        DeclareLaunchArgument(
            "face_mode_max_yaw_step_deg",
            default_value="40.0",
            description="FaceMode max yaw command change per publish. 0 disables step limiting.",
        ),
        DeclareLaunchArgument(
            "face_mode_max_pitch_step_deg",
            default_value="0.0",
            description="FaceMode max pitch command change per publish. 0 disables step limiting.",
        ),
        DeclareLaunchArgument(
            "face_mode_target_frame",
            default_value="official_map",
            description="Frame used by FaceMode target_raw when static calibration is disabled.",
        ),
        DeclareLaunchArgument(
            "face_mode_use_raw_goal_static_calibration",
            default_value="true",
            description="Apply official_map -> map raw-goal calibration inside FaceMode solver.",
        ),
        DeclareLaunchArgument(
            "face_mode_raw_goal_target_frame",
            default_value="map",
            description="Target frame after FaceMode raw-goal static calibration.",
        ),
        DeclareLaunchArgument(
            "face_mode_manual_target_enable",
            default_value="false",
            description="Use signed map-frame manual target directly inside FaceMode solver.",
        ),
        DeclareLaunchArgument(
            "face_mode_manual_target_frame",
            default_value="map",
            description="Frame for signed FaceMode manual target.",
        ),
        DeclareLaunchArgument("face_mode_manual_target_x_m", default_value="0.0"),
        DeclareLaunchArgument("face_mode_manual_target_y_m", default_value="0.0"),
        DeclareLaunchArgument("face_mode_manual_target_z_m", default_value="0.0"),
        DeclareLaunchArgument(
            "outpost_manual_goal_enable",
            default_value="false",
            description="Publish signed map-frame /goal_pose for Outpost test instead of raw UInt16 point.",
        ),
        DeclareLaunchArgument("outpost_manual_goal_x_m", default_value="0.0"),
        DeclareLaunchArgument("outpost_manual_goal_y_m", default_value="0.0"),
        DeclareLaunchArgument("outpost_manual_goal_z_m", default_value="0.0"),
        DeclareLaunchArgument(
            "gimbal_raw_log_enable",
            default_value="false",
            description="Enable gimbal_driver raw serial rx/tx file log.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_log_uplink",
            default_value="true",
            description="Log lower -> upper TypedMessage raw frames when gimbal_raw_log_enable is true.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_log_downlink",
            default_value="true",
            description="Log upper -> lower downlink raw frames when gimbal_raw_log_enable is true.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_log_screen",
            default_value="false",
            description="Also print raw serial log lines to ROS screen output.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_log_flush",
            default_value="true",
            description="Flush gimbal raw serial log file after each line.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_log_dir",
            default_value="~/Log/GimbalRaw",
            description="Directory for gimbal_driver raw serial log files.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_log_type_ids",
            default_value="all",
            description="Comma-separated uplink TypeID list, or all. Downlink is a single control frame.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_topic_enable",
            default_value="false",
            description="Enable binary gimbal_driver raw serial ROS2 topics.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_topic_uplink",
            default_value="true",
            description="Publish lower -> upper raw TypeID frames to /ly/log/gimbal_raw_rx.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_topic_downlink",
            default_value="true",
            description="Publish upper -> lower raw control frames to /ly/log/gimbal_raw_tx.",
        ),
        DeclareLaunchArgument(
            "gimbal_raw_topic_type_ids",
            default_value="all",
            description="Comma-separated uplink TypeID list for raw ROS2 topic, or all.",
        ),
        DeclareLaunchArgument(
            "rosbag",
            default_value="false",
            description="When true, launch ros2 bag record -a for recording all topics.",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/Log/rosbag",
            description="Base directory for timestamped rosbag recordings when rosbag is true.",
        ),
        DeclareLaunchArgument(
            "decision_trace_enabled",
            default_value="false",
            description="Debug only: enable JSONL decision trace for offline pygame replay.",
        ),
        DeclareLaunchArgument(
            "decision_trace_file",
            default_value="",
            description="JSONL decision trace output path. Used only when decision_trace_enabled is true.",
        ),
        DeclareLaunchArgument(
            "decision_trace_every_n_ticks",
            default_value="5",
            description="Write one decision trace record every N behavior_tree ticks when tracing is enabled.",
        ),
        DeclareLaunchArgument("use_gimbal", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "use_navi_tf_bridge",
            default_value="",
            description="Optional override. Empty means load NaviSetting.ToNavi from bt_config_file.",
        ),
        DeclareLaunchArgument(
            "use_face_mode_solver",
            default_value="true",
            description="Launch map_aim_point_node in BT FaceMode mode: /ly/face_mode/target_raw -> /ly/face_mode/angles.",
        ),
        DeclareLaunchArgument(
            "offline",
            default_value="false",
            description="Offline profile: force virtual IO and video replay without editing YAML.",
        ),
        DeclareLaunchArgument("resolved_mode_kind", default_value=""),
        DeclareLaunchArgument("resolved_competition_profile", default_value=""),
        DeclareLaunchArgument("resolved_bt_config_file", default_value=""),
        DeclareLaunchArgument("resolved_rosbag_base_dir", default_value=""),
        DeclareLaunchArgument("resolved_rosbag_path", default_value=""),
        DeclareLaunchArgument("resolved_use_navi_tf_bridge", default_value="true"),
        DeclareLaunchArgument("resolved_chase_preferred_distance_cm", default_value="300"),
        DeclareLaunchArgument("resolved_chase_distance_deadband_cm", default_value="50"),
        DeclareLaunchArgument("resolved_chase_stop_when_no_target", default_value="true"),
        DeclareLaunchArgument("resolved_chase_area_limit_enable", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_boundary_margin_cm", default_value="30.0"),
        DeclareLaunchArgument("resolved_chase_area_limit_chase_enable_cross_area", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_use_area_scope", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_my_area", default_value=""),
        DeclareLaunchArgument("resolved_chase_area_limit_enemy_area", default_value=""),
        DeclareLaunchArgument("resolved_chase_area_limit_common_area", default_value=""),
        DeclareLaunchArgument("resolved_chase_area_limit_hold_when_no_intersection", default_value="true"),
        OpaqueFunction(function=resolve_mode_defaults),
        OpaqueFunction(function=resolve_rosbag_defaults),
        OpaqueFunction(function=resolve_navi_tf_bridge_defaults),
    ]

    truthy_values = "['true', '1', 'yes', 'on']"
    rosbag_enabled_expr = PythonExpression([
        "'", rosbag, "'.lower() in ", truthy_values
    ])
    gimbal_use_virtual_device = PythonExpression([
        "'", offline, "'.lower() in ", truthy_values
    ])
    external_aim_log = PythonExpression([
        "'offline mock: not required' if '", offline, "'.lower() in ", truthy_values,
        " else 'required (/ly/aim/*)'",
    ])
    effective_navi_publish_goal_pose = PythonExpression([
        "'false' if '", outpost_manual_goal_enable, "'.lower() in ", truthy_values,
        " else '", navi_publish_goal_pose, "'",
    ])
    info_logs = [
        LogInfo(msg=["[sentry_all] mode: ", mode]),
        LogInfo(msg=["[sentry_all] config: ", config_file]),
        LogInfo(msg=["[sentry_all] base_config: ", base_config_file]),
        LogInfo(msg=["[sentry_all] gimbal_driver_config: ", gimbal_driver_config_file]),
        LogInfo(msg=["[sentry_all] area_manager_config: ", area_manager_config_file]),
        LogInfo(msg=["[sentry_all] task_config: ", task_config_file]),
        LogInfo(msg=["[sentry_all] chase_config: ", chase_config_file]),
        LogInfo(msg=["[sentry_all] navi_rotate_config: ", navi_rotate_config_file]),
        LogInfo(msg=["[sentry_all] tactical_config: ", tactical_config_file]),
        LogInfo(msg=["[sentry_all] patrol_config: ", patrol_config_file]),
        LogInfo(msg=["[sentry_all] special_config: ", special_config_file]),
        LogInfo(msg=["[sentry_all] output: ", output]),
        LogInfo(msg=["[sentry_all] offline: ", offline]),
        LogInfo(msg=["[sentry_all] external aim: ", external_aim_log]),
        LogInfo(msg=["[sentry_all] competition_profile: ", competition_profile]),
        LogInfo(msg=["[sentry_all] bt_config_file: ", bt_config_file]),
        LogInfo(msg=["[sentry_all] resolved_mode: ", resolved_mode_kind]),
        LogInfo(msg=["[sentry_all] resolved_competition_profile: ", resolved_competition_profile]),
        LogInfo(msg=["[sentry_all] resolved_bt_config_file: ", resolved_bt_config_file]),
        LogInfo(msg=["[sentry_all] bt_tree_file: ", bt_tree_file]),
        LogInfo(msg=["[sentry_all] debug_bypass_is_start: ", debug_bypass_is_start]),
        LogInfo(msg=["[sentry_all] runtime_rearm_start_gate: ", runtime_rearm_start_gate]),
        LogInfo(msg=["[sentry_all] publish_navi_goal: ", publish_navi_goal]),
        LogInfo(msg=["[sentry_all] navi_publish_goal_pose: ", navi_publish_goal_pose]),
        LogInfo(msg=[
            "[sentry_all] effective_navi_publish_goal_pose: ",
            effective_navi_publish_goal_pose,
        ]),
        LogInfo(msg=["[sentry_all] wait_for_game_start_timeout_sec: ", wait_for_game_start_timeout_sec]),
        LogInfo(msg=["[sentry_all] league_referee_stale_timeout_ms: ", league_referee_stale_timeout_ms]),
        LogInfo(msg=[
            "[sentry_all] start_gate_allow_gimbal_patrol_before_start: ",
            start_gate_allow_gimbal_patrol_before_start,
        ]),
        LogInfo(msg=["[sentry_all] firecode_partial_hold_ms: ", firecode_partial_hold_ms]),
        LogInfo(msg=["[sentry_all] velocity_raw_to_mps: ", velocity_raw_to_mps]),
        LogInfo(msg=["[sentry_all] face_mode_max_yaw_step_deg: ", face_mode_max_yaw_step_deg]),
        LogInfo(msg=["[sentry_all] face_mode_max_pitch_step_deg: ", face_mode_max_pitch_step_deg]),
        LogInfo(msg=["[sentry_all] face_mode_target_frame: ", face_mode_target_frame]),
        LogInfo(msg=[
            "[sentry_all] face_mode_use_raw_goal_static_calibration: ",
            face_mode_use_raw_goal_static_calibration,
        ]),
        LogInfo(msg=["[sentry_all] face_mode_raw_goal_target_frame: ", face_mode_raw_goal_target_frame]),
        LogInfo(msg=["[sentry_all] face_mode_manual_target_enable: ", face_mode_manual_target_enable]),
        LogInfo(msg=["[sentry_all] outpost_manual_goal_enable: ", outpost_manual_goal_enable]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_enable: ", gimbal_raw_log_enable]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_uplink: ", gimbal_raw_log_uplink]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_downlink: ", gimbal_raw_log_downlink]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_screen: ", gimbal_raw_log_screen]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_flush: ", gimbal_raw_log_flush]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_dir: ", gimbal_raw_log_dir]),
        LogInfo(msg=["[sentry_all] gimbal_raw_log_type_ids: ", gimbal_raw_log_type_ids]),
        LogInfo(msg=["[sentry_all] gimbal_raw_topic_enable: ", gimbal_raw_topic_enable]),
        LogInfo(msg=["[sentry_all] gimbal_raw_topic_uplink: ", gimbal_raw_topic_uplink]),
        LogInfo(msg=["[sentry_all] gimbal_raw_topic_downlink: ", gimbal_raw_topic_downlink]),
        LogInfo(msg=["[sentry_all] gimbal_raw_topic_type_ids: ", gimbal_raw_topic_type_ids]),
        LogInfo(msg=["[sentry_all] rosbag: ", rosbag]),
        LogInfo(msg=["[sentry_all] rosbag_path: ", rosbag_path]),
        LogInfo(msg=["[sentry_all] resolved_rosbag_path: ", resolved_rosbag_path]),
        LogInfo(msg=["[sentry_all] decision_trace_enabled: ", decision_trace_enabled]),
        LogInfo(msg=["[sentry_all] decision_trace_file: ", decision_trace_file]),
        LogInfo(msg=["[sentry_all] decision_trace_every_n_ticks: ", decision_trace_every_n_ticks]),
        LogInfo(msg=["[sentry_all] use_navi_tf_bridge: ", use_navi_tf_bridge]),
        LogInfo(msg=["[sentry_all] use_face_mode_solver: ", use_face_mode_solver]),
        LogInfo(msg=["[sentry_all] resolved_use_navi_tf_bridge: ", resolved_use_navi_tf_bridge]),
        LogInfo(msg=[
            "[sentry_all] chase_preferred_distance_cm: ",
            resolved_chase_preferred_distance_cm,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_distance_deadband_cm: ",
            resolved_chase_distance_deadband_cm,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_stop_when_no_target: ",
            resolved_chase_stop_when_no_target,
        ]),
        LogInfo(msg=["[sentry_all] chase_area_limit_enable: ", resolved_chase_area_limit_enable]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_boundary_margin_cm: ",
            resolved_chase_area_limit_boundary_margin_cm,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_chase_enable_cross_area: ",
            resolved_chase_area_limit_chase_enable_cross_area,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_use_area_scope: ",
            resolved_chase_area_limit_use_area_scope,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_my_area: ",
            resolved_chase_area_limit_my_area,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_enemy_area: ",
            resolved_chase_area_limit_enemy_area,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_common_area: ",
            resolved_chase_area_limit_common_area,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_hold_when_no_intersection: ",
            resolved_chase_area_limit_hold_when_no_intersection,
        ]),
    ]

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navi_tf_bridge_launch_file),
            condition=IfCondition(resolved_use_navi_tf_bridge),
            launch_arguments={
                "input_topic": "/ly/navi/target_rel",
                "input_goal_pos_raw_topic": "/ly/navi/goal_pos_raw",
                "output_goal_pose_topic": "/goal_pose", #"output_goal_pose_topic": "/goal_pose_debug"
                "publish_goal_pose": effective_navi_publish_goal_pose,
                "publish_goal_pos": "false",
                "enable_goal_pos_raw_bridge": "true",
                # gimbal_driver.launch.py owns the sole /Path_downsampled ->
                # /ly/game/path bridge in the formal stack.
                "enable_game_path_bridge": "false",
                "goal_pos_raw_frame": "map",
                "preferred_distance_cm": resolved_chase_preferred_distance_cm,
                "distance_deadband_cm": resolved_chase_distance_deadband_cm,
                "stop_when_no_target": resolved_chase_stop_when_no_target,
                "chase_area_limit_enable": resolved_chase_area_limit_enable,
                "chase_area_limit_boundary_margin_cm": resolved_chase_area_limit_boundary_margin_cm,
                "chase_area_limit_chase_enable_cross_area": (
                    resolved_chase_area_limit_chase_enable_cross_area
                ),
                "chase_area_limit_use_area_scope": (
                    resolved_chase_area_limit_use_area_scope
                ),
                "chase_area_limit_my_area": (
                    resolved_chase_area_limit_my_area
                ),
                "chase_area_limit_enemy_area": (
                    resolved_chase_area_limit_enemy_area
                ),
                "chase_area_limit_common_area": (
                    resolved_chase_area_limit_common_area
                ),
                "chase_area_limit_hold_when_no_intersection": (
                    resolved_chase_area_limit_hold_when_no_intersection
                ),
            }.items(),
        ),
        Node(
            package="navi_tf_bridge",
            executable="map_aim_point_node",
            name="map_aim_point_node",
            output=output,
            parameters=[{
                "require_initial_target": False,
                "target_frame": ParameterValue(face_mode_target_frame, value_type=str),
                "aim_frame": "gimbal_world",
                "camera_frame": "gx_camera_0",
                "camera_fallback_frame": "gx_camera_1",
                "solve_mode": "relative_geometry",
                "solve_frame": "gimbal_barrel_joint",
                "gimbal_angles_topic": "/ly/gimbal/angles",
                "control_angles_topic": "/ly/face_mode/angles",
                "control_firecode_topic": "/ly/control/firecode",
                "face_target_topic": "/ly/face_mode/target_raw",
                "status_topic": "/ly/gimbal/facemode",
                "publish_firecode": False,
                "aim_mode": True,
                "bridge_config_file": ParameterValue(face_mode_solver_bridge_config_file, value_type=str),
                "use_raw_goal_static_calibration": ParameterValue(
                    face_mode_use_raw_goal_static_calibration, value_type=bool),
                "raw_goal_target_frame": ParameterValue(
                    face_mode_raw_goal_target_frame, value_type=str),
                "manual_target_enable": ParameterValue(
                    face_mode_manual_target_enable, value_type=bool),
                "manual_target_frame": ParameterValue(
                    face_mode_manual_target_frame, value_type=str),
                "manual_target_x_m": ParameterValue(
                    face_mode_manual_target_x_m, value_type=float),
                "manual_target_y_m": ParameterValue(
                    face_mode_manual_target_y_m, value_type=float),
                "manual_target_z_m": ParameterValue(
                    face_mode_manual_target_z_m, value_type=float),
                "publish_hz": 30.0,
                "tf_timeout_sec": 0.05,
                "use_gimbal_stamp_for_tf": False,
                "max_gimbal_stamp_age_sec": 0.50,
                "min_distance_m": 0.10,
                "max_target_distance_m": 100.0,
                "command_filter_alpha": 1.0,
                "yaw_sign": 1.0,
                "pitch_sign": 1.0,
                "yaw_bias_deg": 0.0,
                "pitch_bias_deg": 0.0,
                "max_yaw_step_deg": ParameterValue(
                    face_mode_max_yaw_step_deg, value_type=float),
                "max_pitch_step_deg": ParameterValue(
                    face_mode_max_pitch_step_deg, value_type=float),
            }],
            condition=IfCondition(use_face_mode_solver),
        ),
        ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                "mkdir -p \"$1\" && exec ros2 bag record -a -o \"$2\"",
                "rosbag_record",
                LaunchConfiguration("resolved_rosbag_base_dir"),
                resolved_rosbag_path,
            ],
            output=output,
            condition=IfCondition(rosbag_enabled_expr),
        ),
        # gimbal_driver owns formal baseline + root-config compatibility routing.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gimbal_driver_launch_file),
            condition=IfCondition(use_gimbal),
            launch_arguments={
                "base_config_file": gimbal_driver_config_file,
                "legacy_base_config_file": base_config_file,
                "legacy_config_file": config_file,
                "output": output,
                "use_virtual_device": gimbal_use_virtual_device,
                "enable_path_downsampled_bridge": "true",
                "firecode_partial_hold_ms": firecode_partial_hold_ms,
                "velocity_raw_to_mps": velocity_raw_to_mps,
                "raw_log_enable": gimbal_raw_log_enable,
                "raw_log_uplink": gimbal_raw_log_uplink,
                "raw_log_downlink": gimbal_raw_log_downlink,
                "raw_log_screen": gimbal_raw_log_screen,
                "raw_log_flush": gimbal_raw_log_flush,
                "raw_log_dir": gimbal_raw_log_dir,
                "raw_log_type_ids": gimbal_raw_log_type_ids,
                "raw_topic_enable": gimbal_raw_topic_enable,
                "raw_topic_uplink": gimbal_raw_topic_uplink,
                "raw_topic_downlink": gimbal_raw_topic_downlink,
                "raw_topic_type_ids": gimbal_raw_topic_type_ids,
            }.items(),
        ),
        # 最后启动 behavior_tree（决策接管）
        Node(
            package="behavior_tree",
            executable="behavior_tree_node",
            name="behavior_tree",
            output=output,
            parameters=[
                area_manager_config_file,
                task_config_file,
                chase_config_file,
                navi_rotate_config_file,
                tactical_config_file,
                patrol_config_file,
                special_config_file,
                {
                    "competition_profile": resolved_competition_profile,
                    "bt_config_file": resolved_bt_config_file,
                    "bt_tree_file": bt_tree_file,
                    "debug_bypass_is_start": debug_bypass_is_start,
                    "runtime_rearm_start_gate": runtime_rearm_start_gate,
                    "publish_navi_goal": publish_navi_goal,
                    "wait_for_game_start_timeout_sec": wait_for_game_start_timeout_sec,
                    "league_referee_stale_timeout_ms": league_referee_stale_timeout_ms,
                    "StartGate.AllowGimbalPatrolBeforeStart": ParameterValue(
                        start_gate_allow_gimbal_patrol_before_start, value_type=bool),
                    "StartGate/AllowGimbalPatrolBeforeStart": ParameterValue(
                        start_gate_allow_gimbal_patrol_before_start, value_type=bool),
                    "ExternalAim.Enable": True,
                    "ExternalAim/Enable": True,
                    "Task.OutpostConfirm.ManualGoal.Enable": ParameterValue(
                        outpost_manual_goal_enable, value_type=bool),
                    "Task/OutpostConfirm/ManualGoal/Enable": ParameterValue(
                        outpost_manual_goal_enable, value_type=bool),
                    "Task.OutpostConfirm.ManualGoal.MapXM": ParameterValue(
                        outpost_manual_goal_x_m, value_type=float),
                    "Task/OutpostConfirm/ManualGoal/MapXM": ParameterValue(
                        outpost_manual_goal_x_m, value_type=float),
                    "Task.OutpostConfirm.ManualGoal.MapYM": ParameterValue(
                        outpost_manual_goal_y_m, value_type=float),
                    "Task/OutpostConfirm/ManualGoal/MapYM": ParameterValue(
                        outpost_manual_goal_y_m, value_type=float),
                    "Task.OutpostConfirm.ManualGoal.MapZM": ParameterValue(
                        outpost_manual_goal_z_m, value_type=float),
                    "Task/OutpostConfirm/ManualGoal/MapZM": ParameterValue(
                        outpost_manual_goal_z_m, value_type=float),
                    "decision_trace_enabled": decision_trace_enabled,
                    "decision_trace_file": decision_trace_file,
                    "decision_trace_every_n_ticks": decision_trace_every_n_ticks,
                }
            ],
            on_exit=Shutdown(reason="behavior_tree exited"),
            condition=IfCondition(use_behavior_tree),
        ),
    ]

    return LaunchDescription(launch_args + info_logs + nodes)
