#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
哨兵整链路启动入口（比赛/联调主入口）。

职责：
- 拉起 gimbal_driver / navi_tf_bridge / FaceMode / behavior_tree。
- TF 默認由外部 sentry_tf 提供；需要本倉 fallback 時可設 use_tf_tree:=true。
- 外部 aim 通过 /ly/aim/* 接入；本 launch 不再启动内部相机/辅瞄链。
- 支持通过 offline 参数统一覆盖“虚拟串口 + 视频回放”。

注意：
- behavior_tree 会接管 /ly/control/*，调试外部控制脚本时不要并行启动。
"""
import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration, Shutdown
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

    def resolve_tf_tree_defaults(context):
        tf_tree_params_file_raw = LaunchConfiguration("tf_tree_params_file").perform(context).strip()
        resolved_tf_tree_params_file = (
            tf_tree_params_file_raw if tf_tree_params_file_raw else default_tf_tree_params_file
        )
        return [
            SetLaunchConfiguration("resolved_tf_tree_params_file", resolved_tf_tree_params_file),
        ]

    def resolve_rosbag_defaults(context):
        rosbag_path_raw = LaunchConfiguration("rosbag_path").perform(context).strip()
        resolved_rosbag_path = os.path.expanduser(rosbag_path_raw or "~/Log/rosbag")
        return [
            SetLaunchConfiguration("resolved_rosbag_path", resolved_rosbag_path),
        ]

    def resolve_navi_tf_bridge_defaults(context):
        use_navi_tf_bridge_override = _normalize_bool(
            LaunchConfiguration("use_navi_tf_bridge").perform(context)
        )
        resolved_use_navi_tf_bridge = "true"
        resolved_chase_area_limit_enable = "false"
        resolved_chase_area_limit_boundary_margin_cm = "30.0"
        resolved_chase_area_limit_hold_when_unknown_area = "false"
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
                    area_limit_cfg = chase_cfg.get("AreaLimit", {})
                    if isinstance(area_limit_cfg, dict):
                        resolved_chase_area_limit_enable = (
                            "true" if bool(area_limit_cfg.get("Enable", False)) else "false"
                        )
                        resolved_chase_area_limit_boundary_margin_cm = str(
                            float(area_limit_cfg.get("BoundaryMarginCm", 30.0))
                        )
                        resolved_chase_area_limit_hold_when_unknown_area = (
                            "true"
                            if bool(area_limit_cfg.get("HoldWhenUnknownArea", False))
                            else "false"
                        )
                        resolved_chase_area_limit_hold_when_no_intersection = (
                            "true"
                            if bool(area_limit_cfg.get("HoldWhenNoIntersection", True))
                            else "false"
                        )
            except Exception as ex:
                print(
                    f"[sentry_all] failed to parse bt_config_file '{bt_config_path}': {ex}. "
                    "navi_tf_bridge falls back to enabled."
                )
        if use_navi_tf_bridge_override:
            resolved_use_navi_tf_bridge = use_navi_tf_bridge_override

        return [
            SetLaunchConfiguration("resolved_use_navi_tf_bridge", resolved_use_navi_tf_bridge),
            SetLaunchConfiguration("resolved_chase_area_limit_enable", resolved_chase_area_limit_enable),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_boundary_margin_cm",
                resolved_chase_area_limit_boundary_margin_cm,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_hold_when_unknown_area",
                resolved_chase_area_limit_hold_when_unknown_area,
            ),
            SetLaunchConfiguration(
                "resolved_chase_area_limit_hold_when_no_intersection",
                resolved_chase_area_limit_hold_when_no_intersection,
            ),
        ]

    # 分层配置默认入口：
    #   base + module + optional global override(config_file)
    behavior_tree_share = get_package_share_directory("behavior_tree")
    tf_tree_share = get_package_share_directory("tf_tree")
    behavior_tree_config_root = os.path.join(behavior_tree_share, "config")
    tf_tree_launch_file = os.path.join(tf_tree_share, "launch", "tf_tree.launch.py")
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
    default_tf_tree_params_file = os.path.join(tf_tree_share, "config", "tf_tree.yaml")
    default_base_config_file = os.path.join(behavior_tree_config_root, "base_config.yaml")
    default_override_config_file = os.path.join(behavior_tree_config_root, "override_config.yaml")
    default_area_manager_config_file = os.path.join(behavior_tree_config_root, "AreaManager.yaml")
    default_task_config_file = os.path.join(behavior_tree_config_root, "Task.yaml")
    default_navi_rotate_config_file = os.path.join(behavior_tree_config_root, "NaviRotateControl.yaml")

    mode = LaunchConfiguration("mode")
    config_file = LaunchConfiguration("config_file")
    area_manager_config_file = LaunchConfiguration("area_manager_config_file")
    task_config_file = LaunchConfiguration("task_config_file")
    navi_rotate_config_file = LaunchConfiguration("navi_rotate_config_file")
    base_config_file = LaunchConfiguration("base_config_file")
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
    face_mode_max_yaw_step_deg = LaunchConfiguration("face_mode_max_yaw_step_deg")
    face_mode_max_pitch_step_deg = LaunchConfiguration("face_mode_max_pitch_step_deg")
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
    rosbag_play_enable = LaunchConfiguration("rosbag_play_enable")
    rosbag_path = LaunchConfiguration("rosbag_path")
    resolved_rosbag_path = LaunchConfiguration("resolved_rosbag_path")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    use_tf_tree = LaunchConfiguration("use_tf_tree")
    use_navi_tf_bridge = LaunchConfiguration("use_navi_tf_bridge")
    use_face_mode_solver = LaunchConfiguration("use_face_mode_solver")
    resolved_use_navi_tf_bridge = LaunchConfiguration("resolved_use_navi_tf_bridge")
    resolved_chase_area_limit_enable = LaunchConfiguration("resolved_chase_area_limit_enable")
    resolved_chase_area_limit_boundary_margin_cm = LaunchConfiguration(
        "resolved_chase_area_limit_boundary_margin_cm"
    )
    resolved_chase_area_limit_hold_when_unknown_area = LaunchConfiguration(
        "resolved_chase_area_limit_hold_when_unknown_area"
    )
    resolved_chase_area_limit_hold_when_no_intersection = LaunchConfiguration(
        "resolved_chase_area_limit_hold_when_no_intersection"
    )
    tf_tree_params_file = LaunchConfiguration("tf_tree_params_file")
    resolved_tf_tree_params_file = LaunchConfiguration("resolved_tf_tree_params_file")
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
            description="Base shared YAML for gimbal/io/shared geometry.",
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
            "navi_rotate_config_file",
            default_value=default_navi_rotate_config_file,
            description="External navigation rotate/follow compatibility YAML for behavior_tree.",
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
            description="Log upper -> lower GimbalControlData raw frames when gimbal_raw_log_enable is true.",
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
            "rosbag_play_enable",
            default_value="false",
            description="When true, launch ros2 bag play for replaying external inputs.",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/Log/rosbag",
            description="Rosbag directory passed to `ros2 bag play` when rosbag_play_enable is true.",
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
            "use_tf_tree",
            default_value="false",
            description="Whether to launch local tf_tree fallback. Keep false when external sentry_tf is running.",
        ),
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
            "tf_tree_params_file",
            default_value="",
            description="Optional tf_tree params YAML path. Empty uses package default.",
        ),
        DeclareLaunchArgument(
            "offline",
            default_value="false",
            description="Offline profile: force virtual IO and video replay without editing YAML.",
        ),
        DeclareLaunchArgument("resolved_mode_kind", default_value=""),
        DeclareLaunchArgument("resolved_competition_profile", default_value=""),
        DeclareLaunchArgument("resolved_bt_config_file", default_value=""),
        DeclareLaunchArgument("resolved_tf_tree_params_file", default_value=""),
        DeclareLaunchArgument("resolved_rosbag_path", default_value=""),
        DeclareLaunchArgument("resolved_use_navi_tf_bridge", default_value="true"),
        DeclareLaunchArgument("resolved_chase_area_limit_enable", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_boundary_margin_cm", default_value="30.0"),
        DeclareLaunchArgument("resolved_chase_area_limit_hold_when_unknown_area", default_value="false"),
        DeclareLaunchArgument("resolved_chase_area_limit_hold_when_no_intersection", default_value="true"),
        OpaqueFunction(function=resolve_mode_defaults),
        OpaqueFunction(function=resolve_tf_tree_defaults),
        OpaqueFunction(function=resolve_rosbag_defaults),
        OpaqueFunction(function=resolve_navi_tf_bridge_defaults),
    ]

    truthy_values = "['true', '1', 'yes', 'on']"
    rosbag_enabled_expr = PythonExpression([
        "'", rosbag_play_enable, "'.lower() in ", truthy_values
    ])
    hardware_io_expr = PythonExpression([
        "'", offline, "'.lower() not in ", truthy_values,
        " and '", rosbag_play_enable, "'.lower() not in ", truthy_values
    ])
    virtual_io_expr = PythonExpression([
        "'", offline, "'.lower() in ", truthy_values,
        " or '", rosbag_play_enable, "'.lower() in ", truthy_values
    ])
    info_logs = [
        LogInfo(msg=["[sentry_all] mode: ", mode]),
        LogInfo(msg=["[sentry_all] config: ", config_file]),
        LogInfo(msg=["[sentry_all] base_config: ", base_config_file]),
        LogInfo(msg=["[sentry_all] area_manager_config: ", area_manager_config_file]),
        LogInfo(msg=["[sentry_all] task_config: ", task_config_file]),
        LogInfo(msg=["[sentry_all] navi_rotate_config: ", navi_rotate_config_file]),
        LogInfo(msg=["[sentry_all] output: ", output]),
        LogInfo(msg=["[sentry_all] offline: ", offline]),
        LogInfo(msg=["[sentry_all] external aim: required (/ly/aim/*)"]),
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
        LogInfo(msg=["[sentry_all] rosbag_play_enable: ", rosbag_play_enable]),
        LogInfo(msg=["[sentry_all] rosbag_path: ", rosbag_path]),
        LogInfo(msg=["[sentry_all] resolved_rosbag_path: ", resolved_rosbag_path]),
        LogInfo(msg=["[sentry_all] decision_trace_enabled: ", decision_trace_enabled]),
        LogInfo(msg=["[sentry_all] decision_trace_file: ", decision_trace_file]),
        LogInfo(msg=["[sentry_all] decision_trace_every_n_ticks: ", decision_trace_every_n_ticks]),
        LogInfo(msg=["[sentry_all] use_tf_tree: ", use_tf_tree]),
        LogInfo(msg=["[sentry_all] use_navi_tf_bridge: ", use_navi_tf_bridge]),
        LogInfo(msg=["[sentry_all] use_face_mode_solver: ", use_face_mode_solver]),
        LogInfo(msg=["[sentry_all] resolved_use_navi_tf_bridge: ", resolved_use_navi_tf_bridge]),
        LogInfo(msg=["[sentry_all] chase_area_limit_enable: ", resolved_chase_area_limit_enable]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_boundary_margin_cm: ",
            resolved_chase_area_limit_boundary_margin_cm,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_hold_when_unknown_area: ",
            resolved_chase_area_limit_hold_when_unknown_area,
        ]),
        LogInfo(msg=[
            "[sentry_all] chase_area_limit_hold_when_no_intersection: ",
            resolved_chase_area_limit_hold_when_no_intersection,
        ]),
        LogInfo(msg=["[sentry_all] tf_tree_params_file: ", tf_tree_params_file]),
        LogInfo(msg=["[sentry_all] resolved_tf_tree_params_file: ", resolved_tf_tree_params_file]),
    ]

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_tree_launch_file),
            condition=IfCondition(use_tf_tree),
            launch_arguments={
                "params_file": resolved_tf_tree_params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navi_tf_bridge_launch_file),
            condition=IfCondition(resolved_use_navi_tf_bridge),
            launch_arguments={
                "input_topic": "/ly/navi/target_rel",
                "input_goal_pos_raw_topic": "/ly/navi/goal_pos_raw",
                "output_goal_pose_topic": "/goal_pose", #"output_goal_pose_topic": "/goal_pose_debug"
                "publish_goal_pose": "true", #"publish_goal_pose": navi_publish_goal_pose,
                "publish_goal_pos": "false",
                "enable_goal_pos_raw_bridge": "true",
                "goal_pos_raw_frame": "map",
                "chase_area_limit_enable": resolved_chase_area_limit_enable,
                "chase_area_limit_boundary_margin_cm": resolved_chase_area_limit_boundary_margin_cm,
                "chase_area_limit_hold_when_unknown_area": (
                    resolved_chase_area_limit_hold_when_unknown_area
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
                "target_frame": "official_map",
                "aim_frame": "gimbal_world",
                "camera_frame": "gx_camera",
                "solve_mode": "relative_geometry",
                "solve_frame": "gimbal_barrel_joint",
                "gimbal_angles_topic": "/ly/gimbal/angles",
                "control_angles_topic": "/ly/face_mode/angles",
                "control_firecode_topic": "/ly/control/firecode",
                "face_target_topic": "/ly/face_mode/target_raw",
                "publish_firecode": False,
                "aim_mode": True,
                "bridge_config_file": ParameterValue(face_mode_solver_bridge_config_file, value_type=str),
                "use_raw_goal_static_calibration": True,
                "raw_goal_target_frame": "map",
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
            cmd=["ros2", "bag", "play", resolved_rosbag_path],
            output=output,
            on_exit=[Shutdown(reason="rosbag playback exited")],
            condition=IfCondition(rosbag_enabled_expr),
        ),
        # gimbal_driver: offline=true or rosbag replay force use_virtual_device.
        GroupAction(
            actions=[
                Node(
                    package="gimbal_driver",
                    executable="gimbal_driver_node",
                    name="gimbal_driver",
                    output=output,
                    parameters=[
                        base_config_file,
                        config_file,
                        {
                            "io_config/firecode_partial_hold_ms": ParameterValue(
                                firecode_partial_hold_ms, value_type=int
                            ),
                            "io_config.firecode_partial_hold_ms": ParameterValue(
                                firecode_partial_hold_ms, value_type=int
                            ),
                            "io_config/velocity_raw_to_mps": ParameterValue(
                                velocity_raw_to_mps, value_type=float
                            ),
                            "io_config.velocity_raw_to_mps": ParameterValue(
                                velocity_raw_to_mps, value_type=float
                            ),
                            "io_config/raw_serial_log_enable": ParameterValue(
                                gimbal_raw_log_enable, value_type=bool
                            ),
                            "io_config.raw_serial_log_enable": ParameterValue(
                                gimbal_raw_log_enable, value_type=bool
                            ),
                            "io_config/raw_serial_log_uplink": ParameterValue(
                                gimbal_raw_log_uplink, value_type=bool
                            ),
                            "io_config.raw_serial_log_uplink": ParameterValue(
                                gimbal_raw_log_uplink, value_type=bool
                            ),
                            "io_config/raw_serial_log_downlink": ParameterValue(
                                gimbal_raw_log_downlink, value_type=bool
                            ),
                            "io_config.raw_serial_log_downlink": ParameterValue(
                                gimbal_raw_log_downlink, value_type=bool
                            ),
                            "io_config/raw_serial_log_screen": ParameterValue(
                                gimbal_raw_log_screen, value_type=bool
                            ),
                            "io_config.raw_serial_log_screen": ParameterValue(
                                gimbal_raw_log_screen, value_type=bool
                            ),
                            "io_config/raw_serial_log_flush": ParameterValue(
                                gimbal_raw_log_flush, value_type=bool
                            ),
                            "io_config.raw_serial_log_flush": ParameterValue(
                                gimbal_raw_log_flush, value_type=bool
                            ),
                            "io_config/raw_serial_log_dir": ParameterValue(
                                gimbal_raw_log_dir, value_type=str
                            ),
                            "io_config.raw_serial_log_dir": ParameterValue(
                                gimbal_raw_log_dir, value_type=str
                            ),
                            "io_config/raw_serial_log_type_ids": ParameterValue(
                                gimbal_raw_log_type_ids, value_type=str
                            ),
                            "io_config.raw_serial_log_type_ids": ParameterValue(
                                gimbal_raw_log_type_ids, value_type=str
                            ),
                            "io_config/raw_serial_topic_enable": ParameterValue(
                                gimbal_raw_topic_enable, value_type=bool
                            ),
                            "io_config.raw_serial_topic_enable": ParameterValue(
                                gimbal_raw_topic_enable, value_type=bool
                            ),
                            "io_config/raw_serial_topic_uplink": ParameterValue(
                                gimbal_raw_topic_uplink, value_type=bool
                            ),
                            "io_config.raw_serial_topic_uplink": ParameterValue(
                                gimbal_raw_topic_uplink, value_type=bool
                            ),
                            "io_config/raw_serial_topic_downlink": ParameterValue(
                                gimbal_raw_topic_downlink, value_type=bool
                            ),
                            "io_config.raw_serial_topic_downlink": ParameterValue(
                                gimbal_raw_topic_downlink, value_type=bool
                            ),
                            "io_config/raw_serial_topic_type_ids": ParameterValue(
                                gimbal_raw_topic_type_ids, value_type=str
                            ),
                            "io_config.raw_serial_topic_type_ids": ParameterValue(
                                gimbal_raw_topic_type_ids, value_type=str
                            ),
                        },
                    ],
                    on_exit=Shutdown(reason="gimbal_driver exited"),
                    condition=IfCondition(hardware_io_expr),
                ),
                Node(
                    package="gimbal_driver",
                    executable="gimbal_driver_node",
                    name="gimbal_driver",
                    output=output,
                    parameters=[
                        base_config_file,
                        config_file,
                        {
                            "io_config/use_virtual_device": True,
                            "io_config.use_virtual_device": True,
                            "io_config/firecode_partial_hold_ms": ParameterValue(
                                firecode_partial_hold_ms, value_type=int
                            ),
                            "io_config.firecode_partial_hold_ms": ParameterValue(
                                firecode_partial_hold_ms, value_type=int
                            ),
                            "io_config/velocity_raw_to_mps": ParameterValue(
                                velocity_raw_to_mps, value_type=float
                            ),
                            "io_config.velocity_raw_to_mps": ParameterValue(
                                velocity_raw_to_mps, value_type=float
                            ),
                            "io_config/raw_serial_log_enable": ParameterValue(
                                gimbal_raw_log_enable, value_type=bool
                            ),
                            "io_config.raw_serial_log_enable": ParameterValue(
                                gimbal_raw_log_enable, value_type=bool
                            ),
                            "io_config/raw_serial_log_uplink": ParameterValue(
                                gimbal_raw_log_uplink, value_type=bool
                            ),
                            "io_config.raw_serial_log_uplink": ParameterValue(
                                gimbal_raw_log_uplink, value_type=bool
                            ),
                            "io_config/raw_serial_log_downlink": ParameterValue(
                                gimbal_raw_log_downlink, value_type=bool
                            ),
                            "io_config.raw_serial_log_downlink": ParameterValue(
                                gimbal_raw_log_downlink, value_type=bool
                            ),
                            "io_config/raw_serial_log_screen": ParameterValue(
                                gimbal_raw_log_screen, value_type=bool
                            ),
                            "io_config.raw_serial_log_screen": ParameterValue(
                                gimbal_raw_log_screen, value_type=bool
                            ),
                            "io_config/raw_serial_log_flush": ParameterValue(
                                gimbal_raw_log_flush, value_type=bool
                            ),
                            "io_config.raw_serial_log_flush": ParameterValue(
                                gimbal_raw_log_flush, value_type=bool
                            ),
                            "io_config/raw_serial_log_dir": ParameterValue(
                                gimbal_raw_log_dir, value_type=str
                            ),
                            "io_config.raw_serial_log_dir": ParameterValue(
                                gimbal_raw_log_dir, value_type=str
                            ),
                            "io_config/raw_serial_log_type_ids": ParameterValue(
                                gimbal_raw_log_type_ids, value_type=str
                            ),
                            "io_config.raw_serial_log_type_ids": ParameterValue(
                                gimbal_raw_log_type_ids, value_type=str
                            ),
                            "io_config/raw_serial_topic_enable": ParameterValue(
                                gimbal_raw_topic_enable, value_type=bool
                            ),
                            "io_config.raw_serial_topic_enable": ParameterValue(
                                gimbal_raw_topic_enable, value_type=bool
                            ),
                            "io_config/raw_serial_topic_uplink": ParameterValue(
                                gimbal_raw_topic_uplink, value_type=bool
                            ),
                            "io_config.raw_serial_topic_uplink": ParameterValue(
                                gimbal_raw_topic_uplink, value_type=bool
                            ),
                            "io_config/raw_serial_topic_downlink": ParameterValue(
                                gimbal_raw_topic_downlink, value_type=bool
                            ),
                            "io_config.raw_serial_topic_downlink": ParameterValue(
                                gimbal_raw_topic_downlink, value_type=bool
                            ),
                            "io_config/raw_serial_topic_type_ids": ParameterValue(
                                gimbal_raw_topic_type_ids, value_type=str
                            ),
                            "io_config.raw_serial_topic_type_ids": ParameterValue(
                                gimbal_raw_topic_type_ids, value_type=str
                            ),
                        },
                    ],
                    on_exit=Shutdown(reason="gimbal_driver exited"),
                    condition=IfCondition(virtual_io_expr),
                ),
            ],
            condition=IfCondition(use_gimbal),
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
                navi_rotate_config_file,
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
