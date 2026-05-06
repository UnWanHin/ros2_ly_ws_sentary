#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""
哨兵整链路启动入口（比赛/联调主入口）。

职责：
- 拉起 gimbal_driver / detector / tracker_solver / predictor / outpost_hitter / buff_hitter / behavior_tree。
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
        if use_navi_tf_bridge_override:
            return [
                SetLaunchConfiguration("resolved_use_navi_tf_bridge", use_navi_tf_bridge_override),
            ]

        resolved_use_navi_tf_bridge = "true"
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
            except Exception as ex:
                print(
                    f"[sentry_all] failed to parse bt_config_file '{bt_config_path}': {ex}. "
                    "navi_tf_bridge falls back to enabled."
                )

        return [
            SetLaunchConfiguration("resolved_use_navi_tf_bridge", resolved_use_navi_tf_bridge),
        ]

    # 分层配置默认入口：
    #   base + module + optional global override(config_file)
    behavior_tree_share = get_package_share_directory("behavior_tree")
    detector_share = get_package_share_directory("detector")
    predictor_share = get_package_share_directory("predictor")
    outpost_share = get_package_share_directory("outpost_hitter")
    buff_share = get_package_share_directory("buff_hitter")
    tf_tree_share = get_package_share_directory("tf_tree")
    behavior_tree_config_root = os.path.join(behavior_tree_share, "config")
    tf_tree_launch_file = os.path.join(tf_tree_share, "launch", "tf_tree.launch.py")
    navi_tf_bridge_launch_file = PathJoinSubstitution([
        FindPackageShare("navi_tf_bridge"),
        "launch",
        "target_rel_to_goal_pos.launch.py",
    ])
    default_tf_tree_params_file = os.path.join(tf_tree_share, "config", "tf_tree.yaml")
    default_base_config_file = os.path.join(behavior_tree_config_root, "base_config.yaml")
    default_override_config_file = os.path.join(behavior_tree_config_root, "override_config.yaml")
    default_area_manager_config_file = os.path.join(behavior_tree_config_root, "AreaManager.yaml")
    default_task_config_file = os.path.join(behavior_tree_config_root, "Task.yaml")
    default_detector_config_file = os.path.join(detector_share, "config", "detector_config.yaml")
    default_predictor_config_file = os.path.join(predictor_share, "config", "predictor_config.yaml")
    default_outpost_config_file = os.path.join(outpost_share, "config", "outpost_config.yaml")
    default_buff_config_file = os.path.join(buff_share, "config", "buff_config.yaml")

    mode = LaunchConfiguration("mode")
    config_file = LaunchConfiguration("config_file")
    area_manager_config_file = LaunchConfiguration("area_manager_config_file")
    task_config_file = LaunchConfiguration("task_config_file")
    base_config_file = LaunchConfiguration("base_config_file")
    detector_config_file = LaunchConfiguration("detector_config_file")
    predictor_config_file = LaunchConfiguration("predictor_config_file")
    outpost_config_file = LaunchConfiguration("outpost_config_file")
    buff_config_file = LaunchConfiguration("buff_config_file")
    output = LaunchConfiguration("output")
    competition_profile = LaunchConfiguration("competition_profile")
    bt_config_file = LaunchConfiguration("bt_config_file")
    bt_tree_file = LaunchConfiguration("bt_tree_file")
    debug_bypass_is_start = LaunchConfiguration("debug_bypass_is_start")
    runtime_rearm_start_gate = LaunchConfiguration("runtime_rearm_start_gate")
    publish_navi_goal = LaunchConfiguration("publish_navi_goal")
    wait_for_game_start_timeout_sec = LaunchConfiguration("wait_for_game_start_timeout_sec")
    league_referee_stale_timeout_ms = LaunchConfiguration("league_referee_stale_timeout_ms")
    firecode_partial_hold_ms = LaunchConfiguration("firecode_partial_hold_ms")
    velocity_raw_to_mps = LaunchConfiguration("velocity_raw_to_mps")
    aim_timer_log_enable = LaunchConfiguration("aim_timer_log_enable")
    aim_timer_log_dir = LaunchConfiguration("aim_timer_log_dir")
    decision_trace_enabled = LaunchConfiguration("decision_trace_enabled")
    decision_trace_file = LaunchConfiguration("decision_trace_file")
    decision_trace_every_n_ticks = LaunchConfiguration("decision_trace_every_n_ticks")
    rosbag_play_enable = LaunchConfiguration("rosbag_play_enable")
    rosbag_path = LaunchConfiguration("rosbag_path")
    resolved_rosbag_path = LaunchConfiguration("resolved_rosbag_path")

    use_gimbal = LaunchConfiguration("use_gimbal")
    use_detector = LaunchConfiguration("use_detector")
    use_tracker = LaunchConfiguration("use_tracker")
    use_predictor = LaunchConfiguration("use_predictor")
    use_outpost = LaunchConfiguration("use_outpost")
    use_buff = LaunchConfiguration("use_buff")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    use_tf_tree = LaunchConfiguration("use_tf_tree")
    use_navi_tf_bridge = LaunchConfiguration("use_navi_tf_bridge")
    resolved_use_navi_tf_bridge = LaunchConfiguration("resolved_use_navi_tf_bridge")
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
            description="Base shared YAML for camera/solver/io.",
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
            "detector_config_file",
            default_value=default_detector_config_file,
            description="Detector module YAML.",
        ),
        DeclareLaunchArgument(
            "predictor_config_file",
            default_value=default_predictor_config_file,
            description="Predictor/tracker module YAML.",
        ),
        DeclareLaunchArgument(
            "outpost_config_file",
            default_value=default_outpost_config_file,
            description="Outpost module YAML.",
        ),
        DeclareLaunchArgument(
            "buff_config_file",
            default_value=default_buff_config_file,
            description="Buff module YAML.",
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
            "aim_timer_log_enable",
            default_value="false",
            description="Enable tracker_solver/predictor AimTimer file diagnostics.",
        ),
        DeclareLaunchArgument(
            "aim_timer_log_dir",
            default_value="~/Log/AimTimer",
            description="AimTimer diagnostics output directory.",
        ),
        DeclareLaunchArgument(
            "rosbag_play_enable",
            default_value="false",
            description="When true, launch ros2 bag play and switch detector to rosbag input.",
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
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_tracker", default_value="true"),
        DeclareLaunchArgument("use_predictor", default_value="true"),
        DeclareLaunchArgument("use_outpost", default_value="true"),
        DeclareLaunchArgument("use_buff", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "use_tf_tree",
            default_value="true",
            description="Whether to launch tf_tree TF broadcaster chain.",
        ),
        DeclareLaunchArgument(
            "use_navi_tf_bridge",
            default_value="",
            description="Optional override. Empty means load NaviSetting.ToNavi from bt_config_file.",
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
    offline_video_expr = PythonExpression([
        "'", offline, "'.lower() in ", truthy_values,
        " and '", rosbag_play_enable, "'.lower() not in ", truthy_values
    ])

    info_logs = [
        LogInfo(msg=["[sentry_all] mode: ", mode]),
        LogInfo(msg=["[sentry_all] config: ", config_file]),
        LogInfo(msg=["[sentry_all] base_config: ", base_config_file]),
        LogInfo(msg=["[sentry_all] area_manager_config: ", area_manager_config_file]),
        LogInfo(msg=["[sentry_all] task_config: ", task_config_file]),
        LogInfo(msg=["[sentry_all] detector_config: ", detector_config_file]),
        LogInfo(msg=["[sentry_all] predictor_config: ", predictor_config_file]),
        LogInfo(msg=["[sentry_all] outpost_config: ", outpost_config_file]),
        LogInfo(msg=["[sentry_all] buff_config: ", buff_config_file]),
        LogInfo(msg=["[sentry_all] output: ", output]),
        LogInfo(msg=["[sentry_all] offline: ", offline]),
        LogInfo(msg=["[sentry_all] competition_profile: ", competition_profile]),
        LogInfo(msg=["[sentry_all] bt_config_file: ", bt_config_file]),
        LogInfo(msg=["[sentry_all] resolved_mode: ", resolved_mode_kind]),
        LogInfo(msg=["[sentry_all] resolved_competition_profile: ", resolved_competition_profile]),
        LogInfo(msg=["[sentry_all] resolved_bt_config_file: ", resolved_bt_config_file]),
        LogInfo(msg=["[sentry_all] bt_tree_file: ", bt_tree_file]),
        LogInfo(msg=["[sentry_all] debug_bypass_is_start: ", debug_bypass_is_start]),
        LogInfo(msg=["[sentry_all] runtime_rearm_start_gate: ", runtime_rearm_start_gate]),
        LogInfo(msg=["[sentry_all] publish_navi_goal: ", publish_navi_goal]),
        LogInfo(msg=["[sentry_all] wait_for_game_start_timeout_sec: ", wait_for_game_start_timeout_sec]),
        LogInfo(msg=["[sentry_all] league_referee_stale_timeout_ms: ", league_referee_stale_timeout_ms]),
        LogInfo(msg=["[sentry_all] firecode_partial_hold_ms: ", firecode_partial_hold_ms]),
        LogInfo(msg=["[sentry_all] velocity_raw_to_mps: ", velocity_raw_to_mps]),
        LogInfo(msg=["[sentry_all] aim_timer_log_enable: ", aim_timer_log_enable]),
        LogInfo(msg=["[sentry_all] aim_timer_log_dir: ", aim_timer_log_dir]),
        LogInfo(msg=["[sentry_all] rosbag_play_enable: ", rosbag_play_enable]),
        LogInfo(msg=["[sentry_all] rosbag_path: ", rosbag_path]),
        LogInfo(msg=["[sentry_all] resolved_rosbag_path: ", resolved_rosbag_path]),
        LogInfo(msg=["[sentry_all] decision_trace_enabled: ", decision_trace_enabled]),
        LogInfo(msg=["[sentry_all] decision_trace_file: ", decision_trace_file]),
        LogInfo(msg=["[sentry_all] decision_trace_every_n_ticks: ", decision_trace_every_n_ticks]),
        LogInfo(msg=["[sentry_all] use_tf_tree: ", use_tf_tree]),
        LogInfo(msg=["[sentry_all] use_navi_tf_bridge: ", use_navi_tf_bridge]),
        LogInfo(msg=["[sentry_all] resolved_use_navi_tf_bridge: ", resolved_use_navi_tf_bridge]),
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
                "output_goal_pose_topic": "/goal_pose",
                "publish_goal_pose": "true",
                "publish_goal_pos": "false",
                "enable_goal_pos_raw_bridge": "true",
                "goal_pos_raw_frame": "map",
            }.items(),
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
                        },
                    ],
                    on_exit=Shutdown(reason="gimbal_driver exited"),
                    condition=IfCondition(virtual_io_expr),
                ),
            ],
            condition=IfCondition(use_gimbal),
        ),
        # detector: offline=true uses video replay; rosbag replay uses the compressed image topic.
        GroupAction(
            actions=[
                Node(
                    package="detector",
                    executable="detector_node",
                    name="detector",
                    output=output,
                    parameters=[base_config_file, detector_config_file, config_file],
                    on_exit=Shutdown(reason="detector exited"),
                    condition=IfCondition(hardware_io_expr),
                ),
                Node(
                    package="detector",
                    executable="detector_node",
                    name="detector",
                    output=output,
                    parameters=[
                        base_config_file,
                        detector_config_file,
                        config_file,
                        {
                            "detector_config/use_video": True,
                            "detector_config.use_video": True,
                        },
                    ],
                    on_exit=Shutdown(reason="detector exited"),
                    condition=IfCondition(offline_video_expr),
                ),
                Node(
                    package="detector",
                    executable="detector_node",
                    name="detector",
                    output=output,
                    parameters=[
                        base_config_file,
                        detector_config_file,
                        config_file,
                        {
                            "detector_config/use_video": False,
                            "detector_config.use_video": False,
                            "detector_config/use_ros_bag": True,
                            "detector_config.use_ros_bag": True,
                        },
                    ],
                    on_exit=Shutdown(reason="detector exited"),
                    condition=IfCondition(rosbag_enabled_expr),
                ),
            ],
            condition=IfCondition(use_detector),
        ),
        # 下游链路节点
        Node(
            package="tracker_solver",
            executable="tracker_solver_node",
            name="tracker_solver",
            output=output,
            parameters=[
                base_config_file,
                predictor_config_file,
                config_file,
                {
                    "aim_timer_log.enable": ParameterValue(aim_timer_log_enable, value_type=bool),
                    "aim_timer_log/enable": ParameterValue(aim_timer_log_enable, value_type=bool),
                    "aim_timer_log.dir": ParameterValue(aim_timer_log_dir, value_type=str),
                    "aim_timer_log/dir": ParameterValue(aim_timer_log_dir, value_type=str),
                },
            ],
            on_exit=Shutdown(reason="tracker_solver exited"),
            condition=IfCondition(use_tracker),
        ),
        Node(
            package="predictor",
            executable="predictor_node",
            name="predictor_node",
            output=output,
            parameters=[
                base_config_file,
                predictor_config_file,
                config_file,
                {
                    "aim_timer_log.enable": ParameterValue(aim_timer_log_enable, value_type=bool),
                    "aim_timer_log/enable": ParameterValue(aim_timer_log_enable, value_type=bool),
                    "aim_timer_log.dir": ParameterValue(aim_timer_log_dir, value_type=str),
                    "aim_timer_log/dir": ParameterValue(aim_timer_log_dir, value_type=str),
                },
            ],
            on_exit=Shutdown(reason="predictor_node exited"),
            condition=IfCondition(use_predictor),
        ),
        Node(
            package="outpost_hitter",
            executable="outpost_hitter_node",
            name="outpost_hitter",
            output=output,
            parameters=[base_config_file, outpost_config_file, config_file],
            on_exit=Shutdown(reason="outpost_hitter exited"),
            condition=IfCondition(use_outpost),
        ),
        Node(
            package="buff_hitter",
            executable="buff_hitter_node",
            name="buff_hitter",
            output=output,
            parameters=[base_config_file, buff_config_file, config_file],
            on_exit=Shutdown(reason="buff_hitter exited"),
            condition=IfCondition(use_buff),
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
                {
                    "competition_profile": resolved_competition_profile,
                    "bt_config_file": resolved_bt_config_file,
                    "bt_tree_file": bt_tree_file,
                    "debug_bypass_is_start": debug_bypass_is_start,
                    "runtime_rearm_start_gate": runtime_rearm_start_gate,
                    "publish_navi_goal": publish_navi_goal,
                    "wait_for_game_start_timeout_sec": wait_for_game_start_timeout_sec,
                    "league_referee_stale_timeout_ms": league_referee_stale_timeout_ms,
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
