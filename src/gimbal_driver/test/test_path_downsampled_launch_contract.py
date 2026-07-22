from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
GIMBAL_LAUNCH = WORKSPACE_ROOT / "src" / "gimbal_driver" / "launch" / "gimbal_driver.launch.py"
SENTRY_LAUNCH = WORKSPACE_ROOT / "src" / "behavior_tree" / "launch" / "sentry_all.launch.py"
BRIDGE_LAUNCH = (
    WORKSPACE_ROOT
    / "src"
    / "navi_tf_bridge"
    / "launch"
    / "target_rel_to_goal_pos.launch.py"
)


def test_standalone_gimbal_launch_owns_path_downsampled_to_game_path_bridge() -> None:
    driver_launch = GIMBAL_LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_path_downsampled_bridge", default_value="true")' in driver_launch
    assert 'package="navi_tf_bridge"' in driver_launch
    assert 'executable="map_path_to_game_path_node"' in driver_launch
    assert '"input_topic": LaunchConfiguration("path_downsampled_topic")' in driver_launch
    assert '"output_topic": "/ly/game/path"' in driver_launch
    assert 'default_value="/Path_downsampled"' in driver_launch


def test_formal_stack_has_exactly_one_path_downsampled_bridge_owner() -> None:
    sentry_launch = SENTRY_LAUNCH.read_text(encoding="utf-8")
    bridge_launch = BRIDGE_LAUNCH.read_text(encoding="utf-8")

    assert '"enable_game_path_bridge": "false"' in sentry_launch
    assert '"enable_path_downsampled_bridge": "true"' in sentry_launch
    assert 'default_value="/Path_downsampled"' in bridge_launch
