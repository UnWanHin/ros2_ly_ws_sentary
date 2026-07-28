from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
COMMON_CONFIG = WORKSPACE_ROOT / "config" / "common.yaml"
START_SCRIPT = WORKSPACE_ROOT / "scripts" / "launch" / "start_sentry_all.sh"
SENTRY_LAUNCH = WORKSPACE_ROOT / "src" / "behavior_tree" / "launch" / "sentry_all.launch.py"
GIMBAL_LAUNCH = WORKSPACE_ROOT / "src" / "gimbal_driver" / "launch" / "gimbal_driver.launch.py"
GIMBAL_DRIVER = WORKSPACE_ROOT / "src" / "gimbal_driver" / "main.cpp"


def test_common_team_override_reaches_the_single_formal_publisher() -> None:
    common_config = COMMON_CONFIG.read_text(encoding="utf-8")
    start_script = START_SCRIPT.read_text(encoding="utf-8")
    sentry_launch = SENTRY_LAUNCH.read_text(encoding="utf-8")
    gimbal_launch = GIMBAL_LAUNCH.read_text(encoding="utf-8")
    gimbal_driver = GIMBAL_DRIVER.read_text(encoding="utf-8")

    assert "team_override:\n  Decide_Team: false\n  Red: false\n  Blue: false" in common_config
    assert 'add_common_bool_launch_arg "team_override.Decide_Team" "team_override_enable"' in start_script
    assert 'add_common_bool_launch_arg "team_override.Red" "team_override_red"' in start_script
    assert 'add_common_bool_launch_arg "team_override.Blue" "team_override_blue"' in start_script

    for name in ("team_override_enable", "team_override_red", "team_override_blue"):
        assert f'LaunchConfiguration("{name}")' in sentry_launch
        assert f'"{name}": {name},' in sentry_launch
        assert f'DeclareLaunchArgument(\n            "{name}"' in sentry_launch
        assert f'"{name}"' in gimbal_launch

    assert '"io_config/team_override/enable"' in gimbal_launch
    assert '"io_config/team_override/red"' in gimbal_launch
    assert '"io_config/team_override/blue"' in gimbal_launch
    assert gimbal_driver.count(
        'LY_DEF_ROS_TOPIC(ly_friend_is_team_red, "/ly/friend/is_team_red"'
    ) == 1
    assert "ResolveTeamOverride(\n                teamOverrideConfig_, data.GameCode.IsMyTeamRed)" in gimbal_driver
    assert "msg.data = effective_team.is_team_red;" in gimbal_driver
