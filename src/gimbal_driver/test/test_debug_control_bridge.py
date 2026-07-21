import importlib.util
import sys
from pathlib import Path

import yaml


STATE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "debug_control_state.py"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("debug_control_state", STATE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
STATE_MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = STATE_MODULE
SPEC.loader.exec_module(STATE_MODULE)


def make_state():
    return STATE_MODULE.DebugControlState(
        stale_timeout_ns=500_000_000,
        rotate_level=2,
        follow_mode_when_false=True,
    )


def test_aim_result_follow_is_validity_not_follow_mode():
    state = make_state()
    state.update_should_rotate(False)
    state.update_aim(
        follow=True,
        fire=False,
        yaw=12.0,
        pitch=-3.0,
        yaw_omega=1.5,
        pitch_omega=-2.5,
        yaw_alpha=3.5,
        pitch_alpha=-4.5,
        received_ns=10,
    )

    snapshot = state.snapshot(20, navi_mode=True, aim_mode=True)

    assert snapshot.angles == (12.0, -3.0)
    assert snapshot.aim_mode is True
    assert snapshot.follow_mode is True
    assert snapshot.rotate == 0
    assert snapshot.trajectory == (12.0, -3.0, 1.5, -2.5, 3.5, -4.5)


def test_disabled_inputs_do_not_own_outputs():
    state = make_state()
    state.update_navigation(40, -10, 0)
    state.update_aim(follow=True, fire=True, yaw=1.0, pitch=2.0, received_ns=0)

    snapshot = state.snapshot(10, navi_mode=False, aim_mode=False)

    assert snapshot.velocity is None
    assert snapshot.angles is None
    assert snapshot.aim_mode is False
    assert snapshot.fire_toggle is False


def test_fire_toggles_once_per_fresh_valid_aim_message():
    state = make_state()
    state.update_aim(follow=True, fire=True, yaw=10.0, pitch=4.0, received_ns=100)

    first = state.snapshot(110, navi_mode=False, aim_mode=True)
    second = state.snapshot(120, navi_mode=False, aim_mode=True)

    assert first.fire_toggle is True
    assert second.fire_toggle is False
    assert first.angles == (10.0, 4.0)


def test_stale_aim_never_fires_and_releases_angles_for_patrol():
    state = make_state()
    state.update_aim(follow=True, fire=True, yaw=10.0, pitch=4.0, received_ns=0)

    snapshot = state.snapshot(500_000_001, navi_mode=False, aim_mode=True)

    assert snapshot.aim_mode is False
    assert snapshot.fire_toggle is False
    assert snapshot.angles is None


def test_invalid_aim_never_controls_angles_or_fire():
    state = make_state()
    state.update_aim(
        follow=False,
        fire=True,
        yaw=float("nan"),
        pitch=4.0,
        received_ns=0,
    )

    snapshot = state.snapshot(10, navi_mode=False, aim_mode=True)

    assert snapshot.aim_mode is False
    assert snapshot.fire_toggle is False
    assert snapshot.angles is None
    assert snapshot.trajectory is None


def test_nonfinite_aim_kinematics_never_controls_trajectory():
    state = make_state()
    state.update_aim(
        follow=True,
        fire=False,
        yaw=10.0,
        pitch=4.0,
        yaw_omega=float("nan"),
        pitch_omega=0.0,
        yaw_alpha=0.0,
        pitch_alpha=0.0,
        received_ns=0,
    )

    snapshot = state.snapshot(10, navi_mode=False, aim_mode=True)

    assert snapshot.aim_mode is False
    assert snapshot.angles is None
    assert snapshot.trajectory is None


def test_stale_navigation_publishes_zero_velocity():
    state = make_state()
    state.update_navigation(40, -10, 0)

    snapshot = state.snapshot(500_000_001, navi_mode=True, aim_mode=False)

    assert snapshot.velocity == (0, 0)


def test_debug_raw_downlink_mode_has_an_explicit_default_and_launch_contract():
    debug_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "debug_mode.yaml").read_text(encoding="utf-8")
    )
    debug_parameters = debug_config["/**"]["ros__parameters"]
    debug_launch = (PACKAGE_ROOT / "launch" / "debug_node.launch.py").read_text(encoding="utf-8")
    driver_launch = (PACKAGE_ROOT / "launch" / "gimbal_driver.launch.py").read_text(encoding="utf-8")

    assert debug_parameters["raw_downlink_test_mode"] is False
    assert "load_raw_downlink_test_mode" in debug_launch
    assert '"raw_downlink_test_mode"' in driver_launch
    assert '"io_config/raw_downlink_test_mode"' in driver_launch
