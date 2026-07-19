import importlib.util
import sys
from pathlib import Path


STATE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "debug_control_state.py"
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
    state.update_aim(follow=True, fire=False, yaw=12.0, pitch=-3.0, received_ns=10)

    snapshot = state.snapshot(20, navi_mode=True, aim_mode=True)

    assert snapshot.angles == (12.0, -3.0)
    assert snapshot.aim_mode is True
    assert snapshot.follow_mode is True
    assert snapshot.rotate == 0


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
