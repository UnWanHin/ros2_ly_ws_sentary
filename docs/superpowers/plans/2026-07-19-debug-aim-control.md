# Debug Aim Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend `debug_node.launch.py` so its single control bridge can independently test formal navigation and formal external aim output without starting the behavior tree.

**Architecture:** Keep `navi_vel_to_control_vel.py` as the only debug publisher for `/ly/control/vel`, `/ly/control/angles`, and `/ly/control/firecode`. Add pure helpers for the public mode/stale/fire-state rules, cover them with Python tests, then let the ROS node subscribe only to the inputs enabled by `debug_mode.yaml`. The driver remains unchanged as the formal `/ly/control/*` subscriber and serial owner.

**Tech Stack:** ROS2 Humble, Python `rclpy`, `sentry_msgs/msg/AimResult`, `gimbal_driver` generated messages, `pytest`, `colcon`.

## Global Constraints

- Do not run `debug_node.launch.py` together with `behavior_tree`; both would own formal control topics.
- `AimResult.follow` is aim-result validity, never `FireCode.follow_mode`.
- `navi_mode`, `aim_mode`, and `patrol` are independent boolean parameters in `debug_mode.yaml`.
- FollowMode/Rotate are owned by navigation; AimMode/FireStatus are owned by fresh valid aim results; one bridge emits the merged FireCode snapshot.
- `/ly/control/trajectory` remains an independent gimbal_driver `0x05` path and is out of scope.
- Keep `publish_hz=100.0` and `stale_timeout_ms=500` defaults unless the profile overrides them.
- Do not add the user-owned `docs/rules/.~lock*.pdf#` file.

---

### Task 1: Make debug control state testable and add configuration gates

**Files:**
- Modify: `src/gimbal_driver/scripts/navi_vel_to_control_vel.py`
- Modify: `src/gimbal_driver/config/debug_mode.yaml`
- Modify: `src/gimbal_driver/CMakeLists.txt`
- Create: `src/gimbal_driver/test/test_debug_control_bridge.py`

**Interfaces:**
- Consumes: `gimbal_driver/msg/Vel`, `GimbalAngles`, `FireCode`, `ControlVelocity`; `std_msgs/msg/Bool`; `sentry_msgs/msg/AimResult`.
- Produces: `DebugControlState`, a ROS-independent helper that receives navigation/aim samples and returns a 100 Hz control snapshot.
- Produces: `NaviVelToControlVel` subscriptions gated by `navi_mode` and `aim_mode`.

- [ ] **Step 1: Write failing tests for the mode contract and formal aim validity**

```python
def test_aim_result_follow_is_validity_not_follow_mode():
    state = DebugControlState(stale_timeout_ns=500_000_000, rotate_level=2,
                              follow_mode_when_false=True)
    state.update_should_rotate(False, 0)
    state.update_aim(True, True, 12.0, -3.0, 10)

    snapshot = state.snapshot(20, navi_mode=True, aim_mode=True, patrol=False)

    assert snapshot.angles == (12.0, -3.0)
    assert snapshot.aim_mode is True
    assert snapshot.follow_mode is True
    assert snapshot.rotate == 0


def test_disabled_inputs_do_not_own_outputs():
    state = DebugControlState(stale_timeout_ns=500_000_000, rotate_level=3,
                              follow_mode_when_false=True)
    state.update_navigation(40, -10, 0)
    state.update_aim(True, True, 1.0, 2.0, 0)

    snapshot = state.snapshot(10, navi_mode=False, aim_mode=False, patrol=False)

    assert snapshot.velocity is None
    assert snapshot.angles is None
    assert snapshot.aim_mode is False
    assert snapshot.fire_toggle is False
```

- [ ] **Step 2: Run the focused test and confirm it fails because the helper is absent**

Run: `python3 -m pytest src/gimbal_driver/test/test_debug_control_bridge.py -q`

Expected: collection/import failure for `DebugControlState`.

- [ ] **Step 3: Add the minimal pure state helper and node parameters**

```python
@dataclass(frozen=True)
class ControlSnapshot:
    velocity: tuple[int, int] | None
    angles: tuple[float, float] | None
    follow_mode: bool
    rotate: int
    aim_mode: bool
    fire_toggle: bool


class DebugControlState:
    def __init__(self, stale_timeout_ns: int, rotate_level: int,
                 follow_mode_when_false: bool) -> None:
        self.stale_timeout_ns = stale_timeout_ns
        self.rotate_level = rotate_level
        self.follow_mode_when_false = follow_mode_when_false
        self.last_navigation_ns = None
        self.latest_velocity = (0, 0)
        self.should_rotate = True
        self.last_aim_ns = None
        self.latest_aim = None
        self.pending_fire_toggle = False

    def snapshot(self, now_ns: int, *, navi_mode: bool, aim_mode: bool,
                 patrol: bool) -> ControlSnapshot:
        velocity = self.latest_velocity if (
            navi_mode and self.last_navigation_ns is not None and
            now_ns - self.last_navigation_ns <= self.stale_timeout_ns
        ) else ((0, 0) if navi_mode else None)
        follow_mode = navi_mode and self.follow_mode_when_false and not self.should_rotate
        rotate = self.rotate_level if navi_mode and self.should_rotate else 0
        aim_is_fresh = aim_mode and self.latest_aim is not None and (
            now_ns - self.last_aim_ns <= self.stale_timeout_ns
        )
        angles = self.latest_aim if aim_is_fresh else None
        fire_toggle = aim_is_fresh and self.pending_fire_toggle
        self.pending_fire_toggle = False
        return ControlSnapshot(velocity, angles, follow_mode, rotate,
                               aim_is_fresh, fire_toggle)
```

Declare `navi_mode` and `aim_mode` with defaults `True` and `False`. Construct navigation subscriptions only when `navi_mode` is true; construct the `AimResult` subscription only when `aim_mode` is true. Add `<exec_depend>sentry_msgs</exec_depend>` and `<test_depend>ament_cmake_pytest</test_depend>` to `package.xml`; add `find_package(ament_cmake_pytest REQUIRED)` and:

```cmake
ament_add_pytest_test(test_debug_control_bridge
  test/test_debug_control_bridge.py)
```

- [ ] **Step 4: Run the focused test and confirm it passes**

Run: `python3 -m pytest src/gimbal_driver/test/test_debug_control_bridge.py -q`

Expected: `2 passed`.

- [ ] **Step 5: Commit the testable state and mode gates**

```bash
git add src/gimbal_driver/scripts/navi_vel_to_control_vel.py \
  src/gimbal_driver/config/debug_mode.yaml src/gimbal_driver/CMakeLists.txt \
  src/gimbal_driver/package.xml src/gimbal_driver/test/test_debug_control_bridge.py
git commit -m "gimbal_driver: gate debug navigation and aim inputs"
```

### Task 2: Merge aim, navigation, and patrol into one output

**Files:**
- Modify: `src/gimbal_driver/scripts/navi_vel_to_control_vel.py`
- Modify: `src/gimbal_driver/test/test_debug_control_bridge.py`

**Interfaces:**
- Consumes: `DebugControlState.snapshot()` from Task 1 and current `/ly/gimbal/angles` feedback.
- Produces: exactly one `/ly/control/firecode` snapshot per debug tick; optional velocity/angles outputs according to mode gates.

- [ ] **Step 1: Add failing tests for stale, fire, and patrol priority**

```python
def test_fire_toggles_once_per_fresh_valid_aim_message():
    state = DebugControlState(stale_timeout_ns=500_000_000, rotate_level=0,
                              follow_mode_when_false=True)
    state.update_aim(True, True, 10.0, 4.0, 100)

    first = state.snapshot(110, navi_mode=False, aim_mode=True, patrol=True)
    second = state.snapshot(120, navi_mode=False, aim_mode=True, patrol=True)

    assert first.fire_toggle is True
    assert second.fire_toggle is False
    assert first.angles == (10.0, 4.0)


def test_stale_aim_uses_patrol_but_never_fires():
    state = DebugControlState(stale_timeout_ns=500_000_000, rotate_level=0,
                              follow_mode_when_false=True)
    state.update_aim(True, True, 10.0, 4.0, 0)

    snapshot = state.snapshot(500_000_001, navi_mode=False, aim_mode=True, patrol=True)

    assert snapshot.aim_mode is False
    assert snapshot.fire_toggle is False
    assert snapshot.angles is None  # node replaces this with canonical patrol output
```

- [ ] **Step 2: Run the focused test and confirm it fails**

Run: `python3 -m pytest src/gimbal_driver/test/test_debug_control_bridge.py -q`

Expected: one or more assertions fail because fire is not edge-consumed or stale aim is still active.

- [ ] **Step 3: Wire the helper into the ROS timer**

The timer must:

```python
snapshot = self.control_state.snapshot(
    now.nanoseconds,
    navi_mode=self.navi_mode,
    aim_mode=self.aim_mode,
    patrol=self.patrol,
)
if snapshot.velocity is not None:
    self.publish_velocity(snapshot.velocity, now)
self.publish_merged_firecode(snapshot, now)
if snapshot.angles is not None:
    self.publish_angles(snapshot.angles, now)
elif self.patrol and self.feedback_angles is not None:
    self.publish_angles(self.patrol_angles(now.nanoseconds), now)
elif self.feedback_angles is not None:
    self.publish_angles((self.feedback_angles.yaw, self.feedback_angles.pitch), now)
```

Maintain a local two-bit FireStatus state initialized from `/ly/gimbal/firecode` feedback. Consume a
`fire_toggle` exactly once by XOR-ing that local value with `0b11`. Publish a full `FIELD_ALL`
FireCode snapshot so navigation and aim fields always reach the lower machine together.

- [ ] **Step 4: Run targeted package tests**

Run: `colcon test --packages-select gimbal_driver --event-handlers console_direct+`

Expected: gimbal_driver tests pass, including `test_debug_control_bridge` and existing MPC protocol test.

- [ ] **Step 5: Commit the merged control behavior**

```bash
git add src/gimbal_driver/scripts/navi_vel_to_control_vel.py \
  src/gimbal_driver/test/test_debug_control_bridge.py
git commit -m "gimbal_driver: route formal aim through debug bridge"
```

### Task 3: Update launch contract, documentation, graph, and runtime checks

**Files:**
- Modify: `src/gimbal_driver/launch/debug_node.launch.py`
- Modify: `docs/modules/2026-05-05_gimbal_driver.md`
- Modify: `docs/sentry/internal/ros2_topic_structure.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

**Interfaces:**
- Consumes: implemented mode parameters and the existing launch composition.
- Produces: documented standalone debug entry point with current graph ownership/edges.

- [ ] **Step 1: Add source/launch validation assertions before documentation edits**

Run:

```bash
python3 -m py_compile src/gimbal_driver/scripts/navi_vel_to_control_vel.py \
  src/gimbal_driver/launch/debug_node.launch.py
rg -n 'navi_mode|aim_mode|/ly/aim/result|FIELD_ALL' \
  src/gimbal_driver/scripts/navi_vel_to_control_vel.py \
  src/gimbal_driver/config/debug_mode.yaml
```

Expected: the bridge declares all three debug parameters, subscribes `/ly/aim/result` only behind
`aim_mode`, and emits `FIELD_ALL` FireCode snapshots.

- [ ] **Step 2: Update the public descriptions**

Document the launch command, three flags, control ownership, fresh/stale behavior, and the hard
rule that debug node and BT cannot run together. In the graph, replace the old navigation-only
debug edge with the debug-control bridge edges from navigation, external aim, Patrol.yaml, then
to all three formal control topics. Advance graph metadata to the actual committed source HEAD and
record fallback graph mode if plugin regeneration remains unavailable.

- [ ] **Step 3: Build and run static verification**

Run:

```bash
source /opt/ros/humble/setup.bash
source ../sentry.common/install/setup.bash
colcon build --packages-select gimbal_driver --symlink-install
source install/setup.bash
colcon test --packages-select gimbal_driver
colcon test-result --verbose
./scripts/selfcheck.sh sentry --static-only
python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null
git diff --check
```

Expected: build passes; gimbal_driver tests pass; static selfcheck passes; graph JSON parses; no
whitespace errors. Record any external hardware/runtime check that cannot run locally.

- [ ] **Step 4: Commit integration documentation and graph freshness**

```bash
git add src/gimbal_driver/launch/debug_node.launch.py \
  docs/modules/2026-05-05_gimbal_driver.md \
  docs/sentry/internal/ros2_topic_structure.md \
  .understand-anything/knowledge-graph.json \
  .understand-anything/project-knowledge-graph.md \
  .understand-anything/meta.json
git commit -m "docs: record debug aim control ownership"
```
