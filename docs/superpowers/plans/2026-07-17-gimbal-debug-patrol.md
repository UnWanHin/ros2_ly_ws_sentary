# Gimbal Debug Patrol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make standalone `debug_node` preserve fresh gimbal feedback by default and optionally perform the exact configured formal patrol scan at 100 Hz.

**Architecture:** Move the deterministic Mode1/2/3 scan calculation into a header-only `auto_aim_common` core. `behavior_tree` calls that core for its normal no-target patrol path, while a new C++ `gimbal_driver` debug bridge owns all three formal control topic publishers. The bridge is gated by fresh `/ly/gimbal/angles`; it holds feedback angles when patrol is disabled and runs the shared scanner when enabled.

**Tech Stack:** ROS2 Humble, C++20, rclcpp, gtest, ROS parameter YAML, colcon.

## Global Constraints

- `debug_mode.yaml` owns `patrol.enabled`; its default is `false`.
- `PatrolScan.Mode` and Mode1/2/3 curves exist only in canonical `src/behavior_tree/config/Patrol.yaml`.
- Debug mode must not launch or run beside `behavior_tree`.
- No control topic is published until `/ly/gimbal/angles` is present and fresher than 1200 ms.
- Debug output cadence is 100 Hz by default and uses only `/ly/control/angles`, `/ly/control/vel`, and partial `/ly/control/firecode`.
- Preserve existing external topics and serial `DownlinkTypeID=0x00` format.

---

### Task 1: Add the shared patrol scan core and its deterministic tests

**Files:**
- Create: `src/auto_aim_common/include/auto_aim_common/patrol_scan.hpp`
- Create: `src/auto_aim_common/test/test_patrol_scan.cpp`
- Modify: `src/auto_aim_common/CMakeLists.txt`

**Interfaces:**
- Produces `ly_auto_aim::patrol::Config`, `Angles`, `Scanner::Reset`, and `Scanner::Step`.
- `Step` consumes one feedback angle, one monotonic elapsed time in milliseconds, a mode `1..3`, and a `Config`; it returns the next absolute yaw/pitch command.

- [ ] **Step 1: Write failing scanner tests**

```cpp
#include <auto_aim_common/patrol_scan.hpp>
#include <gtest/gtest.h>

TEST(PatrolScanner, Mode1AdvancesYawAndUsesPitchCurve) {
  ly_auto_aim::patrol::Config config{};
  config.mode1_yaw_step_deg = 9.0;
  config.mode1_pitch_center_deg = 5.0;
  config.mode1_pitch_half_range_deg = 15.0;
  config.mode1_pitch_period_ms = 2000.0;
  ly_auto_aim::patrol::Scanner scanner;
  const auto command = scanner.Step({10.0, 4.0}, 0, 1, config);
  EXPECT_DOUBLE_EQ(command.yaw_deg, 19.0);
  EXPECT_DOUBLE_EQ(command.pitch_deg, 5.0);
}

TEST(PatrolScanner, Mode2AnchorsAtFeedbackAndOscillates) {
  ly_auto_aim::patrol::Config config{};
  config.mode2_yaw_step_deg = 1.0;
  config.mode2_yaw_half_range_deg = 30.0;
  ly_auto_aim::patrol::Scanner scanner;
  const auto first = scanner.Step({42.0, 0.0}, 0, 2, config);
  EXPECT_GT(first.yaw_deg, 42.0);
  scanner.Reset();
  const auto reseeded = scanner.Step({7.0, 0.0}, 0, 2, config);
  EXPECT_GT(reseeded.yaw_deg, 7.0);
}

TEST(PatrolScanner, Mode3UsesConfiguredHighPitchCurve) {
  ly_auto_aim::patrol::Config config{};
  config.mode3_yaw_step_deg = 6.0;
  config.mode3_pitch_offset_deg = 20.0;
  config.mode3_pitch_half_range_deg = 0.0;
  ly_auto_aim::patrol::Scanner scanner;
  const auto command = scanner.Step({-5.0, 0.0}, 100, 3, config);
  EXPECT_DOUBLE_EQ(command.yaw_deg, 1.0);
  EXPECT_DOUBLE_EQ(command.pitch_deg, 20.0);
}
```

- [ ] **Step 2: Run the new test target and verify compilation fails because the header is absent**

Run: `colcon test --packages-select auto_aim_common --ctest-args -R test_patrol_scan --output-on-failure`

Expected: compilation failure mentioning `auto_aim_common/patrol_scan.hpp`.

- [ ] **Step 3: Add the header-only scanner**

```cpp
namespace ly_auto_aim::patrol {
struct Angles { double yaw_deg{0.0}; double pitch_deg{0.0}; };
struct Config { /* Mode1/2/3 fields matching Patrol.yaml */ };
class Scanner {
 public:
  void Reset() noexcept;
  Angles Step(Angles feedback, double elapsed_ms, int mode, const Config& config);
 private:
  int active_mode_{0};
  double center_yaw_deg_{0.0};
  double phase_rad_{0.0};
  bool initialized_{false};
};
}
```

Implement the current `GameLoop.cpp` formulas exactly: Mode1/3 advance from
feedback yaw; Mode2 initializes its center from feedback yaw, advances phase by
`yaw_step / max(half_range, 1)`, applies center drift per cycle, and uses the
same pitch sine curve. Keep all units in degrees and milliseconds.

- [ ] **Step 4: Register the test target**

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_patrol_scan test/test_patrol_scan.cpp)
  target_link_libraries(test_patrol_scan ${PROJECT_NAME}_interface)
endif()
```

- [ ] **Step 5: Run the core tests and commit**

Run: `colcon test --packages-select auto_aim_common --ctest-args -R test_patrol_scan --output-on-failure`

Expected: all three tests pass.

```bash
git add src/auto_aim_common
git commit -m "auto_aim_common: add patrol scan core"
```

### Task 2: Move formal BT patrol calculation onto the shared core

**Files:**
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/CMakeLists.txt`
- Test: `src/auto_aim_common/test/test_patrol_scan.cpp`

**Interfaces:**
- Consumes `ly_auto_aim::patrol::Scanner` and `Config` from Task 1.
- Produces unchanged `/ly/control/angles` behavior for no-target Mode1/2/3 patrol.

- [ ] **Step 1: Add a regression assertion for the Mode2 default profile**

Extend `test_patrol_scan.cpp` with canonical Mode2 values from `Patrol.yaml`
and assert the first command has yaw `30 * sin(1 / 30)` degrees above its
feedback anchor and pitch `0` at elapsed `0`.

- [ ] **Step 2: Run the test before integration**

Run: `colcon test --packages-select auto_aim_common --ctest-args -R test_patrol_scan --output-on-failure`

Expected: PASS; this locks the calculation used by BT before replacing its
inline copy.

- [ ] **Step 3: Replace only the normal GameLoop patrol formula**

Add `ly_auto_aim::patrol::Scanner patrolScanner_` to `Application` and map
`LangYa::PatrolScanSetting` into the shared `Config` in one local helper.
Replace the block beginning at `int patrol_mode = config.PatrolScanSettings.Mode`
with:

```cpp
const auto patrol_angles = patrolScanner_.Step(
    {gimbalAngles.Yaw, gimbalAngles.Pitch}, pitch_elapsed_ms, patrol_mode,
    MakePatrolConfig(config.PatrolScanSettings));
nextAngles = GimbalAnglesType{
    static_cast<AngleType>(patrol_angles.yaw_deg),
    static_cast<AngleType>(patrol_angles.pitch_deg)};
```

Keep current task-level mode selection, damage boost choice, and Outpost pitch
offset in `GameLoop.cpp`; reset `patrolScanner_` everywhere the existing code
calls `reset_patrol_scan_state`.

- [ ] **Step 4: Build and run targeted formal tests**

Run: `colcon build --packages-select auto_aim_common behavior_tree --symlink-install`

Run: `colcon test --packages-select auto_aim_common behavior_tree --ctest-args -R 'test_patrol_scan|test_face_mode_manager' --output-on-failure`

Expected: both targets pass; no public topic/message changes.

- [ ] **Step 5: Commit the formal integration**

```bash
git add src/behavior_tree src/auto_aim_common/test/test_patrol_scan.cpp
git commit -m "behavior_tree: share patrol scan calculation"
```

### Task 3: Replace the unsafe Python debug bridge with a C++ formal-control bridge

**Files:**
- Create: `src/gimbal_driver/src/debug_control_bridge.cpp`
- Create: `src/gimbal_driver/test/test_debug_control_bridge.cpp`
- Modify: `src/gimbal_driver/CMakeLists.txt`
- Modify: `src/gimbal_driver/package.xml`
- Delete: `src/gimbal_driver/scripts/navi_vel_to_control_vel.py`

**Interfaces:**
- Consumes `/ly/gimbal/angles`, `/ly/navi/vel`, and `/ly/navi/should_rotate`.
- Produces `/ly/control/angles`, `/ly/control/vel`, and partial `/ly/control/firecode` at `publish_hz`.
- Uses `auto_aim_common` `Scanner` only when `patrol.enabled=true`.

- [ ] **Step 1: Write a unit-testable command-state model and failing tests**

```cpp
TEST(DebugControlState, RefusesControlBeforeFreshFeedback) {
  DebugControlState state(DebugOptions{});
  EXPECT_FALSE(state.MakeCommand(100).has_value());
}

TEST(DebugControlState, DisabledPatrolHoldsFeedbackAngle) {
  DebugControlState state(DebugOptions{.patrol_enabled = false});
  state.OnFeedback({12.0, -3.0}, 1000);
  const auto command = state.MakeCommand(1100).value();
  EXPECT_DOUBLE_EQ(command.angle.yaw, 12.0);
  EXPECT_DOUBLE_EQ(command.angle.pitch, -3.0);
}

TEST(DebugControlState, EnabledPatrolUsesSharedScanner) {
  DebugControlState state(DebugOptions{.patrol_enabled = true, .patrol_mode = 2});
  state.OnFeedback({12.0, -3.0}, 1000);
  const auto command = state.MakeCommand(1010).value();
  EXPECT_NE(command.angle.yaw, 12.0);
}
```

- [ ] **Step 2: Run the debug bridge test and verify it fails before implementation**

Run: `colcon test --packages-select gimbal_driver --ctest-args -R test_debug_control_bridge --output-on-failure`

Expected: target or symbols are absent.

- [ ] **Step 3: Implement the state model and ROS node**

Declare these parameters: `rotate_level`, `follow_mode_when_false`,
`stale_timeout_ms`, `publish_hz`, `patrol.enabled`, and
`angle_feedback_stale_timeout_ms`, plus all `PatrolScan.*` fields supplied by
the canonical parameter file. On each timer tick:

```cpp
const auto command = state.MakeCommand(now_ms);
if (!command) return;
publish_angles(command->angle);
publish_velocity(command->velocity);
publish_partial_firecode(command->should_rotate);
```

`MakeCommand` returns no value before first feedback or after its 1200 ms
freshness limit. It resets the scanner whenever feedback becomes stale. With
patrol disabled it returns the last feedback angle unchanged; with patrol
enabled it calls the Task 1 shared scanner using `PatrolScan.Mode`.

- [ ] **Step 4: Build and run debug bridge tests**

Run: `colcon build --packages-select auto_aim_common gimbal_driver --symlink-install`

Run: `colcon test --packages-select auto_aim_common gimbal_driver --ctest-args -R 'test_patrol_scan|test_debug_control_bridge' --output-on-failure`

Expected: all tests pass and `ros2 pkg executables gimbal_driver` includes
`debug_control_bridge`.

- [ ] **Step 5: Commit the safe bridge**

```bash
git add src/gimbal_driver src/auto_aim_common
git commit -m "gimbal_driver: add safe debug patrol bridge"
```

### Task 4: Wire canonical configuration, launch, documentation, and graph

**Files:**
- Modify: `src/gimbal_driver/config/debug_mode.yaml`
- Modify: `src/gimbal_driver/launch/debug_node.launch.py`
- Modify: `src/behavior_tree/config/Patrol.yaml`
- Modify: `src/gimbal_driver/CMakeLists.txt`
- Modify: `docs/modules/2026-05-05_gimbal_driver.md`
- Modify: `docs/sentry/internal/ros2_topic_structure.md`
- Modify: `docs/sentry/internal/ros2_topic_tree.md`
- Modify: `docs/architecture/2026-07-12_project_link_graph.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`
- Modify: `scripts/selfcheck/sentry.sh`

**Interfaces:**
- `debug_mode.yaml` supplies `patrol.enabled=false` and the 1200 ms feedback
  freshness timeout.
- `debug_node.launch.py` supplies `patrol_config_file`, defaulting to the
  installed `behavior_tree/config/Patrol.yaml`, only to `debug_control_bridge`.

- [ ] **Step 1: Add the failing static contract**

Add a `selfcheck` assertion that `debug_node.launch.py` starts
`debug_control_bridge`, passes the canonical patrol config, and does not start
`behavior_tree`; assert `debug_mode.yaml` defaults `patrol.enabled` to false.

- [ ] **Step 2: Run the contract before wiring**

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: debug patrol contract fails until the launch/config changes exist.

- [ ] **Step 3: Apply launch/config wiring**

Use `/**` as the root in `Patrol.yaml`, because that file is passed only to the
formal BT or debug bridge and must apply to either node name without duplicating
the patrol parameters. Install the C++ bridge executable and remove the old
Python script from `install(PROGRAMS)`. In debug YAML add:

```yaml
    patrol:
      enabled: false
    angle_feedback_stale_timeout_ms: 1200
```

The launch must pass both `debug_mode.yaml` and `patrol_config_file` to the
bridge, but never pass the debug YAML to `gimbal_driver` itself.

- [ ] **Step 4: Update docs and graph**

Document both debug states, feedback gating, 100 Hz publishers, the canonical
profile relationship, and the no-BT coexistence rule. Update the graph nodes
and edges so `/ly/gimbal/angles -> debug_control_bridge -> /ly/control/angles`
is explicit; update graph metadata to the current HEAD and source-checked
note.

- [ ] **Step 5: Verify static contracts, JSON, and build**

Run:

```bash
python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null
python3 -m json.tool .understand-anything/meta.json >/dev/null
git diff --check
./scripts/selfcheck.sh sentry --static-only
colcon build --packages-select auto_aim_common behavior_tree gimbal_driver --symlink-install
```

Expected: JSON and diff checks pass; static selfcheck reports the debug patrol
contract as PASS; the three package builds pass. Record any external
`sentry_msgs` limitation rather than attributing it to this change.

- [ ] **Step 6: Commit and push the integration**

```bash
git add src/gimbal_driver src/behavior_tree src/auto_aim_common docs scripts .understand-anything
git commit -m "gimbal_driver: align debug patrol with formal profile"
git push origin Behavion
```

### Task 5: Validate on the NUC hardware environment

**Files:**
- No repository changes expected.

**Interfaces:**
- Verifies deployed `debug_node` formal control ownership and DownlinkTypeID
  `0x00` output.

- [ ] **Step 1: Synchronize and build**

```bash
cd ~/ros2_ly_ws_sentry
git pull --ff-only origin Behavion
colcon build --packages-select auto_aim_common behavior_tree gimbal_driver --symlink-install
source install/setup.bash
```

- [ ] **Step 2: Validate feedback-hold mode**

Set `patrol.enabled: false`, launch debug node, then run:

```bash
ros2 topic info -v /ly/control/angles
ros2 topic echo /ly/control/angles
ros2 topic hz /ly/control/angles
```

Expected: exactly one publisher (`debug_control_bridge`), feedback-matching
yaw/pitch, and approximately 100 Hz only after gimbal feedback exists.

- [ ] **Step 3: Validate patrol mode**

Set `patrol.enabled: true` in `debug_mode.yaml`, relaunch, and repeat the
angle checks. Confirm the observed mode is the `PatrolScan.Mode` from the
active `patrol_config_file`; change only that canonical profile to validate a
Mode1-to-Mode2 switch.

- [ ] **Step 4: Validate complete downlink control frame**

Enable `io_config.serial_mode=true` and `download.typeid0x00`, then:

```bash
ros2 topic echo /ly/download/typeid0x00
ros2 topic echo /ly/control/vel
ros2 topic echo /ly/control/firecode
```

Expected: each debug command path uses the normal 0x00 frame; `should_rotate`
true yields configured `rotate_level`, while false yields Rotate=0 and the
configured FollowMode partial command.

## Plan Self-Review

- Spec coverage: Tasks 1-2 preserve a single patrol algorithm; Task 3 gates
  all debug output on feedback and owns formal controls; Task 4 puts the
  switch in debug YAML and updates documentation/graph; Task 5 covers NUC
  runtime and raw serial proof.
- Scope: no target selection, FaceMode, navigation goal, posture, or BT task
  fallback is added to debug mode.
- Interface consistency: `patrol.enabled` is the only debug enable switch;
  `PatrolScan.Mode` remains canonical and is passed through one
  `patrol_config_file` launch argument.
