# Navi Target Official Fallback Toggle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `config/common.yaml` able to disable camera-derived official enemy positions from entering BT decisions while preserving fresh lower-machine precedence.

**Architecture:** The formal starter reads one common-YAML boolean into a launch argument. `sentry_all.launch.py` passes that value only to `behavior_tree` as a `Chase` parameter override. The existing `/ly/navi/target_official` subscriber applies a pure acceptance predicate before mutating enemy state; `/ly/position/data` remains untouched.

**Tech Stack:** ROS2 Humble launch Python, Bash, C++20, GoogleTest, YAML, JSON configuration.

## Global Constraints

- New setting is `chase.enable_navi_target_official_fallback: true` in `config/common.yaml`.
- Default `true` preserves current behavior.
- `false` suppresses only BT writes from `/ly/navi/target_official`; it must not stop bridge publication, aim/fire, `/ly/navi/target_rel`, `/goal_pose`, or `/ly/position/data`.
- Fresh, non-zero lower-machine data remains higher priority when the setting is `true`.
- Do not change ROS topic names, message formats, or lower-machine protocol.

---

### Task 1: Define And Test The BT Acceptance Rule

**Files:**
- Create: `src/behavior_tree/include/EnemyPositionSourcePolicy.hpp`
- Create: `src/behavior_tree/test/test_enemy_position_source_policy.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- Consumes: `LangYa::ChaseSetting::EnableNaviTargetOfficialFallback` and the existing calculated `has_fresh_position_data` boolean.
- Produces: `BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(const LangYa::ChaseSetting&, bool)`.

- [x] **Step 1: Write the failing test**

```cpp
TEST(EnemyPositionSourcePolicyTest, RejectsCameraFallbackWhenDisabled) {
    LangYa::ChaseSetting setting;
    setting.EnableNaviTargetOfficialFallback = false;
    EXPECT_FALSE(BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(setting, false));
}

TEST(EnemyPositionSourcePolicyTest, PreservesFreshLowerMachinePrecedence) {
    LangYa::ChaseSetting setting;
    setting.EnableNaviTargetOfficialFallback = true;
    EXPECT_FALSE(BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(setting, true));
    EXPECT_TRUE(BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(setting, false));
}
```

- [x] **Step 2: Run the test to verify it fails**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_enemy_position_source_policy`

Expected: the test target is missing before it is added.

- [x] **Step 3: Add the smallest policy helper and CMake target**

```cpp
inline bool ShouldAcceptNaviTargetOfficialFallback(
    const LangYa::ChaseSetting& setting,
    const bool has_fresh_position_data) {
    return setting.EnableNaviTargetOfficialFallback && !has_fresh_position_data;
}
```

Add `ament_add_gtest(test_enemy_position_source_policy test/test_enemy_position_source_policy.cpp)` with the existing `include` and `module` include paths.

- [x] **Step 4: Run the focused test to verify it passes**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_enemy_position_source_policy && colcon test-result --verbose`

Expected: `test_enemy_position_source_policy` passes with two assertions groups.

### Task 2: Wire The Common-YAML Toggle Through Formal Launch And BT

**Files:**
- Modify: `config/common.yaml`
- Modify: `scripts/launch/start_sentry_all.sh`
- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/src/SubscribeMessage.cpp`

**Interfaces:**
- Consumes: `chase.enable_navi_target_official_fallback` from `config/common.yaml`.
- Produces: ROS parameter aliases `Chase.EnableNaviTargetOfficialFallback` and `Chase/EnableNaviTargetOfficialFallback`, then `config.ChaseSettings.EnableNaviTargetOfficialFallback`.

- [x] **Step 1: Add the configuration default and formal launch propagation**

```yaml
chase:
  # false: camera/TF official coordinates remain observable but do not enter BT decisions.
  enable_navi_target_official_fallback: true
```

In the starter, map this key to `enable_navi_target_official_fallback`. In `sentry_all.launch.py`, declare that launch argument with default `true`, log its effective value, and pass both dotted and slash aliases to the BT node as booleans.

- [x] **Step 2: Add configuration ownership**

Add `bool EnableNaviTargetOfficialFallback{true};` to `LangYa::ChaseSetting`, read the same-name JSON field in `from_json`, and let `ApplyChaseParameterOverrides()` read both ROS parameter aliases. Log its final value alongside existing Chase settings.

- [x] **Step 3: Gate only the camera-derived subscriber**

After the existing lower-machine freshness calculation in `SubscribeMessage.cpp`, return when `ShouldAcceptNaviTargetOfficialFallback(...)` is false. Keep the existing position-data callback unchanged.

- [x] **Step 4: Build and run focused regression coverage**

Run: `colcon build --packages-select behavior_tree && source install/setup.bash && colcon test --packages-select behavior_tree --ctest-args -R 'test_enemy_position_source_policy|test_chase_policy' && colcon test-result --verbose`

Expected: build completes and both focused tests pass.

### Task 3: Synchronize Current Documentation And Run Runtime-Adjacent Checks

**Files:**
- Modify: `docs/sentry/regional/bt_aim_navi_coordinate_chain.md`
- Modify: `docs/modules/2026-05-05_behavior_tree.md`

**Interfaces:**
- Documents: the exact common-YAML key, default, and the unchanged lower-machine-over-camera priority.

- [x] **Step 1: Update current source-of-truth docs**

State that `chase.enable_navi_target_official_fallback=false` suppresses only BT ingestion of `/ly/navi/target_official`; the bridge still publishes it for observation and fresh non-zero `/ly/position/data` remains authoritative when the gate is enabled.

- [x] **Step 2: Run repository checks**

Run: `bash -n scripts/launch/start_sentry_all.sh && python3 -m py_compile src/behavior_tree/launch/sentry_all.launch.py && ./scripts/selfcheck.sh sentry --static-only && git diff --check`

Expected: each command exits `0`.

Result: build, focused tests, shell/Python syntax, launch argument discovery, and diff checks passed.
`selfcheck.sh sentry --static-only` reached all local static contracts but returned non-zero because this
machine has no sourced `sentry.aim` overlay and therefore no `sentry_msgs` ROS interfaces.

- [ ] **Step 3: Commit the implementation**

Run: `git add config/common.yaml scripts/launch/start_sentry_all.sh src/behavior_tree docs/sentry/regional/bt_aim_navi_coordinate_chain.md docs/modules/2026-05-05_behavior_tree.md docs/superpowers/plans/2026-07-31-navi-target-official-fallback-toggle.md && git commit -m 'behavior_tree: gate camera official target fallback'`

Expected: only this feature and its plan are committed; do not stage `docs/rules/.~lock.*`.
