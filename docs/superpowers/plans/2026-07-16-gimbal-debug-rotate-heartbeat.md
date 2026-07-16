# Gimbal Debug Rotate Heartbeat Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make standalone debug keep Rotate at 100 Hz and route navigation velocity through `/ly/control/vel`.

**Architecture:** A private driver scheduler invokes `ApplyNavigationModeRotate()` only when navigation direct-debug is enabled. `debug_node` additionally launches a 100 Hz `/ly/navi/vel -> /ly/control/vel` bridge with a 500 ms zero-velocity stale fallback; the driver keeps consuming the formal control topic. This leaves the formal BT chain untouched.

**Tech Stack:** ROS 2 Humble, C++20, Bash/Python selfcheck, Markdown.

## Global Constraints

- The heartbeat is only active when `io_config.navigation_mode.enabled=true`.
- The interval is 10 ms, targeting 100 Hz while retaining the 250 Hz main loop.
- The debug bridge is the sole `/ly/control/vel` publisher when `debug_node` runs; it cannot run beside BT.
- Do not stage the user-owned `docs/rules` deletions or lock file.

---

### Task 1: Add the debug heartbeat regression contract

**Files:**
- Modify: `scripts/selfcheck/sentry.sh`

**Interfaces:**
- Consumes: `src/gimbal_driver/main.cpp` source.
- Produces: a failing static selfcheck until the 10 ms heartbeat implementation exists.

- [ ] **Step 1: Write the failing test**

Require `kNavigationModeRotatePublishInterval = 10ms`,
`MaybeApplyNavigationModeRotateHeartbeat()`, and its main-loop invocation in
the gimbal debug profile contract.

- [ ] **Step 2: Run test to verify it fails**

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: the gimbal debug profile contract reports missing Rotate heartbeat
source tokens.

### Task 2: Implement the debug-only 100 Hz heartbeat

**Files:**
- Modify: `src/gimbal_driver/main.cpp`

**Interfaces:**
- Consumes: `navigationModeEnable_`, `navigationModeShouldRotate_`,
  `navigationModeRotateLevel_`, and the existing `ApplyNavigationModeRotate()`.
- Produces: `MaybeApplyNavigationModeRotateHeartbeat()` with a 10 ms gate.

- [ ] **Step 1: Add scheduler state and helper**

Add a 10 ms `std::chrono` interval and next-send timestamp. The helper returns
without writing when direct debug is disabled or the next deadline has not
arrived; otherwise it writes the current Rotate state and advances the deadline.

- [ ] **Step 2: Preserve immediate startup and callback writes**

After serial initialization, retain the immediate write and schedule the first
heartbeat deadline. Keep `should_rotate` callbacks immediate.

- [ ] **Step 3: Invoke from the existing main loop**

Call the helper beside the stale fallbacks. The formal mode remains inert
because `navigationModeEnable_` is false.

- [ ] **Step 4: Run the regression contract and build**

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: gimbal debug profile contract passes.

Run: `colcon build --packages-select gimbal_driver`

Expected: `gimbal_driver` builds successfully.

### Task 3: Record the operator-facing semantics

**Files:**
- Modify: `src/gimbal_driver/config/debug_mode.yaml`
- Create: `src/gimbal_driver/scripts/navi_vel_to_control_vel.py`
- Modify: `src/gimbal_driver/launch/debug_node.launch.py`
- Modify: `docs/modules/2026-05-05_gimbal_driver.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

**Interfaces:**
- Consumes: the implemented debug-only scheduling behavior.
- Produces: current configuration, module, and graph records.

- [ ] **Step 1: State the 100 Hz direct-debug contract**

Document that `rotate_level` is emitted without `/ly/navi/vel`, retained across
velocity updates, and overridden by fresh `should_rotate` values.

- [ ] **Step 2: Validate graph and documentation artifacts**

Run: `python3 -m json.tool .understand-anything/knowledge-graph.json`

Expected: valid JSON.

Run: `git diff --check`

Expected: no whitespace errors.

### Task 4: Review and publish the scoped change

**Files:**
- Review only: all files changed by Tasks 1-3.

- [ ] **Step 1: Inspect the staged diff and verify no user-owned rule files are staged**

Run: `git diff --staged --check`

Expected: no whitespace errors and no `docs/rules` entries.

- [ ] **Step 2: Commit and push**

Run: `git commit -m "gimbal_driver: heartbeat debug rotate"`

Run: `git push origin Behavion`

Expected: the remote `Behavion` branch receives the scoped commit.
