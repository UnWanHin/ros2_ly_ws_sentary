# Map Command Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make official referee `0x0303` coordinate commands navigate the sentry for a deduplicated 45-second hold through the existing calibrated raw-goal bridge.

**Architecture:** A focused BT helper owns acceptance, coordinate deduplication, and hold expiry. `Application` feeds it from the existing `/ly/game/map_command` cache and, when active, publishes official-map centimeters on `/ly/navi/goal_pos_raw`. Strategy gives this operator task priority below Hard Recovery and above normal Regional tasks.

**Tech Stack:** ROS 2 Humble, C++20, GoogleTest, `gimbal_driver/msg/MapCommand`, `navi_tf_bridge` raw-goal calibration.

## Global Constraints

- Accept only coordinate mode: `has_target_position=true`, finite coordinates, and not `(0, 0)`.
- Keep target-robot mode cache-only and preserve all existing topics/messages.
- Reuse `/ly/navi/goal_pos_raw`; do not publish `/goal_pose` from BT.
- Same coordinates within `DedupDistanceCm` never extend a hold.
- Hard Recovery cancels active MapCommand ownership; no automatic resume.
- Update current embedded/regional documentation and Understand Anything fallback graph.

---

### Task 1: Add a testable MapCommand hold policy

**Files:**
- Create: `src/behavior_tree/include/MapCommandTask.hpp`
- Modify: `src/behavior_tree/test/test_pre_ready_roadland_tasks.cpp`

**Interfaces:**
- Consumes: coordinate-mode `gimbal_driver::msg::MapCommand` values and `steady_clock` time.
- Produces: `Accept`, `Active`, `RawXCentimeters`, `RawYCentimeters`, and `Cancel` operations.

- [ ] Write failing tests for `(0,0)` rejection, first coordinate acceptance, duplicate non-extension,
  coordinate replacement, expiry, and cancellation.
- [ ] Implement the minimal stateful helper that rounds official meters to UInt16 centimeters only after
  finite/range validation.
- [ ] Build `behavior_tree` and run `test_pre_ready_roadland_tasks`.

### Task 2: Connect the policy to formal Regional strategy

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/config/Task.yaml`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/SubscribeMessage.cpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/StrategyManager.cpp`

**Interfaces:**
- Consumes: existing `/ly/game/map_command`, `CheckPositionRecovery()`, and `pub_navi_goal_pos_raw_`.
- Produces: an operator-owned raw goal with typed `DecisionIntent` reason `map_command`.

- [ ] Add `Task.MapCommand.Enable`, `HoldSec`, and `DedupDistanceCm` with defaults `true`, `45`, `20`.
- [ ] Feed only new received MapCommand messages into the helper; do not reaccept the cached 1 Hz resend.
- [ ] Run MapCommand strategy after Hard Recovery and before Regional Tactical/Default branches.
- [ ] Cancel the task when Hard Recovery owns the tick and do not resume it later.
- [ ] Build and run all current behavior-tree CTests.

### Task 3: Record the public behavior and verify the graph

**Files:**
- Modify: `docs/sentry/embedded/map_command_typeid9.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/record/2026-07-19_map_command_navigation.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

**Interfaces:**
- Documents official `0x0303` coordinate semantics, duplicate behavior, calibrated raw-goal routing, and
  Hard Recovery precedence.

- [ ] Record the V2.0 source evidence and state that target-robot mode cannot navigate without a coordinate.
- [ ] Update graph nodes/edges for TypeID 9, BT MapCommand policy, and `/ly/navi/goal_pos_raw`.
- [ ] Validate graph JSON, `git diff --check`, run static self-check, review, commit, and push.
