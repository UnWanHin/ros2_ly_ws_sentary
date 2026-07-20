# Tactical Damage Rotate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the formal Regional default Rotate gear and damage ramp timing from hard-coded control logic into `Tactical.yaml`.

**Architecture:** `Tactical.DamageRotate` is loaded as a normal ROS parameter file into `behavior_tree`, copied into `LangYa::Config`, and consumed by a small pure ramp policy helper. PointManager is removed because its all-disabled table had no runtime effect. Existing navigation stop requests remain after the policy calculation.

**Tech Stack:** ROS2 Humble parameters, C++20, GoogleTest, YAML.

## Global Constraints

- Preserve `/ly/control/firecode` and `/ly/navi/should_rotate` interfaces.
- Default values reproduce the prior timing: gear 0 for 220 ms, 1 for 220 ms, 2 for 220 ms, then 3; reset after 1800 ms without damage.
- A fresh navigation stop request continues to force `FollowMode=1` and `Rotate=0`.

---

### Task 1: Define Tactical DamageRotate Contract

**Files:**
- Create: `src/behavior_tree/config/Tactical.yaml`
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/launch/behavior_tree.launch.py`
- Modify: `scripts/launch/start_sentry_all.sh`

- [x] Add `Tactical.DamageRotate.DefaultGear`, `NoHitTimeoutMs`, `Gear0HoldMs`, `Gear1HoldMs`, `Gear2HoldMs`, `ScanBoostWindowMs`, and `ScanYawPhaseMs` with the current constants as defaults.
- [x] Load the YAML in the behavior-tree launch and copy parameters into the new configuration structure with bounds validation.
- [x] Make the Tactical default the sole ordinary Rotate source.

### Task 2: Test and Consume Ramp Policy

**Files:**
- Create: `src/behavior_tree/include/DamageRotatePolicy.hpp`
- Create: `src/behavior_tree/test/test_damage_rotate_policy.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`
- Modify: `src/behavior_tree/src/GameLoop.cpp`

- [x] Add a failing test for the 220/440/660 ms gear boundaries and a higher configured default gear.
- [x] Add the pure gear calculation helper and replace the hard-coded GameLoop ramp constants with Tactical settings.
- [ ] Build `behavior_tree` and run CTest (requires a rebuilt external `sentry.aim` `sentry_msgs`; the local old `sentry.common` interface lacks AimResult dynamics).

### Task 3: Remove Dead Point Configuration and Update Records

**Files:**
- Delete: `src/behavior_tree/config/PointManager.yaml`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

- [x] Remove the all-disabled PointManager table and its launch/configuration plumbing.
- [x] Document the priority order and validate JSON graph files plus `git diff --check`.
