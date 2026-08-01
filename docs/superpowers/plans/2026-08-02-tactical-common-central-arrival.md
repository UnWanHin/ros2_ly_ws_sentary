# Tactical CommonCentral Arrival Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Give Tactical CommonCentral its own YAML enable gate and make its four-point search (`17 -> 29 -> 30 -> 24`, then reverse) advance only through the existing composite arrival and navigation-progress watchdog semantics.

**Architecture:** Keep `RegionalDefenseSearchKind::CommonCentral` in `GameLoop` as the owner of the four-goal tactical search. Add an enable/hold setting to `RegionalDefenseSetting`, read `Tactical.RegionalDefense.CommonCentral.Enable/HoldSec` from YAML, and gate only the CommonCentral branch. Reuse `EvaluateBaseGoalReach()` for reached/unreachable status and the existing `NaviProgressWatchdog` runtime/config for no-progress fallback; do not create a second arrival implementation or change other RegionalDefense branches.

**Tech Stack:** C++17, ROS2 parameter YAML, GoogleTest, existing behavior-tree configuration and documentation.

## Global Constraints

- Preserve `/ly/*` topics, message types, posture outputs, target selection, and task priority ordering.
- Preserve Default `AreaManager.yaml` CommonCentral; the new gate controls only Tactical CommonCentral.
- Preserve existing goal IDs 0-28, including `CentralLeft.A/B (26/27)` and `ProtectOutpost (28)`; CommonCentral uses new IDs `29/30`.
- Use `HoleRoad (17) -> CentralHigh (29) -> CentralLow (30) -> OutpostGuard (24)`, reverse at `OutpostGuard`, with team-relative coordinate resolution.
- Do not change the global `RegionalDefense.SearchHoldSec` or `SearchNoTargetSec` semantics for other branches.
- Keep generated `build/`, `install/`, and `log/` out of commits.

### Task 1: Add the Tactical configuration gate

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp:855-869`
- Modify: `src/behavior_tree/src/Configuration.cpp:941-955, 1900-1980, 2617-2629, 3264-3308`
- Modify: `src/behavior_tree/config/Tactical.yaml:1-110`
- Test: `src/behavior_tree/test/test_tactical_common_central.cpp`

**Interfaces:**
- Produces `RegionalDefenseSetting::CommonCentralEnable`, defaulting to `true`.
- Reads both `Tactical.RegionalDefense.CommonCentral.Enable` and slash-form compatibility paths.
- Logs the effective gate beside the existing RegionalDefense settings.

- [x] Add and read `Tactical.RegionalDefense.CommonCentral.Enable/HoldSec` with a safe default.
- [x] Update `scripts/selfcheck/sentry.sh` static contract checks for the gate and point IDs.

### Task 2: Gate and stabilize Tactical CommonCentral progression

**Files:**
- Modify: `src/behavior_tree/src/GameLoop.cpp:4497-4541, 4584-4630`
- Modify: `src/behavior_tree/src/AreaManager.cpp:773-875` only if a small existing watchdog helper is needed
- Modify: `src/behavior_tree/include/AreaManager.hpp` only if the helper signature requires it
- Test: `src/behavior_tree/test/test_tactical_common_central.cpp`

**Interfaces:**
- Consumes `EvaluateBaseGoalReach()` and `AreaManager::ProgressWatchdogRuntime()`.
- Produces the same `RegionalDefense common_central` decision intent and navigation goal IDs.

- [x] Add regression coverage for travel, arrival hold, watchdog and external-unreachable progression.
- [x] Keep the traveling goal stable until the shared watchdog or explicit unreachable result.
- [x] Implement the four-point forward/reverse sequence without changing legacy IDs.

### Task 3: Synchronize current regional documentation

**Files:**
- Modify: `docs/sentry/regional/current_behavior.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `docs/README.md` only if the nearest index entry needs a current-status note

- [x] Document the gate, four-point route, coordinates, and 15-second arrival hold.
- [x] Document shared 80 cm / 14 s watchdog behavior and legacy ID preservation.

### Task 4: Verification and review

- [x] Targeted policy test: 3/3 passed; full behavior-tree suite: 20/20 passed.
- [x] `behavior_tree` and `simulator` build succeeded.
- [x] Simulator suite: 238 tests passed; tactical catalog: 10 tests passed.
- [x] `git diff --check` and JSON/YAML point-data validation passed.
- [ ] Full selfcheck remains environment-limited when external `sentry_msgs` is not sourced.
