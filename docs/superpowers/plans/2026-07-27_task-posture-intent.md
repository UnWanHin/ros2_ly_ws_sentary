# Task Posture Intent Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make BT posture selection follow each active task's goal-scoped travel or arrived-hold state without changing ROS, navigation, Tactical priority, or referee interfaces.

**Architecture:** `Application` derives one internal task posture intent after strategy selection from the task that owns the current navigation goal. `PostureLogic` uses that intent to preserve Move during travel or return to normal Attack/Defense scoring at a confirmed hold. `PostureManager` receives a narrow policy flag that prevents automatic early rotation from defeating required degraded Move or hard task posture.

**Tech Stack:** ROS2 Humble C++, gtest/CTest, existing BehaviorTree posture and decision-trace facilities.

## Global Constraints

- Preserve all ROS topics, message schemas, YAML keys, navigation output rules, and Tactical priority order.
- Use `EvaluateNaviGoalReach` or explicit state-machine hold phases; never use an unscoped raw `/ly/navi/reached` value.
- Recovery and fresh `should_rotate=false` require Move; outpost engagement lock retains its existing behavior.
- Do not modify the user-owned LibreOffice PDF lock file under `docs/rules/`.

---

### Task 1: Protect Required Posture From Generic Early Rotation

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/src/PostureManager.cpp`
- Modify: `src/behavior_tree/test/test_posture_manager.cpp`

**Interfaces:**
- Extend `PostureRequestPolicy` with `AllowEarlyRotate` defaulting to `true`.
- `PostureManager::Tick` skips generic early rotation only when the request policy sets it to `false`.

- [ ] Add a gtest that initializes a fresh referee Move timer at zero, requests Move with early rotation disabled, and expects no Attack/Defense command.
- [ ] Run the focused test and confirm it fails before the implementation.
- [ ] Add the policy member and guard the early-rotation branch.
- [ ] Run the focused test and all posture-manager tests.

### Task 2: Derive One Goal-Owned Task Intent

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/test/test_posture_logic.cpp` or the nearest existing posture test target.

**Interfaces:**
- Add internal `TaskPostureIntent` values: `None`, `SoftTransit`, `SoftArrived`, `HardMove`, `HardAttack`, `HardDefense`.
- Add `Application::ResolveTaskPostureIntent(TimePoint)` that considers only the active goal owner and current goal coordinates.
- `UpdatePostureCommand` maps the intent to an existing posture request and policy, then preserves existing outpost-lock priority.

- [ ] Add failing unit tests for intent-to-request mapping: soft transit reserves Move, soft arrived does not force Move, and hard Move preserves Move at zero official remaining.
- [ ] Implement the enum/string helper and mapping with no new ROS or YAML configuration.
- [ ] Run focused posture tests.

### Task 3: Connect Existing Task State Machines

**Files:**
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/test/test_tactical_protection_policy.cpp` and/or existing task tests.

**Interfaces:**
- Default regional: current `RegionalAreaTaskPostureHint` remains the source.
- ProtectOutpost: `Travel` is soft transit and `SearchHold` is soft arrived.
- ProtectHero, regional defense, and SpecialPatrol expose soft arrived only when their active output still owns the current goal and canonical reach/hold state confirms it.
- Recovery, Buff, dynamic Chase, fresh navigation non-rotate, damage burst, and Outpost lock retain their documented special handling.

- [ ] Add tests covering ProtectOutpost phase mapping and stale-goal rejection.
- [ ] Implement only state inspection around existing task data; do not alter task goal selection, hold duration, or preemption.
- [ ] Build `behavior_tree` and run all CTest cases.

### Task 4: Observability and Current Documentation

**Files:**
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`

**Interfaces:**
- Posture logs and decision trace expose task intent, goal ownership, and reserve/forced-degraded choice.

- [ ] Add trace/log fields without changing existing trace field meanings.
- [ ] Update current documentation with source order, exceptions, referee budget behavior, and 5 s cooldown/10 s manager hold distinction.
- [ ] Run `git diff --check`, focused build/tests, and `./scripts/selfcheck.sh sentry --static-only`; report unavailable external aim dependency separately if present.

## Review Checklist

- Every travel state uses the current owned goal, never an old reached event.
- Every arrived state is either an existing explicit hold phase or canonical composite arrival.
- Generic rotation cannot replace required degraded Move.
- Soft arrival keeps existing health, target, and resource scoring rather than blindly forcing Attack.
- No ROS, serial, YAML, navigation, or simulator contract changes occur.
