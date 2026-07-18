# Regional Goal Commitment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent Default Regional navigation from reversing a partially executed area task after a temporary tactical preemption, while keeping the configured Outpost opening hold stable at BuffOutpost.

**Architecture:** `DefaultStrategyManager` records an eligible Default area that was preempted by a resumable tactical layer. Its next candidate build promotes that area only if it remains enabled, in scope, healthy, and not in cooldown; an ineligible next selection discards the one-shot resume. The opening Outpost hold is represented by one Application predicate and used to keep navigation ownership at BuffOutpost except for existing Recovery and explicit own-base defense paths.

**Tech Stack:** ROS 2 Humble, C++17, GoogleTest, YAML/JSON runtime configuration, DecisionTrace JSONL.

## Global Constraints

- Preserve ROS topics, message schemas, launch interfaces, and external navigation contracts.
- `GoalReachState` remains the only final arrival/unreachable contract.
- Recovery and hard RegionalDefense remain allowed to preempt navigation.
- Do not reintroduce `BuffOutpost` into the Default candidate set.
- Update current Regional behavior documentation and Understand Anything fallback graph evidence.

---

### Task 1: Make Default preemption resumable

**Files:**
- Modify: `src/behavior_tree/include/DefaultStrategyManager.hpp`
- Modify: `src/behavior_tree/src/DefaultStrategyManager.cpp`
- Modify: `src/behavior_tree/test/test_pre_ready_roadland_tasks.cpp`

**Interfaces:**
- Consumes: `RecordRegionalAreaResult(type, reason, now, setting)` from existing task cancellation paths.
- Produces: `preempted` result semantics: the next eligible Default selection resumes that `RegionalAreaTaskType` before normal scoring.

- [x] **Step 1: Write the failing tests**
  Add tests proving a preempted PreRoadland candidate is promoted on the next eligible selection, and that a canceled candidate remains governed by the existing failure cooldown.

- [x] **Step 2: Run the focused test binary and observe the new assertion fail**
  Run: `colcon test --packages-select behavior_tree --ctest-args -R test_pre_ready_roadland_tasks --output-on-failure`
  Expected before implementation: the preempted candidate is not first because normal scoring selects MyBase.

- [x] **Step 3: Implement the smallest state change**
  Store one preempted task type. When candidate construction has an eligible matching task, move it to the front. Clear it on successful commit, failure, completion, and policy reset; preserve it only for reason `preempted`.

- [x] **Step 4: Run focused tests**
  Run the command from Step 2.
  Expected: `test_pre_ready_roadland_tasks` passes.

### Task 2: Preserve resumable Default work across tactical preemption

**Files:**
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/include/DecisionIntent.hpp`

**Interfaces:**
- Consumes: existing cancellation sites for aim tasks, RegionalDefense, Special Patrol, and Recovery.
- Produces: explicit `preempted` result recording for resumable overlays and `DecisionTrace` reason data that distinguishes tactical ownership from Default ownership.

- [x] **Step 1: Replace only resumable cancellation reasons**
  Use `preempted` for AimMode/RegionalDefense/Special cancellation of a yieldable Default task. Keep Recovery as `canceled` and keep completed/timeout/unreachable behavior unchanged.

- [x] **Step 2: Preserve trace reason mapping**
  Add the `preempted` text mapping only when it is used to set decision intent; do not change topic or trace schema.

- [x] **Step 3: Build and run focused behavior_tree tests**
  Run: `colcon build --packages-select behavior_tree --symlink-install`
  Run: `colcon test --packages-select behavior_tree --ctest-args --output-on-failure`

### Task 3: Make the configured opening Outpost hold an explicit navigation commitment

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/test/test_outpost_engagement_lock.cpp` or a focused new pure-logic test if extraction is required

**Interfaces:**
- Consumes: `Task.OutpostConfirm.OpeningHoldSec`, `OpeningHoldUntilWindowEnd`, `OpeningHighPriority`, and elapsed game time.
- Produces: a single predicate used by Outpost selection and tactical navigation ownership.

- [x] **Step 1: Write the failing coverage for the opening-hold predicate**
  Cover active before 120 seconds and inactive at/after 120 seconds; verify disabled Outpost or disabled hold never activates it.

- [x] **Step 2: Extract the predicate and use it in the Outpost path**
  Prevent ordinary Default/special/soft tactical changes from replacing BuffOutpost during an active opening hold. Do not suppress Recovery or explicit own-base defense.

- [x] **Step 3: Build and test**
  Run the focused tests, then all `behavior_tree` tests.

### Task 4: Record and verify the behavioral contract

**Files:**
- Modify: `docs/record/2026-07-18_default_area_patrol.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

- [x] **Step 1: Document precedence and the allowed exceptions**
  Record the resume contract, the `preempted` reason, opening-hold ownership, and Recovery/own-base-defense exceptions.

- [x] **Step 2: Refresh the source-checked fallback graph**
  Update the Regional-flow summary and metadata to the current HEAD after source verification.

- [x] **Step 3: Validate and run repository gates**
  Run: `python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null`
  Run: `python3 -m json.tool .understand-anything/meta.json >/dev/null`
  Run: `./scripts/selfcheck.sh sentry --static-only`
  Run: `git diff --check`
