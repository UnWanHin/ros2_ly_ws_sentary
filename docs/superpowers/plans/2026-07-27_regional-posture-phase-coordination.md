# Regional Posture Phase Coordination Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make posture use the existing Default area-task travel and verified-arrival dwell state while preserving all existing navigation and task behavior.

**Architecture:** `AreaManager` remains the single owner of route phases and arrival truth. It exposes a small internal `RegionalAreaTaskPostureHint`; `PostureLogic` uses the hint after normal selection. Fresh referee TypeID 10 posture budgets decide whether Transit may reserve Move and use a better-budgeted base posture instead.

**Tech Stack:** ROS2 Humble, C++20, GoogleTest, colcon, existing behavior_tree configuration and documentation.

## Global Constraints

- Do not add ROS topics, messages, YAML keys, or a second dwell timer.
- Do not alter regional task selection, navigation goals, or priority ordering.
- Treat timeout/unreachable as Transit, never as verified arrival.
- Preserve Recovery and outpost-engagement behavior.
- Use TypeID 10 referee remaining seconds only while existing freshness checks accept them.

---

### Task 1: Define and test AreaManager posture hints

**Files:**
- Modify: `src/behavior_tree/include/AreaManager.hpp`
- Modify: `src/behavior_tree/src/AreaManager.cpp`
- Modify: `src/behavior_tree/test/test_pre_ready_roadland_tasks.cpp`

**Interfaces:**
- Produces: `RegionalAreaTaskPostureHint ResolveRegionalAreaTaskPostureHint(const RegionalAreaTaskRuntime&) noexcept`
- Produces: runtime evidence that a phase began with verified arrival.

- [ ] **Step 1: Write failing tests**

```cpp
EXPECT_EQ(ResolveRegionalAreaTaskPostureHint(runtime), RegionalAreaTaskPostureHint::Transit);
runtime.Phase = RegionalAreaTaskPhase::PreRoadlandHold;
runtime.PhaseArrived = true;
EXPECT_EQ(ResolveRegionalAreaTaskPostureHint(runtime), RegionalAreaTaskPostureHint::ArrivedHold);
runtime.PhaseArrived = false;
EXPECT_EQ(ResolveRegionalAreaTaskPostureHint(runtime), RegionalAreaTaskPostureHint::Transit);
```

- [ ] **Step 2: Run the focused test and observe it fail**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_pre_ready_roadland_tasks --output-on-failure`

- [ ] **Step 3: Add the runtime evidence and pure hint resolver**

Reset the evidence on each new goal/phase. Set it only when a phase transition
was caused by a composite-arrived input. Map verified hold phases to
`ArrivedHold`; map all active travel or uncertain phases to `Transit`.

- [ ] **Step 4: Run the focused test and build**

Run: `colcon build --packages-select behavior_tree --symlink-install && colcon test --packages-select behavior_tree --ctest-args -R test_pre_ready_roadland_tasks --output-on-failure`

### Task 2: Add budget-aware Transit posture selection

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Create: `src/behavior_tree/test/test_regional_task_posture.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- Consumes: `RegionalAreaTaskPostureHint`, `PostureRuntime`, `PostureSetting`.
- Produces: a pure Transit override that returns Move while its fresh official
  budget exceeds the warning reserve; otherwise chooses the best usable
  Attack/Defense budget, preferring Defense on an equal budget.

- [ ] **Step 1: Write failing tests**

```cpp
EXPECT_EQ(SelectTransitPosture(runtime, setting), SentryPosture::Move);
runtime.RefereeRemainingSec[3] = 10;
runtime.RefereeRemainingSec[2] = 60;
EXPECT_EQ(SelectTransitPosture(runtime, setting), SentryPosture::Defense);
runtime.RefereeRemainingSec[1] = 80;
EXPECT_EQ(SelectTransitPosture(runtime, setting), SentryPosture::Attack);
```

- [ ] **Step 2: Run the focused test and observe it fail**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_regional_task_posture --output-on-failure`

- [ ] **Step 3: Implement the pure selector and connect it**

Keep ordinary `SelectDesiredPosture()` unchanged. In `UpdatePostureCommand()`,
apply the verified AreaManager hint only after normal selection and before the
existing navigation and outpost-lock policy. Recovery remains Move. The helper
uses fresh referee budgets only; without them Transit stays Move.

- [ ] **Step 4: Run focused tests and build**

Run: `colcon build --packages-select behavior_tree --symlink-install && colcon test --packages-select behavior_tree --ctest-args -R 'test_regional_task_posture|test_posture_manager|test_pre_ready_roadland_tasks' --output-on-failure`

### Task 3: Document and validate the runtime contract

**Files:**
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `docs/sentry/regional/decision_framework.md`

- [ ] **Step 1: Update current decision documentation**

Document Transit, verified ArrivedHold, timeout/unreachable safety behavior,
the TypeID 10 timer path, and retained posture manager gates.

- [ ] **Step 2: Run full required verification**

Run: `colcon build --packages-select behavior_tree --symlink-install && colcon test --packages-select behavior_tree && colcon test-result --verbose && ./scripts/selfcheck.sh sentry --static-only && git diff --check`

- [ ] **Step 3: Commit and push**

```bash
git add src/behavior_tree docs/superpowers docs/sentry/regional
git commit -m "behavior_tree: coordinate regional posture and arrival holds"
git push origin Behavion
```
