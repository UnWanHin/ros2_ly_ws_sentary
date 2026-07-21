# Decision Explain Detail Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Explain Recovery, Default area tasks, and idle patrol at final navigation transitions without changing decision behavior or producing high-rate logs.

**Architecture:** Reuse `Application::lastDecisionIntent_` and the final-output fingerprint introduced by `DecisionExplain.hpp`. Record stable intent detail only in code that has already selected a new goal; do not add branch-local publishers, timers, or a parallel decision state.

**Tech Stack:** C++20, ROS2 Humble, existing `DecisionIntent`, gtest, Utils Logger.

## Global Constraints

- Do not change navigation goal selection, control output, priorities, or YAML interfaces.
- Do not add ROS topics, timers, or per-tick logs.
- A detail mutation is allowed only with a final navigation point transition.
- Preserve the existing `Default regional policy` score log; score is not part of the de-duplicated final intent detail.

### Task 1: Add stable reason and formatter regression coverage

**Files:**
- Modify: `src/behavior_tree/include/DecisionIntent.hpp`
- Modify: `src/behavior_tree/test/test_decision_explain.cpp`

**Interfaces:**
- `DecisionReason::Recovery` maps to `"recovery"`, Hard layer, priority 400.
- `DecisionExplain::Fingerprint` continues to distinguish an intentional detail change.

- [ ] **Step 1: Write failing tests**

```cpp
TEST(DecisionExplain, FormatsRecoveryAsHardDecision) {
    const auto observation = MakeObservation(
        BehaviorTree::DecisionReason::Recovery, 2U, 102U, 250U, 200U,
        "enter_default hp=120 ammo=45");
    const auto line = BehaviorTree::DecisionExplain::FormatNavigationLine(observation);
    EXPECT_NE(line.find("layer=hard"), std::string::npos);
    EXPECT_NE(line.find("reason=recovery"), std::string::npos);
}

TEST(DecisionExplain, StableDetailSuppressesRepeatedPointPublish) {
    const auto first = MakeObservation(
        BehaviorTree::DecisionReason::DefaultAreaPolicy, 8U, 108U, 1600U, 720U,
        "area=MyHighland phase=patrol");
    EXPECT_EQ(BehaviorTree::DecisionExplain::MakeFingerprint(first),
              BehaviorTree::DecisionExplain::MakeFingerprint(first));
}
```

- [ ] **Step 2: Run RED**

Run:

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
cmake --build build/behavior_tree --target test_decision_explain -j2
./build/behavior_tree/test_decision_explain
```

Expected: `DecisionReason::Recovery` does not exist.

- [ ] **Step 3: Implement the reason mapping**

Append `Recovery` to `DecisionReason`, return `"recovery"` from
`DecisionReasonToString`, parse it in `DecisionReasonFromString`, and map it to
`DecisionLayer::Hard`. Do not alter any existing reason priority.

- [ ] **Step 4: Run GREEN and commit**

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
cmake --build build/behavior_tree --target test_decision_explain -j2
./build/behavior_tree/test_decision_explain
git add src/behavior_tree/include/DecisionIntent.hpp src/behavior_tree/test/test_decision_explain.cpp
git commit -m "behavior_tree: label recovery navigation"
```

### Task 2: Record detailed intent at real point transitions

**Files:**
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/test/test_decision_explain.cpp`

**Interfaces:**
- Regional recovery direct targets record `Recovery` only from
  `command_recovery_position` when `changed` is true.
- Default candidate selection records `DefaultAreaPolicy` with
  `area=<candidate.Name>` only after `TrySetScopedPositionByBaseGoal` succeeds.
- Active area tasks record `DefaultAreaPolicy` with
  `area_task=<type> phase=<phase>` only if their final goal differs from the
  preceding goal.
- Idle patrol records `RegionalIdlePatrol` with `index=<candidate.Index>
  hold_sec=<hold_sec>` only after a successful new candidate selection.

- [ ] **Step 1: Add pure formatter tests for representative detail strings**

```cpp
EXPECT_NE(BehaviorTree::DecisionExplain::FormatNavigationLine(default_task)
              .find("detail=area_task=MyHighland phase=patrol"),
          std::string::npos);
EXPECT_NE(BehaviorTree::DecisionExplain::FormatNavigationLine(idle)
              .find("detail=index=2 hold_sec=8"),
          std::string::npos);
```

- [ ] **Step 2: Run RED**

Run `./build/behavior_tree/test_decision_explain` and confirm the detail
fixtures cannot compile until `Recovery` is added.

- [ ] **Step 3: Record metadata at the selected-goal sites**

In `TickRegionalAreaTask`, save the old resolved goal and coordinate before
`SetPositionByBaseGoal`; after it, call `RecordDecisionIntent` only if the
goal or coordinate changed. Build a stable `detail` from
`RegionalAreaTaskTypeToString(result.Type)` and
`RegionalAreaTaskPhaseToString(result.Phase)`.

In `TrySetDefaultRegionalAreaTaskGoal`, overwrite the generic scoped intent
after a successful selection with detail `area=<candidate.Name>`.

In `TrySetRegionalIdlePatrolGoal`, overwrite the generic scoped intent after a
successful candidate with `index=<candidate.Index> hold_sec=<hold_sec>`.

In `CheckPositionRecovery`, make `command_recovery_position` record
`Recovery` only within its existing `changed` block, using the existing
reason and a snapshot `hp=<myselfHealth> ammo=<ammoLeft>`. Do not make health
or ammo updates alone emit a new intent.

- [ ] **Step 4: Run focused build and tests**

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
cmake --build build/behavior_tree --target behavior_tree_node test_decision_explain -j2
./build/behavior_tree/test_decision_explain
```

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/src/GameLoop.cpp src/behavior_tree/test/test_decision_explain.cpp
git commit -m "behavior_tree: detail patrol decision logs"
```

### Task 3: Document and verify the operator contract

**Files:**
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `docs/sentry/regional/current_behavior.md`

- [ ] **Step 1: Document detailed transition-only output**

State that Recovery emits its trigger and point snapshot at a target change;
Default area tasks emit area and phase; idle patrol emits index and hold time.
State that repeated publication of the same point does not update these logs.

- [ ] **Step 2: Run required checks**

```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-select behavior_tree --symlink-install
source install/setup.zsh
ctest --test-dir build/behavior_tree --output-on-failure -R 'decision_explain|tactical_protection_policy|decision_trace_control_output'
./scripts/selfcheck.sh sentry --static-only
git diff --check
```

- [ ] **Step 3: Commit documentation**

```bash
git add docs/sentry/regional/2026-07-12_regional_decision_graph.md docs/sentry/regional/current_behavior.md docs/superpowers/specs/2026-07-21-decision-explain-detail-design.md docs/superpowers/plans/2026-07-21-decision-explain-detail.md
git commit -m "docs: describe detailed decision logs"
```
