# Default Area Patrol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Regional Default patrol rotate among YAML-enabled main areas and remove implicit Default ownership of `BuffOutpost`.

**Architecture:** The JSON `DecisionAutonomy.NaviGoal` area switches remain the only operator-owned eligibility contract. C++ owns the Default area rotation and four-point Base patrol; Buff/Outpost tactical remains the sole producer of `BuffOutpost`.

**Tech Stack:** C++20, ROS2 Humble, BehaviorTree.CPP, GoogleTest, colcon.

## Global Constraints

- Preserve ROS topics, message schemas, BaseGoal IDs, lower-controller protocol, and Outpost tactical behavior.
- `BuffOutpost`, `HoleRoad`, and `OutpostGuard` must not be MyBase Default patrol candidates.
- Default candidate eligibility must continue to honor `DecisionAutonomy.NaviGoal` area switches.
- Update current docs, fallback graph, and generated Obsidian indexes.

---

### Task 1: Rotate Default Area Candidates

**Files:**
- Modify: `src/behavior_tree/src/DefaultStrategyManager.cpp`
- Modify: `src/behavior_tree/test/test_pre_ready_roadland_tasks.cpp`

**Interfaces:**
- Produces: `BuildRegionalAreaCandidates()` returns the highest-scored candidate that is not `last_selected_task_` when another eligible candidate exists.

- [x] **Step 1: Write the failing test**

```cpp
TEST(PreReadyRoadlandTaskTest, DefaultPolicyDoesNotImmediatelyRepeatLastEligibleArea) {
    auto config = SplitConfig();
    config.RegionalAreaTaskSettings.MyBase.Enable = true;
    config.DecisionAutonomySettings.NaviGoal.MyArea = {
        "base", "pre_roadland", "ready_roadland"};
    DefaultStrategyManager manager;
    const DefaultRegionalPolicyInput input{
        .Config = &config, .MyTeam = UnitTeam::Red,
        .HealthFresh = true, .AmmoFresh = true,
        .Health = 400, .Ammo = 100,
        .Now = std::chrono::steady_clock::now()};
    const auto first = manager.BuildRegionalAreaCandidates(input);
    ASSERT_EQ(first.front().TaskType, RegionalAreaTaskType::MyBase);
    manager.CommitRegionalAreaSelection(first.front(), input.Now);
    const auto next = manager.BuildRegionalAreaCandidates(input);
    ASSERT_GT(next.size(), 1U);
    EXPECT_NE(next.front().TaskType, RegionalAreaTaskType::MyBase);
}
```

- [x] **Step 2: Verify RED**

Run `bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon build --packages-select behavior_tree --symlink-install && source install/setup.bash && ctest --test-dir build/behavior_tree -R test_pre_ready_roadland_tasks --output-on-failure'`.

Expected: the new assertion fails because MyBase remains first.

- [x] **Step 3: Implement and verify GREEN**

After the existing stable sort, find the candidate whose type equals `last_selected_task_`, erase it, and append it only if `candidates.size() > 1`. Re-run the Step 2 command; all focused tests must pass.

- [ ] **Step 4: Commit**

Run `git add src/behavior_tree/src/DefaultStrategyManager.cpp src/behavior_tree/test/test_pre_ready_roadland_tasks.cpp && git commit -m 'behavior_tree: rotate default area patrol'`.

### Task 2: Make MyBase Route Code-Owned and Tactical-Free

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Modify: `scripts/launch/start_sentry_all.sh`
- Modify: `scripts/selfcheck/sentry.sh`
- Modify: `src/behavior_tree/test/test_pre_ready_roadland_tasks.cpp`
- Delete: `src/behavior_tree/config/Base.yaml`

**Interfaces:**
- Produces: `MyBaseAreaTaskSetting` defaults to a 15-second hold and exactly `CastleLeft1`, `CastleLeft2`, `CastleRight2`, and `CastleRight1`; `sentry_all.launch.py` has no `base_strategy_config_file` argument.

- [x] **Step 1: Write the failing test**

```cpp
TEST(PreReadyRoadlandTaskTest, DefaultBasePatrolUsesOnlyCastlePoints) {
    const LangYa::MyBaseAreaTaskSetting setting;
    const std::array<std::uint8_t, 4> expected{
        LangYa::CastleLeft1.ID, LangYa::CastleLeft2.ID,
        LangYa::CastleRight2.ID, LangYa::CastleRight1.ID};
    ASSERT_EQ(setting.PatrolGoals.size(), expected.size());
    EXPECT_EQ(setting.GoalHoldSec, 15);
    for (const auto& goal : setting.PatrolGoals) {
        EXPECT_NE(std::find(expected.begin(), expected.end(), goal.BaseGoalId), expected.end());
    }
}
```

- [x] **Step 2: Verify RED**

Run the focused command from Task 1. Expected: it fails because current defaults include `HoleRoad`, `OutpostGuard`, and `BuffOutpost`.

- [x] **Step 3: Implement the fixed route and remove the overlay**

Set the four default Castle weights to `1.0`; remove the three tactical points from `MyBaseAreaTaskSetting::PatrolGoals`; delete `Base.yaml`; remove its `base_strategy_config_file` default, launch argument, log line, and Node parameter from `sentry_all.launch.py`; remove its wrapper forwarding from `start_sentry_all.sh`; remove its static-file check from `scripts/selfcheck/sentry.sh`.

- [x] **Step 4: Verify and commit**

Run `bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon build --packages-select behavior_tree --symlink-install && source install/setup.bash && ctest --test-dir build/behavior_tree --output-on-failure && ./scripts/selfcheck.sh sentry --static-only'`.

Expected: build succeeds, all BT tests pass, and static self-check reports `FAIL: 0`. Commit with `git commit -m 'behavior_tree: keep default patrol out of outpost'`.

### Task 3: Record and Verify the Behavior

**Files:**
- Modify: `docs/modules/2026-05-05_behavior_tree.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/regional/strategy_layers_and_navigation_reach.md`
- Modify: `docs/README.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`
- Regenerate: `docs/obsidian/_generated/`

- [x] **Step 1: Update current records**

Document that Default uses enabled areas only, avoids an immediate repeat when alternatives exist, uses four code-owned Base Castle points, and never publishes `BuffOutpost`; state that Buff/Outpost tactical owns that point.

- [ ] **Step 2: Validate graph and vault**

Run `python3 scripts/obsidian_sync.py && python3 scripts/obsidian_sync.py --check && python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null && python3 -m json.tool .understand-anything/meta.json >/dev/null && git diff --check`.

Expected: Obsidian check reports `writes=0 deletes=0 conflicts=0` and all validation commands exit `0`.

- [ ] **Step 3: Final verification and push**

Run `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_regional_area_overlay.py src/simulator/test/test_trace_contract.py src/simulator/test/test_validation.py`, then `git fetch origin Behavion`, `git rev-list --left-right --count HEAD...origin/Behavion`, and `git push origin Behavion` after remote-only count is zero.
