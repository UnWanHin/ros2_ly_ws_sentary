# Near-Goal Arrival Confirmation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent a regional goal from advancing immediately when fused position first enters the 20 cm arrival radius while external navigation is still converging.

**Architecture:** Keep `/ly/navi/reached` and `/ly/navi/reachable` unchanged. Add a goal-scoped, internal confirmation timer to `GoalReachState`: external `reachable=false` and `reached=true` retain priority; after goal-start grace, the first valid in-radius position starts a 1500 ms confirmation interval; only expiry while still in-radius permits the existing position-distance fallback.

**Tech Stack:** C++20, ROS2 Humble, gtest, existing JSON configuration and decision trace.

## Global Constraints

- Do not change ROS topics, messages, navigation-side behavior, or lower-machine protocol.
- Keep fresh `reachable=false` as immediate `Unreachable` and fresh `reached=true` as immediate `Reached`.
- Reset confirmation state when the goal changes, position becomes stale, or position leaves the arrival radius.
- Set formal regional `DecisionAutonomy.NaviGoal.HighlandCompat.NearGoalConfirmWaitMs` to `1500`.
- Update the current regional decision documentation and run targeted tests, a `behavior_tree` build, and static self-check.

---

### Task 1: Prove and encapsulate near-goal confirmation timing

**Files:**
- Create: `src/behavior_tree/include/NearGoalArrivalConfirm.hpp`
- Create: `src/behavior_tree/test/test_near_goal_arrival_confirm.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- Produces `BehaviorTree::NearGoalArrivalConfirm::Observe(goal_id, goal_position, position_is_fresh_and_in_radius, now, wait_ms)`.
- Produces `BehaviorTree::NearGoalArrivalConfirm::Reset()`.
- Returns `true` only after continuous in-radius confirmation for `wait_ms`; `wait_ms <= 0` preserves immediate distance fallback.

- [x] **Step 1: Write failing gtests**

```cpp
TEST(NearGoalArrivalConfirm, HoldsInsideRadiusUntilWaitExpires) {
  BehaviorTree::NearGoalArrivalConfirm confirm;
  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(1);
  const Area::Point<std::uint16_t> goal{100, 200};
  EXPECT_FALSE(confirm.Observe(7, goal, true, start, 1500));
  EXPECT_FALSE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(1499), 1500));
  EXPECT_TRUE(confirm.Observe(7, goal, true, start + std::chrono::milliseconds(1500), 1500));
}
```

- [x] **Step 2: Run the test and confirm it fails before implementation**

Run: `colcon test --packages-select behavior_tree --ctest-args -R near_goal_arrival_confirm --output-on-failure`

Expected: test target cannot compile because `NearGoalArrivalConfirm` does not exist.

- [x] **Step 3: Implement the minimal state helper**

```cpp
bool Observe(std::uint8_t goal_id, Area::Point<std::uint16_t> goal, bool inside, TimePoint now, int wait_ms) {
  if (wait_ms <= 0) { Reset(); return inside; }
  if (!inside) { Reset(); return false; }
  if (!Matches(goal_id, goal)) { Start(goal_id, goal, now); return false; }
  return now - entered_at_ >= std::chrono::milliseconds(wait_ms);
}
```

- [x] **Step 4: Add and run regression cases**

Add tests proving that leaving the radius resets the timer, changing goal resets the timer, and zero wait allows immediate fallback.

Run: `colcon test --packages-select behavior_tree --ctest-args -R near_goal_arrival_confirm --output-on-failure`

Expected: all near-goal confirmation tests pass.

### Task 2: Integrate the confirmation state into composite GoalReachState

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`

**Interfaces:**
- Extends `LangYa::NaviGoalAutonomySetting` with `NearGoalConfirmWaitMs{1500}`.
- Adds `GoalReachReason::NearGoalConfirmPending` and observable pending/elapsed fields to `GoalReachState`.
- `EvaluateNaviGoalReach()` owns only internal goal-scoped timing and retains its external ROS contract.

- [x] **Step 1: Add the new JSON setting and validation**

Read `NearGoalConfirmWaitMs` from the existing `HighlandCompat` object and top-level compatibility keys, clamp negative values to `0`, and include it in the startup configuration log. Set regional formal JSON to `1500`.

- [x] **Step 2: Integrate after external priority checks**

Keep this order:

```cpp
if (external_reachable == false) { Reset(); return Unreachable; }
if (external_reach == true) { Reset(); return ExternalReached; }
if (goal_start_grace_active) { Reset(); return GraceActive; }
if (!fresh_position || !within_arrive_distance) { Observe(..., false, ...); return Traveling; }
if (!confirm.Observe(..., true, now, wait_ms)) { return Traveling/NearGoalConfirmPending; }
return Reached/PositionDistance;
```

- [x] **Step 3: Extend gtests with integration-level priority coverage**

Verify external reached bypasses confirmation and external unreachable bypasses it. Ensure radius exit clears the pending timer before a later re-entry.

- [x] **Step 4: Build and run focused tests**

Run:

```bash
colcon build --packages-select behavior_tree --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select behavior_tree --ctest-args -R 'near_goal_arrival_confirm|aim_source|outpost_engagement_lock' --output-on-failure
colcon test-result --verbose
```

Expected: build succeeds and all selected test executables pass.

### Task 3: Expose the decision reason and document the formal semantics

**Files:**
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/regional/strategy_layers_and_navigation_reach.md`

**Interfaces:**
- Decision trace `goal_reach_state` includes pending confirmation fields.
- Documentation distinguishes goal-start grace from near-goal confirmation wait.

- [x] **Step 1: Add trace fields from `GoalReachState`**

Include `near_goal_confirm_pending` and `near_goal_confirm_elapsed_ms` beside existing distance and reason fields; do not change any existing key.

- [x] **Step 2: Update the two current decision documents**

Document 1500 ms continuous in-radius confirmation, immediate external reached/unreachable priority, and reset conditions. Update their `Updated:` dates.

- [ ] **Step 3: Run final static checks**

Run:

```bash
git diff --check
./scripts/selfcheck.sh sentry --static-only
```

Expected: both commands exit successfully.

`git diff --check` passed. `./scripts/selfcheck.sh sentry --static-only` was executed but cannot complete in this local environment because the external `sentry.aim` overlay, including `sentry_msgs/AimResult`, is not installed; no workspace source check failed.
