# ProtectOutpost Tactical Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Send the Regional sentry to C3/C4 on a fresh own-outpost health decrease, search for 30 seconds after arrival, and arbitrate it by configurable Tactical priority without navigation churn.

**Architecture:** Keep health-change detection and lifecycle transitions as a pure policy in `TacticalProtectionPolicy.hpp`; `Application` owns only ROS receipt freshness and navigation projection. `StrategyManager` retains Hard/Task/Aim precedence, then uses the YAML priority table to order ProtectCastle, ProtectOutpost, ProtectHero, and Chase.

**Tech Stack:** ROS2 Humble C++20, `ament_cmake_gtest`, YAML parameters, existing `DecisionIntent` / JSONL trace, shell static self-check.

## Global Constraints

- C3/C4 are official-map centimeters: Red `(1011, 429)`, Blue `(1789, 1071)`.
- No ROS topic, message, launch argument, serial protocol, or navigation-bridge API changes.
- A stable low outpost HP cannot re-trigger defense; only a new strict decrease from a fresh sample can.
- Hard and Task layers remain above all Tactical priority values; Buff/Outpost aim behavior remains unchanged.
- Default ordering is `ProtectCastle: 1`, `ProtectOutpost: 2`, `ProtectHero: 3`, `Chase: 4`; lower integer wins and ties use that same listed order.
- Do not include unrelated working-tree changes in commits.

---

### Task 1: Define and prove the pure ProtectOutpost lifecycle

**Files:**
- Modify: `src/behavior_tree/include/TacticalProtectionPolicy.hpp`
- Modify: `src/behavior_tree/test/test_tactical_protection_policy.cpp`

**Interfaces:**
- Produces `ProtectOutpostPhase { Idle, Travel, SearchHold, Cooldown, Complete }`.
- Produces `ProtectOutpostState` with last sampled HP, one active event generation, hold start, and cooldown deadline.
- Produces `ObserveProtectOutpostHealth(state, health, fresh, now)` and `TickProtectOutpost(state, fresh, reached, unreachable, now, settings)`.

- [ ] **Step 1: Add failing lifecycle tests**

```cpp
TEST(TacticalProtectionPolicy, ProtectOutpostStartsOnlyOnFreshStrictDecrease) {
    ProtectOutpostState state;
    const auto now = Clock::now();
    EXPECT_FALSE(ObserveProtectOutpostHealth(state, 1500, true, now));
    EXPECT_FALSE(ObserveProtectOutpostHealth(state, 1500, true, now + 1s));
    EXPECT_FALSE(ObserveProtectOutpostHealth(state, 1499, false, now + 2s));
    EXPECT_TRUE(ObserveProtectOutpostHealth(state, 1499, true, now + 3s));
    EXPECT_EQ(state.Phase, ProtectOutpostPhase::Travel);
}

TEST(TacticalProtectionPolicy, ProtectOutpostHoldsAfterArrivalAndConsumesEvent) {
    ProtectOutpostState state = ActiveProtectOutpostTravelState();
    const auto now = Clock::now();
    EXPECT_EQ(TickProtectOutpost(state, true, true, false, now, 30s).Phase,
              ProtectOutpostPhase::SearchHold);
    EXPECT_EQ(TickProtectOutpost(state, true, true, false, now + 29s, 30s).Phase,
              ProtectOutpostPhase::SearchHold);
    EXPECT_EQ(TickProtectOutpost(state, true, true, false, now + 30s, 30s).Phase,
              ProtectOutpostPhase::Complete);
}
```

Also cover new-damage hold refresh, stale health preservation, unreachable-to-cooldown, cooldown expiry, and a preempted tick retaining its active event.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
colcon test --packages-select behavior_tree --ctest-args -R test_tactical_protection_policy --output-on-failure
```

Expected: the test target fails to compile because the new policy symbols do not yet exist.

- [ ] **Step 3: Implement the smallest policy state machine**

Use a single `ProtectOutpostState` with no ROS dependency. A first fresh sample records baseline only. A strict non-zero decrease creates `Travel`; `Reached` switches to `SearchHold`; a later fresh decrease while holding resets the hold start; `Unreachable` moves to `Cooldown`; expiry moves the state to `Complete`; `Complete` only becomes `Travel` after another strict decrease. `TickProtectOutpost` must not change target coordinates or issue navigation commands.

- [ ] **Step 4: Re-run the focused policy tests**

Run the command from Step 2.

Expected: `test_tactical_protection_policy` passes whenever the local ament environment is available.

- [ ] **Step 5: Commit the pure policy slice**

```bash
git add src/behavior_tree/include/TacticalProtectionPolicy.hpp \
  src/behavior_tree/test/test_tactical_protection_policy.cpp
git commit -m "behavior_tree: add ProtectOutpost lifecycle policy"
```

### Task 2: Project the lifecycle through the Regional decision chain

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/module/Area.hpp`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/SubscribeMessage.cpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/StrategyManager.cpp`
- Modify: `src/behavior_tree/include/DecisionIntent.hpp`

**Interfaces:**
- Consumes fresh `/ly/friend/op_hp` samples and `Tactical.ProtectOutpost` settings.
- Produces `TrySetProtectOutpostGoal(my_team, enemy_team)` and a new `DecisionReason::ProtectOutpost`.
- Produces `Area::ProtectOutpostPoint(team)` returning C3/C4.

- [ ] **Step 1: Extend configuration types and YAML reads**

Add these exact configuration shapes:

```cpp
struct TacticalPrioritySetting {
    int ProtectCastle{1};
    int ProtectOutpost{2};
    int ProtectHero{3};
    int Chase{4};
};

struct ProtectOutpostSetting {
    bool Enable{true};
    int HealthFreshMs{2000};
    int SearchHoldSec{30};
    int UnreachableCooldownSec{10};
};
```

Add both to `TacticalSetting`, implement `from_json`, read every `Tactical.Priority.*` and `Tactical.ProtectOutpost.*` parameter with the established optional parameter helpers, and clamp freshness/hold/cooldown to non-negative safe values. Add the default YAML block exactly as specified in the design.

- [ ] **Step 2: Add exact map point and freshness state**

In `Area.hpp`, add one named team-dependent `Location`:

```cpp
static const Location<std::uint16_t> ProtectOutpostPoint{
    {1011, 429}, {1789, 1071}
};
```

In `Application.hpp`, add own-outpost receipt freshness and `ProtectOutpostState`. In the existing `/ly/friend/op_hp` subscriber, set `hasReceivedSelfOutpostHealth_`, `lastSelfOutpostHealthRxTime_`, and `selfOutpostHealth`; do not infer fresh time from the message header.

- [ ] **Step 3: Add the navigation projection and decision evidence**

`TrySetProtectOutpostGoal` must:

1. reject non-Regional, disabled, stale, or inactive policy state;
2. evaluate the currently published navigation target with existing composite reach/unreachable semantics;
3. keep one stable C3/C4 coordinate while the same event is in Travel/SearchHold;
4. publish the coordinate through the normal `/ly/navi/goal_pos(_raw)` path, set normal movement speed, and never route it through `AreaManager` default selection;
5. record a `protect_outpost_damage` decision intent whose detail reports the phase and event generation;
6. return false after completion/cooldown so no stale event owns navigation.

Add `DecisionReason::ProtectOutpost`, string/layer mapping, and exact goal-detail logging for damage, arrival, hold, unreachable, completion, and preemption. Keep no direct publisher in this task: the existing final publisher remains the sole output owner.

- [ ] **Step 4: Replace the fixed four-action tail with YAML priority ordering**

After the existing Hard/Task/opening/Buff/Outpost-aim branches, build a fixed four-entry candidate sequence and `stable_sort` it by the configured integer and listed-order tie-breaker. Execute these existing/new actions in order:

```cpp
ProtectCastle  -> TrySetRegionalDefenseGoal(my_team, enemy_team)
ProtectOutpost -> TrySetProtectOutpostGoal(my_team, enemy_team)
ProtectHero    -> TrySetProtectHeroGoal(my_team, enemy_team)
Chase          -> TryApplyChaseTactical()
```

Only mark the Tactical layer handled for the first action that returns true. The state machine itself remains active during a higher-priority preemption, but no repeated goal publication is allowed while its C3/C4 output is already selected.

- [ ] **Step 5: Run focused build/test and inspect the decision path**

Run:

```bash
colcon build --packages-select behavior_tree --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select behavior_tree --ctest-args -R test_tactical_protection_policy --output-on-failure
```

Expected: the policy test passes and compilation proves the new Application/StrategyManager contract. If `/usr/bin/python3` cannot import `ament_package`, record that environmental failure without claiming a successful build.

- [ ] **Step 6: Commit the runtime slice**

```bash
git add src/behavior_tree/module/BasicTypes.hpp src/behavior_tree/module/Area.hpp \
  src/behavior_tree/include/Application.hpp src/behavior_tree/src/SubscribeMessage.cpp \
  src/behavior_tree/src/Configuration.cpp src/behavior_tree/src/GameLoop.cpp \
  src/behavior_tree/src/StrategyManager.cpp src/behavior_tree/include/DecisionIntent.hpp
git commit -m "behavior_tree: prioritize own outpost defense"
```

### Task 3: Surface and validate the new Tactical contract

**Files:**
- Modify: `src/behavior_tree/include/DecisionExplain.hpp`
- Modify: `src/behavior_tree/src/Application.cpp`
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`
- Modify: `docs/modules/2026-05-05_behavior_tree.md`
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `scripts/selfcheck/sentry.sh` only if the Tactical config contract is explicitly enumerated there

**Interfaces:**
- Produces one startup configuration log containing the ProtectOutpost enable/state timing and all four priority values.
- Produces decision-trace Tactical evidence for current own-outpost HP freshness and ProtectOutpost phase.

- [ ] **Step 1: Add the failing decision-explain expectation**

Extend `test_tactical_protection_policy.cpp` or `test_decision_explain.cpp` to assert that config output contains `protect_outpost=enabled`, `search_hold_sec=30`, and the four named priorities. This prevents a YAML feature that cannot be verified from startup logs.

- [ ] **Step 2: Implement observability and trace fields**

Extend `DecisionExplain::ConfigSnapshot` and `LogDecisionConfigurationOnce()` with the new settings. In `DecisionTrace.cpp`, add a `tactical.protect_outpost` object with `enabled`, `hp`, `hp_fresh`, `phase`, `event_generation`, `search_hold_sec`, and `priority`. Reuse the normal decision-output goal data; do not add simulator-only derived state.

- [ ] **Step 3: Update current documentation**

Document C3/C4, fresh strict-decrease trigger, 30-second arrival hold, cooldown behavior, exact priority order, and the fact that Hard/Task/Aim behavior remains outside this priority table. Update each dated document's `Updated: 2026-07-22` line.

- [ ] **Step 4: Run final verification**

Run:

```bash
python3 - <<'PY'
import yaml
yaml.safe_load(open('src/behavior_tree/config/Tactical.yaml'))
PY
./scripts/selfcheck.sh sentry --static-only
git diff --check
```

Also run the focused behavior-tree build/test commands from Task 2 when dependencies permit. Verify the static check sees `Tactical.Priority` and `Tactical.ProtectOutpost` after any required selfcheck update.

- [ ] **Step 5: Commit observability and documentation**

```bash
git add src/behavior_tree/include/DecisionExplain.hpp src/behavior_tree/src/Application.cpp \
  src/behavior_tree/src/DecisionTrace.cpp docs/modules/2026-05-05_behavior_tree.md \
  docs/sentry/regional/2026-07-12_regional_decision_graph.md \
  docs/sentry/regional/decision_framework.md scripts/selfcheck/sentry.sh
git commit -m "behavior_tree: document ProtectOutpost tactical priority"
```

## Plan Self-Review

- Spec coverage: fresh strict decrease, C3/C4, 30-second post-arrival hold, new damage refresh, cooldown, priority ordering, preemption, output ownership, logs, trace, docs, and tests each map to a task.
- Placeholder scan: no TBD/TODO/implicit implementation steps remain.
- Type consistency: the policy state is pure and owned by `Application`; the Tactical scheduler calls only `TrySetProtectOutpostGoal` and existing strategy methods.
