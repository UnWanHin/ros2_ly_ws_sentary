# ProtectCastle RFID Stay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Tactical YAML switch that makes a fresh opponent occupation of the team's fortress drive the sentry to `Castle` and keep its chassis there after arrival.

**Architecture:** Extend the existing `ProtectCastleSetting` and `TacticalProtectionPolicy` rather than adding a strategy layer. The RegionalDefense RFID branch will use `Castle` as its sole candidate while the switch is active; chase authorization will reject movement only after composite Castle arrival. The decision trace and simulator continue to consume BT-authored evidence.

**Tech Stack:** ROS2 Humble C++20, `ament_cmake_gtest`, YAML parameters, Python simulator trace parser, shell static self-check.

## Global Constraints

- Only the fresh referee `event_data` RFID branch (`status == 2 || status == 3`) may activate this stay behaviour.
- `EnemyPos` semantics and all other ProtectCastle behaviour remain unchanged.
- No topic, message, launch argument, or module changes.
- `StayWhenRfid: false` preserves the current four-Castle-point RFID search behaviour.
- Aim, rotate, and fire remain active while only chassis navigation chase/velocity is suppressed after Castle arrival.

---

### Task 1: Specify and test the additive Tactical switch

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp:616-620`
- Modify: `src/behavior_tree/config/Tactical.yaml:5-10`
- Modify: `src/behavior_tree/src/Configuration.cpp:1792-1821,2263-2265`
- Modify: `src/behavior_tree/include/TacticalProtectionPolicy.hpp:13-18`
- Modify: `src/behavior_tree/test/test_tactical_protection_policy.cpp:17-22`
- Modify: `scripts/selfcheck/sentry.sh:552-586`

**Interfaces:**
- Produces: `config.TacticalSettings.ProtectCastle.StayWhenRfid`.
- Produces: `IsProtectCastleRfidStayEnabled(enable, rfid, stay_when_rfid, event_active)`.

- [ ] **Step 1: Write the failing policy test**

```cpp
TEST(TacticalProtectionPolicy, ProtectCastleRfidStayRequiresAllFourGates) {
    EXPECT_TRUE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(false, true, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, false, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, false, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, true, false));
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_tactical_protection_policy --output-on-failure`

Expected: compile failure because `IsProtectCastleRfidStayEnabled` does not exist.

- [ ] **Step 3: Add the setting and policy predicate**

```cpp
struct ProtectCastleSetting {
    bool Enable{true};
    bool RFID{true};
    bool EnemyPos{true};
    bool StayWhenRfid{false};
};

inline bool IsProtectCastleRfidStayEnabled(
    const bool protect_castle_enable,
    const bool rfid_enable,
    const bool stay_when_rfid,
    const bool event_active) noexcept {
    return protect_castle_enable && rfid_enable && stay_when_rfid && event_active;
}
```

Read `Tactical.ProtectCastle.StayWhenRfid` through the existing optional-bool
pattern and `ResolveTacticalFeatureEnable`; write `StayWhenRfid: true` below
`RFID` in `Tactical.yaml`. Add the config debug line and make the static
self-check require both the YAML key and its `Configuration.cpp` read.

- [ ] **Step 4: Run the focused test and static contract check**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_tactical_protection_policy --output-on-failure`

Expected: policy test passes.

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: Tactical ProtectCastle contract passes; unrelated unavailable external ROS interfaces may remain environment warnings/failures.

- [ ] **Step 5: Commit the configuration contract**

```bash
git add src/behavior_tree/module/BasicTypes.hpp src/behavior_tree/config/Tactical.yaml \
  src/behavior_tree/src/Configuration.cpp src/behavior_tree/include/TacticalProtectionPolicy.hpp \
  src/behavior_tree/test/test_tactical_protection_policy.cpp scripts/selfcheck/sentry.sh
git commit -m "behavior_tree: add fortress RFID stay switch"
```

### Task 2: Apply the RFID-only Castle hold to RegionalDefense and chase

**Files:**
- Modify: `src/behavior_tree/src/GameLoop.cpp:3799-3810,4029-4037`
- Modify: `src/behavior_tree/src/StrategyManager.cpp:59-78`
- Modify: `src/behavior_tree/test/test_tactical_protection_policy.cpp`

**Interfaces:**
- Consumes: `ProtectCastle.StayWhenRfid` and the Task 1 predicate.
- Produces: an RFID RegionalDefense goal candidate list of exactly `{Castle}` while active.
- Produces: no navigation chase authorization after composite arrival at Castle while active.

- [ ] **Step 1: Extend the failing policy test with the inactive-switch regression**

```cpp
TEST(TacticalProtectionPolicy, ProtectCastleRfidStayDoesNotEnableFromEnemyPositionOnly) {
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidStayEnabled(true, true, true, false));
}
```

This locks the required boundary: EnemyPos alone cannot activate the stay lock.

- [ ] **Step 2: Run the focused test and verify RED if the predicate was not added in Task 1**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_tactical_protection_policy --output-on-failure`

Expected: it fails before Task 1 implementation and passes after Task 1.

- [ ] **Step 3: Implement only the two movement decisions**

In `TrySetRegionalDefenseGoal`, compute the existing fresh RFID event once and
use the Task 1 predicate. Replace the RFID branch candidate list with:

```cpp
candidates = protect_castle_rfid_stay
    ? std::vector<std::uint8_t>{LangYa::Castle.ID}
    : order_nearest_base_candidates({
        LangYa::CastleLeft1.ID,
        LangYa::CastleLeft2.ID,
        LangYa::CastleRight1.ID,
        LangYa::CastleRight2.ID,
    });
```

In `CanAuthorizeChaseTactical`, reject chase only when all are true:

```cpp
const bool fortress_stay_after_arrival =
    IsProtectCastleRfidStayEnabled(
        config.TacticalSettings.ProtectCastle.Enable,
        config.TacticalSettings.ProtectCastle.RFID,
        config.TacticalSettings.ProtectCastle.StayWhenRfid,
        IsFortressGainPointEnemyOccupiedEventFresh(referee_fresh_ms)) &&
    regionalDefenseSearchKind_ == RegionalDefenseSearchKind::OwnFortressGainPoint &&
    IsBaseGoalArrived(LangYa::Castle.ID, team, true);
```

Return `false` for this condition before aim-mode authorization. Do not clear
aim data, fire code, rotate, or FaceMode; the existing caller then retains the
RegionalDefense Castle command and simply emits no chase velocity/goal.

- [ ] **Step 4: Run behavior_tree tests**

Run: `colcon test --packages-select behavior_tree --ctest-args --output-on-failure`

Expected: all available behavior_tree tests pass.

- [ ] **Step 5: Commit the runtime behaviour**

```bash
git add src/behavior_tree/src/GameLoop.cpp src/behavior_tree/src/StrategyManager.cpp \
  src/behavior_tree/test/test_tactical_protection_policy.cpp
git commit -m "behavior_tree: hold castle for enemy RFID occupation"
```

### Task 3: Surface the switch in decision traces, simulator, and current docs

**Files:**
- Modify: `src/behavior_tree/src/DecisionTrace.cpp:621-630`
- Modify: `src/simulator/simulator/model.py:412-443`
- Modify: `src/simulator/simulator/trace.py:554-601`
- Modify: `src/simulator/simulator/tactical_web.py:442`
- Modify: `src/simulator/test/test_trace_contract.py`
- Modify: `src/simulator/simulator/web_visual_check.py:141-146`
- Modify: `docs/sentry/regional/current_behavior.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/internal/simulator.md`

**Interfaces:**
- Produces: `tactical.protect_castle.stay_when_rfid_enabled` in every v4 trace.
- The simulator preserves old traces by treating a missing key as `None`.

- [ ] **Step 1: Add failing trace parser expectations**

Add a v4 trace fixture assertion that `stay_when_rfid_enabled` is normalized
to `True`, emitted in `TacticalDecisionState.as_payload()`, and absent legacy
records remain readable with `None`.

- [ ] **Step 2: Run the focused simulator trace test and verify RED**

Run: `PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_trace_contract.py -q`

Expected: the new expectation fails before the parser/model field exists.

- [ ] **Step 3: Add BT evidence and simulator normalization**

Emit the key from `DecisionTrace.cpp` alongside `rfid_enabled`; add the
nullable field to `TacticalDecisionState`, `empty_tactical_state`,
`normalize_tactical`, and `as_payload`. Add one Tactical inspector row:

```javascript
['RFID stay', castle.stay_when_rfid_enabled]
```

Keep the browser observer-only; it displays BT trace evidence and does not
change simulation inputs or decision logic.

- [ ] **Step 4: Update documentation and sample expectations**

Document that `StayWhenRfid` only applies to fresh `event_data` status `2/3`,
goes to `Castle`, and blocks chassis chase only after arrival. Explicitly
state that `EnemyPos` is unaffected. Update the dated simulator document's
`Updated:` value to `2026-07-21`.

- [ ] **Step 5: Run trace, simulator, and static verification**

Run: `PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_trace_contract.py src/simulator/test/test_scenarios.py -q`

Expected: all selected tests pass.

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: Tactical contract passes; report external `sentry_msgs` availability separately if this host lacks it.

Run: `git diff --check`

Expected: no output.

- [ ] **Step 6: Commit trace and documentation evidence**

```bash
git add src/behavior_tree/src/DecisionTrace.cpp src/simulator/simulator/model.py \
  src/simulator/simulator/trace.py src/simulator/simulator/tactical_web.py \
  src/simulator/test/test_trace_contract.py src/simulator/simulator/web_visual_check.py \
  docs/sentry/regional/current_behavior.md docs/sentry/regional/decision_framework.md \
  docs/sentry/internal/simulator.md
git commit -m "simulator: expose fortress RFID stay evidence"
```

### Task 4: Final integration verification

**Files:**
- Verify only: all Task 1-3 files.

- [ ] **Step 1: Build the changed packages**

Run: `colcon build --packages-select behavior_tree simulator`

Expected: successful build when the environment has ROS2, BehaviorTree.CPP v4,
and current `sentry_msgs` from `sentry.aim`.

- [ ] **Step 2: Run all available changed-package tests**

Run: `colcon test --packages-select behavior_tree simulator --ctest-args --output-on-failure`

Expected: no test failures.

- [ ] **Step 3: Inspect the final diff and commit state**

Run: `git diff --check && git status --short`

Expected: no diff-check output and a clean worktree after commits.

- [ ] **Step 4: Commit only if a verification-only correction was needed**

```bash
git add <corrected-files>
git commit -m "test: verify fortress RFID stay policy"
```

