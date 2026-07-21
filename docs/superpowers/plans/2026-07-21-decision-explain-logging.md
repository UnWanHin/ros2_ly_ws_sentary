# Decision Explain Logging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Print the effective Area/Tactical configuration once at BT start and print one compact, de-duplicated explanation whenever the final publishable navigation decision changes.

**Architecture:** Keep formatting and de-duplication as a pure header helper so it has fast unit coverage. `Application` only builds an observation from existing `lastDecisionIntent_`, `naviCommandGoal`, `naviGoalPosition`, and configuration, then invokes the helper at the already-final `Run` and `PublishMessageAll` boundaries.

**Tech Stack:** C++20, ROS2 Humble, existing `Utils::Logger`, gtest, existing behavior-tree configuration and decision intent types.

## Global Constraints

- Do not add ROS topics, parameters, timers, serial writes, or navigation/control behavior.
- Do not log from individual Tactical/Default/Task branches.
- Log configuration after parser defaults and YAML overrides have been resolved.
- Log navigation only when `publishNaviGoal_` and `naviGoalPublishAllowed_` are both true.
- A new Tactical owner of navigation must add/reuse a non-unknown `DecisionReason` and call `RecordDecisionIntent(MakeDecisionIntent(...))` after choosing its final goal.
- Preserve the user-owned `docs/rules/.~lock.*` file; never stage it.

### Task 1: Add pure explanation formatting and transition tests

**Files:**
- Create: `src/behavior_tree/include/DecisionExplain.hpp`
- Create: `src/behavior_tree/test/test_decision_explain.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- Produces `DecisionExplain::ConfigSnapshot`, `DecisionExplain::NavigationObservation`, `DecisionExplain::Fingerprint`, `MakeFingerprint`, `FormatConfigLines`, and `FormatNavigationLine`.
- `NavigationObservation` contains `DecisionIntent`, the final goal ID and coordinate, the final publish flags, and `MapCommandActive`.

- [ ] **Step 1: Write the failing gtest cases**

```cpp
TEST(DecisionExplain, FormatsEffectiveAreaAndTacticalSettings) {
    const BehaviorTree::DecisionExplain::ConfigSnapshot snapshot{
        .RegionalAreaTaskEnable = true,
        .MyBaseEnable = true,
        .MyHighlandEnable = false,
        .MyPreRoadlandEnable = true,
        .MyReadyRoadlandEnable = true,
        .CommonCentralEnable = false,
        .ProtectCastleEnable = true,
        .ProtectCastleRfidEnable = true,
        .ProtectCastleEnemyPosEnable = false,
        .ProtectCastleStayWhenRfid = true,
        .ProtectHeroEnable = true,
        .DamageRotateDefaultGear = 0,
        .DamageRotateNoHitTimeoutMs = 1800,
        .DamageRotateGear0HoldMs = 220,
        .DamageRotateGear1HoldMs = 220,
        .DamageRotateGear2HoldMs = 220,
        .DamageRotateScanBoostWindowMs = 1300,
        .DamageRotateScanYawPhaseMs = 160,
    };
    const auto lines = BehaviorTree::DecisionExplain::FormatConfigLines(snapshot);
    EXPECT_EQ(lines.size(), 3U);
    EXPECT_NE(lines[0].find("MyHighland=0"), std::string::npos);
    EXPECT_NE(lines[1].find("enemy_pos=0"), std::string::npos);
}

TEST(DecisionExplain, SuppressesIdenticalPublishedNavigation) {
    const auto observation = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    const auto first = BehaviorTree::DecisionExplain::MakeFingerprint(observation);
    EXPECT_TRUE(first.has_value());
    EXPECT_EQ(first, BehaviorTree::DecisionExplain::MakeFingerprint(observation));
}

TEST(DecisionExplain, IncludesReasonChangeAndRawMapCommandCoordinate) {
    const auto first = MakeObservation(
        BehaviorTree::DecisionReason::DefaultAreaPolicy, 8U, 108U, 1600U, 720U, "MyHighland");
    const auto second = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    EXPECT_NE(BehaviorTree::DecisionExplain::MakeFingerprint(first),
              BehaviorTree::DecisionExplain::MakeFingerprint(second));
    const auto map = MakeMapCommandObservation(927U, 563U);
    EXPECT_NE(BehaviorTree::DecisionExplain::FormatNavigationLine(map).find("raw_map_command"),
              std::string::npos);
}

TEST(DecisionExplain, DoesNotProduceFingerprintForDisabledNavigation) {
    auto observation = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    observation.NaviGoalPublishAllowed = false;
    EXPECT_FALSE(BehaviorTree::DecisionExplain::MakeFingerprint(observation).has_value());
}
```

- [ ] **Step 2: Run the focused test and confirm RED**

Run:

```bash
source /opt/ros/humble/setup.zsh
cmake --build build/behavior_tree --target test_decision_explain -j2
```

Expected: target or header is missing.

- [ ] **Step 3: Implement the pure helper**

Create `DecisionExplain.hpp` with only standard-library and `DecisionIntent.hpp` dependencies. Define:

```cpp
namespace BehaviorTree::DecisionExplain {
struct ConfigSnapshot { /* exact fields asserted above */ };
struct NavigationObservation {
    DecisionIntent Intent{};
    std::uint8_t PublishedGoalId{};
    std::uint16_t XCentimeter{};
    std::uint16_t YCentimeter{};
    bool PublishNaviGoal{};
    bool NaviGoalPublishAllowed{};
    bool MapCommandActive{};
};
struct Fingerprint { /* all observation fields that explain a decision */
    bool operator==(const Fingerprint&) const = default;
};
std::array<std::string, 3> FormatConfigLines(const ConfigSnapshot& snapshot);
std::optional<Fingerprint> MakeFingerprint(const NavigationObservation& observation);
std::string FormatNavigationLine(const NavigationObservation& observation);
}
```

`MakeFingerprint` returns `std::nullopt` unless both publish flags are true.
`FormatNavigationLine` uses `raw_map_command` for `MapCommandActive`; otherwise it writes the goal ID and final cm coordinate. It always writes `DecisionLayerToString`, `DecisionReasonToString`, intent base/resolved goal IDs, `UnitTeam` as `red`, `blue`, or `unknown`, priority, and detail.

- [ ] **Step 4: Register and run the green test**

Append this target beside the existing standalone policy tests in `src/behavior_tree/CMakeLists.txt`:

```cmake
ament_add_gtest(test_decision_explain test/test_decision_explain.cpp)
target_include_directories(test_decision_explain PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
)
```

Run:

```bash
source /opt/ros/humble/setup.zsh
cmake --build build/behavior_tree --target test_decision_explain -j2
./build/behavior_tree/test_decision_explain
```

Expected: all DecisionExplain tests pass.

- [ ] **Step 5: Commit the helper and regression tests**

```bash
git add src/behavior_tree/include/DecisionExplain.hpp src/behavior_tree/test/test_decision_explain.cpp src/behavior_tree/CMakeLists.txt
git commit -m "behavior_tree: format decision explanations"
```

### Task 2: Emit startup configuration and final navigation transitions

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/Application.cpp`
- Modify: `src/behavior_tree/src/PublishMessage.cpp`

**Interfaces:**
- `Application::LogDecisionConfigurationOnce()` reads the effective `config` fields and emits the three helper lines.
- `Application::MaybeLogNavigationDecision()` reads the final state, emits one helper line on fingerprint change, and records that fingerprint only after emission.

- [ ] **Step 1: Add the integration assertions before implementation**

Extend `test_decision_explain.cpp` with a fixture-level expectation that `FormatNavigationLine` includes all final fields:

```cpp
EXPECT_NE(line.find("layer=tactical"), std::string::npos);
EXPECT_NE(line.find("reason=protect_hero"), std::string::npos);
EXPECT_NE(line.find("base_goal=8"), std::string::npos);
EXPECT_NE(line.find("resolved_goal=108"), std::string::npos);
EXPECT_NE(line.find("pos_cm=(1600,720)"), std::string::npos);
EXPECT_NE(line.find("detail=protect_hero"), std::string::npos);
```

- [ ] **Step 2: Run the test and confirm RED**

Run:

```bash
./build/behavior_tree/test_decision_explain
```

Expected: the exact field-name assertion fails before formatter completion.

- [ ] **Step 3: Add Application state and methods**

In `Application.hpp`, include `DecisionExplain.hpp`, add:

```cpp
bool decisionConfigurationLogged_{false};
std::optional<DecisionExplain::Fingerprint> lastDecisionExplainFingerprint_{};
void LogDecisionConfigurationOnce();
void MaybeLogNavigationDecision();
```

`LogDecisionConfigurationOnce()` creates `ConfigSnapshot` from:

```cpp
config.RegionalAreaTaskSettings.Enable
config.RegionalAreaTaskSettings.MyBase.Enable
config.RegionalAreaTaskSettings.MyHighland.Enable
config.RegionalAreaTaskSettings.MyPreRoadland.Enable
config.RegionalAreaTaskSettings.MyReadyRoadland.Enable
config.RegionalAreaTaskSettings.CommonCentral.Enable
config.TacticalSettings.ProtectCastle.Enable
config.TacticalSettings.ProtectCastle.RFID
config.TacticalSettings.ProtectCastle.EnemyPos
config.TacticalSettings.ProtectCastle.StayWhenRfid
config.TacticalSettings.ProtectHero.Enable
config.TacticalSettings.DamageRotate.DefaultGear
config.TacticalSettings.DamageRotate.NoHitTimeoutMs
config.TacticalSettings.DamageRotate.Gear0HoldMs
config.TacticalSettings.DamageRotate.Gear1HoldMs
config.TacticalSettings.DamageRotate.Gear2HoldMs
config.TacticalSettings.DamageRotate.ScanBoostWindowMs
config.TacticalSettings.DamageRotate.ScanYawPhaseMs
```

It writes each `FormatConfigLines` item with `LoggerPtr->Info("{}", line)` and
sets `decisionConfigurationLogged_` only after all three lines are emitted.

`MaybeLogNavigationDecision()` builds `NavigationObservation` from
`lastDecisionIntent_`, `naviCommandGoal`, `naviGoalPosition`, `publishNaviGoal_`,
`naviGoalPublishAllowed_`, and `activeMapCommandGoal_.has_value()`. It returns
without writing when `MakeFingerprint` is empty or equals
`lastDecisionExplainFingerprint_`; otherwise it writes `FormatNavigationLine`
and stores the new fingerprint.

- [ ] **Step 4: Place calls at the final boundaries**

At the top of `Application::Run()` in `src/behavior_tree/src/Application.cpp`,
call `LogDecisionConfigurationOnce()` before `WaitBeforeGame()` so configuration
is visible even when the match has not started.

In `PublishMessageAll()`, after `naviGoalPublishAllowed_` has passed and before
each of `PubMapCommandGoalPos()`, `PubNaviGoalPos()`, and `PubNaviGoal()`, call
`MaybeLogNavigationDecision()`. Do not call it before the flag guard and do not
call it from `PubNaviGoal*` functions, because those helpers are not the single
selection boundary.

- [ ] **Step 5: Build and run targeted tests**

Run:

```bash
source /opt/ros/humble/setup.zsh
cmake --build build/behavior_tree --target behavior_tree_node test_decision_explain -j2
./build/behavior_tree/test_decision_explain
```

Expected: node build succeeds and all DecisionExplain tests pass.

- [ ] **Step 6: Commit runtime wiring**

```bash
git add src/behavior_tree/include/Application.hpp src/behavior_tree/src/Application.cpp src/behavior_tree/src/PublishMessage.cpp src/behavior_tree/test/test_decision_explain.cpp
git commit -m "behavior_tree: log final navigation decisions"
```

### Task 3: Record the operator contract and verify the package boundary

**Files:**
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `docs/sentry/regional/current_behavior.md`

- [ ] **Step 1: Document the startup and transition log contract**

Add one current-status paragraph to the regional decision graph that states:

```text
behavior_tree prints one effective AreaManager/Tactical snapshot before waiting
for the match. It then prints [Decision] only when the final publishable
navigation fingerprint changes. The line is derived from DecisionIntent plus
the final goal ID and official cm coordinate; it is not a per-tick trace.
```

Add a maintenance sentence to `current_behavior.md` requiring future Tactical
navigation owners to record a specific `DecisionReason` and detail in the same
change.

- [ ] **Step 2: Run the required checks**

Run:

```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-select behavior_tree --symlink-install
source install/setup.zsh
ctest --test-dir build/behavior_tree --output-on-failure -R 'decision_explain|tactical_protection_policy|decision_trace_control_output'
./scripts/selfcheck.sh sentry --static-only
git diff --check
```

Expected: targeted behavior-tree tests pass. If static self-check reports only
missing external `sentry_msgs`, report that external dependency separately.

- [ ] **Step 3: Commit documentation**

```bash
git add docs/sentry/regional/2026-07-12_regional_decision_graph.md docs/sentry/regional/current_behavior.md
git commit -m "docs: describe decision explain logs"
```
