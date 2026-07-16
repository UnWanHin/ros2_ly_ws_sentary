# 前哨強化交戰鎖 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 讓前哨 id `7` 的已確認交戰先鎖普通進攻，敵方前哨 HP 新鮮下降後安全使用強化進攻，並以 200/250 HP 門檻控制轉火與退出。

**Architecture:** 新增純 `OutpostEngagementLock`，只產生高優先級 intent 和 target-hold 決策。`PostureManager` 維持唯一姿態命令 owner，但從普通姿態擴充為「基礎類別 + 是否強化」的複合 ACK。BT 把鎖結果接到現有 aim/posture tick；trace 和 simulator 消費同一份狀態。

**Tech Stack:** ROS2 Humble、C++20、gtest、nlohmann JSON、Python simulator、YAML、Understand Anything fallback graph。

## Global Constraints

- 不改既有 ROS topic/message、裁判 `0x020D`／`0x0120` 位布局或 gimbal serial frame。
- 只有 `PostureManager` 可以產生 `/ly/control/posture` 命令。
- 強化 `4` 的 ACK 必須是新鮮 `posture=1 && enhanced_posture=true`；強化姿態不可 optimistic ACK。
- 前哨 HP 為零/stale、導航 unreachable、或對應 200/250 HP 安全閾值命中時，必須取消前哨 owned pending request。
- 不改 Buff、Regional、一般巡邏與非前哨姿態接口。
- 不提交 `build/`、`install/`、`log/` 或既有 PDF metadata/lock 檔。

---

### Task 1: 複合姿態模式與 ACK 管理

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/include/PostureManager.hpp`
- Modify: `src/behavior_tree/src/PostureManager.cpp`
- Create: `src/behavior_tree/test/test_posture_manager.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- `PostureMode { SentryPosture Base; bool Enhanced; }`
- `PostureFeedback { uint8_t Base; bool Enhanced; bool Fresh; bool EnhancedFresh; }`; `Fresh` is the base-posture freshness and `EnhancedFresh` is the independent referee enhanced-bit freshness.
- `PostureRequestPolicy { bool AllowOptimisticAck; bool PreserveCurrentOnRetryExhausted; }`
- `uint8_t ToPostureCommandValue(PostureMode)` maps normal `1..3` and strong `4..6`.
- `PostureManager::CancelPending()` clears only an unconfirmed request.
- `PostureManager::Tick(TimePoint, PostureMode, PostureFeedback, PostureRefereeTimer, PostureRequestPolicy)` is the new mode-aware overload; the existing `Tick(TimePoint, SentryPosture, uint8_t, PostureRefereeTimer)` remains a wrapper using `{false, false}` policy.

- [x] **Step 1: Add failing gtests for command 4 ACK and retry preservation**

```cpp
#include "../include/PostureManager.hpp"

#include <chrono>
#include <gtest/gtest.h>

using namespace std::chrono_literals;

namespace {

LangYa::PostureSetting EnabledPostureSetting() {
    LangYa::PostureSetting setting;
    setting.Enable = true;
    setting.SwitchCooldownSec = 5;
    setting.MinHoldSec = 0;
    setting.PendingAckTimeoutMs = 600;
    setting.RetryIntervalMs = 300;
    setting.MaxRetryCount = 3;
    setting.OptimisticAck = true;
    return setting;
}

BehaviorTree::PostureFeedback FreshAttackFeedback(const bool enhanced) {
    return {1U, enhanced, true, true};
}

}  // namespace

TEST(PostureManagerTest, EnhancedAttackNeedsEnhancedFeedbackForAck) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);
    EXPECT_EQ(4U, manager.Tick(
        now + 5s,
        {BehaviorTree::SentryPosture::Attack, true},
        FreshAttackFeedback(false), {},
        BehaviorTree::PostureRequestPolicy::OutpostLock()).Command);
    EXPECT_TRUE(manager.Runtime().HasPending);
    manager.Tick(
        now + 5s + 1ms,
        {BehaviorTree::SentryPosture::Attack, true},
        FreshAttackFeedback(true), {},
        BehaviorTree::PostureRequestPolicy::OutpostLock());
    EXPECT_FALSE(manager.Runtime().HasPending);
    EXPECT_TRUE(manager.Runtime().Current.Enhanced);
}

TEST(PostureManagerTest, FailedOutpostRequestDoesNotIssueDefenseOrMove) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);
    const BehaviorTree::PostureMode enhanced_attack{BehaviorTree::SentryPosture::Attack, true};
    const auto policy = BehaviorTree::PostureRequestPolicy::OutpostLock();

    const auto first = manager.Tick(now + 5s, enhanced_attack, FreshAttackFeedback(false), {}, policy);
    const auto retry_one = manager.Tick(now + 5600ms, enhanced_attack, FreshAttackFeedback(false), {}, policy);
    const auto retry_two = manager.Tick(now + 5900ms, enhanced_attack, FreshAttackFeedback(false), {}, policy);
    const auto exhausted = manager.Tick(now + 6200ms, enhanced_attack, FreshAttackFeedback(false), {}, policy);

    EXPECT_EQ(4U, first.Command);
    EXPECT_EQ(4U, retry_one.Command);
    EXPECT_EQ(4U, retry_two.Command);
    EXPECT_EQ(0U, exhausted.Command);
    EXPECT_EQ(BehaviorTree::SentryPosture::Attack, manager.Runtime().Current.Base);
    EXPECT_FALSE(manager.Runtime().Current.Enhanced);
    EXPECT_FALSE(manager.Runtime().HasPending);
    EXPECT_EQ("pending_preserved", exhausted.Reason);
}

TEST(PostureManagerTest, CooldownBeginsAtCompositeAck) {
    BehaviorTree::PostureManager manager;
    manager.Configure(EnabledPostureSetting());
    const auto now = BehaviorTree::PostureManager::TimePoint{};
    manager.Reset(now, BehaviorTree::SentryPosture::Attack);
    const auto policy = BehaviorTree::PostureRequestPolicy::OutpostLock();
    manager.Tick(now + 5s, {BehaviorTree::SentryPosture::Attack, true}, FreshAttackFeedback(false), {}, policy);
    manager.Tick(now + 5200ms, {BehaviorTree::SentryPosture::Attack, true}, FreshAttackFeedback(true), {}, policy);
    const auto decision = manager.Tick(
        now + 10199ms,
        {BehaviorTree::SentryPosture::Move, false},
        FreshAttackFeedback(true), {},
        BehaviorTree::PostureRequestPolicy{});

    EXPECT_EQ(0U, decision.Command);
    EXPECT_STREQ("cooldown", decision.Reason);
}
```

- [x] **Step 2: Register the test and confirm it initially fails on the missing API**

Add `ament_add_gtest(test_posture_manager test/test_posture_manager.cpp src/PostureManager.cpp)` with the established include paths. Run:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test --packages-select behavior_tree --ctest-args -R test_posture_manager --output-on-failure'
```

Expected: compile failure until the new mode/feedback API exists.

- [x] **Step 3: Implement composite mode without changing base score semantics**

```cpp
constexpr uint8_t ToPostureCommandValue(const PostureMode mode) noexcept {
    const auto base = ToPostureValue(mode.Base);
    return base == 0 ? 0 : static_cast<uint8_t>(base + (mode.Enhanced ? 3 : 0));
}

constexpr bool MatchesFeedback(const PostureMode mode, const PostureFeedback feedback) noexcept {
    return feedback.Fresh && ToPosture(feedback.Base) == mode.Base &&
        feedback.Enhanced == mode.Enhanced;
}
```

Keep accumulation/degradation arrays indexed by base `1..3`. Convert pending/current/desired command state to `PostureMode`; start `last_switch_` only after `MatchesFeedback()`. When `PreserveCurrentOnRetryExhausted=true`, clear pending with reason `pending_preserved` and do not call `choose_alternative_posture()`. Preserve the old base-only `Tick()` overload for all callers outside this feature.

- [x] **Step 4: Run posture regression tests**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test --packages-select behavior_tree --ctest-args -R "test_posture_manager|test_navi_rotate_posture" --output-on-failure'
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test-result --verbose'
```

Expected: selected tests pass; `NaviRotate` still requests ordinary Move.

- [x] **Step 5: Commit**

```bash
git add src/behavior_tree/include/PostureTypes.hpp src/behavior_tree/include/PostureManager.hpp src/behavior_tree/src/PostureManager.cpp src/behavior_tree/test/test_posture_manager.cpp src/behavior_tree/CMakeLists.txt
git commit -m "behavior_tree: confirm composite posture commands"
```

### Task 2: 純前哨交戰鎖狀態機

**Files:**
- Create: `src/behavior_tree/include/OutpostEngagementLock.hpp`
- Create: `src/behavior_tree/src/OutpostEngagementLock.cpp`
- Create: `src/behavior_tree/test/test_outpost_engagement_lock.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- `OutpostEngagementInput` is `{ bool Target7Fresh; bool SelectedTarget7; bool EnemyHpFresh; uint16_t EnemyHp; bool SelfHpFresh; uint16_t SelfHp; bool NavigationReachable; PostureRuntime Posture; bool PostureCooldownReady; bool EnhancedAttackRemainingFresh; uint8_t EnhancedAttackRemainingSec; }`.
- `OutpostEngagementDecision` is `{ bool Active; bool HoldTarget; bool CancelPending; bool EnhancedArmed; bool EnhancedUnavailable; bool EnhancedPending; bool EnhancedActive; std::optional<PostureMode> Intent; OutpostEngagementExitReason ExitReason; }`.
- `OutpostEngagementLock::Tick(TimePoint, const OutpostEngagementInput&)` returns `OutpostEngagementDecision`; `Reset()` clears the HP baseline and the one-attempt enhanced latch.
- The class is pure C++: no `Application.hpp`, ROS, publisher, or topic dependency.

- [ ] **Step 1: Add failing state-machine tests**

```cpp
#include "../include/OutpostEngagementLock.hpp"

#include <chrono>
#include <gtest/gtest.h>

namespace {

BehaviorTree::OutpostEngagementInput HealthyOutpostInput() {
    BehaviorTree::OutpostEngagementInput input{};
    input.Target7Fresh = true;
    input.SelectedTarget7 = true;
    input.EnemyHpFresh = true;
    input.EnemyHp = 1000;
    input.SelfHpFresh = true;
    input.SelfHp = 400;
    input.NavigationReachable = true;
    input.Posture.Current = {BehaviorTree::SentryPosture::Attack, false};
    input.Posture.Desired = {BehaviorTree::SentryPosture::Attack, false};
    input.Posture.Pending = {BehaviorTree::SentryPosture::Unknown, false};
    input.PostureCooldownReady = true;
    input.EnhancedAttackRemainingFresh = true;
    input.EnhancedAttackRemainingSec = 15;
    return input;
}

}  // namespace

TEST(OutpostEngagementLockTest, TargetSevenRequestsNormalAttack) {
    BehaviorTree::OutpostEngagementLock lock;
    const auto out = lock.Tick(BehaviorTree::OutpostEngagementLock::TimePoint{}, HealthyOutpostInput());
    ASSERT_TRUE(out.Active);
    ASSERT_TRUE(out.HoldTarget);
    ASSERT_TRUE(out.Intent.has_value());
    EXPECT_EQ(1U, BehaviorTree::ToPostureCommandValue(*out.Intent));
}

TEST(OutpostEngagementLockTest, FreshHpDropArmsExactlyOneEnhancedAttack) {
    BehaviorTree::OutpostEngagementLock lock;
    const auto now = BehaviorTree::OutpostEngagementLock::TimePoint{};
    auto input = HealthyOutpostInput();
    lock.Tick(now, input);
    input.EnemyHp = 999;
    const auto armed = lock.Tick(now + std::chrono::milliseconds(1), input);
    ASSERT_TRUE(armed.EnhancedArmed);
    ASSERT_TRUE(armed.Intent.has_value());
    EXPECT_EQ(4U, BehaviorTree::ToPostureCommandValue(*armed.Intent));

    input.EnemyHp = 998;
    input.Posture.Current = {BehaviorTree::SentryPosture::Attack, true};
    const auto active = lock.Tick(now + std::chrono::milliseconds(2), input);
    EXPECT_TRUE(active.EnhancedActive);
    EXPECT_FALSE(active.EnhancedArmed);
    EXPECT_FALSE(active.Intent.has_value());
}

TEST(OutpostEngagementLockTest, LockThresholdsAre200And250) {
    BehaviorTree::OutpostEngagementLock normal_lock;
    auto normal = HealthyOutpostInput();
    normal.SelfHp = 201;
    const auto now = BehaviorTree::OutpostEngagementLock::TimePoint{};
    EXPECT_TRUE(normal_lock.Tick(now, normal).Active);
    normal.SelfHp = 200;
    const auto normal_exit = normal_lock.Tick(now + std::chrono::milliseconds(1), normal);
    EXPECT_FALSE(normal_exit.Active);
    EXPECT_EQ(BehaviorTree::OutpostEngagementExitReason::NormalHealthThreshold, normal_exit.ExitReason);

    BehaviorTree::OutpostEngagementLock enhanced_lock;
    auto enhanced = HealthyOutpostInput();
    enhanced.EnemyHp = 999;
    enhanced_lock.Tick(now, HealthyOutpostInput());
    enhanced.Posture.HasPending = true;
    enhanced.Posture.Pending = {BehaviorTree::SentryPosture::Attack, true};
    enhanced.SelfHp = 251;
    EXPECT_TRUE(enhanced_lock.Tick(now + std::chrono::milliseconds(1), enhanced).Active);
    enhanced.SelfHp = 250;
    const auto enhanced_exit = enhanced_lock.Tick(now + std::chrono::milliseconds(2), enhanced);
    EXPECT_FALSE(enhanced_exit.Active);
    EXPECT_EQ(BehaviorTree::OutpostEngagementExitReason::EnhancedHealthThreshold, enhanced_exit.ExitReason);
}

TEST(OutpostEngagementLockTest, StaleZeroAndUnreachableCancelPending) {
    for (const auto reason : {0, 1, 2}) {
        BehaviorTree::OutpostEngagementLock lock;
        const auto now = BehaviorTree::OutpostEngagementLock::TimePoint{};
        auto input = HealthyOutpostInput();
        lock.Tick(now, input);
        if (reason == 0) input.EnemyHpFresh = false;
        if (reason == 1) input.EnemyHp = 0;
        if (reason == 2) input.NavigationReachable = false;
        const auto out = lock.Tick(now + std::chrono::milliseconds(1), input);
        EXPECT_FALSE(out.Active);
        EXPECT_FALSE(out.HoldTarget);
        EXPECT_TRUE(out.CancelPending);
    }
}
```

- [ ] **Step 2: Implement input priority and HP baseline**

On lock entry cache the first fresh positive HP. Later arm only for a lower adjacent fresh sample; a higher sample refreshes baseline without arming. Arm once per continuous lock only when `EnhancedAttackRemainingFresh && EnhancedAttackRemainingSec > 0`. Process exits in this order: HP stale/zero, unreachable, normal-lock fresh own HP `<=200`, enhanced pending/active fresh own HP `<=250`, then target-7 no longer fresh. `Posture.Pending={Attack,true}` with `HasPending=true` and `Posture.Current={Attack,true}` both use the enhanced threshold. An armed state yields `{Attack,true}` only after normal Attack is confirmed and cooldown-ready. Retry exhaustion marks `EnhancedUnavailable` while retaining normal Attack and target hold.

- [ ] **Step 3: Register and run the pure test target**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test --packages-select behavior_tree --ctest-args -R test_outpost_engagement_lock --output-on-failure'
```

Expected: all four state-machine tests pass without launching ROS.

- [ ] **Step 4: Commit**

```bash
git add src/behavior_tree/include/OutpostEngagementLock.hpp src/behavior_tree/src/OutpostEngagementLock.cpp src/behavior_tree/test/test_outpost_engagement_lock.cpp src/behavior_tree/CMakeLists.txt
git commit -m "behavior_tree: add outpost engagement lock"
```

### Task 3: 配置與 BT 控制鏈整合

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/config/Task.yaml`
- Modify: `src/behavior_tree/config/OutpostRegionalTest.yaml`
- Modify: `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/include/BTNodes.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Modify: `src/behavior_tree/src/PublishMessage.cpp`

- [ ] **Step 1: Add config ownership and validation**

Extend `OutpostConfirmSetting`, JSON parsing, ROS parameter overrides, debug print, and validation with:

```yaml
Task:
  OutpostConfirm:
    TrustEnemyOutpostHp: true
    EnhancedAttackOnEnemyHpDrop: true
    NormalAttackLockExitHp: 200
    EnhancedAttackLockExitHp: 250
```

Set `Task.yaml` formal default `TrustEnemyOutpostHp: true`; set the same key in `OutpostRegionalTest.yaml` so its launch also exercises official HP gating. Add `EnhancedAttackOnEnemyHpDrop{true}`, `NormalAttackLockExitHp{200}`, and `EnhancedAttackLockExitHp{250}` to `LangYa::OutpostConfirmSetting`; parse the exact JSON/YAML keys above and accept the matching ROS override spellings `Task.OutpostConfirm.<Key>` and `Task/OutpostConfirm/<Key>`. Clamp both exit HP fields to `0..400`; a negative override restores its documented default. Keep every existing VisualScout key unchanged.

- [ ] **Step 2: Refresh lock after `SetAimTarget()`**

At the end of `SelectAimTargetNode::tick()` call `app_->RefreshOutpostEngagementLock()`. Construct the pure input from target id, fresh external target cache, `enemyOutpostHealth`/timestamp, own HP/timestamp, `EvaluateBaseGoalReach()` status, `postureManager_.Runtime()`, and fresh `postureRefereeTimer_.EnhancedRemainingSec[Attack]`. Call `postureManager_.CancelPending()` before the next posture tick when the decision sets `CancelPending`.

- [ ] **Step 3: Preserve 7 only while the lock owns it**

In `SetAimMode()`, prevent the ordinary armor interrupt and damage-abort branch from replacing a held target. Keep HP zero/stale, unreachable, RuntimeGuard, and the lock's 200/250 exits higher priority. In `SetAimTarget()`, use the existing `set_outpost_target()` path while `HoldTarget`; otherwise leave ordinary selection untouched. In `UpdatePostureCommand()`, submit the lock intent using `PostureRequestPolicy::OutpostLock()`; otherwise use existing posture selection.

- [ ] **Step 4: Permit existing posture publisher to send 1..6**

Change only the range guard in `PubPostureControlData()` from `1..3` to `1..6`; preserve `FIELD_POSTURE` and `raw = postureCommand << 21`.

- [ ] **Step 5: Build and test integration**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon build --packages-select behavior_tree --event-handlers console_direct+'
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test --packages-select behavior_tree --event-handlers console_direct+ --ctest-args --output-on-failure'
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test-result --verbose'
```

- [ ] **Step 6: Commit**

```bash
git add src/behavior_tree/module/BasicTypes.hpp src/behavior_tree/src/Configuration.cpp src/behavior_tree/config/Task.yaml src/behavior_tree/config/OutpostRegionalTest.yaml src/behavior_tree/Scripts/ConfigJson/regional_competition.json src/behavior_tree/include/Application.hpp src/behavior_tree/include/BTNodes.hpp src/behavior_tree/src/GameLoop.cpp src/behavior_tree/src/PostureLogic.cpp src/behavior_tree/src/PublishMessage.cpp
git commit -m "behavior_tree: lock outpost engagement through enhanced attack"
```

### Task 4: DecisionTrace 與 simulator

**Files:**
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`
- Modify: `src/simulator/simulator/model.py`
- Modify: `src/simulator/simulator/trace.py`
- Modify: `src/simulator/simulator/validation.py`
- Modify: `src/simulator/simulator/viewer.py`
- Modify: `src/simulator/simulator/foxglove_export.py`
- Modify: `src/simulator/sample/scenarios/outpost_attack.jsonl`
- Modify: `src/simulator/sample/scenarios/manifest.json`
- Modify: `src/simulator/test/test_trace_contract.py`
- Modify: `src/simulator/test/test_validation.py`
- Modify: `src/simulator/config/default.yaml`

- [ ] **Step 1: Add a failing trace-contract fixture**

Add optional schema-v2 JSON:

```json
"outpost_engagement_lock": {
  "active": true,
  "hold_target": true,
  "enhanced_armed": true,
  "enhanced_pending": true,
  "enhanced_active": false,
  "enhanced_unavailable": false,
  "exit_reason": "none",
  "normal_exit_hp": 200,
  "enhanced_exit_hp": 250
}
```

Add this exact test beside `stable_trace_row()` in `test_trace_contract.py`:

```python
def test_trace_normalizes_optional_outpost_engagement_lock() -> None:
    raw = stable_trace_row()
    raw["outpost_engagement_lock"] = {
        "active": True,
        "hold_target": True,
        "enhanced_armed": True,
        "enhanced_pending": True,
        "enhanced_active": False,
        "enhanced_unavailable": False,
        "exit_reason": "none",
        "normal_exit_hp": 200,
        "enhanced_exit_hp": 250,
    }
    record = normalize_record(raw, 0, {18: "OccupyArea"})
    assert record.outpost_engagement_lock.active is True
    assert record.outpost_engagement_lock.enhanced_pending is True
    assert record.outpost_engagement_lock.enhanced_exit_hp == 250

    legacy = stable_trace_row()
    normalized_legacy = normalize_record(legacy, 0, {18: "OccupyArea"})
    assert normalized_legacy.outpost_engagement_lock.active is False
    assert normalized_legacy.outpost_engagement_lock.exit_reason == "none"
```

- [ ] **Step 2: Implement trace, view, export, and validation**

Write the object from `DecisionTrace.cpp`; add an immutable simulator model, normalizer, viewer Runtime rows, and Foxglove object. Add validation warnings for active lock with invalid thresholds or enhanced active without target hold. Keep `schema_version=2` because the new object is optional and backward compatible.

- [ ] **Step 3: Add a complete outpost scenario and run offline acceptance**

Cover normal lock, HP-drop arm, command 4 pending, strong ACK, 251 HP hold, 250 HP release, and stale-HP release. Update manifest counts. Run:

```bash
python3 -m pytest src/simulator/test/test_trace_contract.py src/simulator/test/test_validation.py src/simulator/test/test_scenarios.py -q
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/outpost_attack.jsonl --validate-only
```

- [ ] **Step 4: Commit**

```bash
git add src/behavior_tree/src/DecisionTrace.cpp src/simulator/simulator/model.py src/simulator/simulator/trace.py src/simulator/simulator/validation.py src/simulator/simulator/viewer.py src/simulator/simulator/foxglove_export.py src/simulator/sample/scenarios/outpost_attack.jsonl src/simulator/sample/scenarios/manifest.json src/simulator/test/test_trace_contract.py src/simulator/test/test_validation.py src/simulator/config/default.yaml
git commit -m "simulator: trace outpost enhanced engagement lock"
```

### Task 5: 文件、Obsidian 與圖譜更新

**Files:**
- Modify: `docs/sentry/regional/current_behavior.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `docs/sentry/internal/simulator.md`
- Modify: `docs/obsidian/notes/2026-07-16-rmuc-v2-enhanced-postures.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`
- Modify: `.understand-anything/intermediate/scan-result.json` only if inventory changes

- [ ] **Step 1: Record implemented behavior, not proposal**

In the four Regional/simulator documents, state the exact condition `selected_target=7 && fresh_target && fresh_enemy_op_hp>0`, then record the one-per-lock HP decrease arm, `posture=1 && enhanced_posture=true` ACK for command `4`, normal/enhanced exits `<=200`/`<=250`, retry preservation, and the unchanged `/ly/*` topic plus serial contract. Update the Regional flow to `SelectAimTarget -> RefreshOutpostEngagementLock -> UpdatePostureCommand -> PublishAll`; each flow node links to `GameLoop.cpp`, `PostureLogic.cpp`, `PostureManager.cpp`, or `Task.yaml` as appropriate.

- [ ] **Step 2: Update the fallback graph**

Add source-backed nodes for `OutpostEngagementLock` and the composite `PostureManager` mode, then create edges `/ly/enemy/op_hp -> OutpostEngagementLock`, `/ly/game/sentry/info -> PostureManager`, `/ly/gimbal/posture -> PostureManager`, `OutpostEngagementLock -> PostureManager`, and `PostureManager -> /ly/control/posture`. Refresh `Generated`, checked `HEAD`, worktree note, node/edge counts, and the graph dashboard Regional flow membership in all three canonical graph artifacts.

- [ ] **Step 3: Validate documentation artifacts**

```bash
python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null
python3 -m json.tool .understand-anything/meta.json >/dev/null
python3 -m json.tool .understand-anything/intermediate/scan-result.json >/dev/null
python3 scripts/obsidian_sync.py --check
git diff --check
```

- [ ] **Step 4: Commit**

```bash
git add docs/sentry/regional/current_behavior.md docs/sentry/regional/decision_framework.md docs/sentry/regional/2026-07-12_regional_decision_graph.md docs/sentry/internal/simulator.md docs/obsidian/notes/2026-07-16-rmuc-v2-enhanced-postures.md .understand-anything/knowledge-graph.json .understand-anything/project-knowledge-graph.md .understand-anything/meta.json .understand-anything/intermediate/scan-result.json
git commit -m "docs: record enhanced outpost engagement policy"
```

### Task 6: 全量驗收與審查

**Files:**
- Modify only task-local defects discovered by this verification.

- [ ] **Step 1: Run package and static gates**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon build --packages-select behavior_tree simulator --event-handlers console_direct+'
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test --packages-select behavior_tree simulator --event-handlers console_direct+ --ctest-args --output-on-failure'
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test-result --verbose'
./scripts/selfcheck.sh sentry --static-only
python3 scripts/obsidian_sync.py --check
git diff --check
```

Expected: build succeeds, tests have 0 errors/failures, static selfcheck passes, Obsidian has no drift, and diff has no whitespace errors.

- [ ] **Step 2: Fresh-context review and scope check**

Inspect the final diff against the spec acceptance criteria, then run the targeted commands below. Each command must pass before staging a correction; re-run the first two after any correction:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/sentry.common/install/setup.bash && colcon test --packages-select behavior_tree --ctest-args -R "test_posture_manager|test_outpost_engagement_lock|test_navi_rotate_posture" --output-on-failure'
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/outpost_attack.jsonl --validate-only
git diff --check
git status --short
```

The review rejects: a lock created from stale/zero enemy HP; command `4` ACKed without both fresh `posture=1` and `enhanced_posture=true`; emitted posture values outside `1..6`; a pending `4` surviving a safety exit; changed non-outpost posture behavior; or a trace field not parsed by the simulator. Confirm the final status does not stage the pre-existing `Zone.Identifier` deletions or document lock file.

- [ ] **Step 3: Push only after explicit request**

```bash
git log --oneline --max-count=6
git status --short
```

Do not push from this task unless the user explicitly requests it after reviewing the completed implementation.
