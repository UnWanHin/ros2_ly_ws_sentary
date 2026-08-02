# Posture Transit Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep Attack or Defense during navigation transit when they are valid posture candidates, tolerate a short target-loss window, and give enhanced posture requests enough internal ACK time without changing ROS or lower-machine interfaces.

**Architecture:** Preserve the single `PostureManager` command path. `SoftTransit` will describe navigation ownership only; its scored Attack/Defense candidate is accepted directly, while scored Move still uses the existing dynamic Move reserve selector. `OutpostEngagementLock` will reuse `Posture.TargetKeepMs` (800 ms in the active Regional JSON) as an internal target-loss grace and will stop clearing pending posture requests for target-loss alone. `PostureManager` will use enhanced-only retry timing for command values 4/5/6 and HardMove will preserve its current safe posture when retries are exhausted.

**Tech Stack:** C++17, ROS2/colcon, nlohmann JSON configuration, GoogleTest.

## Global Constraints

- Do not change ROS topics, message types, navigation outputs, gimbal-driver protocol, or lower-machine behavior.
- Keep Recovery and `navi_should_rotate=false` HardMove higher priority than SoftTransit.
- Keep `follow=true` sufficient for Attack posture; `fire=true` is not required.
- Keep `TargetKeepMs=800` as the single target-loss grace authority.
- Do not alter unrelated task, tactical, simulator, or map logic.

### Task 1: SoftTransit posture contract

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Test: `src/behavior_tree/test/test_posture_manager.cpp`

**Interfaces:**
- `ResolveTaskPostureRequest(TaskPostureIntent::SoftTransit, ...)` accepts scored Attack and Defense directly; scored Move continues through `SelectTransitPosture`.
- `SelectDesiredPosture` no longer returns Move solely because an Outpost task owns a transit goal.

- [x] Add a regression assertion that SoftTransit accepts Attack without a raw-target boolean gate, while retaining the existing Move-reserve assertion.
- [x] Remove only the Outpost-transit hard Move return; leave low-health, arrived, damage-burst, and Recovery rules unchanged.
- [x] Make HardMove policy preserve the current posture on retry exhaustion; add a regression assertion that it cannot select Attack as a fallback.
- [x] Build and run the focused posture test binary before moving to the next slice.

### Task 2: Target-loss grace and pending preservation

**Files:**
- Modify: `src/behavior_tree/include/OutpostEngagementLock.hpp`
- Modify: `src/behavior_tree/src/OutpostEngagementLock.cpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Test: `src/behavior_tree/test/test_outpost_engagement_lock.cpp`

**Interfaces:**
- `OutpostEngagementSetting` receives an internal `TargetLostGraceMs` value populated from `Posture.TargetKeepMs`.
- Target loss within grace keeps the active lock and does not cancel pending posture; target loss after grace releases the lock without cancelling generic pending posture.
- Definitive exits (stale/zero enemy HP, unreachable navigation, health thresholds) retain their existing pending cancellation behavior.

- [x] Add tests for target loss inside and outside the 800 ms grace window.
- [x] Implement a monotonic grace timer that resets on a fresh selected Target 7.
- [x] Route only definitive exits through `CancelPending`; TargetLost returns `CancelPending=false`.
- [x] Build and run the focused lock test binary.

### Task 3: Enhanced posture ACK window

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/src/PostureManager.cpp`
- Modify: `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- Modify: `src/behavior_tree/Scripts/config.json`
- Test: `src/behavior_tree/test/test_posture_manager.cpp`

**Interfaces:**
- New internal config fields: `EnhancedPendingAckTimeoutMs=800`, `EnhancedRetryIntervalMs=300`, `EnhancedMaxRetryCount=5`.
- Normal posture retry settings remain unchanged.

- [x] Add a test showing an enhanced request remains pending at 1.2 seconds and can be confirmed at approximately 1.4 seconds.
- [x] Parse, log, and validate the enhanced settings with positive safe fallbacks.
- [x] Select enhanced timing only while `runtime_.Pending.Enhanced` is true.
- [x] Build, run both focused test binaries, and inspect that no ROS/topic files changed.

### Task 4: Documentation and final verification

**Files:**
- Modify: `docs/record/2026-08-02_posture_bag_analysis.md`
- Modify: `docs/sentry/regional/current_behavior.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/modules/2026-05-05_behavior_tree.md` (only if the current posture config description is stale)

- [x] Document the 800 ms grace, SoftTransit semantics, HardMove fallback, and enhanced ACK window.
- [x] Run `colcon build --packages-select behavior_tree`.
- [x] Run `build/behavior_tree/test_posture_manager` and `build/behavior_tree/test_outpost_engagement_lock`.
- [x] Run `./scripts/selfcheck.sh sentry --skip-hz` and `git diff --check` (blocked only by the unsourced external ROS environment and the pre-existing Aim.yaml override).
- [x] Review the final diff for unchanged ROS, navigation, gimbal, and lower-machine interfaces.
