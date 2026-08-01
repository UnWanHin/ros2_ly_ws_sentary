# Task/Tactical Ownership and Enhanced Posture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use inline execution with the task sequence below and verify each checkpoint before continuing.

**Goal:** Make Recovery and Task navigation ownership authoritative over Tactical decisions, make all three enhanced postures reliable through the single posture publisher, document CommonCentral/ProtectOutpost ownership, and ensure Castle damage can produce the configured rotate gear when navigation does not explicitly suppress it.

**Architecture:** Keep the existing ROS topics and configuration contracts. Reorder the strategy evaluation so Recovery, MapCommand, and explicit competition Tasks are resolved before Tactical; add owner-aware watchdog fallback instead of changing arbitrary goals. Extend the existing `PostureManager` runtime with explicit enhanced-task priority and retry semantics so normal pending transitions cannot permanently consume an enhanced request. Keep all posture output on the existing `/ly/control/posture` publisher and keep rotate output on the existing firecode path.

**Tech Stack:** ROS2 C++, BehaviorTree.CPP, GoogleTest, existing decision trace and simulator replay tools.

## Global Constraints

- Recovery remains the only decision allowed to preempt an active Task.
- MapCommand remains the highest non-Recovery navigation command and keeps its existing 0,0, field-bound, 20cm deduplication, and 45s hold semantics.
- Tactical ProtectCastle, ProtectOutpost, ProtectHero, and Chase must never clear or replace an active Task owner.
- No ROS topic, message type, public API, serial frame, or lower-machine code is renamed or rerouted.
- Preserve unrelated user changes and do not commit generated build/install/log artifacts.
- Enhanced posture requests require fresh referee Info3 and a positive corresponding enhanced budget; zero budget, stale feedback, contradiction quarantine, death, and stale target/health always fall back safely.
- A posture request that is blocked by an older lower-priority pending request remains retryable; it is not marked unavailable merely because the first tick could not enqueue it.
- `common_central` in the Default RegionalAreaTask config is distinct from Tactical `RegionalDefenseSearchKind::CommonCentral`; documentation must name both explicitly.
- ProtectOutpost uses the team-specific official C3/C4 ProtectOutpost point and its 30-second SearchHold; no alternate BuffOutpost point is substituted.

## Task 1: Lock the strategy ownership contract

**Files:**
- Modify: `src/behavior_tree/src/StrategyManager.cpp`
- Modify: `src/behavior_tree/include/StrategyManager.hpp`
- Modify: `src/behavior_tree/include/BTNodes.hpp`
- Test: `src/behavior_tree/test/test_strategy_ownership.cpp`
- Update: `docs/sentry/regional/current_behavior.md`

**Interfaces:**
- Existing `Application::RunStrategyLayer*()` methods remain the public BT node interface.
- Add an internal `StrategyLayerOwner`/handled state query used only by the strategy manager and tests.

- [ ] Add failing tests for: active Outpost Task blocks camera-triggered ProtectCastle; active MapCommand blocks all Tactical; no Task still permits ProtectCastle; non-Recovery watchdog cannot replace an active Task goal.
- [ ] Run the targeted test and confirm it fails against the current ordering.
- [ ] Make the BT strategy sequence evaluate `Hard -> Task -> Tactical -> Special -> Default -> Finalizer` while preserving hard Recovery behavior.
- [ ] Move opening Outpost/Buff task entry decisions out of Tactical-only gating into `RunTask`; make `RunTactical` handle only the four tactical candidates.
- [ ] Make `TickRegionalAreaTask` and ReadyRoadland hard-lock paths yield whenever a non-Recovery Task owns navigation.
- [ ] Record the winning owner in the existing decision intent/blackboard diagnostics without changing topic schemas.
- [ ] Run the new strategy ownership tests and the existing pre-ready/regional task tests.

## Task 2: Make watchdog fallback owner-aware

**Files:**
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/AreaManager.cpp`
- Modify: `src/behavior_tree/include/AreaManager.hpp`
- Test: `src/behavior_tree/test/test_strategy_ownership.cpp`
- Update: `docs/sentry/regional/current_behavior.md`

**Interfaces:**
- Extend the internal progress-watchdog input with the active navigation owner and permitted fallback scope.
- Keep existing `NaviProgressWatchdogSettings` YAML keys and fallback log format compatible.

- [ ] Add a failing test showing an Outpost-owned watchdog cannot return Castle `goal=6`.
- [ ] Pass the current owner into the watchdog and constrain fallback candidates to the owner’s task scope.
- [ ] Return false or reissue the current task goal when no scoped fallback is valid; never silently select a Tactical Castle point.
- [ ] Preserve the existing generic fallback behavior only when no Task owner is active.
- [ ] Run watchdog, arrival, map-command, and regional task tests.

## Task 3: Supersede stale posture pending commands by priority

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/include/PostureManager.hpp`
- Modify: `src/behavior_tree/src/PostureManager.cpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Test: `src/behavior_tree/test/test_posture_manager.cpp`
- Update: `docs/record/2026-03-04_sentry_posture_system.md`

**Interfaces:**
- Extend the internal `PostureRequestPolicy` with a request priority and source label; keep all ROS messages unchanged.
- `PostureManager::Tick()` remains the sole posture decision function.

- [ ] Add a failing test for pending Attack followed by higher-priority HardMove; assert the next command is Move and the reason identifies supersession.
- [ ] Add tests proving equal/lower priority does not churn pending state, and that enhanced posture guards still reject exhausted budgets.
- [ ] When a higher-priority desired mode differs from pending, clear the stale pending/retry state and enqueue the new desired mode once.
- [ ] Mark `navi_should_rotate_false`/HardMove as the movement safety priority; preserve switch cooldown and acknowledgment behavior after supersession.
- [ ] Keep `/ly/control/posture` as the only BT posture publisher and retain `/ly/control/sentry_cmd` energy-only behavior.
- [ ] Run all posture and regional-task posture tests.

## Task 4: Make enhanced Attack an accepted-and-retryable outpost request

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/include/OutpostEngagementLock.hpp`
- Modify: `src/behavior_tree/src/OutpostEngagementLock.cpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Test: `src/behavior_tree/test/test_outpost_engagement_lock.cpp`
- Update: `docs/sentry/regional/current_behavior.md`

**Interfaces:**
- Keep `OutpostEngagementLock` and `/ly/control/posture` signatures unchanged.
- Add only internal state/priority metadata; do not expose new ROS messages.

- [ ] Add a RED test where HP `1500 -> 1460`, target 7 is selected, enhanced Attack has 15 seconds, and a lower-priority pending Move exists; assert the lock remains armed and does not mark unavailable.
- [ ] Add a RED test where the lower-priority pending request clears and the next tick emits command `4` exactly once.
- [ ] Add tests for target loss, HP zero, stale HP, exhausted budget, and repeated HP drops after the accepted request.
- [ ] Change `enhanced_attempted_` to mean “enhanced command was accepted/emitted”, not “HP drop was observed”.
- [ ] Give the outpost enhanced request a task-owned priority that supersedes ordinary transit pending state but never Recovery hard ownership.
- [ ] Mark the lock unavailable only after a genuine accepted-request failure/expiry, not while it is waiting behind a lower-priority request.
- [ ] Run the focused outpost and posture tests.

## Task 5: Make enhanced Move and enhanced Defense first-class task requests

**Files:**
- Modify: `src/behavior_tree/include/PostureTypes.hpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Modify: `src/behavior_tree/src/PostureManager.cpp`
- Test: `src/behavior_tree/test/test_regional_task_posture.cpp`
- Test: `src/behavior_tree/test/test_posture_manager.cpp`
- Test: `src/behavior_tree/test/test_tactical_protection_policy.cpp`
- Update: `docs/record/2026-03-04_sentry_posture_system.md`

**Interfaces:**
- Preserve ordinary posture values (1 Attack, 2 Defense, 3 Move) and enhanced command encoding (4/5/6).
- Keep Recovery HP==0 excluded; low positive HP during Recovery travel may request enhanced Move.

- [ ] Add RED tests for low-health Recovery travel with fresh enhanced Move budget and a pending normal Move, and for ProtectHero damage burst with fresh enhanced Defense budget and a pending normal Defense/Move.
- [ ] Add RED tests proving HP 0, stale Info3, zero budget, contradiction quarantine, and no damage burst all fall back to ordinary posture without repeated enhanced commands.
- [ ] Implement a shared task-owned enhanced priority/pending replacement path in `PostureManager::Tick()`.
- [ ] Keep enhanced requests retryable through the switch cooldown and acknowledgement path; do not set `enhancedRecoveryMoveUnavailable_` or `protectHeroEnhancedDefenseUnavailable_` on a lower-priority pending wait.
- [ ] Preserve external `/ly/navi/should_rotate`, `/ly/control/trajectory`, and speed-level behavior; this slice changes only the BT posture command decision.
- [ ] Run all posture, recovery, and tactical protection tests.

## Task 6: Verify/document CommonCentral and ProtectOutpost ownership

**Files:**
- Test: `src/behavior_tree/test/test_regional_task_posture.cpp` or a focused AreaManager test target
- Update: `docs/sentry/regional/decision_framework.md`
- Update: `docs/sentry/regional/current_behavior.md`
- Update: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`

- [ ] Add a test asserting ProtectOutpost resolves Red to C3 `(1011,429)` cm and Blue to C4 `(1789,1071)` cm, with SearchHold remaining 30 seconds.
- [ ] Add a test/documented trace showing Default `RegionalAreaTask.CommonCentral` is gated by `AreaManager.yaml`, while Tactical `RegionalDefenseSearchKind::CommonCentral` is selected from threat classification and uses HoleRoad/Castle candidates.
- [ ] Document why the Tactical branch can visit goal 17 (HoleRoad) and goal 6 (Castle), and distinguish it from the Default CommonCentral patrol sequence.
- [ ] Update dated documentation lines to `Updated: 2026-08-01`.

## Task 7: Make Castle damage rotate behavior observable and correct

**Files:**
- Modify: `src/behavior_tree/include/DamageRotatePolicy.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Test: `src/behavior_tree/test/test_damage_rotate_policy.cpp`
- Update: `docs/sentry/regional/current_behavior.md`

- [ ] Add a RED pure-policy test proving a fresh damage event produces a nonzero configured gear when the Castle goal is active and no external rotate suppression is present.
- [ ] Add a RED test proving fresh `/ly/navi/should_rotate=false` and FollowMode still force gear 0, with the reason distinguishable in logs.
- [ ] Refactor the final rotate resolution into a small testable helper that records the suppression source (`StopRotate`, Highland compatibility, navigation false, FollowMode) without changing the firecode message.
- [ ] Ensure Castle/ProtectCastle does not implicitly suppress damage rotation; only explicit external suppression or configured debug suppression may do so.
- [ ] Add a structured log field for `damage_detected`, `damage_gear`, `final_gear`, and `suppressed_by` so the next replay can prove the path.
- [ ] Run focused rotate tests and a local/replayed Castle damage scenario.

## Task 8: Integration verification and documentation freshness

**Files:**
- Update: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Update: `docs/sentry/internal/simulator.md`
- Update: `docs/sentry/regional/current_behavior.md`

- [ ] Build `behavior_tree` and run its registered tests.
- [ ] Run `./scripts/selfcheck.sh sentry --static-only`; record any environment-only missing overlay separately.
- [ ] Replay the remote 2026-08-01 outpost/Castle trace and verify the enhanced request and rotate suppression reasons are visible.
- [ ] Run `git diff --check` and verify no generated files or user lock file are modified.
- [ ] Confirm all changed runtime behavior is reflected in current Markdown documentation so the documentation graph remains authoritative.
