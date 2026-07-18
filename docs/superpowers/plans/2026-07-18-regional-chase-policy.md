# Regional Chase Policy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Permit Regional navigation Chase only for a fresh enemy located in the exact Default area currently planned by the sentry, with a simple per-area `Chase.yaml` switch.

**Architecture:** Add a pure `ChasePolicy` module which maps an active yieldable `RegionalAreaTask` to its planned `AreaKey`, compares it with an exact (not nearest-fallback) fresh enemy `AreaKey`, and returns a stable authorization reason. `Application::TryApplyChaseTactical()` keeps target/output construction but uses this policy before it creates any chase output. The existing `Chase.AreaLimit` remains the independent geometry clamp; a denied policy tick naturally exposes the retained Default task's prior goal.

**Tech Stack:** ROS 2 Humble, C++17, GoogleTest, ROS parameter YAML, JSON behavior profile, launch Python, DecisionTrace JSONL.

## Global Constraints

- Preserve all existing ROS topics, messages, serial data, and `GoalReachState` semantics.
- `Chase.yaml` controls only Regional strategic area permissions; existing JSON `Chase` settings retain distance, source, motion, and output ownership.
- Exact containment is required for a Regional chase permit. Never use nearest-area fallback to make an enemy chaseable.
- Keep Recovery, MapCommand, unyieldable ReadyRoadland, Highland transition, Buff/Outpost, RegionalDefense, ProtectHero, and Special Patrol above Chase.
- Keep League, showcase, and explicit debug profiles on their current Chase behavior.
- Update source-backed docs, simulator trace only if trace fields change, and Understand Anything graph artifacts in the same runtime change.

---

### Task 1: Define and prove the pure regional authorization contract

**Files:**
- Create: `src/behavior_tree/include/ChasePolicy.hpp`
- Create: `src/behavior_tree/src/ChasePolicy.cpp`
- Create: `src/behavior_tree/test/test_chase_policy.cpp`
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- Consumes: `ChasePolicySetting`, an optional planned `AreaKey`, an optional exact target `ResolvedAreaKey`, Regional-mode and yieldable-owner flags.
- Produces: `ChasePolicyResult { bool Allowed; ChasePolicyReason Reason; }` from `EvaluateRegionalChasePolicy(...)`.

- [x] **Step 1: Write failing pure-policy tests**

```cpp
EXPECT_TRUE(EvaluateRegionalChasePolicy(setting, same_area_context).Allowed);
EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, other_area_context).Allowed);
EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, nearest_fallback_context).Allowed);
EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, disabled_area_context).Allowed);
```

Cover every configured area mapping, side mismatch, kind mismatch, missing plan,
non-yieldable plan, missing/stale target position, `ChasePolicy.Enable=false`,
and non-Regional profile bypass.

- [x] **Step 2: Run the focused test before implementation**

Run: `colcon test --packages-select behavior_tree --ctest-args -R test_chase_policy --output-on-failure`

Expected: test target is absent or fails because the policy interface does not exist.

- [x] **Step 3: Add the minimal policy data and implementation**

Define `ChasePolicySetting` with only `Enable`, `MyBase`, `MyHighland`,
`MyPreRoadland`, `MyReadyRoadland`, and `CommonCentral`. Define one helper that
maps an active task to its planned `AreaKey`, and one evaluator that requires
an exact, fresh, matching target key and enabled planned area. Return enum
reasons instead of formatting strings in the policy.

- [x] **Step 4: Register and run focused tests**

Add `test_chase_policy` to `CMakeLists.txt`, build the package, and run the
command from Step 2. Expected: all policy cases pass.

- [x] **Step 5: Commit the isolated policy unit**

```bash
git add src/behavior_tree/include/ChasePolicy.hpp src/behavior_tree/src/ChasePolicy.cpp \
  src/behavior_tree/test/test_chase_policy.cpp src/behavior_tree/module/BasicTypes.hpp \
  src/behavior_tree/CMakeLists.txt
git commit -m "behavior_tree: add regional chase policy"
```

### Task 2: Load the minimal `Chase.yaml` profile

**Files:**
- Create: `src/behavior_tree/config/Chase.yaml`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Modify: applicable test launch files that enumerate normal BT parameter files

**Interfaces:**
- Consumes: ROS parameter paths `ChasePolicy.Enable`, `ChasePolicy.MyBase`,
  `ChasePolicy.MyHighland`, `ChasePolicy.MyPreRoadland`, `ChasePolicy.MyReadyRoadland`, and `ChasePolicy.CommonCentral`.
- Produces: `config.ChasePolicySettings`, defaulting every missing value to `false`.

- [ ] **Step 1: Write a launch/static assertion before adding the profile**

Add a focused launch-source test or existing static assertion proving that
`sentry_all.launch.py` has a `chase_config_file` argument and passes it only
to `behavior_tree`.

- [ ] **Step 2: Add the safe default profile**

Create `Chase.yaml` with the six accepted keys. Use an explicit safe default
for every area and do not duplicate any JSON `Chase` distance/velocity key.

- [ ] **Step 3: Wire parameter loading and launch composition**

Follow the existing `Task.yaml`/`Special.yaml` pattern: add the launch
argument, pass the YAML to the BT node, and read dotted/slash-compatible
parameters in `Configuration.cpp`. Include the effective values in config
logging. Do not pass this file to `gimbal_driver` or `navi_tf_bridge`.

- [ ] **Step 4: Build and check the launch source**

Run: `colcon build --packages-select behavior_tree --symlink-install`

Run: `python3 -m py_compile src/behavior_tree/launch/sentry_all.launch.py`

Expected: build and launch syntax pass; the installed share contains
`config/Chase.yaml`.

- [ ] **Step 5: Commit configuration ownership separately**

```bash
git add src/behavior_tree/config/Chase.yaml src/behavior_tree/src/Configuration.cpp \
  src/behavior_tree/launch/sentry_all.launch.py src/behavior_tree/test
git commit -m "behavior_tree: add chase area profile"
```

### Task 3: Apply the policy at the chase authorization boundary

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/StrategyManager.cpp`
- Modify: `src/behavior_tree/test/test_chase_policy.cpp`

**Interfaces:**
- Consumes: active `AreaManager::RegionalAreaTask`, its yieldability, target
  unit's `GetEnemyPositionState(...)`, and exact `ResolveAreaKeyForPoint(...)`.
- Produces: Chase output only after `ChasePolicyResult.Allowed`; denial retains
  the active Default task and emits no `target_rel`/official chase output.

- [ ] **Step 1: Extend the failing test to model the application boundary**

Add integration-style coverage for a current `MyHighland` plan: target in
MyHighland authorizes Chase; target in `CommonCentral` does not; an external
aim point with stale/no official coordinate does not; a denied result leaves
the prior regional task active.

- [ ] **Step 2: Replace the global-only area-scope test**

In `TryApplyChaseTactical()`, retain the existing macro gates and selected
target validation, then call the policy before constructing relative/official
chase output. Resolve target coordinates with `ResolveAreaKeyForPoint`, not
`ResolveAreaKeyForPointWithNearest`. Remove only the obsolete
`IsAreaKeyAllowedForChaseTarget()` authorization path; retain all geometry
functions used by `Chase.AreaLimit`.

- [ ] **Step 3: Keep temporary tactical ownership non-destructive**

Do not clear, cancel, complete, or re-score the active RegionalAreaTask when
Chase starts or stops. Preserve the existing per-tick output reset so failed
policy authorization cannot leave an old chase target published.

- [ ] **Step 4: Run target and package tests**

Run: `colcon build --packages-select behavior_tree --symlink-install`

Run: `colcon test --packages-select behavior_tree --ctest-args --output-on-failure`

Expected: all behavior-tree tests pass, including `test_chase_policy`.

- [ ] **Step 5: Commit the runtime ownership change**

```bash
git add src/behavior_tree/include/Application.hpp src/behavior_tree/src/GameLoop.cpp \
  src/behavior_tree/src/StrategyManager.cpp src/behavior_tree/test/test_chase_policy.cpp
git commit -m "behavior_tree: require planned area for chase"
```

### Task 4: Record the live contract and verify the workspace

**Files:**
- Modify: `docs/modules/2026-05-05_behavior_tree.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`
- Modify: `.understand-anything/intermediate/scan-result.json` if graph inventory changes

**Interfaces:**
- Consumes: source-verified `ChasePolicy`, `Chase.yaml`, and existing
  `/ly/navi/target_rel` / `/ly/navi/goal_pos_raw` output contract.
- Produces: current docs and graph showing Chase as a Regional Default-area
  tactical overlay, not a cross-area owner.

- [ ] **Step 1: Update current documentation**

Describe the exact planned-area match, strict coordinate requirement, YAML
ownership, preserved geometry limit, and return-to-retained-task behavior.
Do not claim that aim/fire is disabled when navigation Chase is denied.

- [ ] **Step 2: Refresh the fallback graph**

Update graph nodes/edges/tags and the Regional flow block with `ChasePolicy`
and `Chase.yaml`; set metadata HEAD and graph counts to the source-checked
post-change state.

- [ ] **Step 3: Run all required validation**

Run:

```bash
colcon build --packages-select behavior_tree --symlink-install
colcon test --packages-select behavior_tree --ctest-args --output-on-failure
./scripts/selfcheck.sh sentry --static-only
python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null
python3 -m json.tool .understand-anything/meta.json >/dev/null
git diff --check
```

Also serve/check the graph dashboard root, `/graph.json`, `/project.md`, and
`/regional.md`; do not leave the server running.

- [ ] **Step 4: Commit docs and graph separately**

```bash
git add docs/modules/2026-05-05_behavior_tree.md docs/sentry/regional \
  .understand-anything
git commit -m "docs: record regional chase ownership"
```
