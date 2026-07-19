# Regional Chase Policy Design

## Status

Accepted: 2026-07-18

## Context

Regional strategy currently treats `Chase` as a Tactical overlay.  The existing
`Chase.AreaLimit` protects the output geometry: with cross-area chase disabled,
the chase goal remains within the robot's current physical main area.  The
existing `IsAreaKeyAllowedForChaseTarget()` check is different: it only asks
whether the target area is enabled somewhere in `DecisionAutonomy.NaviGoal`.
It does not require the enemy to be in the same area as the regional task that
the sentry has already committed to.

That difference permits an enabled target area to take navigation ownership
away from an unrelated Default area task.  The selection and output mechanics
also live together in `GameLoop.cpp`, making it difficult to audit which
conditions authorize a chase.

The desired Regional behavior is opportunity-based: chase only while a
yieldable Default regional task owns the plan, and only when the selected
enemy is inside that exact planned area and that area is explicitly enabled
for chasing.  A target in another enabled area must not redirect the sentry.

## Decision

Introduce a behavior-tree-local `ChasePolicy` module.  It owns strategic chase
authorization only; it neither selects aim targets nor publishes navigation
messages.

For Regional mode, a Chase is authorized only when all of these are true:

1. Existing global and mode gates permit Chase (`Chase.Enable`,
   `FollowAimTarget`, supported aim mode, no outpost/special suppression).
2. No higher owner is active: Recovery, an unyieldable ReadyRoadland crossing,
   MapCommand, Highland transition, Buff/Outpost navigation, RegionalDefense,
   or ProtectHero. Chase is a Tactical decision and therefore runs before the
   later Special layer. If Special Patrol is enabled in the future,
   `Special.Patrol.SuppressChase=true` is its explicit opt-out.
3. A yieldable Default `RegionalAreaTask` is active.  Its task type resolves
   to one planned `AreaKey`: `MyBase`, `MyHighland`, `MyPreRoadland`,
   `MyReadyRoadland`, or `CommonCentral`.
4. The selected enemy has a fresh official-map position.  The position may
   come from the established `/ly/position/data` source or the existing
   `/ly/navi/target_official` fallback, but it must resolve by exact polygon
   containment.  Nearest-area fallback is not sufficient for authorization.
5. The enemy `AreaKey` exactly equals the planned task `AreaKey`, including
   side, main-area kind, and team where applicable.  The matching planned
   area is enabled in `Chase.yaml`.

The policy returns an allow/deny result and a stable reason.  `Application`
continues to own the existing target freshness, relative-target construction,
and `/ly/navi/target_rel` or `/ly/navi/goal_pos_raw` publication.  The current
area-limit geometry remains in place as a second, independent safety boundary;
the policy does not weaken `Chase.AreaLimit` or `ChaseEnableCrossArea`.

When the policy denies a new tick, the Tactical chase output is cleared by the
existing per-tick reset and the still-active Default regional task resumes its
same goal/phase.  It does not re-run Default scoring or select a new point.
The existing `LostTargetHoldMs` remains the short target-observation grace;
it must never authorize an unlocated or cross-area target after the official
position is stale.

## Configuration Contract

Create `src/behavior_tree/config/Chase.yaml` and load it through a new
`chase_config_file` launch argument.  The file is deliberately limited to
regional strategic permissions; distance, velocity, aim-mode, target-freshness
and output geometry remain in the existing profile JSON `Chase` block.

```yaml
behavior_tree:
  ros__parameters:
    ChasePolicy:
      Enable: true
      MyBase: false
      MyHighland: true
      MyPreRoadland: true
      MyReadyRoadland: false
      CommonCentral: true
```

Unknown/missing keys use the safe value `false`.  The policy has no `Enemy*`
area keys: a chase follows the current planned Default area only, so an enemy
side area can never become a chase destination merely by configuration.

`ChasePolicy.Enable=false` disables the new Regional policy and therefore
disables Regional chase navigation.  League, showcase, and explicit debug
profiles preserve their established Chase behavior; this change does not add
Regional ownership requirements to those profiles.

## Ownership Flow

```text
Hard / MapCommand / protected Task
  -> owns navigation; ChasePolicy denies

yieldable Default RegionalAreaTask
  -> resolve planned AreaKey
  -> fresh selected-enemy official position
  -> exact same AreaKey and Chase.yaml=true
  -> Chase Tactical overlay publishes chase input

target lost, stale, outside planned area, disabled area, or higher owner
  -> clear chase output
  -> retained RegionalAreaTask resumes its existing goal/phase
```

## Non-Goals

- No ROS topic, message schema, serial protocol, or `GoalReachState` contract
  changes.
- No new area scoring, patrol point weighting, or cross-area pursuit.
- No duplicate pursuit controller: `navi_tf_bridge` remains the owner of
  relative-target conversion and the established geometry limiter.
- No changes to aim selection or fire permission.  A visible target may still
  be aimed/fired upon while navigation Chase is denied.

## Validation

- Add focused unit tests for each planned-area mapping, disabled area, exact
  same-area permit, side mismatch, kind mismatch, absent/stale target position,
  and nearest-area fallback rejection.
- Add strategy integration coverage proving a valid same-area target produces
  chase authorization, while a target in a different enabled Default area
  leaves the original regional task/goal intact.
- Build and run the `behavior_tree` CTest suite, then run
  `./scripts/selfcheck.sh sentry --static-only` and `git diff --check`.
- Update current Regional and behavior-tree documentation plus the
  Understand Anything fallback graph, including the Regional flow ownership
  and `Chase.yaml` configuration edge.  Validate graph JSON and dashboard
  routes after the runtime change.

## Rejected Alternatives

- **Any enabled area may chase:** simple but lets a target in an unrelated
  enabled zone replace an already committed plan.
- **Patrol-point radius:** prevents some deviations but makes the effective
  engagement region too small and treats a patrol waypoint as a tactical
  boundary.
- **Only `Chase.AreaLimit`:** it limits geometry relative to the physical
  robot location, not the selected regional plan, so it cannot express the
  required strategic ownership rule.
