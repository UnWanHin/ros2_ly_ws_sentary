# ProtectOutpost Tactical Design

Updated: 2026-07-22

## Goal

Add a Regional tactical defense task for a strictly decreasing, fresh own-outpost
health report. The sentinel travels to the official-map outpost-defense point,
searches there after arrival, and yields to higher-priority work without route
churn.

## Inputs and Targets

- Input: `/ly/friend/op_hp` (`std_msgs/UInt16`), the own-outpost health report.
- A report is valid only while it is fresh. The subscriber records receipt time
  and the tactical state machine ignores missing or stale reports.
- A newly observed strict health decrease starts one defense event. A steady low
  value never starts another event.
- Official-map coordinates are centimeters:
  - Red: `C3 = (1011, 429)`
  - Blue: `C4 = (1789, 1071)`

## YAML Contract

`Tactical.yaml` gains one priority table and one feature block:

```yaml
Tactical:
  Priority:
    ProtectCastle: 1
    ProtectOutpost: 2
    ProtectHero: 3
    Chase: 4
  ProtectOutpost:
    Enable: true
    HealthFreshMs: 2000
    SearchHoldSec: 30
    UnreachableCooldownSec: 10
```

Smaller values are higher priority. This table orders only the four listed
Regional tactical entries. Hard and Task layers retain their current authority;
unlisted/invalid ties use the listed order as a deterministic tie-breaker.

## State Machine

1. `Idle`: waits for a fresh first sample, then a strict health decrease.
2. `Travel`: owns exactly one C3/C4 target for the new damage event.
3. `SearchHold`: begins only when the navigation reach evaluator confirms
   arrival; holds that target for `SearchHoldSec`.
4. `Cooldown`: a navigation unreachable result ends the event and suppresses
   immediate retriggering for `UnreachableCooldownSec`.
5. `Complete`: after the hold, the event is consumed. A subsequent strict
   health decrease is required to start another event.

A new damage report while traveling or holding refreshes the same event and,
when already holding, restarts the hold timer. A higher-priority selection may
preempt this task, but the state remains intact and can resume only while its
same event is still active. It never emits alternating targets each tick and
does not reactivate from an unchanged health value.

## Scheduling

The Regional Tactical layer evaluates the configured four-entry priority order.
`ProtectOutpost` is above `ProtectHero` and Chase by default, while the
existing Castle-defense source remains highest. Existing opening behavior,
Buff/Outpost aiming, Recovery, map command, transition, and watchdog behavior
remain unchanged. A new decision reason and decision logs expose `damage`,
`travel`, `search_hold`, `preempted`, `unreachable`, and `complete`.

## Verification

- Add pure state-machine tests for first sample, decrease, no-repeat on steady
  low health, reach/30-second hold, new damage refresh, stale input,
  unreachable cooldown, and preemption/resume.
- Add configuration parsing tests for the feature and priority values.
- Run focused `behavior_tree` tests/build where ROS dependencies are available,
  static self-check, YAML parsing, and `git diff --check`.
- Update the current Regional decision-flow and behavior-tree documentation;
  the documentation graph then updates automatically.
