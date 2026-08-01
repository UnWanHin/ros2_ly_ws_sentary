# Tactical CommonCentral Arrival Design

## Status

Accepted for implementation

## Date

2026-08-02

## Context

Tactical `RegionalDefenseSearchKind::CommonCentral` is an event-driven search
overlay with four candidate goals: `HoleRoad (17)`, `CentralHigh (29)`,
`CentralLow (30)`, and `OutpostGuard (24)`. It is
distinct from Default `RegionalAreaTask.CommonCentral`, which owns the normal
eight-point central patrol route.

The tactical branch previously shared `RegionalDefense.SearchHoldSec` and could
advance after a short elapsed interval even while the robot was still traveling.
That does not match the workspace's composite navigation arrival contract.

## Decision

Add an independent YAML gate at `Tactical.RegionalDefense.CommonCentral.Enable`.
The default remains `true` to preserve current behavior. The gate only disables
the Tactical CommonCentral threat branch; it does not disable Default
`AreaManager.yaml` central patrol or other RegionalDefense branches.

Tactical CommonCentral will use the existing navigation abstractions and the
team-relative coordinates in `Area.hpp`:

- Red `CentralHigh=(1000,1007)`, `CentralLow=(989,496)`;
- Blue `CentralHigh=(1800,493)`, `CentralLow=(1811,1004)`;
- the forward order is `17 -> 29 -> 30 -> 24`, then it reverses back toward
  `17`; legacy `CentralLeft.A/B (26/27)` remain unchanged.

- `EvaluateBaseGoalReach()` remains the only arrival decision. It combines
  `/ly/navi/reached`, `/ly/navi/reachable`, position distance, and the existing
  near-goal confirmation window.
- Existing `NaviProgressWatchdog` values (`MoveProgressCm=80`,
  `NoMoveTimeoutSec=14`) are reused for a traveling goal that has not reached.
  No new independent distance or timeout semantics are introduced.
- A reached goal advances only after the `Tactical.RegionalDefense.CommonCentral.HoldSec`
  hold/no-target contract (default 15 seconds); a traveling goal does not advance merely because the old short
  search timer elapsed.
- Unreachable goals still advance immediately through the existing fallback
  path, with the same decision logging and unique navigation output.

No ROS topic, message, posture, target, or task-priority contract changes.

## Alternatives Rejected

### Change global `RegionalDefense.SearchHoldSec`

Rejected because it changes ProtectCastle, Highland, road-corridor, and soft
enemy-side searches together.

### Add a second CommonCentral-specific arrival implementation

Rejected because it would duplicate the existing reached/reachable/position
and watchdog semantics and could diverge again.

## Verification

Add regression coverage for:

1. the YAML gate disabling only Tactical CommonCentral;
2. a CommonCentral traveling goal remaining on the current point before the
   shared watchdog reports no progress;
3. a reached CommonCentral goal advancing only after the existing hold;
4. an external unreachable result advancing immediately;
5. no changes to the existing ProtectCastle and other RegionalDefense paths.

Run the targeted behavior-tree tests, the full behavior-tree test suite, the
workspace static self-check, and `git diff --check`.
