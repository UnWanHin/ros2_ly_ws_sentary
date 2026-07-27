# Regional Posture Phase Coordination Design

Updated: 2026-07-27

## Goal

Coordinate existing Default regional area-task travel and arrived-hold phases with
the posture manager, without changing navigation goals, task priority, ROS topics,
or the existing area-task dwell timers.

## Current Evidence

`AreaManager` owns Default task phase and dwell state. Base, PreRoadland,
Highland, ReadyRoadland, and CommonCentral have their own routes and dwell
durations. `PostureLogic` currently selects posture from aim, health, ammo,
damage, and navigation rotation state, but does not consume area-task state.

The official posture budgets arrive through lower-machine TypeID 10 as referee
`0x020D sentry_info_3`, then reach `/ly/game/sentry/info`. BT uses those values
only while fresh; otherwise it retains its existing local posture accumulation
fallback.

## Decision

`AreaManager` will expose an internal posture hint derived from the active
regional task runtime:

- `Transit`: the task is travelling or its apparent hold phase was entered by
  timeout/unreachable rather than a verified arrival.
- `ArrivedHold`: the task has verified composite arrival and is in its existing
  dwell period.
- `None`: no active regional Default task controls posture.

`PostureLogic` consumes this hint after its ordinary selection:

- `Transit` normally requests Move.
- When fresh referee timing says Move is inside the existing warning window,
  Transit instead chooses the non-degraded base posture with the greatest
  official remaining time; Defense wins an equal remaining-time tie. This
  preserves the useful defensive-travel fallback without inventing a second
  timer.
- `ArrivedHold` makes no forced selection. Existing target, damage, health, and
  ammo scoring therefore decides Attack or Defense at the held point.
- Recovery remains Move. Existing outpost-engagement request policy remains
  unchanged.

## Arrival Integrity

The runtime records whether a phase entered with verified arrival. It is reset
whenever a new phase/goal begins. Timeout and unreachable transitions do not
set it. This prevents an unreachable Highland or ReadyRoadland phase from being
treated as an arrived firing position.

## Boundaries

This is an internal behavior-tree contract only. It adds no ROS topic, message,
YAML key, navigation goal, task priority, or simulator trace schema. Existing
posture cooldown, minimum hold, acknowledgement, retry, and referee-timer
semantics stay authoritative.

## Validation

Unit tests cover phase hint classification and budget-aware Transit selection.
The behavior-tree package build, targeted CTest suite, static sentry self-check,
and diff check validate integration. Current regional decision documentation is
updated with the resulting priority and data source.
