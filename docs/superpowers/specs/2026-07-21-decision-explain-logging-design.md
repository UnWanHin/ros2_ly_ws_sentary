# Decision Explain Logging Design

## Goal

Make the regional behavior-tree decision visible in the normal node log without
changing task priority, YAML semantics, navigation outputs, or control outputs.

## Scope

The feature has two outputs, both written through the existing `LoggerPtr` at
`INFO` level:

1. A one-time startup snapshot of the resolved AreaManager and Tactical
   settings that govern regional decisions.
2. A compact transition log whenever the final navigation intent changes its
   layer, reason, target, side, or target coordinate.

It does not add ROS topics, parameters, timers, persistence, or new behavior.

## Existing Source Of Truth

`Application::lastDecisionIntent_` is already the normalized explanation for a
selected decision. It contains `DecisionLayer`, `DecisionReason`, base and
resolved goal IDs, goal side, priority, and free-form detail. Decision trace
already serializes this exact state. `naviCommandGoal` and `naviGoalPosition`
are the final navigation values published by the BT.

The logger must consume those final values after the strategy pipeline has
completed. It must not log inside individual Tactical branches, because a
higher-priority branch can replace their target in the same tick.

## Startup Snapshot

After `AreaManager.yaml` and `Tactical.yaml` have been loaded into the active
`Configuration`, the application prints once:

```text
[DecisionConfig] AreaManager RegionalAreaTask: enable=1 MyBase=1 MyHighland=1 MyPreRoadland=1 MyReadyRoadland=1 CommonCentral=1
[DecisionConfig] Tactical ProtectCastle: enable=1 rfid=1 enemy_pos=1 stay_when_rfid=1; ProtectHero: enable=1
[DecisionConfig] Tactical DamageRotate: default_gear=0 no_hit_timeout_ms=1800 gear0_hold_ms=220 gear1_hold_ms=220 gear2_hold_ms=220 scan_boost_window_ms=1300 scan_yaw_phase_ms=160
```

The values are the parsed runtime configuration, not hard-coded YAML defaults.
The snapshot makes a bad launch/config overlay visible immediately.

## Transition Log

The application keeps one private fingerprint of the last logged final intent:

```text
layer + reason + base goal + resolved goal + team + target coordinate + detail
```

It emits one line when the fingerprint differs and navigation is publishable.
Its field order is `layer`, `reason`, `goal`, `base_goal`, `resolved_goal`,
`team`, `pos_cm`, `priority`, and `detail`. The values come respectively from
`DecisionLayerToString`, `DecisionReasonToString`, `GoalName`, the final
intent IDs, `UnitTeamToString`, `naviGoalPosition`, final intent priority, and
final intent detail.

`MapCommand` uses its raw official coordinate rather than a base goal and is
logged with `goal=raw_map_command`. No identical 100 Hz tick is logged again.

The exact reason strings remain those from `DecisionReasonToString`; therefore
they stay consistent with decision traces and tests.

## Tactical Extension Contract

Every Tactical feature that can own or replace navigation must do both of the
following in the same change:

1. Add or reuse a specific `DecisionReason`, with its layer and priority
   mapping in `DecisionIntent.hpp`.
2. Call `RecordDecisionIntent(MakeDecisionIntent(...))` after it has selected
   its final goal, putting trigger-specific facts in `Detail`.

The centralized logger then needs no per-feature branch and automatically
prints that Tactical feature at startup/config time and at runtime. A test will
cover the reason-to-log rendering and the change-only fingerprint behavior.

New fields under the current `Tactical` configuration must be appended to the
one-time snapshot beside their owning Tactical feature. This is intentional:
the log names the semantic setting rather than dumping untyped YAML.

## Error And Performance Behavior

- If there is no publishable navigation intent, no transition line is emitted.
- A changed intent with the same target is still logged, since the reason has
  changed and is operationally important.
- Log work is a small string/fingerprint comparison once per BT tick; it adds
  no DDS, serial, or control work.

## Verification

- Unit-test startup formatting from a representative runtime config.
- Unit-test transition de-duplication, reason changes, base-goal changes, raw
  map-command coordinates, and disabled navigation.
- Build `behavior_tree` and run its targeted tests.
- Update the current regional decision documentation with the observability
  contract and run static self-check.
