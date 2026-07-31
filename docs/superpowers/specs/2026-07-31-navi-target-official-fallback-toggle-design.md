# Navi Target Official Fallback Toggle

Updated: 2026-07-31

## Purpose

Allow field operators to prevent camera-derived official-map enemy positions from entering
BT decision state, without changing aim, navigation, serial, or lower-machine position inputs.

## Configuration Contract

`config/common.yaml` owns one boolean:

```yaml
Chase:
  EnableNaviTargetOfficialFallback: true
```

The default is `true`, preserving the current released behavior.

## Behavior

When `true`, BT consumes `/ly/navi/target_official` as its existing per-armor fallback. A
fresh, non-zero `/ly/position/data` update for the same unit remains authoritative and blocks
that fallback for `Chase.OfficialPositionFreshMs`.

When `false`, BT ignores `/ly/navi/target_official` before it writes `enemyRobots`, position
source metadata, or `/ly/enemy/info`. The bridge may continue publishing the topic for
diagnosis. `/ly/aim/armor_targets`, aim/fire, `/ly/navi/target_rel`, `/goal_pose`, and lower
machine `/ly/position/data` behavior are unchanged.

## Scope And Verification

The change is limited to common-config loading, the BT target-official subscriber, a focused
unit test covering enabled/disabled acceptance and lower-machine precedence, and the current
coordinate-chain documentation. Build `behavior_tree`, run its focused test, then run the
static sentry self-check.
