# Tactical Damage Rotate Design

## Status

Accepted: 2026-07-19

## Context

The formal Regional control loop currently hard-codes the chassis Rotate ramp after damage. The old
`PointManager` table only contained disabled overrides and has been removed, so a tactical global
default is the sole ordinary Rotate source. Navigation `should_rotate=false`
must continue to force `FollowMode=1` and `Rotate=0` after every tactical decision.

## Decision

Create `config/Tactical.yaml` with `Tactical.DamageRotate`. It owns the global default gear, the
no-hit timeout, the three ramp holds, and the damage scan timing. The removed PointManager table has
no replacement: tactical settings are the single ordinary Rotate source.

Priority is fixed as follows:

```text
Tactical default -> damage ramp (max gear)
-> existing hard safety/debug constraints -> fresh NaviRotateControl stop request
```

No ROS message or topic changes are introduced.

## Validation

- A small unit test verifies the 0 -> 1 -> 2 -> 3 timing and preserves a higher configured default gear.
- Targeted `behavior_tree` build and its CTest suite pass when `sentry_msgs` is available.
- Graph JSON validation and `git diff --check` pass.
