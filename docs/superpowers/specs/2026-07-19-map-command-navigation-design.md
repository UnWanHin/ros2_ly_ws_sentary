# Map Command Navigation Design

## Status

Accepted: 2026-07-19

## Context

Official referee `0x0303 map_command_t` reaches this workspace through lower-machine `TypeID=9` as
`/ly/game/map_command`. The official V2.0 protocol defines coordinate mode as
`target_robot_id == 0` with `target_position_x/y` in official-map meters. It repeats a triggered
packet five times at 100 ms intervals and then repeats the latest packet at 1 Hz.

The existing BT subscriber only caches the message. `navi_tf_bridge` already converts the existing
`/ly/navi/goal_pos_raw` UInt16-centimeter official-map input through the configured
`official_map -> map` calibration and publishes `/goal_pose` for navigation.

## Decision

Add an opt-in `Task.MapCommand` BT task. A received command is accepted only when it is coordinate
mode, both coordinates are finite, and it is not the lower-machine default `(0, 0)`. BT converts the
official-map meters to the existing UInt16-centimeter raw-goal message and publishes no new navigation
topic.

An accepted coordinate owns navigation for `HoldSec` (default 45). Repeated packets with the same
coordinate within `DedupDistanceCm` do not extend the deadline. A changed coordinate creates a new
hold. The protocol has no sequence or fresh-click flag, so an identical point cannot be deliberately
rearmed until it changes; this prevents its 1 Hz resend from creating an infinite task.

MapCommand is an explicit operator navigation task: it takes precedence over Default, Outpost, normal
RegionalDefense and Special navigation. Hard Recovery remains above it. While recovery is active,
the MapCommand deadline continues to elapse and the original point is not automatically resumed.
Target-robot mode remains cache-only because the protocol deliberately supplies no coordinate.

## Configuration

```yaml
Task:
  MapCommand:
    Enable: true
    HoldSec: 45
    DedupDistanceCm: 20
```

## Validation

- Unit tests cover default-zero rejection, coordinate acceptance, duplicate suppression, coordinate
  replacement, deadline expiry, and recovery cancellation.
- Targeted `behavior_tree` build and CTest must pass.
- `selfcheck.sh sentry --static-only`, graph JSON validation, and `git diff --check` must pass.
