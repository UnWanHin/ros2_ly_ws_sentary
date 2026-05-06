# RFID Area Logic

Updated: 2026-05-07

## Purpose

RFID from the referee system is treated as a **region trigger**, not a single point.
The BT side now has a small reusable module for future regional tasks:

- `src/behavior_tree/include/RfidAreaManager.hpp`
- `src/behavior_tree/src/RfidAreaManager.cpp`

This module does not publish ROS topics and does not change current decision output yet.
It only provides reusable logic for later task integration.

## Input State

The live RFID source is still:

```text
/ly/game/rfid -> gimbal_driver/msg/RfidStatus -> behavior_tree RfidMatchState
```

`RfidMatchState.Fresh` is maintained in `GameLoop.cpp` with the existing 1s freshness timeout.
If RFID is stale, `IsRfidAreaKindTriggered()` returns false.

## Region Spec

Each future RFID region can be described with:

```cpp
BehaviorTree::RfidAreaSpec{
    .Name = "SelfHighlandRfid",
    .Kind = BehaviorTree::RfidAreaKind::SelfHighlandGainPoint,
    .Boundary = {
        {x0, y0},
        {x1, y1},
        {x2, y2},
        // ...
    },
    .Priority = 10,
    .Enabled = true,
};
```

Coordinates are official-map centimeters, same convention as `Area.hpp`.
If red/blue regions differ, the caller should pass the already resolved team-side boundary.

## Available Helpers

`IsRfidAreaKindTriggered(state, kind)`

Checks whether the semantic RFID bit/group is currently active.

`IsPointInsideRfidArea(spec, x, y)`

Checks whether an official-map coordinate lies inside the configured polygon.

`ComputeRfidAreaCenter(boundary)`

Computes the polygon centroid. If the polygon is degenerate, it falls back to averaging vertices.

`ComputeRfidAreaCenterGoal(boundary)`

Returns the same center rounded/clamped to `Area::Point<uint16_t>`, suitable for `/ly/navi/goal_pos_raw`.

`SelectHighestPriorityTriggeredRfidArea(state, specs)`

Filters enabled regions, requires fresh RFID and a valid center, then chooses the highest-priority triggered region.
Ties keep the earlier region in the input list.

## Current Status

No regional task consumes this module yet. The next step is to add concrete RFID polygons and decide which task layer should use them.
