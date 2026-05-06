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
    .Shape = BehaviorTree::Area::ShapeType::Polygon,
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

Circle-ring RFID regions are also supported:

```cpp
BehaviorTree::RfidAreaSpec{
    .Name = "CenterRingRfid",
    .Kind = BehaviorTree::RfidAreaKind::CenterGainPoint,
    .Shape = BehaviorTree::Area::ShapeType::CircleRing,
    .CircleRing = {
        .center = {1400.0, 750.0},
        .innerRadiusCm = 80.0,
        .outerRadiusCm = 140.0,
        .segmentCount = 8,
        .startAngleDeg = 0.0,
    },
    .Priority = 20,
    .Enabled = true,
};
```

For a ring, the geometric center may be inside the hole, so it is not used as the navigation point.
The module generates `segmentCount` representative points on the middle radius:

```text
radius = (innerRadiusCm + outerRadiusCm) / 2
angle_i = startAngleDeg + 360deg * i / segmentCount
point_i = center + radius * [cos(angle_i), sin(angle_i)]
```

The reusable shape format lives in `Area.hpp`, so fixed regions can be kept next to other field points:

```cpp
static const CircleRingLocation<double> ExampleRfidRing{
    {
        .center = {1400.0, 750.0},
        .innerRadiusCm = 80.0,
        .outerRadiusCm = 140.0,
        .segmentCount = 8,
        .startAngleDeg = 0.0,
    },
    {
        .center = {1400.0, 750.0},
        .innerRadiusCm = 80.0,
        .outerRadiusCm = 140.0,
        .segmentCount = 8,
        .startAngleDeg = 0.0,
    },
};
```

## Available Helpers

`IsRfidAreaKindTriggered(state, kind)`

Checks whether the semantic RFID bit/group is currently active.

`IsPointInsideRfidArea(spec, x, y)`

Checks whether an official-map coordinate lies inside the configured polygon or circle ring.

`ComputeRfidAreaCenter(boundary)`

Computes the polygon centroid. If the polygon is degenerate, it falls back to averaging vertices.

`ComputeRfidAreaCenterGoal(boundary)`

Returns the same center rounded/clamped to `Area::Point<uint16_t>`, suitable for `/ly/navi/goal_pos_raw`.

`ComputeRfidAreaRepresentativePoints(spec)`

For polygon specs, returns one centroid point. For circle-ring specs, returns the equal-division middle-radius points.

`ComputeRfidAreaRepresentativeGoals(spec)`

Returns the representative points rounded/clamped to `Area::Point<uint16_t>`.

`ComputeRfidAreaCenter(spec)`

Returns the first representative point. For ring specs this is intentionally not the ring center.

`SelectHighestPriorityTriggeredRfidArea(state, specs)`

Filters enabled regions, requires fresh RFID and a valid center, then chooses the highest-priority triggered region.
Ties keep the earlier region in the input list.

## Current Status

No regional task consumes this module yet. The next step is to add concrete RFID polygons and decide which task layer should use them.
