# Aim / Task / Patrol Mode Flow

Updated: 2026-07-12

This document distinguishes the Regional decision layers that remain after the internal vision packages were removed. Aim selection, task selection, gimbal scan, regional navigation, FollowMode, and FaceMode are separate controls.

## Layers

| Name | Owner | Role |
|---|---|---|
| External aim | external `/ly/aim/*` provider | Publishes target candidates and final follow/yaw/pitch/fire result. |
| `Task.Buff` / `Task.Outpost` | BT config | Allows the corresponding Regional task to enter the strategy candidate set. |
| `AimMode` and `/ly/vision/mode` | `behavior_tree` | Represents the active decision mode; it no longer switches an internal camera pipeline. |
| Gimbal patrol scan | `behavior_tree` | Produces `/ly/control/angles` while no fresh external aim result is controlling the gimbal. |
| Regional patrol | `AreaManager` / strategy layer | Chooses navigation goals and area transitions. |
| FollowMode | `FireCode` semantic bit | Is sent through `/ly/control/firecode`; it is not a task selector. |
| FaceMode | BT + `map_aim_point_node` | Provides a map-point yaw/pitch fallback through `/ly/face_mode/angles`. |

## Normal Armor / Rotate Scan

When neither special task is active, Regional normally uses `AimMode::RotateScan` and publishes `/ly/vision/mode=1`. The formal target and fire path is still external aim:

```text
external /ly/aim/armor_targets + /ly/aim/result
  -> behavior_tree
  -> /ly/control/angles + /ly/control/firecode
```

If no fresh external result is available, the BT may use its configured patrol scan and FaceMode fallback. It does not start a detector or predictor in this workspace.

## Buff And Outpost Tasks

`Task.Buff=true` permits the energy-mechanism task and `Task.Outpost=true` permits the outpost task. Their task gates, navigation goals, posture selection, referee data, and `SentryCmd` confirmation behavior remain in `behavior_tree`; their visual target input must come from the external aim provider through `/ly/aim/*`.

`/ly/vision/mode=2` and `=3` are observability values for these active modes. They are not commands to resurrect the deleted internal buff/outpost pipeline.

## FaceMode And Regional Patrol

Regional navigation is selected through the usual strategy layers and `AreaManager`. FaceMode receives an official-map target at `/ly/face_mode/target_raw`, resolves it to `/ly/face_mode/angles`, and the BT decides whether to forward it to `/ly/control/angles` according to task and target priority.

`FollowMode`, gimbal scanning, FaceMode, navigation, and fire suppression remain independent outputs. Changing one must not be used as a substitute for changing another.

## Maintenance Rule

Current aim integration changes must update this document, `vision_mode_semantics.md`, `ros2_topic_structure.md`, `ros2_topic_tree.md`, and the Understand Anything graph. Do not add back `/ly/detector/*`, `/ly/predictor/*`, `/ly/buff/*`, or `/ly/outpost/*` endpoints without an explicit interface migration.
