# Aim Mode Semantics

Updated: 2026-07-12

## Purpose

`/ly/vision/mode` remains a behavior-tree observation topic. It no longer selects or starts an in-repository camera, detector, tracker, predictor, buff, or outpost node: those packages were removed when the formal aim source moved outside this workspace.

The formal control chain is:

```text
external /ly/aim/armor_targets + /ly/aim/result
  -> behavior_tree
  -> /ly/control/angles + /ly/control/firecode
  -> gimbal_driver
  -> lower controller
```

## Topic

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/ly/vision/mode` | `std_msgs/msg/UInt8` | `behavior_tree` | debug tools and observers only |

Mode values continue to mirror the active BT `AimMode`:

| Value | Name | Meaning |
|---:|---|---|
| `0` | `DISABLED` | No dedicated aim mode is active. |
| `1` | `ARMOR` | Normal armor / rotate-scan behavior. |
| `2` | `BUFF` | Energy-mechanism task behavior. |
| `3` | `OUTPOST` | Outpost task behavior. |

`/ly/vision/mode` is not sent to the lower controller. The lower controller receives only the semantic control topics such as `/ly/control/angles`, `/ly/control/firecode`, and `/ly/control/sentry_cmd`.

## External Aim Contract

For every mode, the external aim stack remains responsible for publishing:

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/ly/aim/armor_targets` | `sentry_msgs/msg/AimTargetArray` | external aim -> BT | Fresh target candidates, target distance, and optional TF frame point for chase. |
| `/ly/aim/result` | `sentry_msgs/msg/AimResult` | external aim -> BT | Final yaw/pitch, follow state, and fire gate. |
| `/ly/aim/select_target` | `sentry_msgs/msg/AimTarget` | BT -> external aim | BT-selected target id and latest candidate position. |

When `ExternalAim.UseTargetArrayAsArmorList=true`, a valid `/ly/aim/result` needs fresh matching target-array context. This prevents stale aim results from producing control output after target context disappears.

## Task Relationship

`Task.Buff` and `Task.Outpost` are BT configuration switches, not vision-pipeline switches. They determine which regional task can be chosen; once a task is active, the BT exposes the corresponding mode value and continues to consume the same external `/ly/aim/*` contract.

Do not create new scripts, launches, or tests that publish `/ly/detector/*`, `/ly/predictor/*`, `/ly/buff/*`, or `/ly/outpost/*`. Those were interfaces of removed internal packages.
