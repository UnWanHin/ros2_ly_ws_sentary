# Vision Mode Semantics

Updated: 2026-05-06

## Purpose

`/ly/vision/mode` is the semantic mode topic for selecting the active vision pipeline.
It is an upper-machine ROS2 topic and is not sent to the lower machine serial link directly.

The lower machine still receives gimbal/fire control through `/ly/control/firecode`.

## Topic

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/ly/vision/mode` | `std_msgs/msg/UInt8` | `behavior_tree` | `detector`, `buff_hitter` |

Mode values:

| Value | Name | Meaning |
|---:|---|---|
| `0` | `DISABLED` | No dedicated vision pipeline requested |
| `1` | `ARMOR` | Armor detection and normal auto-aim pipeline |
| `2` | `BUFF` | Energy mechanism detection / buff hitter pipeline |
| `3` | `OUTPOST` | Outpost detection / outpost hitter pipeline |

## Behavior Tree Mapping

`behavior_tree` maps its internal `AimMode` to `/ly/vision/mode`:

| Internal `AimMode` | `/ly/vision/mode` |
|---|---:|
| `AutoAim` | `1` |
| `RotateScan` | `1` |
| `Buff` | `2` |
| `Outpost` | `3` |
| other modes | `0` |

This mode only selects the vision pipeline. It does not mean the gimbal is in follow mode, FaceMode, or fire mode.

## Deprecated Topics

The old Bool mode topics are removed from the active chain:

| Deprecated topic | Replacement |
|---|---|
| `/ly/aa/enable` | `/ly/vision/mode=1` |
| `/ly/ra/enable` | `/ly/vision/mode=2` |
| `/ly/outpost/enable` | `/ly/vision/mode=3` |

Do not use the old Bool topics for new scripts, launch files, or runtime checks.

## Downstream Behavior

`detector` expands `/ly/vision/mode` into its internal gates:

| `/ly/vision/mode` | `aa_enable` | `ra_enable` | `outpost_enable` |
|---:|---|---|---|
| `0` | false | false | false |
| `1` | true | false | false |
| `2` | false | true | false |
| `3` | false | false | true |

Effects:

- `aa_enable=true`: detector publishes normal armor results to `/ly/detector/armors`.
- `ra_enable=true`: detector publishes angle images to `/ly/ra/angle_image` for `buff_hitter`.
- `outpost_enable=true`: detector publishes outpost armor results to `/ly/outpost/armors`.

`buff_hitter` uses `/ly/vision/mode` as:

- mode `2`: enable buff solving.
- mode `1`: normal armor mode is active, so buff solving is skipped.

## Manual Test Commands

Armor mode:

```bash
ros2 topic pub --once /ly/vision/mode std_msgs/msg/UInt8 "{data: 1}"
```

Buff mode:

```bash
ros2 topic pub --once /ly/vision/mode std_msgs/msg/UInt8 "{data: 2}"
```

Outpost mode:

```bash
ros2 topic pub --once /ly/vision/mode std_msgs/msg/UInt8 "{data: 3}"
```

Disable vision pipelines:

```bash
ros2 topic pub --once /ly/vision/mode std_msgs/msg/UInt8 "{data: 0}"
```

Inspect the graph:

```bash
ros2 topic info /ly/vision/mode -v
ros2 topic echo /ly/vision/mode
```

## Migration Rule

Use `/ly/vision/mode` for new behavior-tree, detector, buff, and outpost mode logic.
