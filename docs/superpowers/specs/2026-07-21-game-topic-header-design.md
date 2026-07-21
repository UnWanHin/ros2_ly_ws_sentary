# `/ly/game/*` Header Migration

## Status

Proposed

## Date

2026-07-21

## Scope

Only semantic `/ly/game/*` topics whose source is lower-machine uplink data are in scope.
Existing non-`/ly/game/*` topics, raw `/ly/upload/typeid*` topics, navigation topics, and
downlink/control topics are unchanged.

## Goal

Every in-scope semantic game message exposes a ROS `std_msgs/Header` so consumers can
reason about freshness. The stamp means **the time `gimbal_driver` received and decoded the
lower-machine serial frame**. It is not an MCU sampling timestamp and must not be interpreted
as one.

## Contract

Already stamped interfaces retain their current contract and fields:

- `/ly/game/event_data` (`gimbal_driver/msg/EventData`)
- `/ly/game/rfid` (`gimbal_driver/msg/RfidStatus`)
- `/ly/game/sentry/info` (`gimbal_driver/msg/SentryInfo`)
- `/ly/game/bullet` (`gimbal_driver/msg/BulletInfo`)
- `/ly/game/map_command` (`gimbal_driver/msg/MapCommand`)

The following legacy interfaces change type in place because this workspace is their sole
consumer:

| Topic | Old type | New type |
|---|---|---|
| `/ly/game/all` | `gimbal_driver/msg/GameData` | same message with `std_msgs/Header header` added |
| `/ly/game/time_left` | `std_msgs/UInt16` | `gimbal_driver/msg/StampedUInt16` |
| `/ly/game/damage_difference` | `std_msgs/Int16` | `gimbal_driver/msg/StampedInt16` |

`/ly/game/is_start` intentionally remains `std_msgs/Bool`: it is the existing startup gate and
is not migrated in this change. The two scalar wrappers contain only `std_msgs/Header header`
and a `data` field of their
named scalar type. They do not combine unrelated game fields.

## Data Flow

```text
lower serial TypeID
  -> gimbal_driver receive/decode time
  -> header.stamp on every /ly/game/* semantic message
  -> behavior_tree subscription
  -> per-field receive/freshness state
```

`/ly/game/time_left` uses the same TypeID=1 receive/decode stamp as `/ly/game/all`.
`/ly/game/damage_difference` uses the TypeID=6 receive/decode stamp.

## Consumer Changes

`behavior_tree` updates its typed topic aliases and subscriptions for the four migrated
contracts. Existing decision values and launch topic names do not change. The migration must
not alter serial packet layouts or lower-machine firmware contracts.

## Verification

1. Build `gimbal_driver` and `behavior_tree`.
2. Run all affected unit/CTest tests and static sentry self-check.
3. Add focused tests or compile-time checks proving each migrated message has a Header.
4. Validate publisher/subscriber types with `ros2 topic info` in a ROS-capable environment.
5. Update the current topic and serial mapping documentation with message types and timestamp
   semantics.

## Non-goals

- No source-time clock synchronization with the lower machine.
- No TypeID wire-layout changes.
- No topic renames, mirror topics, or compatibility shims for the replaced legacy scalar types.
