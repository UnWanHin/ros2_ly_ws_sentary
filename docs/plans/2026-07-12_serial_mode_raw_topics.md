# SerialMode Raw Topic Plan

Updated: 2026-07-12

## Goal

Expose one timestamped raw ROS topic per current serial protocol ID without changing the existing semantic
topics used by behavior_tree and bridges.

## Contract

- `io_config.serial_mode` is the master switch and defaults to `false`.
- Upload raw topics are `/ly/upload/typeid0` through `/ly/upload/typeid10`, enabled by `upload.typeid0` through `upload.typeid10`.
- Download raw topics are `/ly/download/typeid0x00` through `/ly/download/typeid0x04`, enabled by `download.typeid0x00` through `download.typeid0x04`.
- All topics use `gimbal_driver/msg/GimbalRawFrame`; `header.stamp` is recorded when the upper machine
  receives an uplink frame or emits a downlink frame.
- `type_id` is the real upload `TypeID` or download `DownlinkTypeID` for the per-ID topics.
- Existing semantic topics such as `/ly/game/sentry/info` and `/ly/gimbal/angles` remain unchanged.
- A publisher only builds and publishes a raw message when its per-ID publisher has at least one subscriber.

## Configuration Ownership

`src/gimbal_driver/config/gimbal_driver_config.yaml` owns the master, direction, and per-ID switches.
The YAML keeps only nested `io_config` keys; `gimbal_driver` still accepts slash and dot runtime parameter
names for launch/CLI compatibility.
When a protocol TypeID or DownlinkTypeID is added, update the structure mapping, these YAML switches,
raw-topic publisher mapping, serial documentation, and the Understand Anything graph in the same change.

## Verification

- Build `gimbal_driver`, `behavior_tree`, and `navi_tf_bridge`.
- Confirm the baseline YAML contains all current upload `0..10` and download `0x00..0x04` switches.
- Confirm source maps every current raw frame path to exactly one per-ID topic.
- Run `./scripts/selfcheck.sh sentry --static-only`.
