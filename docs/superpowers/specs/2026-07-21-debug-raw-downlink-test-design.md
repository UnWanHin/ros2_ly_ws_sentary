# Debug Raw Downlink Test Design

Updated: 2026-07-21

## Goal

Add a hardware communication test mode to `gimbal_driver`'s existing
`debug_node.launch.py`. When enabled from `debug_mode.yaml`, an operator can
send a complete lower-machine serial frame through a per-ID `/ly/download/*`
topic and observe the complete RX/TX serial frames without any normal control
traffic sharing the serial device.

## Configuration

`debug_mode.yaml` gains `raw_downlink_test_mode`, defaulting to `false`.

- `false`: preserve current debug behavior. `/ly/download/typeid0x00` through
  `/ly/download/typeid0x05` are observation-only TX mirrors when serial-mode
  raw observation is enabled.
- `true`: raw downlink test mode has highest priority over `navi_mode`,
  `aim_mode`, patrol, and every normal gimbal-driver control subscriber.

The debug launch reads this one driver-specific switch from the debug profile
and passes only that value to `gimbal_driver`; bridge-owned parameters remain
private to `debug.py`.

## Raw Input Contract

When raw test mode is active, these topics become the direct serial input
surface:

| Topic | Required `type_id` / byte 1 | Allowed physical frame |
|---|---:|---:|
| `/ly/download/typeid0x00` | `0x00` | 13B `GimbalControlFrame` |
| `/ly/download/typeid0x01` | `0x01` | 6B `SentryCommandFrame` |
| `/ly/download/typeid0x02` | `0x02` | one 64B `MapPathFragmentFrame` |
| `/ly/download/typeid0x03` | `0x03` | 36B `CustomInfoFrame` |
| `/ly/download/typeid0x04` | `0x04` | 17B `SentryCoordinateFrame` |
| `/ly/download/typeid0x05` | `0x05` | 26B `GimbalTrajectoryFrame` |

Each input is `gimbal_driver/msg/GimbalRawFrame`. It must set
`direction=DIRECTION_TX`, use the matching `type_id`, and put the complete
physical bytes in `data`. The driver rejects malformed frames before any serial
write: incorrect head flag (`0x21`), topic/type mismatch, byte-1 mismatch,
incorrect fixed length, invalid map-path CRC16, or invalid sentry-coordinate
CRC8.

`0x02` callers publish each already-formed 64B fragment separately; the driver
does not reinterpret or regenerate its sequence, payload, or CRC.

## Exclusivity And Observation

Raw test mode is an exclusive serial writer:

- `/ly/control/angles`, `/ly/control/firecode`, `/ly/control/vel`,
  `/ly/control/trajectory`, `/ly/control/posture`, `/ly/control/sentry_cmd`,
  `/ly/control/map_path`, `/ly/game/path`, `/ly/control/custom_info`, and
  `/ly/bt/sentry_position` cannot write serial frames.
- Scheduled posture retries, coordinate retries, stale control fallbacks, and
  debug-bridge outputs also cannot write a frame.
- RX parsing and semantic feedback publication remain active.

The driver must not publish its TX observation to `/ly/download/*` in this
mode, because the same topic is the injection subscriber and would otherwise
feed its own TX frame back into serial writes. Complete TX observation remains
available through `/ly/log/gimbal_raw_tx` and raw screen/file logging. The
debug profile enables complete RX and TX hex logging to the screen while this
mode is active.

Disabling the switch restores the current observation-only meaning of all
`/ly/download/*` topics and re-enables the normal debug bridge/control chain.

## Verification

1. Add unit coverage for every accepted downlink frame type and each rejection
   condition, especially CRC failures for `0x02` and `0x04`.
2. Add launch/config coverage proving the debug profile passes the new switch
   only to `gimbal_driver`.
3. Build `gimbal_driver`, run its focused C++/Python tests, and run the static
   sentry self-check.
4. With a virtual serial device, publish a valid raw test frame and verify one
   TX observation; publish an invalid frame and verify no TX observation.

## Non-Goals

- No formal launch behavior, protocol layout, message definition, or normal
  `/ly/control/*` contract changes.
- No raw-injection capability outside `debug_node.launch.py`.
- No change to the observation-only behavior of `/ly/download/*` when the
  test switch is disabled.
