# Gimbal Debug Rotate Heartbeat Design

## Status

Accepted by the operator on 2026-07-16.

## Context

`debug_node.launch.py` owns the standalone driver debug path. Its
`debug_mode.yaml` sets `navigation_mode.enabled=true` and `rotate_level=1`.
The driver already writes that Rotate value once after serial initialization,
but a single packet is not sufficiently observable or robust for field
debugging when no `/ly/navi/vel` message is present. The standalone debug
path must also expose navigation velocity through the formal `/ly/control/vel`
contract instead of bypassing the control topic.

## Decision

While and only while `io_config.navigation_mode.enabled=true`,
`gimbal_driver` will write the current navigation Rotate state at a best-effort
100 Hz interval (10 ms):

- Before any `/ly/navi/should_rotate` message, the default state remains true
  and writes `rotate_level`.
- A received `false` writes and continues to write `Rotate=0`; the existing
  optional `FollowMode=1` behavior remains unchanged.
- A received `true` immediately restores and continues to write
  `rotate_level`.
- `debug_node` starts a 100 Hz bridge that converts `/ly/navi/vel` to
  `/ly/control/vel` with `use_raw=true`; it publishes zero after 500 ms without
  navigation input. `gimbal_driver` consumes that formal control topic, so each
  outgoing control frame retains the current Rotate state.

The formal `sentry_all -> behavior_tree -> /ly/control/* -> gimbal_driver`
chain remains unchanged because it does not enable `navigation_mode`.

## Consequences

- Standalone debug sends a control frame even without navigation velocity.
- The main driver loop remains at 250 Hz; a monotonic-clock gate bounds this
  debug-only control-frame heartbeat to approximately 100 Hz.
- No ROS topic or message contract changes; only the debug launch composition
  adds the bridge node.
- The existing selfcheck gains a source-level contract to prevent removal or
  accidental relocation of the debug-only heartbeat.
