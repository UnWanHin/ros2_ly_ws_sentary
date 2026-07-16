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

`debug_node` runs `navi_vel_to_control_vel.py`, which publishes the current
navigation control state to the formal control topics at a best-effort 100 Hz
interval (10 ms):

- Before any `/ly/navi/should_rotate` message, the default state remains true
  and the bridge publishes `rotate_level` with `FollowMode=false` to
  `/ly/control/firecode`.
- A received `false` publishes `Rotate=0` and the configured FollowMode value;
  a received `true` restores `rotate_level` and clears FollowMode.
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
  adds the bridge as the sole debug publisher of `/ly/control/vel` and
  `/ly/control/firecode`.
- The existing selfcheck gains a source-level contract to prevent removal or
  accidental relocation of the debug-only heartbeat.
