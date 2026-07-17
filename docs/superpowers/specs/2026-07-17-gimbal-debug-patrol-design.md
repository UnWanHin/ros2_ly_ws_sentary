# Gimbal Debug Patrol Design

Date: 2026-07-17

## Goal

Make `ros2 launch gimbal_driver debug_node.launch.py` safe for standalone
navigation and rotation testing while allowing an opt-in patrol scan that uses
the same canonical patrol profile as the formal behavior-tree chain.

## Scope

- Keep `debug_node` standalone: it starts `gimbal_driver` plus its debug
  bridge, never `behavior_tree`, FaceMode, target selection, posture control,
  or navigation-goal logic.
- Keep `/ly/control/vel` and partial `/ly/control/firecode` as the formal
  driver inputs. The bridge publishes them at the configured 100 Hz rate.
- Add angle ownership to the bridge so every outgoing `GimbalControlFrame`
  has a safe angle value.

## Configuration Contract

`src/gimbal_driver/config/debug_mode.yaml` owns debug behavior:

```yaml
/**:
  ros__parameters:
    patrol:
      enabled: false
    angle_feedback_stale_timeout_ms: 1200
```

`debug_node.launch.py` adds `patrol_config_file`, defaulting to the installed
`behavior_tree/config/Patrol.yaml`. It passes this path to the bridge. The
debug profile does not duplicate `PatrolScan.Mode` or any mode curve.

`patrol.enabled=false` is the safe default. The bridge must receive a fresh
`/ly/gimbal/angles` feedback sample before publishing any `/ly/control/*`
command. It publishes that feedback angle unchanged to `/ly/control/angles`,
so the lower controller holds the current gimbal angle while velocity and
Rotate/FollowMode commands are tested. If feedback is absent or older than
`angle_feedback_stale_timeout_ms`, it publishes no control commands.

`patrol.enabled=true` still requires fresh feedback. The first feedback yaw
anchors the scan. The bridge reads the formal `PatrolScan.Mode` and all mode
parameters from `patrol_config_file`, then publishes the calculated patrol
angle to `/ly/control/angles` at the same 100 Hz control cadence. Therefore a
formal profile change is shared by debug without a separate Mode1/Mode2
switch in `debug_mode.yaml`.

## Patrol Semantics

- Mode1: single-direction yaw scan plus its configured pitch sine curve.
- Mode2: yaw oscillation around the feedback-anchored center with configured
  drift and pitch sine curve.
- Mode3: single-direction yaw with its high-pitch profile.

The debug bridge uses only the configured default `PatrolScan.Mode`. It does
not reproduce BT task selection such as FaceMode-missing or outpost fallback:
those require BT task/aim state and are deliberately absent from standalone
debug mode.

## Implementation Boundary

The patrol math is extracted from the behavior-tree scan path into a small,
deterministic shared implementation used by the formal BT and a dedicated
debug patrol publisher. This avoids a second independently-maintained Python
copy of the Mode1/2/3 curves. The debug bridge remains the sole debug owner
of `/ly/control/vel`, `/ly/control/firecode`, and `/ly/control/angles`.

The debug launch must not run with a formal `behavior_tree`, because both
would publish `/ly/control/angles`, `/ly/control/vel`, and
`/ly/control/firecode`.

## Validation

- Unit tests cover no-feedback, stale-feedback, feedback-hold, and all three
  patrol modes using fixed time and feedback inputs.
- A launch/static contract check verifies that debug mode has one publisher
  for each formal `/ly/control/*` topic and no BT process.
- Targeted builds cover `gimbal_driver` and `behavior_tree` after extracting
  the shared patrol implementation.
- Runtime validation on the NUC checks `/ly/control/angles`,
  `/ly/control/vel`, and `/ly/control/firecode` at 100 Hz and uses raw
  DownlinkTypeID `0x00` observation when serial-mode tracing is enabled.

## Rejected Alternatives

- Duplicating Mode1/2/3 math in the Python bridge: low initial effort, but it
  would drift from the formal chain.
- Starting the full BT in debug launch: it would also own target, firecode,
  velocity, posture, and task outputs, violating standalone debug ownership.
- Sending velocity/firecode before a gimbal angle feedback sample: the driver
  control shadow initializes angles to zero, which can command an unintended
  yaw/pitch target on the complete 0x00 frame.
