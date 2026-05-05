# Auto-Aim Latched Target Hold

Date: 2026-04-28

## Background

Compared with `~/sentry.aim`, this workspace keeps the original chain:

- `/ly/tracker/results`
- `/ly/predictor/target`
- behavior_tree `PublishTogether()`
- `/ly/control/angles` and `/ly/control/firecode`

`sentry.aim` gets good follow/response mainly from a bounded target cache: its controller runs at 100 Hz, reuses the newest target briefly, and falls back after `target_msg_timeout_sec` (`0.10 s`).

The previous BT behavior reused latched auto-aim angles whenever no fresh target arrived, until the later search branch took over. That made short callback jitter smooth, but also made the gimbal visibly stick to stale angles after target loss.

## Change

Added `AimDebug.LatchedTargetHoldMs` to the BT JSON config, default `100`.

In `src/behavior_tree/src/GameLoop.cpp`:

- Fresh predictor/buff/outpost target still takes priority.
- If no fresh target arrives, the last valid latched angle is reused only while it is newer than `LatchedTargetHoldMs`.
- During the short hold window, BT keeps `FireCode.AimMode = 1` and continues publishing the latched angle.
- Fire toggling still requires a fresh target callback; held targets do not create new fire toggles.
- After the hold window expires, BT no longer reuses the stale aim angle and clears `FireCode.AimMode`.

All BT JSON files with `AimDebug` now explicitly set:

```json
"LatchedTargetHoldMs": 100
```

## Scope

This is a low-risk first step toward the `sentry.aim` response model. It does not port the external TF/controller packages and does not change detector, tracker, predictor, ROS topics, messages, or lower-machine communication.
