# Simulator Schema V2 Viewer Completion

Updated: 2026-05-23

## Scope

This record covers the first completion slice for `src/simulator`.
It does not change behavior-tree trace production, ROS topics, launch defaults, navigation commands, gimbal commands, or lower-machine outputs.

## Changes

- `src/simulator/simulator/model.py`
  - Added normalized viewer models for `target_state`, `events`, `navi_relative_target`, `gimbal`, and `runtime_guard`.
  - Extended `TraceRecord` so the viewer can use schema v2 fields already written by `src/behavior_tree/src/DecisionTrace.cpp`.
- `src/simulator/simulator/trace.py`
  - Added parsers for schema v2 decision context fields.
  - Kept legacy records readable by defaulting missing fields to safe display values.
  - Expanded recent-change detection to include decision intent, event condition changes, and runtime guard fault changes.
- `src/simulator/simulator/viewer.py`
  - Added right-panel tabs: `Decision`, `Events`, `Runtime`.
  - Added mouse tab switching and keyboard shortcuts `1`, `2`, `3`.
  - Moved detailed event/target/relative-target/referee/unit state into `Events`.
  - Moved ROS output/posture/gimbal/fire-code/runtime guard state into `Runtime`.
- `src/simulator/simulator/validation.py`
  - Warns on legacy `schema_version < 2`.
  - Checks schema v2 records for missing `decision_output`.
  - Warns when `relative_target_bridge` output lacks a valid `navi_relative_target`.
- `src/simulator/sample/sample_trace.jsonl`
  - Replaced the old schema v1 sample with a compact schema v2 sample.
  - Sample now covers game start, armor target, buff activation, relative target bridge, outpost window, and low-resource recovery.
- `src/simulator/README.md`
  - Documented the new tabs and keyboard shortcuts.
- `docs/sentry/internal/simulator.md`
  - Updated viewer behavior, schema coverage, and maintenance rules.

## Verification

- `python3 -m py_compile $(rg --files src/simulator/simulator -g '*.py')`
- `PYTHONPATH=src/simulator python3 -m simulator.main --validate-only`
  - Result: `records=6 errors=0 warnings=0`
- `SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test --no-web-stream`
  - Result: exited 0. ALSA warnings are from the headless environment and do not affect the dummy pygame draw check.
- `git diff --check`
- `colcon build --packages-select simulator`
- `colcon test --packages-select simulator && colcon test-result --verbose`
  - Result: `0 tests, 0 errors, 0 failures, 0 skipped`.

## Remaining Work

- Run a real GUI session on the robot laptop or WSL display to inspect visual spacing and readability.
- Record a fresh live trace from `behavior_tree` and confirm the new panels show real EventManager, gimbal, and runtime guard values as expected.
- If future decision outputs add fields, update `DecisionTrace.cpp`, parser/model, validation, sample trace, and docs in the same change.
