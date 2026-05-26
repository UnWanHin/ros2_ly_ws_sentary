# Simulator Base Points Correction

Updated: 2026-05-23

## Scope

Corrected the simulator map coordinate for both bases.
This is a simulator map/config correction only; it does not change ROS topics, messages, behavior-tree decision logic, or launch flow.

## Changes

- `src/simulator/config/default.yaml`
  - Updated `simulator_inputs.structure_positions.base.red` from `[401, 691]` to `[245, 750]`.
  - Updated `simulator_inputs.structure_positions.base.blue` from `[2400, 811]` to `[2555, 750]`.
  - Updated navigation goal ID `1` (`Base`) red point from `[401, 691]` to `[245, 750]`.
  - Updated navigation goal ID `1` (`Base`) blue point from `[2400, 811]` to `[2555, 750]`.
- `docs/sentry/internal/simulator.md`
  - Documented Base goal ID `1` as red `(245, 750)` and blue `(2555, 750)`.
  - Documented that base HP structure badges use those same points.

## Verification

- `PYTHONPATH=src/simulator python3 -m simulator.main --validate-only`
- `SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test --no-web-stream`
- `git diff --check`
