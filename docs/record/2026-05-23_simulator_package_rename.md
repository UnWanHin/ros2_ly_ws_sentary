# Simulator Package Rename

Updated: 2026-05-23

## Scope

The offline decision replay/viewer package was renamed from `decision_viz` to `simulator`.
This is a package/interface rename only. It does not change behavior-tree trace production, ROS navigation topics, gimbal topics, referee topics, launch defaults for the robot stack, or lower-machine command ownership.

## File And Interface Changes

- Source package path:
  - Old: `src/decision_viz`
  - New: `src/simulator`
- Python module:
  - Old: `decision_viz`
  - New: `simulator`
- ROS package name:
  - Old: `decision_viz`
  - New: `simulator`
- Ament resource marker:
  - Old: `src/decision_viz/resource/decision_viz`
  - New: `src/simulator/resource/simulator`
- Package metadata:
  - `src/simulator/package.xml` now declares `<name>simulator</name>`.
  - `src/simulator/setup.py` now uses `package_name = "simulator"`.
  - `src/simulator/setup.cfg` now installs scripts under `lib/simulator`.
- Console scripts:
  - Old: `decision-viz`
  - New: `simulator`
  - Old: `decision-viz-start`
  - New: `simulator-start`
  - Old: `decision-viz-mock-inputs`
  - New: `simulator-mock-inputs`
  - Old: `decision-viz-ros-topic-monitor`
  - New: `simulator-ros-topic-monitor`
- Python module commands:
  - Old: `python3 -m decision_viz.main`
  - New: `python3 -m simulator.main`
  - Old: `python3 -m decision_viz.start`
  - New: `python3 -m simulator.start`
  - Old: `python3 -m decision_viz.mock_inputs`
  - New: `python3 -m simulator.mock_inputs`
  - Old: `python3 -m decision_viz.ros_topic_monitor`
  - New: `python3 -m simulator.ros_topic_monitor`
- One-command wrapper:
  - `scripts/python/start.py` now builds `PYTHONPATH=src/simulator` and runs `python3 -m simulator.start`.
  - The printed label is now `offline simulator starter`.
- Runtime temp/default files:
  - Old: `/tmp/decision_viz_match_control.jsonl`
  - New: `/tmp/simulator_match_control.jsonl`
  - Old: `/tmp/decision_viz_ros_topics.json`
  - New: `/tmp/simulator_ros_topics.json`
- Generated offline config output:
  - Old: `log/decision_viz/`
  - New: `log/simulator/`
- Config schema string:
  - Old: `ly_decision_viz_config_v1`
  - New: `ly_simulator_config_v1`
- Live ROS monitor state schema:
  - Old: `ly_decision_viz_ros_topics_v1`
  - New: `ly_simulator_ros_topics_v1`
- Internal code names:
  - `DecisionVizWebStream` became `SimulatorWebStream`.
  - `DecisionVizRosTopicMonitor` became `SimulatorRosTopicMonitor`.
  - Pygame panel title became `Simulator`.
- Documentation:
  - `docs/sentry/internal/decision_visualization.md` became `docs/sentry/internal/simulator.md`.
  - `docs/sentry/internal/README.md`, `docs/README.md`, `docs/modules/2026-05-05_behavior_tree.md`, `docs/agents/domain.md`, and `AGENTS.md` now point to `src/simulator` and `docs/sentry/internal/simulator.md`.
  - `src/simulator/README.md` documents the new commands and package name.

## Verification

- `rg -n "decision_viz|decision-viz|Decision Viz|DecisionViz|decision_visualization|ly_decision_viz|desicion|decision_riv" src scripts docs AGENTS.md README.md`
  - Result after source/docs rename: no matches except this historical record.
- `python3 -m py_compile $(rg --files src/simulator/simulator -g '*.py')`
- `PYTHONPATH=src/simulator python3 -m simulator.main --validate-only`
  - Result: `records=6 errors=0 warnings=0`
- `SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test --no-web-stream`
  - Result: exited 0. ALSA warnings are from the headless environment.
- `git diff --check`
- `colcon list --packages-select simulator`
  - Result: `simulator src/simulator (ros.ament_python)`
- `python3 scripts/python/start.py --dry-run --no-view`
  - Result: generated command uses `PYTHONPATH=/home/unwanhin/ros2_ly_ws_sentry/src/simulator python3 -m simulator.start`.
- `colcon build --packages-select simulator`
- `colcon test --packages-select simulator && colcon test-result --verbose`
  - Result: `0 tests, 0 errors, 0 failures, 0 skipped`.
- `source install/setup.bash && ros2 run simulator simulator --validate-only`
  - Result: `records=6 errors=0 warnings=0`.

## Cleanup

Removed stale generated package artifacts for the old package from:

- `build/decision_viz`
- `install/decision_viz`
- `log/decision_viz`

Historical `log/build_*` and `log/test_*` directories may still contain old package names because they are past colcon logs.
