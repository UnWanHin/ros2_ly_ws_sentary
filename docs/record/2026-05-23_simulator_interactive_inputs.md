# Simulator Interactive Inputs

Updated: 2026-05-23

## Scope

This record covers the simulator change that lets offline decision tests control structure HP and drag unit pieces into the field map.
It keeps the existing ROS topics and message types; no new ROS message, topic, launch chain, or lower-machine output was added.

## Behavior Changes

- `src/simulator/simulator/viewer.py`
  - Added an `Inputs` right-panel tab.
  - Added keyboard shortcut `4` for the `Inputs` tab.
  - Added clickable HP controls for:
    - enemy outpost
    - enemy base
    - friend outpost
    - friend base
  - Added map badges for those structures, with side color and HP bar.
  - Added a draggable unit palette for friend/enemy:
    - Hero
    - Engineer
    - Infantry1
    - Infantry2
    - Sentry
  - Added placed-piece rendering on top of the map, with HP bars and labels when the `Inputs` tab or labels are visible.
  - Added drag-to-move for pieces already placed on the map.
  - Added placed-piece HP controls and a remove button in the panel.
  - Added screen-to-field coordinate conversion for drops. Viewer drop coordinates stay in official map centimeters with left-bottom origin.
  - Added simulator command handling for `set_structure_health`, `set_unit`, `set_unit_hp`, `remove_unit`, and `clear_units`.
  - Reused the existing JSONL command bus at `match_control.control_file`; commands only affect behavior_tree in live follow mode with a control file.

- `src/simulator/simulator/mock_inputs.py`
  - Added mock structure HP state with defaults:
    - friend outpost: `60`
    - enemy outpost: `60`
    - friend base: `5000`
    - enemy base: `5000`
  - Added publishers for:
    - `/ly/friend/op_hp`
    - `/ly/enemy/op_hp`
    - `/ly/friend/base_hp`
    - `/ly/enemy/base_hp`
  - Added simulated unit state keyed by side and unit type.
  - Added command handling for:
    - `set_structure_health` / `set_structure_hp`
    - `set_unit`
    - `set_unit_hp`
    - `remove_unit`
    - `clear_units`
  - Publishes placed-piece HP through existing `gimbal_driver/msg/Health` topics:
    - `/ly/friend/hp`
    - `/ly/enemy/hp`
  - Publishes placed-piece positions through existing `gimbal_driver/msg/PositionData` on `/ly/position/data`.
  - Sends enemy unit IDs as `100 + UnitType`, matching the lower-machine protocol comment; behavior_tree already normalizes enemy IDs with `% 100`.
  - Converts official viewer y to raw `PositionData` y as `1500 - official_y`, matching behavior_tree normalization.
  - Handles ROS external shutdown cleanly when the wrapper terminates the mock process.

- `src/behavior_tree/src/SubscribeMessage.cpp`
  - Structure HP subscribers now accept `0` as a valid value for:
    - `/ly/enemy/op_hp`
    - `/ly/friend/op_hp`
    - `/ly/friend/base_hp`
    - `/ly/enemy/base_hp`
  - This lets the simulator mark an outpost/base as destroyed. Before this change, `0` was ignored and the blackboard could not be driven to a destroyed state from the existing HP topics.
  - Unit HP subscribers were not changed; they still follow the existing robot-health freshness/zero handling.

- `src/simulator/config/default.yaml`
  - Added `layers.simulator_inputs`.
  - Added `simulator_inputs.structure_positions` for red/blue outpost and base map badges.
  - Added `simulator_inputs.structures` for default HP, max HP, and step size.
  - Added `simulator_inputs.unit_palette` for friend/enemy unit pieces and default HP/max HP.

- `src/simulator/README.md`
  - Documented the `Inputs` tab, keyboard shortcut `4`, structure HP controls, draggable pieces, and the existing topics used to feed behavior_tree.

- `docs/sentry/internal/simulator.md`
  - Documented the new tab, offline-only control-bus requirement, topic mapping, coordinate convention, and maintenance rule for future mock input controls.

## Decision Interface Mapping

- Structure state enters behavior_tree through existing blackboard fields:
  - `EnemyOutpostHealth`
  - `SelfOutpostHealth`
  - `SelfBaseHealth`
  - `EnemyBaseHealth`
- Unit state enters behavior_tree through existing robot snapshots:
  - `FriendRobots`
  - `EnemyRobots`
- Position drops use existing `PositionData` fields:
  - friend piece: `friendcarid`, `friendx`, `friendy`
  - enemy piece: `enemycarid`, `enemyx`, `enemyy`

## Validation

- `python3 -m py_compile $(rg --files src/simulator/simulator -g '*.py')`
- `PYTHONPATH=src/simulator python3 -m simulator.main --validate-only`
  - Result: `records=6 errors=0 warnings=0`
- `SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test --no-web-stream`
  - Result: exited 0. ALSA warnings are from the headless environment.
- `colcon build --packages-select simulator behavior_tree`
  - Result: both packages built successfully.
- `colcon test --packages-select simulator behavior_tree && colcon test-result --verbose`
  - Result: `0 tests, 0 errors, 0 failures, 0 skipped`.
- `python3 scripts/python/start.py --dry-run --no-view`
  - Result: generated command still runs `python3 -m simulator.start --offline-decision --mode regional`.
- `timeout 3 ... /usr/bin/python3 -m simulator.mock_inputs --hz 5 --control-file /tmp/simulator_test_control.jsonl`
  - Result: mock inputs started and exited cleanly after timeout. The sandbox printed Fast DDS UDP permission warnings, but the node initialized and no Python traceback remained.
- `git diff --check`
  - Result: no whitespace errors.

## Remaining Check

- Run a real GUI session and verify drag/drop feel, panel spacing, and whether the WSL/robot display can receive mouse drag events normally.
- Run offline live mode against behavior_tree and confirm a dragged unit appears in the next decision trace under `units.friend` or `units.enemy`.
