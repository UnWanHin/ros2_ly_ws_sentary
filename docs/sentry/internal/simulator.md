# Simulator Trace And Viewer

Updated: 2026-05-23

## Purpose

`simulator` is the maintained offline viewer for sentry decision behavior.
It replays JSONL rows written by `behavior_tree` and draws the current decision on a 2D field map.
The viewer shows whether the current navigation output is bridge `/goal_pose`, direct `UseXY` (`/ly/navi/goal_pos`), or goal-ID (`/ly/navi/goal`) mode.
The right panel shows live ROS topic values from `/goal_pose`, `/ly/navi/goal_pos_raw`, `/ly/navi/goal`, and `/ly/navi/speed_level` when started through the offline live wrapper.
In live mode, the current-goal marker and recent path prefer live `/goal_pose`; if that topic is absent, the viewer falls back to legacy `/ly/navi/goal_pos`, trace records, and labels the marker as `TRACE`.
The right panel is split into three tabs:

- `Decision`: current strategy, aim mode, stable decision output, decision intent, and recent semantic changes.
- `Events`: EventManager conditions, target freshness, hitable/reliable target sets, relative target bridge fields, referee resource state, energy/RFID data, and unit HP summaries.
- `Runtime`: live ROS output monitor, posture runtime, gimbal/fire-code state, and runtime guard state.
- `Inputs`: offline mock input controls for structure HP and draggable friend/enemy unit pieces.

This is for decision review and offline decision simulation. The viewer itself does not publish ROS topics.
In offline mode, it can send file-based control commands to `simulator.mock_inputs` for match clock and mock referee/unit inputs.

## Components

- Trace writer: `src/behavior_tree/src/DecisionTrace.cpp`
- Launch parameters: `decision_trace_enabled`, `decision_trace_file`, `decision_trace_every_n_ticks`
- Viewer package: `src/simulator`
- Viewer config: `src/simulator/config/default.yaml`
- Default map: `tools/maps/basemaps/buff_map_field.png`
- Rule-aware structure overlay source: `src/simulator/config/default.yaml` -> `structures`
- The structure overlay includes `RoadlandFollow.*`, `Recovery.*`, `MiniRoadland.*`, `CentralLeft.*`, and `SettleArea.*`; Recovery, MiniRoadland and CentralLeft are rendered as `special_zone`, not as FollowMode regions.
- Scripted route overlay source: `src/simulator/config/default.yaml` -> `scripted_path`
- Interactive mock input source: `src/simulator/config/default.yaml` -> `simulator_inputs`
- Field coordinate conversion: `src/simulator/simulator/field.py`
- Offline input state and command application: `src/simulator/simulator/interactive_inputs.py`
- Inputs tab rendering and hitboxes: `src/simulator/simulator/inputs_panel.py`
- File/API command bus schema: `src/simulator/simulator/control_bus.py`
- Energy mechanism debug fields are recorded in `decision_output`: `has_sentry_info`, `sentry_can_activate_energy`, and `energy_activate_confirm_pulse`.

## Trace Recording

Tracing is opt-in and stays off for normal competition runs.
`decision_trace_enabled` defaults to `false`; `decision_trace_file` is only used after that switch is explicitly enabled.

```bash
./scripts/start.sh nogate --mode league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

Direct launch:

```bash
ros2 launch behavior_tree sentry_all.launch.py mode:=league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

Leaving `decision_trace_enabled:=false` means no trace file is opened or written, even if a path is accidentally supplied.

Optional indexed wrapper (reads `src/behavior_tree/Scripts/ConfigJson`):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-configs
PYTHONPATH=src/simulator python3 -m simulator.start --mode league --bt-config league/chase_only_competition.json --entry nogate
```

Offline decision test (not replay, behavior_tree + mock topics only):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --bt-config regional_competition.json
```

Offline decision + pygame live view:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --live-view
```

Before launching live view, `simulator.start` auto-cleans stale old `simulator.main` viewer processes.

For regional super confrontation timing, use a 7-minute match clock (`420` seconds):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --live-view --match-duration-sec 420
```

One-command fixed regional wrapper:

```bash
python3 scripts/python/start.py
```

Offline mode keeps `/ly/game/is_start` gate enabled by default; press `Start` in viewer/web to publish game-start and enter match phase.
Offline mode enables `runtime_rearm_start_gate:=true` by default. After `Reset`, behavior_tree re-enters start gate,
holds safe-control, publishes Home navigation goal, and waits for next `Start`.
Offline mode also forces `NaviSetting.ToNavi=false` via a generated temp config, so `/ly/navi/goal_pos` remains official map coordinates.
Use `--keep-to-navi` only when you need transformed bridge output.

Live view now also serves the same pygame frame to HTTP by default (port from YAML `web_stream.port`, default `9000`):

- `http://127.0.0.1:9000/` for local browser
- `http://<your-ip>:9000/` for LAN browser

Override live-view web port:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --live-view --live-web-port 9010
```

With the default 100 Hz BT tick rate:

- `decision_trace_every_n_ticks:=1` records about 100 rows/s.
- `decision_trace_every_n_ticks:=5` records about 20 rows/s.
- `decision_trace_every_n_ticks:=10` records about 10 rows/s.

## Viewer

Run from source:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl
```

Open the sample:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main
```

Select a different map:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --map tools/maps/basemaps/RMUC2026_V1.2.0_topview_cad_field.png
```

Validate a trace without opening pygame:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --validate-only
```

Disable HTTP stream:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --no-web-stream
```

Change stream endpoint:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --web-host 0.0.0.0 \
  --web-port 9000
```

The viewer includes a structure layer for quick rule-context checks:

- Walls / major barrier blocks
- Energy mechanism (rule section `4.3.2.2`)
- Outpost stations (rule section `4.3.2.3`)

These are currently approximated from map artwork + rule figure alignment and are intended for offline simulator, not millimeter-level navigation constraints.

YAML default stream config is under `src/simulator/config/default.yaml`:

- `web_stream.enabled`
- `web_stream.host`
- `web_stream.port`
- `web_stream.fps`
- `web_stream.jpeg_quality`
- `map_tags.goal_tags_expanded`
- `map_tags.hover_goal_tags`
- `map_tags.hover_radius_px`
- `ros_monitor.state_file`
- `ros_monitor.poll_sec`
- `ros_monitor.stale_sec`

Offline match-control defaults are in the same YAML:

- `match_control.duration_sec`
- `match_control.rewind_step_sec`
- `match_control.forward_step_sec`
- `match_control.control_file`

Scripted route defaults are in the same YAML:

- `scripted_path.enabled`
- `scripted_path.goal_ids` / `scripted_path.points_cm`
- `scripted_path.speed_cmps`
- `scripted_path.side` / `scripted_path.loop`
- `scripted_path.show_future`

Live follow mode panel includes `Start`, `Pause`, `+10s`, `-10s`, `Reset` controls for offline mock match timing.
The HTTP page (`/`) provides the same control buttons and writes commands to `match_control.control_file`.
Match clock is rendered as a real-time countdown; pressing `Start` starts countdown immediately.
Live view opens the full viewer immediately, then updates when real trace rows arrive.
The right panel tab can be changed with mouse clicks or keyboard `1` / `2` / `3` / `4`.
The `Inputs` tab is only connected to behavior-tree decisions when running with `--offline-decision --live-view`;
in normal trace playback it remains a visual UI and reports that the live control bus is unavailable.

Interactive inputs use existing ROS contracts:

- Structure HP buttons publish `std_msgs/UInt16` through `simulator.mock_inputs` to `/ly/friend/op_hp`, `/ly/enemy/op_hp`, `/ly/friend/base_hp`, and `/ly/enemy/base_hp`.
- Dragged unit HP publishes through `gimbal_driver/msg/Health` on `/ly/friend/hp` or `/ly/enemy/hp`.
- Dragged unit positions publish through `gimbal_driver/msg/PositionData` on `/ly/position/data`.
- Viewer coordinates are official field centimeters with left-bottom origin. `PositionData.friendy` / `enemyy` are sent as `1500 - official_y` because `behavior_tree/src/SubscribeMessage.cpp` normalizes them back to official-map y.

The `0` HP value is valid for outpost/base topics, so behavior_tree accepts zero on those four structure HP subscribers.
Per-unit HP callbacks still follow the existing robot-health freshness behavior; removing a placed piece stops future mock position publishes, so downstream freshness timeout controls when that piece disappears from decisions.
`scripted_path` is disabled by default because it is a visual-only simulated route overlay, not a behavior_tree output.
When scripted path is enabled, route marker movement uses `speed_cmps` and elapsed match time.
By default, only the visited route plus the current target segment is drawn; set `show_future: true` to draw the full planned route.

After build:

```bash
colcon build --packages-select simulator
source install/setup.bash
ros2 run simulator simulator log/decision_trace.jsonl
```

## Trace Schema

Each line is one JSON object. Important top-level fields:

- `schema`: currently `ly_decision_trace_v1`
- `schema_version`: `2` adds `decision_output`, `decision_intent`, `events`, `target_state`, `navi_relative_target`, `gimbal`, and `runtime_guard`; the viewer still reads older rows without those fields
- `event`: `game_start`, `tick`, or `stop`
- `t`: seconds from `gameStartTime`
- `field_cm`: map frame and field size
- `competition_profile`, `strategy_mode`, `team`; current live strategy names include `LeagueSimple` and `Regional`
- `aim_mode`, `target_armor`, `target_state`
- `events`: EventManager semantic snapshot, including Buff/Outpost/RegionalDefense/resource/damage/navigation event flags
- `decision_output`: stable viewer-facing output model; includes final output kind, topic, goal ID, `goal_pos_cm`, publish flags, and bridge hints
- `decision_intent`: typed decision metadata for why the current output was selected; includes layer, reason, base goal ID, resolved goal ID, goal team, team-offset flag, priority, and detail
- `navi_goal`: ID, base ID, side, speed, publish flags, and `position_cm`
- `navi_relative_target`: chase/bridge relative target, including x/y/z, distance, yaw/pitch error, armor type, and aim mode
- `posture`: command, state, runtime desired/current/pending, reason
- `referee`: HP, ammo, time, outpost/base HP, RFID/RFID2 raw state, `rfid_match`, event-data energy/fortress gain-point state, and buff state
- `gimbal.fire_code.follow_mode`: semantic firecode bit4; old `hole_mode` naming should no longer be used in new traces/docs
- `units`: friend/enemy unit records with type, HP, distance, and `position_cm`
- `runtime_guard`: current fault and recovery state

The viewer uses coordinates in this order:

1. `decision_output.goal_pos_cm`, then legacy `navi_goal.position_cm`, and `units.*.position_cm` from trace.
2. `src/simulator/config/default.yaml` goal coordinates.
3. Optional `--points-json` map plugin coordinates when non-zero.

Terrain height is configured, not inferred. `config/default.yaml` contains an approximate 2D elevation overlay based on the visible map artwork. Treat it as a debug layer only until verified against official CAD/rule metadata.

Navigation goal ID `1` is `Base`, at red `(245, 750)` and blue `(2555, 750)`.
The simulator base HP structure badges use the same points.
Navigation goal ID `19` is `Highland`, a regional Highland compatibility/via point at red `(744, 1263)` and blue `(2056, 237)`. Current regional Highland compatibility arrival radius is 20 cm; during enter/via/leave transitions, behavior_tree may set `follow_mode` while routing through Highland or `CastleLeft1`. `CastleLeft1` is ID `5` at red `(510, 964)` and blue `(2290, 536)`; `CastleLeft2` is ID `20` at red `(831, 960)` and blue `(1969, 540)`.

## Maintenance Rules

When decision data changes, update the simulator in the same change:

- New/renamed trace fields: update `src/behavior_tree/src/DecisionTrace.cpp`, `src/simulator/simulator/model.py`, and `src/simulator/simulator/trace.py`.
- New output checks: update `src/simulator/simulator/validation.py`.
- New schema sample coverage: update `src/simulator/sample/sample_trace.jsonl` so parser and smoke tests exercise the current trace version.
- New point IDs or map assumptions: update `src/simulator/config/default.yaml`.
- New visual layer or style: update `src/simulator/simulator/viewer.py` and this document.
- New offline mock input control: update `src/simulator/simulator/interactive_inputs.py`, `src/simulator/simulator/inputs_panel.py`, `src/simulator/simulator/mock_inputs.py`, `src/simulator/simulator/control_bus.py`, `src/simulator/config/default.yaml`, and this document.
- New field coordinate or raw `PositionData` conversion: update `src/simulator/simulator/field.py` and simulator tests.
- New decision interface docs: update the current internal or regional document under `docs/sentry/` and add a dated implementation record under `docs/record/` when the change needs historical context.

Do not move this tool back under `tools/`; it is a maintained source package because it tracks the decision interface.
