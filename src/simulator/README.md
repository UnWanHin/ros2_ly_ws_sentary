# simulator

Offline pygame viewer for sentry behavior-tree decision traces.

Updated: 2026-05-23

## Scope

- Runs offline from JSONL traces produced by `behavior_tree`.
- Normalizes trace rows into `DecisionOutput` records so future decision internals can change while the viewer stays centered on the final goal output.
- Draws a 2D map, navigation goals, recent goal path, friendly/enemy units, HP bars, strategy, aim mode, posture, target, ammo, terrain overlays, and recent decision changes.
- Splits the right panel into Decision / Events / Runtime tabs so schema v2 traces can expose decision intent, event conditions, target freshness, relative target bridge data, gimbal fire-code state, and runtime guard state without changing the trace producer.
- Shows whether the current decision output is bridge `/goal_pose`, direct `UseXY` (`/ly/navi/goal_pos`), or goal-ID (`/ly/navi/goal`) mode.
- Shows live ROS topic values from `/goal_pose`, `/ly/navi/goal_pos_raw`, `/ly/navi/goal`, and `/ly/navi/speed_level` when started through the offline live wrapper.
- In live mode, the current-goal marker and recent path prefer live `/goal_pose`; if that topic is absent, the viewer falls back to legacy `/ly/navi/goal_pos`, trace records, and labels the marker as `TRACE`.
- Draws rule-aware structure overlays (walls, energy mechanism, outposts) on top of the 2D map.
- Supports a YAML-configured scripted path overlay with configurable waypoint list and movement speed.
- Adds offline match-time control for super confrontation regional tests: start, pause, rewind, forward, reset.
- Adds an `Inputs` tab for offline decision simulation: click enemy/friend outpost/base HP controls, drag friend/enemy unit pieces onto the map, drag placed pieces to move them, and adjust placed-piece HP.
- In offline live mode, input controls write to the simulator command bus; `simulator.mock_inputs` publishes the resulting structure HP, unit HP, and unit positions into the existing behavior-tree input topics.
- Uses `tools/maps/basemaps/buff_map_field.png` by default, but the map is selectable.
- Keeps window size, field size, colors, point coordinates, terrain overlays, unit styles, layer switches, and web stream defaults in `config/default.yaml`.

## Record

Tracing is disabled unless `decision_trace_enabled:=true`.

```bash
./scripts/start.sh nogate --mode league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

At 100 Hz BT tick rate, `decision_trace_every_n_ticks:=5` records about 20 rows per second.

### Which Decision Config Is Running

`./scripts/start.sh ... --mode league` resolves to:

- `bt_config_file := Scripts/ConfigJson/league_competition.json`

`--mode regional` resolves to `regional_competition.json`, and `--mode showcase` resolves to `regional/debug/showcase_competition.json`.

If you pass `bt_config_file:=...`, that explicit file overrides the mode default.

### Indexed Recorder Start Script

Use the wrapper to index `src/behavior_tree/Scripts/ConfigJson` and start trace recording:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-configs
```

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --mode league \
  --bt-config league/chase_only_competition.json \
  --entry nogate
```

In normal mode (without `--offline-decision`), the wrapper enables trace and prints the exact forwarded command.
It writes trace to `log/decision_trace_<timestamp>.jsonl` by default.

Auto-open pygame after record exits:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --mode league --play
```

Open pygame while decision is running:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --mode league --live-view
```

When live view starts, it also opens HTTP frame streaming by default (port from YAML `web_stream.port`, default `9000`):

- `http://127.0.0.1:9000/` (local browser)
- `http://<your-ip>:9000/` (LAN browser)

Override live-view stream port from wrapper:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --mode league --live-view --live-web-port 9010
```

### Offline Decision Test (Not Replay)

Run behavior-tree decision offline with built-in mock topic publishers (no detector/gimbal/predictor/outpost/buff nodes):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode league \
  --bt-config league/chase_only_competition.json
```

Add pygame live view in offline mode:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode league \
  --live-view
```

Before launching live view, `simulator.start` now auto-cleans stale old `simulator.main` viewer processes.

For super confrontation regional logic, keep `--mode regional` and use a 7-minute match clock (`420` seconds):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --match-duration-sec 420
```

One-command fixed regional wrapper:

```bash
python3 scripts/python/start.py
```

Offline mode now keeps `/ly/game/is_start` gate enabled by default (no `debug_bypass_is_start`).
Use the viewer/web `Start` control to publish game-start and begin countdown.
Offline mode also enables `runtime_rearm_start_gate:=true` by default:
`Reset` closes the gate again, holds safe-control, publishes Home navigation goal, and waits for next `Start`.

By default, offline mode also writes a temporary BT config with `NaviSetting.ToNavi=false`,
so goal position publish uses official map coordinates (no transformed bridge output).
Use `--keep-to-navi` if you explicitly need transformed output.

By default, `--offline-decision` keeps trace off. Add `--trace-on` if you want JSONL output at the same time:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode league \
  --trace-on
```

You can choose which target stream is active in offline test:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-target predictor
```

## Playback From Source Tree

```bash
python3 -m simulator.main log/decision_trace.jsonl
```

If `PYTHONPATH` does not include this package:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl
```

Open the bundled sample:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main
```

Use a different map or config:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --map tools/maps/basemaps/RMUC2026_V1.2.0_topview_cad_field.png \
  --config src/simulator/config/default.yaml
```

Disable web streaming:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --no-web-stream
```

Change stream host/port:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --web-host 0.0.0.0 \
  --web-port 9000
```

Stream defaults can be configured in YAML:

- `src/simulator/config/default.yaml` -> `web_stream.enabled`
- `src/simulator/config/default.yaml` -> `web_stream.host`
- `src/simulator/config/default.yaml` -> `web_stream.port`
- `src/simulator/config/default.yaml` -> `web_stream.fps`
- `src/simulator/config/default.yaml` -> `web_stream.jpeg_quality`
- `src/simulator/config/default.yaml` -> `structures` (walls/energy mechanism/outposts)
- `src/simulator/config/default.yaml` -> `map_tags` (hover/expanded goal ID tags)
- `src/simulator/config/default.yaml` -> `ros_monitor` (live ROS topic state file)
- `src/simulator/config/default.yaml` -> `match_control` (duration/control-file/button step)
- `src/simulator/config/default.yaml` -> `simulator_inputs` (structure HP defaults, marker positions, draggable unit palette)
- `src/simulator/config/default.yaml` -> `scripted_path` (goal route / custom points / speed)

Headless smoke test:

```bash
SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test
```

Offline consistency validation:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --validate-only
```

After building this package:

```bash
colcon build --packages-select simulator
source install/setup.bash
ros2 run simulator simulator log/decision_trace.jsonl
```

## Dependencies

```bash
python3 -m pip install -r src/simulator/requirements.txt
```

`PyYAML` is already present in the current workspace environment. `pygame` may need to be installed locally.

## Controls

- `Space`: play or pause
- `Left` / `Right`: step one record
- `PageUp` / `PageDown`: jump by `timeline.jump_step`
- `Home` / `End`: jump to start or end
- `+` / `-`: playback speed
- `L`: toggle point/unit labels
- `1` / `2` / `3` / `4`: switch right panel tabs: Decision, Events, Runtime, Inputs
- Mouse click the timeline to seek
- In follow mode with offline mock control: click panel buttons for `Start`, `Pause`, `+10s`, `-10s`, `Reset`
- In the `Inputs` tab, click structure HP buttons to publish `/ly/friend/op_hp`, `/ly/enemy/op_hp`, `/ly/friend/base_hp`, and `/ly/enemy/base_hp` through `simulator.mock_inputs`
- Drag a friend/enemy unit piece from the palette onto the field; the mock publisher sends its HP through `/ly/friend/hp` or `/ly/enemy/hp` and its map position through `/ly/position/data`
- Drag a placed piece again to move it; use the placed-piece HP buttons to update its published HP
- The HTTP page (`/`) now has the same control buttons and writes commands to `match_control.control_file`
- Keyboard shortcuts for offline mock control: `S` start, `P` pause, `[` rewind, `]` forward, `R` reset
- Match clock is a real-time countdown: `Start` begins countdown immediately, then syncs with incoming trace `time_left`
- Follow mode opens the full viewer immediately, then updates when real trace rows arrive.

## Scripted Path Config

Use `scripted_path` in `src/simulator/config/default.yaml`:

```yaml
scripted_path:
  enabled: false
  side: auto      # auto/red/blue
  speed_cmps: 320
  loop: true
  show_future: false
  label: SimPath
  points_cm: []   # if non-empty, this is used first
  goal_ids: [0, 6, 10, 14, 17, 10, 6]
```

`scripted_path` is a visual-only simulated route overlay, not a behavior_tree output and not `/goal_pose`.
When enabled, the marker moves by `speed_cmps` and match elapsed time. By default, only the visited route plus the current target segment is drawn; set `show_future: true` to draw the full planned route.
In follow offline mode, elapsed time comes from match clock (`Start/Pause/Rewind/Forward/Reset`).

## Maintenance Contract

When a behavior-tree decision output changes, update this package in the same PR:

- `src/behavior_tree/src/DecisionTrace.cpp`
- `src/simulator/simulator/model.py`
- `src/simulator/simulator/trace.py`
- `src/simulator/simulator/validation.py` if output checks need to change
- `src/simulator/config/default.yaml` if field, map, point, unit, layer, or style assumptions changed
- `src/simulator/simulator/field.py` if field size, coordinate frame, or raw `PositionData` y-conversion changes
- `src/simulator/simulator/interactive_inputs.py` if structure/unit mock input state or command payloads change
- `src/simulator/simulator/inputs_panel.py` if Inputs tab controls or hitboxes change
- `src/simulator/simulator/control_bus.py`, `mock_inputs.py`, and `web_stream.py` if simulator command bus commands change
- `src/simulator/test/` when simulator state, command, or coordinate assumptions change
- `docs/sentry/internal/simulator.md` or a newer simulator document

Do not rely on `tools/maps/map_plugin.json` for required point positions until it has non-zero calibrated coordinates. The viewer currently uses trace positions first, YAML points second, and optional map plugin coordinates third.
Terrain height is an approximate YAML overlay, not inferred from the image at runtime. Use official CAD/rules or measured map metadata before using it for path-cost decisions.
