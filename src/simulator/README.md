# simulator

Offline pygame viewer for sentry behavior-tree decision traces.

Updated: 2026-06-02

## Scope

- Runs offline from JSONL traces produced by `behavior_tree`.
- Normalizes trace rows into `DecisionOutput` records so future decision internals can change while the viewer stays centered on the final goal output.
- Treats `simulator.trace -> simulator.model.TraceRecord` as the stable simulator-facing contract; BT-only debug fields can change without viewer changes.
- Draws a 2D map, navigation goals, recent goal path, friendly/enemy units, HP bars, strategy, aim mode, posture, target, ammo, terrain overlays, and recent decision changes.
- Splits the right panel into Decision / Events / Runtime / Inputs / Layers tabs so schema v2 traces can expose decision intent, event conditions, detailed goal reach, navigation status/velocity, active aim-source freshness, relative target bridge data, gimbal fire-code state, runtime guard state, offline mock-input state, and map-layer/asset status without changing the trace producer.
- Shows whether the current decision output is bridge `/goal_pose`, direct `UseXY` (`/ly/navi/goal_pos`), or goal-ID (`/ly/navi/goal`) mode.
- Shows live ROS topic values from `/goal_pose`, `/ly/navi/goal_pos_raw`, `/ly/navi/goal`, `/ly/navi/speed_level`, `/ly/navi/should_rotate`, and `/ly/control/vel` when started through the offline live wrapper.
- In live mode, the current-goal marker and recent path prefer live `/goal_pose`; if that topic is absent, the viewer falls back to legacy `/ly/navi/goal_pos`, trace records, and labels the marker as `TRACE`.
- Draws rule-aware structure overlays (walls, energy mechanism, outposts) on top of the 2D map.
- Supports a YAML-configured scripted path overlay with configurable waypoint list and movement speed.
- Adds offline match-time control for super confrontation regional tests: start, pause, rewind, forward, reset.
- Adds an `Inputs` tab for offline decision simulation: click enemy/friend outpost/base HP controls, drag friend/enemy unit pieces onto the map, drag placed pieces to move them, and adjust placed-piece HP.
- In offline live mode, input controls write to the simulator command bus; `simulator.mock_inputs` publishes the resulting structure HP, unit HP, and unit positions into the existing behavior-tree input topics.
- Can preload friend/enemy units from a JSON/YAML unit scene with `--unit-scene`; the launcher passes the same file to the live viewer and mock publishers.
- Annotates each draggable/scene unit with its offline decision channel (`BT:HP,POS,UI`, `PUB:HP,POS noUI`, or `PUB:POS noUI`) so published mock facts are not confused with fields currently consumed by behavior_tree `UnitInfo`.
- Publishes offline referee, RFID, team-buff, sentry-info, gimbal, bullet, navigation-status, self-position, official target fallback, and optional external-aim inputs for behavior-tree decision tests.
- Uses normalized red/blue unit PNG assets from `src/simulator/assets/` when available, with circle-marker fallback when a sprite is missing.
- Shows asset manifest provenance and license/redistribution status in the `Layers` tab.
- Runs automated full-roster sprite visibility QA against a clean visual config, while keeping the normal full-roster smoke screenshot for manual UI/overlay review.
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

Run behavior-tree decision offline with built-in mock topic publishers (no internal vision nodes):

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

Preload units into the live viewer and mock topics:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-unit-scenes

PYTHONPATH=src/simulator python3 -m simulator.unit_scene \
  src/simulator/sample/unit_scene.json \
  --team red

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --unit-scene src/simulator/sample/unit_scene.json
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

### Offline Mock Decision Inputs

`simulator.start --offline-decision` launches `simulator.mock_inputs`, which publishes only existing behavior-tree input topics. It does not modify formal subscribers or message definitions.

Named mock presets are simulator-only overlays for common regional decision contexts. They expand into the same `--mock-*` flags below, require `--offline-decision`, and explicit CLI flags win over preset values:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-mock-presets

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset buff-ready \
  --live-view

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset uwb-fusion \
  --live-view

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset multi-unit-regional \
  --mock-ammo 120 \
  --live-view

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset full-roster-regional \
  --live-view
```

Current presets:

- `buff-ready`: buff target, sentry activation, buff energy, center RFID, and regional self position.
- `outpost-dead`: enemy outpost already dead with resource state still present.
- `nav-unreachable`: target-visible context with `/ly/navi/reachable=false`.
- `official-target-sentry`: `/ly/navi/target_official` fallback for Sentry armor ID `6`.
- `uwb-fusion`: opt-in `/ly/friend/uwb_pos` self-position fusion rehearsal with CLI coordinates in official field centimeters.
- `bullet-resource`: BulletInfo resource snapshot with speed, shoot data, projectile allowance, and gold coin fields.
- `multi-unit-regional`: preloads `src/simulator/sample/unit_scene.json` and publishes multi-unit HP/position context.
- `full-roster-regional`: preloads `src/simulator/sample/unit_scenes/full_roster.json` for full packaged unit-art, formal health-unit HP mapping, and placed-unit PositionData coverage.
- `low-resource`: low HP and low ammo recovery context.

Preset names describe mocked input state, not guaranteed behavior-tree outcomes. The active BT config still controls task enablement, target selection, chase, recovery, and publish gates.

Scripted mock sequences cover temporal decision context that static presets cannot express. They append existing simulator control-bus commands after `simulator.mock_inputs` starts; they do not publish ROS topics directly and do not add a parallel runtime input path.

```bash
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence --list-samples

PYTHONPATH=src/simulator python3 -m simulator.start --list-mock-sequences

PYTHONPATH=src/simulator python3 -m simulator.mock_sequence \
  src/simulator/sample/mock_sequences/regional_timed_context.json \
  --control-file /tmp/simulator_match_control.jsonl \
  --dry-run

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --mock-preset multi-unit-regional \
  --mock-sequence src/simulator/sample/mock_sequences/regional_timed_context.json \
  --live-view
```

Bundled sequence examples:

- `regional_timed_context.json`: starts the offline gate, sets match time, updates self position, moves friend/enemy Hero and enemy Infantry3 units, changes unit/self HP, lowers ammo, changes posture, and drops enemy outpost HP to zero.
- `buff_timeout_context.json`: starts from a buff-ready regional context, moves through posture/ammo/target HP changes, jumps match time, and removes the enemy Sentry target to rehearse timeout/fallback behavior.
- `low_resource_recovery_exit.json`: starts near base with low self HP/ammo, then restores HP/ammo, moves self position forward, and keeps an enemy Hero context to rehearse leaving recovery state.
- `official_target_fallback_companion.json`: companion script for `--mock-preset official-target-sentry`; opens the gate and changes nearby unit/self facts while the preset publishes `/ly/navi/target_official`.
- `multi_unit_target_priority_rehearsal.json`: multi-unit HP/position/resource rehearsal for watching target-priority behavior under changing enemy context.

Sequence timestamps use monotonic elapsed seconds from the scheduler start, not match-clock time; use explicit `set_time_left` actions when a test needs match-clock jumps.

Offline workflow playbooks pair presets, sequences, unit scenes, expected evidence, and verification commands for common regional debugging jobs:

```bash
PYTHONPATH=src/simulator python3 -m simulator.offline_workflow

PYTHONPATH=src/simulator python3 -m simulator.offline_workflow multi-unit-target-priority

PYTHONPATH=src/simulator python3 -m simulator.offline_workflow official-target-fallback --json
```

Current workflow IDs:

- `regional-buff-timeout`
- `regional-outpost-collapse`
- `official-target-fallback`
- `uwb-position-fusion`
- `bullet-info-resource-snapshot`
- `multi-unit-target-priority`
- `low-resource-recovery-exit`
- `full-roster-visual-inputs`

Each workflow prints a start command, preflight dry-runs, post-run trace checks, and evidence to inspect in the viewer, `/status.json`, or `simulator.unit_trace`.
The catalog is simulator-only guidance; it does not publish ROS topics and does not alter formal launch behavior.

Decision-input coverage is tracked as a simulator-owned catalog that maps formal behavior-tree inputs to mock flags, control-bus commands, trace fields, fixtures, workflows, and known gaps:

```bash
PYTHONPATH=src/simulator python3 -m simulator.decision_input_coverage

PYTHONPATH=src/simulator python3 -m simulator.decision_input_coverage unit_hp_position

PYTHONPATH=src/simulator python3 -m simulator.decision_input_coverage --json
```

The catalog covers match state, structure HP, unit HP/position, self position, navigation status/velocity, referee event/energy data, team buff, RFID, external aim target/result streams, official target fallback, gimbal/fire/posture state, and BulletInfo resource state.
It also records known non-complete areas: Drone and Infantry3 are visual/offline context for current formal UnitInfo, and optional external aim requires a sourced `sentry_msgs` workspace and matching BT config.
BulletInfo is now traceable through `bullet_info` rows, `/status.json.current_record.bullet_info`, the Runtime tab, and Foxglove export; current behavior-tree decisions still use the existing legacy ammo/speed gates unless the formal logic changes separately.


`--mock-armor` uses `ArmorType:DISTANCE_M`; distance is meters and is copied by `behavior_tree` into target distance evidence. `ArmorType::Sentry` is `6`, while draggable simulator Sentry units use `UnitType` ID `7`.

Common referee/resource knobs:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-ammo 35 \
  --mock-self-health 260 \
  --mock-enemy-outpost-health 0 \
  --mock-enemy-base-health 4200
```

Energy, team buff, and RFID examples:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-sentry-can-activate-energy true \
  --mock-event-self-small-energy-status 2 \
  --mock-team-buff-attack 1 \
  --mock-team-buff-remaining-energy 35 \
  --mock-rfid-center-gain-point true \
  --mock-rfid-self-outpost true
```

Navigation status and official target fallback examples:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-navi-reached false \
  --mock-navi-reachable true \
  --mock-navi-should-rotate true \
  --mock-self-position-x 1220 \
  --mock-self-position-y 760 \
  --mock-official-target-valid true \
  --mock-official-target-x 1505 \
  --mock-official-target-y 905 \
  --mock-official-target-armor-type 6
```

UWB self-position fusion can be rehearsed explicitly. The CLI takes official field centimeters, but the mock publisher sends `/ly/friend/uwb_pos` with the raw-y convention expected by the behavior-tree subscriber:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-publish-uwb-position true \
  --mock-uwb-position-x 1220 \
  --mock-uwb-position-y 760
```

External aim can also be mocked when the selected BT config reads the sentry external aim path:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-external-aim true \
  --mock-external-aim-target-id 6 \
  --mock-external-aim-yaw 7.5 \
  --mock-external-aim-pitch -1.2
```

This publishes `/ly/aim/armor_targets` and `/ly/aim/result` from `simulator.mock_inputs` using existing `sentry_msgs` message types. It is optional, simulator-only, and requires `sentry_msgs` Python imports from the sourced workspace.

Mock publisher coverage:

- `/ly/gimbal/angles`, `/ly/gimbal/posture`, `/ly/gimbal/firecode`, `/ly/gimbal/chassis`, `/ly/gimbal/capV`: gimbal state used by aim/posture/fire-code runtime logic.
- `/ly/game/event_data`: semantic energy/gain-point fields consumed by `EventManager`.
- `/ly/game/bullet`: bullet initial speed, shoot data, shooter number, launching frequency, projectile allowance, and remaining gold coin fields.
- `/ly/game/sentry/info`: `can_activate_energy_mechanism`.
- `/ly/team/buff`: recovery/cooling/defence/vulnerability/attack/remaining-energy fields.
- `/ly/game/rfid`: semantic RFID bool fields; `raw` is trace context, not decoded by behavior_tree.
- `/ly/navi/reached`, `/ly/navi/reachable`, `/ly/navi/should_rotate`, `/ly/navi/vel`, `/ly/navi/lower_head`: navigation status, velocity, and lower-head state.
- `/ly/navi/position`: self position in official field centimeters, no y inversion.
- `/ly/friend/uwb_pos`: optional UWB self position. Simulator CLI inputs are official field centimeters; published `data[1]` is raw y so behavior_tree reconstructs official y as `1500 - raw_y`.
- `/ly/navi/target_official`: `[x_cm, y_cm, armor_type]` fallback for enemy chase position.
- `/ly/aim/armor_targets`, `/ly/aim/result`: optional external aim mock when `--mock-external-aim true`.

### Control Bus Commands

The viewer, `/api/control`, and `simulator.mock_sequence` all write the same JSONL command bus. `simulator.mock_inputs` is the only offline process that converts those commands into ROS topics.

Supported command groups:

- Match clock: `start`, `pause`, `reset`, `rewind`, `forward`, `set_time_left`.
- Self state: `set_self_health`, `set_ammo`, `set_posture`, `set_self_position`.
- Structures: `set_structure_health` / `set_structure_hp`.
- Units: `set_unit`, `set_units`, `set_unit_hp`, `remove_unit`, `clear_units`.

`set_self_position` accepts `x`/`y`, `position_cm`, `position`, or `pos` payloads in official field centimeters. Invalid or non-finite coordinates are ignored instead of stopping the mock publisher.

Important ID note: simulator draggable unit IDs follow `UnitType` (`Drone=6`, `Sentry=7`), while `--mock-official-target-armor-type` follows behavior-tree `ArmorType` (`Sentry=6`). Drone art is visual/offline input context only; it is not a formal `Health.msg` or `UnitInfo` decision field.

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

HTTP endpoints:

- `GET /`: compact browser dashboard with the latest pygame frame, match-control buttons, stream readiness, trace/replay/current-decision summaries, simulator-input state, placed units, and alert lists.
- `GET /frame.jpg`: latest pygame frame as JPEG; returns `503` until the first frame is published.
- `GET /status.json`: machine-readable stream status, including `ready`, `has_frame`, `frame_id`, `frame_age_sec`, control-file availability, host/port, FPS, JPEG quality, trace metadata, validation status, replay state, selected-record summary, and simulator input state.
- `GET /healthz`: readiness probe; returns `200` after the first frame is available and `503` before then.
- `POST /api/control`: appends simulator command-bus JSONL commands when `match_control.control_file` is configured.

`/healthz` reports readiness, not ongoing frame freshness. Automation that needs to detect a stalled viewer should also inspect `frame_age_sec` from `/status.json`.
HTTP responses expose only whether control is enabled and the control-file basename; the resolved local path is kept server-side.

When a trace is loaded, `/status.json` also includes simulator metadata:

- `trace`: basename-only trace name, follow mode, record count, bad-line count, duration, and tick range.
- `validation`: the same `ly_simulator_validation_report_v1` payload used by `--validate-format json`.
- `replay`: play/pause state, replay speed, current record index, total records, active panel tab, match-clock state, and whether local controls are available.
- `current_record`: compact selected-record debug summary with tick/event, team, strategy, aim, target, goal, output topic/frame, official chase metadata, goal reach state, navigation status/velocity, relative target payload, intent, posture, referee resources, and runtime guard state.
- `simulator_inputs`: whether offline input controls are enabled, last command status, runtime self HP/ammo/posture/self-position, structure HP, placed units, unit palette, low-HP summaries, and `PositionData` raw-coordinate rows.

The metadata payload is strict JSON: non-finite floats are serialized as `null`, and resolved local trace/control-file paths are not exposed.
The browser page escapes the control-file basename before rendering it as HTML.

Quick local readiness check:

```bash
curl http://127.0.0.1:9000/status.json
curl -f http://127.0.0.1:9000/healthz
```

Optional browser visual QA starts a temporary stream with a synthetic simulator frame, opens the dashboard in Chromium through Playwright, checks desktop and narrow layouts, and writes screenshots to `/tmp/ly-simulator-web-visual`:

```bash
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check --json
```

If Playwright or its Chromium browser is not installed, the command reports `status: skip` by default. Install the optional browser tooling and make missing browser support a hard failure when you want a real-browser gate:

```bash
python3 -m pip install -r src/simulator/requirements-browser.txt
playwright install chromium
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check --require-browser
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
- `src/simulator/config/default.yaml` -> `simulator_inputs` (structure HP defaults, marker positions, draggable unit palette, optional `unit_scene_file`)
- `src/simulator/config/default.yaml` -> `scripted_path` (goal route / custom points / speed)

Headless smoke test:

```bash
SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test
```

Offline consistency validation:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --validate-only
```

The validation report includes PASS/WARN/FAIL status, trace shape summaries, stable issue codes, and per-issue suggestions.
Treat `FAIL` as a blocker before using a trace for replay review, Foxglove export, or offline decision regression evidence.
`WARN` reports are grouped by domain (`schema`, `trace`, `output`, `unit`, `runtime`, `referee`, `scenario`) so operators can scan likely root cause quickly.

For CI or scripts, use the JSON report format:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/route_churn_warning.jsonl \
  --validate-only \
  --validate-format json
```

The JSON report schema is `ly_simulator_validation_report_v1`.
It includes `status`, `summary`, `issues`, `issue_groups`, and `next_actions`.
`--validate-format json` is only valid with `--validate-only`; text remains the default format.

Scenario-level warning codes currently include:

- `scenario.route_churn`: rapid goal changes inside a short replay window.
- `scenario.stale_chase_target`: chase or relative-target output while all target freshness flags are false.
- `output.goal_reach_mismatch`: detailed goal reach state does not match the emitted decision output goal.
- `output.relative_target_frame_missing`: valid relative target output lacks a frame ID.
- `output.navi_velocity_scale`: traced raw navigation velocity has a non-positive conversion scale.
- `output.should_rotate_missing`: `/ly/navi/should_rotate` is marked fresh without a value.
- `runtime.posture_lag`: posture command/feedback remains pending for multiple seconds.
- `scenario.match_time_jump`: referee match time changes much faster or slower than trace time.
- `referee.missing_resource_state`: outpost/base HP fields needed for offline resource reasoning are absent.

## Scenario Fixtures

Bundled regression fixtures live under `src/simulator/sample/scenarios/`.
`manifest.json` names each scenario and records its expected validation status, schema/output coverage, intent layer/reason, goal, aim mode, and important event flags.

Current fixtures cover:

- `startup_home_hold`
- `target_acquisition`
- `buff_activation`
- `relative_target_bridge`
- `goal_id_output`
- `goal_pos_raw_bridge`
- `chase_goal_pos`
- `chase_goal_pos_raw_bridge`
- `outpost_attack`
- `low_resource_recovery`
- `start_gate_lifecycle`
- `route_churn_warning`
- `multi_unit_decision_context`

Validate a single fixture:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/relative_target_bridge.jsonl \
  --validate-only
```

Validate every bundled fixture against the manifest:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator \
  python3 -m pytest src/simulator/test/test_scenarios.py -q
```

Scripted mock sequence examples live under `src/simulator/sample/mock_sequences/`.
They are launch-time input scripts for offline decision runs, not replay fixtures; list them with `simulator.mock_sequence --list-samples` or `simulator.start --list-mock-sequences`, then validate a selected file with `simulator.mock_sequence --dry-run` or `simulator.start --offline-decision --mock-sequence ... --dry-run`.
Current examples are `regional_timed_context.json`, `buff_timeout_context.json`, `low_resource_recovery_exit.json`, `official_target_fallback_companion.json`, and `multi_unit_target_priority_rehearsal.json`.

Foxglove offline export:

```bash
python3 -m pip install -r src/simulator/requirements-foxglove.txt
PYTHONPATH=src/simulator python3 -m simulator.foxglove_export \
  log/decision_trace.jsonl \
  -o log/decision_trace.mcap
```

Or through the viewer entrypoint without opening pygame:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --export-foxglove log/decision_trace.mcap
```

This is offline post-processing only. Normal robot runs do not import the MCAP writer and do not pay this cost.

Run the standard simulator-only quality gate from the workspace root:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality
```

The default gate runs the simulator pytest suite, Python bytecode compilation, sample validation, expected-WARN route-churn JSON validation, unit-scene catalog/summary checks, offline workflow catalog checks, decision-input coverage catalog checks, pygame asset decode/scale/blit smoke coverage, multi-unit UnitInfo trace evidence, headless web smoke tests including the full-roster scene, clean visual asset QA, and simulator-related text whitespace checks, including untracked files.
The normal full-roster smoke writes `/tmp/ly-simulator-full-roster-smoke.png` so asset scale, overlay interaction, and panel layout can be inspected after a headless run.
The automated pixel-level sprite check uses `src/simulator/config/visual_asset_qa.yaml` and `/tmp/ly-simulator-full-roster-visual-qa.png`, where non-essential overlays are disabled so map labels, current-goal markers, and HP bars do not hide unit art and create false failures.
Add the optional real-browser dashboard visual gate when Playwright/Chromium is installed:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --with-browser-visual
```

Add package integration build when needed:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --with-build
```

Preview the exact commands without running them:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --dry-run --with-build
```

After building this package:

```bash
colcon build --packages-select simulator
source install/setup.bash
ros2 run simulator simulator log/decision_trace.jsonl
```

After `source install/setup.bash`, the same quality gate is available as:

```bash
ros2 run simulator simulator-quality --with-build
```

## Dependencies

```bash
python3 -m pip install -r src/simulator/requirements.txt
```

`PyYAML` is already present in the current workspace environment. `pygame` may need to be installed locally.
Browser visual QA is optional:

```bash
python3 -m pip install -r src/simulator/requirements-browser.txt
playwright install chromium
```

## Controls

- `Space`: play or pause
- `Left` / `Right`: step one record
- `PageUp` / `PageDown`: jump by `timeline.jump_step`
- `Home` / `End`: jump to start or end
- `+` / `-`: playback speed
- `L`: toggle point/unit labels
- `1` / `2` / `3` / `4` / `5`: switch right panel tabs: Decision, Events, Runtime, Inputs, Layers
- Mouse wheel over the right panel: scroll dense tab content without moving the timeline
- `Up` / `Down` / `PageUp` / `PageDown` / `Home` / `End` over the right panel: scroll that tab instead of seeking the trace
- Mouse click the timeline to seek
- In follow mode with offline mock control: click panel buttons for `Start`, `Pause`, `+10s`, `-10s`, `Reset`
- In the `Inputs` tab, click structure HP buttons to publish `/ly/friend/op_hp`, `/ly/enemy/op_hp`, `/ly/friend/base_hp`, and `/ly/enemy/base_hp` through `simulator.mock_inputs`
- Drag a friend/enemy unit piece from the palette onto the field; the mock publisher sends its HP through `/ly/friend/hp` or `/ly/enemy/hp` and its map position through `/ly/position/data`
- Use `--unit-scene path/to/unit_scene.json` to preload placed units before dragging or publishing starts
- Drag a placed piece again to move it; use the placed-piece HP buttons to update its published HP
- In the `Layers` tab, click layer buttons to toggle terrain, structures, simulator inputs, grid, goals, paths, units, HP bars, and recent changes.
- The HTTP page (`/`) now has the same control buttons and writes commands to `match_control.control_file`
- Keyboard shortcuts for offline mock control: `S` start, `P` pause, `[` rewind, `]` forward, `R` reset
- Match clock is a real-time countdown: `Start` begins countdown immediately, then syncs with incoming trace `time_left`
- Follow mode opens the full viewer immediately, then updates when real trace rows arrive.

## Unit Scene Import

`--unit-scene` accepts JSON or YAML. It can be a top-level list, `{ "units": [...] }`, or `{ "units": { "friend": [...], "enemy": [...] } }`.
Each unit needs `side`, `type` or `type_id`, and either `x`/`y` or `position_cm`.
List bundled scenes with `simulator.unit_scene --list-samples` or `simulator.start --list-unit-scenes`.
Inspect a scene before launch with `simulator.unit_scene <scene> --team red|blue`; the summary shows the resulting `Health.msg` field, `/ly/position/data` car ID/raw-y value, field-side mapping, and sprite availability.
Use `--json` for a machine-readable `ly_simulator_unit_scene_summary_v1` payload.
Scene summaries, Inputs-tab rows, and map labels include decision-channel badges:

- `BT:HP,POS,UI`: Hero, Engineer, Infantry1, Infantry2, and Sentry publish mock HP/position facts that the current behavior-tree UnitInfo path consumes and can emit back into trace `unit_info`.
- `PUB:HP,POS noUI`: Infantry3 publishes `Health.msg.reserve` plus `/ly/position/data`, but the current formal `RobotLists`/UnitInfo path does not consume it as a decision unit.
- `PUB:POS noUI`: Drone publishes `/ly/position/data` and is drawn as tactical/visual context, but it has no formal `Health.msg` field and no current UnitInfo output.

For enemy units, `BT:HP` also means the current HP field can affect target/resource reasoning in the selected BT config.
For friend units, the same badge means the fact is available on the formal mock input channel; the selected BT config still decides whether that friend-side fact changes behavior.

```json
{
  "units": [
    {"side": "friend", "type_id": 7, "hp": 400, "position_cm": {"x": 393, "y": 810}},
    {"side": "enemy", "type": "Hero", "hp": 200, "x": 2555, "y": 900}
  ]
}
```

When started through `simulator.start` or `scripts/python/start.py`, the same scene path is passed to `simulator.main` and `simulator.mock_inputs`, so the drawn units and published mock topics stay aligned.
`src/simulator/sample/unit_scenes/full_roster.json` is a visual QA scene that covers red/blue Hero, Engineer, Infantry, Sentry, and Drone sprites.
Infantry3 and Drone are intentionally included in that scene because they are useful tactical context and asset coverage, but they are not current formal BT UnitInfo decision units.

After recording an offline trace, compare a scene against formal `unit_info` evidence with:

```bash
PYTHONPATH=src/simulator python3 -m simulator.unit_trace \
  src/simulator/sample/scenarios/multi_unit_decision_context.jsonl \
  --unit-scene src/simulator/sample/unit_scenes/multi_unit_trace_contract.json
```

`simulator.unit_trace` requires traceable units from the scene to appear in `unit_info` with matching HP and official-map position. `Infantry3` and `Drone` are reported as skipped because the current formal `RobotLists`/`UnitInfoArray` contract excludes those unit types.

## Visual Assets

Simulator runtime uses the extracted asset tree, not the local source archive:

- `src/simulator/assets/manifest.yaml`
- `src/simulator/assets/units/red/*.png`
- `src/simulator/assets/units/blue/*.png`
- `src/simulator/assets/armor/*.png`

`config/default.yaml` controls sprite loading under `assets.enabled`, `assets.manifest`, `assets.unit_size_px`, `assets.trace_unit_size_px`, and `assets.drag_unit_size_px`.
Hero, Engineer, Infantry, Sentry, and Drone art is available for both red and blue. `Infantry1`, `Infantry2`, and `Infantry3` share the infantry sprite through manifest aliases.
Armor images from the same manifest are used in the right-panel target preview when available.
If an asset is missing or disabled, the viewer falls back to the previous circle-marker rendering.
The manifest records local provenance for `素材.zip`, including SHA-256 `8716faeaadce88422023023f923af5679c404241151b81d9a7a96b126fe963f4`.
The current license status is `unknown`, so redistribution is documented as `local_project_only_until_license_confirmed`.
The pygame `Layers` tab exposes asset catalog status, aliases, source archive name, import date, and license/redistribution status.
`simulator-quality` includes a headless pygame asset-render smoke test that decodes every manifest unit/armor PNG, scales it, and blits it to a transparent surface.

Pixel-level full-roster sprite visibility can be checked directly:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy \
  python3 -m simulator.main \
  --smoke-test \
  --web-port 9023 \
  --config src/simulator/config/visual_asset_qa.yaml \
  --unit-scene src/simulator/sample/unit_scenes/full_roster.json \
  --smoke-screenshot /tmp/ly-simulator-full-roster-visual-qa.png

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy \
  python3 -m simulator.visual_asset_qa \
  --screenshot /tmp/ly-simulator-full-roster-visual-qa.png \
  --unit-scene src/simulator/sample/unit_scenes/full_roster.json \
  --config src/simulator/config/visual_asset_qa.yaml \
  --team red \
  --json
```

Use the clean QA screenshot for automated pass/fail decisions.
Use `/tmp/ly-simulator-full-roster-smoke.png` for manual review of the normal UI because normal overlays may intentionally cover pieces.

## HP Zero Semantics

The `0` HP value is valid for outpost/base topics, so behavior_tree accepts zero on the four structure HP subscribers.
Self HP and per-unit HP are different in the current formal subscribers: `/ly/game/all.selfhealth` updates only when `selfhealth > 0`, and `/ly/friend/hp` / `/ly/enemy/hp` update formal Hero, Engineer, Infantry1, Infantry2, and Sentry HP only when each Health field is nonzero.
The simulator still allows `set_self_health` and `set_unit_hp` to publish `0` for visual/offline rehearsal, but validation warns with `referee.zero_self_hp_runtime_mismatch` or `unit.zero_hp_runtime_mismatch` when a trace row could be mistaken for formal BT zero-HP evidence.
Use positive low-health values for recovery/posture rehearsal unless the formal behavior_tree subscriber contract is changed separately.

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

The simulator boundary is the normalized `TraceRecord` model in `src/simulator/simulator/model.py`.
`src/simulator/simulator/trace.py` is the adapter from raw behavior-tree JSONL into that model.
Behavior-tree debug-only fields may be added under optional names such as `debug` or `bt_debug` without changing simulator code.
The ongoing offline-debug roadmap is documented in `docs/sentry/internal/simulator_offline_debug_roadmap.md`.

When the stable simulator-facing contract changes, update this package in the same PR:

- `src/behavior_tree/src/DecisionTrace.cpp`
- `src/simulator/simulator/model.py`
- `src/simulator/simulator/trace.py`
- `src/simulator/simulator/validation.py` if output checks need to change
- `src/simulator/simulator/foxglove_export.py` if Foxglove-exported fields change
- `src/simulator/config/default.yaml` if field, map, point, unit, layer, or style assumptions changed
- `src/simulator/simulator/field.py` if field size, coordinate frame, or raw `PositionData` y-conversion changes
- `src/simulator/simulator/interactive_inputs.py` if structure/unit mock input state or command payloads change
- `src/simulator/simulator/inputs_panel.py` if Inputs tab controls or hitboxes change
- `src/simulator/simulator/control_bus.py`, `mock_inputs.py`, and `web_stream.py` if simulator command bus commands change
- `src/simulator/simulator/decision_input_coverage.py` when formal decision inputs, mock flags, trace evidence, fixtures, workflows, or known simulator-only gaps change
- `src/simulator/test/` when stable trace, export, simulator state, command, or coordinate assumptions change
- `src/simulator/sample/scenarios/manifest.json` and the relevant fixture when stable decision scenarios or coverage expectations change
- `src/simulator/simulator/offline_workflow.py` when adding/removing named offline decision workflows or changing recommended preset/sequence/unit-scene pairings
- `docs/sentry/internal/simulator.md` or a newer simulator document

Do not rely on `tools/maps/map_plugin.json` for required point positions until it has non-zero calibrated coordinates. The viewer currently uses trace positions first, YAML points second, and optional map plugin coordinates third.
Terrain height is an approximate YAML overlay, not inferred from the image at runtime. Use official CAD/rules or measured map metadata before using it for path-cost decisions.
