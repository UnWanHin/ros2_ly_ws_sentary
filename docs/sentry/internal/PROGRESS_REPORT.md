# Simulator Offline Debug Progress Report

Updated: 2026-06-02

## Completed Work

This milestone strengthened `src/simulator` as an offline behavior-tree debugging workbench without changing the formal ROS2 runtime chain.

- Added actionable validation reports with stable issue codes, grouped domains, PASS/WARN/FAIL status, schema counters, output-kind counters, and per-issue suggestions.
- Added warning-only scenario diagnostics for route churn, stale chase targets, posture lag, referee match-time jumps, and missing outpost/base HP state.
- Added a compact scenario fixture suite under `src/simulator/sample/scenarios/` with a manifest-driven contract.
- Added the first multi-record transition fixture for reset/start gate/Home hold to active regional patrol goal.
- Added the first expected-WARN fixture for route churn, with warning-code assertions in the manifest test.
- Added web stream health/readiness endpoints so remote tools can inspect the simulator without parsing pixels.
- Added `/status.json` trace, validation, replay, selected-record, and simulator-input metadata for automation and remote debugging.
- Added a compact browser debug dashboard on `GET /` that renders the existing frame stream plus trace, replay, current decision, simulator-input, placed-unit, and alert summaries from `/status.json`.
- Added optional browser visual QA via `simulator.web_visual_check`: it starts a temporary simulator stream, publishes a synthetic pygame frame and status payload, opens the dashboard in Playwright Chromium at desktop and narrow viewports, checks core layout/readiness/overflow signals, and writes screenshots when browser tooling is installed.
- Sanitized web stream control status so LAN clients no longer receive resolved local control-file paths.
- Hardened web status serialization: `/status.json` is strict JSON, non-finite floats become `null`, and the browser page escapes the control-file basename before rendering HTML.
- Added machine-readable validation JSON output for CI and scripts while preserving the default text report.
- Added `simulator-quality` as the standard simulator-only quality gate with dry-run, keep-going, text-whitespace, smoke, validation, pytest, py_compile, optional browser visual QA, and optional package-build steps.
- Normalized provided unit art into `src/simulator/assets/` with a manifest-driven red/blue sprite catalog and packaged assets for install.
- Added sprite rendering for trace units, offline input units, and drag previews, with circle-marker fallback when art is disabled or missing.
- Extended offline input state and fixtures to cover Hero, Engineer, Infantry1/2/3, Sentry, and visual Drone pieces while preserving formal `Health.msg` fields.
- Added `simulator.unit_scene`, a simulator-only scene inspection tool that lists bundled unit scenes and summarizes `Health.msg`, `/ly/position/data`, field-side, and sprite availability before launch.
- Added `src/simulator/sample/unit_scenes/full_roster.json`, a full red/blue Hero, Engineer, Infantry, Sentry, and Drone visual QA scene for checking the provided art and mock input mapping.
- Added per-unit decision-channel metadata so scene summaries, Inputs-tab rows, and map labels distinguish `BT:HP,POS,UI`, `PUB:HP,POS noUI`, and `PUB:POS noUI` units.
- Tightened full-roster wording so Drone is documented as PositionData-only visual/offline context, and Infantry3 is documented as published `Health.msg.reserve`/PositionData but not consumed by the current formal UnitInfo path.
- Added `simulator.unit_trace`, a simulator-only trace evidence checker that compares a unit scene against behavior-tree `unit_info` HP/position rows and reports visual-only units as explicit skips.
- Added `src/simulator/sample/unit_scenes/multi_unit_trace_contract.json`, a scene aligned with `multi_unit_decision_context.jsonl` for formal UnitInfo evidence checks.
- Added armor-asset parsing and a right-panel target preview so the existing armor images in the extracted asset tree are used by the pygame UI.
- Added asset provenance fields for the local `素材.zip` source archive, including SHA-256, import date, unknown license status, and local-project-only redistribution status.
- Added pygame `Layers` tab controls for terrain, structures, simulator inputs, grid, goals, paths, units, HP bars, and recent changes, plus asset catalog/provenance status in the UI.
- Added right-panel body scrolling with per-tab scroll offsets, mouse-wheel support, panel-local keyboard scrolling, clipping, and a compact scrollbar so dense Decision/Events/Runtime/Inputs/Layers content can be inspected at fixed window sizes.
- Extracted right-panel scroll bookkeeping into `PanelScrollState`, with pure unit tests for clamping, invalid values, per-tab offsets, and the current `content - body + padding` scroll-range semantics.
- Added pygame-level asset smoke coverage that decodes, scales, and blits every manifest unit/armor PNG instead of only checking manifest paths and PNG headers.
- Added `--smoke-screenshot` for `simulator.main --smoke-test`; the full-roster quality smoke writes `/tmp/ly-simulator-full-roster-smoke.png` for manual visual QA of sprite scale, overlap, and panel layout after a headless run.
- Added `simulator.visual_asset_qa` plus `src/simulator/config/visual_asset_qa.yaml` so the quality gate can render a clean full-roster screenshot and pixel-check every packaged unit sprite without normal map overlays causing false failures.
- Added formal `unit_info` trace parsing, status payloads, Events-tab summaries, and validation warnings for missing/out-of-bounds UnitInfo positions and reliable-position evidence gaps.
- Added `multi_unit_decision_context`, a fixture that captures multi-unit HP, fresh UnitInfo positions, area/source evidence, RFID, team buff, and resource state.
- Added formal decision-data trace coverage for `goal_reach_state`, `navi_status`, `navi_velocity`, relative target `frame_id`, and official chase metadata.
- Extended web/current-record status payloads, pygame Events/Runtime panels, validation, and Foxglove export to expose the same formal navigation/reach/velocity/chase fields.
- Added scenario fixtures for all currently traced output kinds: `goal_id`, `goal_pos`, `goal_pos_raw_bridge`, `relative_target_bridge`, `chase_goal_pos`, and `chase_goal_pos_raw_bridge`.
- Hardened `simulator.unit_trace` so skipped bad JSONL rows make trace-evidence status fail instead of silently passing.
- Extended `simulator.mock_inputs` to publish referee/event/sentry/RFID/team-buff/gimbal/bullet/navigation/self-position/official-target fallback inputs on existing behavior-tree topics.
- Added optional external aim mocks for `/ly/aim/armor_targets` and `/ly/aim/result` without enabling them by default.
- Extended `simulator.start` so `--offline-decision` can forward those mock decision inputs from one wrapper command.
- Added named `--mock-preset` overlays for common offline decision input states: buff-ready, outpost-dead, nav-unreachable, official-target-sentry, uwb-fusion, bullet-resource, detector-armors, multi-unit-regional, full-roster-regional, and low-resource.
- Added `--list-mock-presets` with expanded overlay flags so operators can audit preset inputs before launching.
- Added a simulator-only scripted mock sequence runner for temporal control-bus actions such as start, time-left changes, self HP/ammo/posture/self-position changes, unit HP/position updates, and structure HP transitions.
- Added `src/simulator/sample/mock_sequences/regional_timed_context.json`, `buff_timeout_context.json`, and `low_resource_recovery_exit.json` as timed regional input scripts.
- Added `official_target_fallback_companion.json` and `multi_unit_target_priority_rehearsal.json` as additional timed input scripts that stay within the existing control-bus command set.
- Extended `simulator.start --offline-decision` with `--mock-sequence` and dry-run sequence validation before launching child processes.
- Added `simulator.mock_sequence --list-samples` and `simulator.start --list-mock-sequences` so operators can discover bundled timed input scripts with descriptions and action counts before launching.
- Added `simulator.offline_workflow`, a simulator-only playbook catalog that pairs presets, sequences, unit scenes, expected evidence, preflight commands, and post-run checks for common offline decision debugging jobs.
- Added `simulator.decision_input_coverage`, a simulator-only coverage catalog that maps formal behavior-tree decision input groups to mock flags, command-bus commands, trace fields, fixtures, workflows, viewer surfaces, and known gaps.
- Added opt-in UWB self-position mocking for `/ly/friend/uwb_pos`; CLI inputs stay in official field centimeters while the publisher sends the raw-y convention consumed by behavior_tree.
- Added a discoverable `uwb-fusion` preset and `uwb-position-fusion` offline workflow so UWB position-fusion rehearsal appears in the same preset/workflow catalogs as the other decision-input checks.
- Added trace-only BulletInfo evidence for cached `/ly/game/bullet` facts, including initial speed, shoot data, projectile allowance, and remaining gold coin fields.
- Added normalized BulletInfo parser/model/status/Foxglove/viewer support plus a `bullet_info_resource` fixture, `bullet-resource` preset, and `bullet-info-resource-snapshot` offline workflow.
- Added simulator-only `/ly/detector/armors` publishing with single-target and repeatable multi-target CLI support so detector armor-list decisions can be rehearsed while the detector node remains disabled.
- Added the `detector-armors` preset, `detector-armors-target-list` workflow, and `detector_armors_target_list` scenario fixture with manifest assertions for `target_armor.distance_m` and `target_state.hitable_targets`.
- Added a JSON-safe `SimulatorInputState.snapshot()` and exposed it through `/status.json` as `simulator_inputs`.
- Improved the Inputs tab with sprite-backed palette chips, placed-unit rows, and HP bars so the multi-unit offline state is easier to scan.
- Fixed mock publisher startup ordering for posture initialization and made dynamic `set_self_position` ignore invalid/non-finite payloads instead of crashing.
- Added regression tests for validation output, diagnostic codes, scenario fixture coverage, and packaged fixture inclusion.
- Updated simulator docs and roadmap to describe trace contracts, scenario fixtures, validation diagnostics, and current verification workflow.

## Systems Modified

- `src/simulator/simulator/validation.py`: validation issue model, grouped report formatting, trace summaries, and offline scenario diagnostics.
- `src/simulator/test/test_validation.py`: focused regression tests for validation reports and diagnostic warnings.
- `src/simulator/simulator/main.py`: `--validate-format text|json` CLI output selection, basename-only trace status metadata, and finite `--web-fps` validation.
- `src/simulator/test/test_validation_cli.py`: subprocess tests for machine-readable validation output, CLI error handling, and non-finite FPS rejection.
- `src/simulator/test/test_scenarios.py`: manifest-driven scenario fixture regression tests.
- `src/simulator/sample/scenarios/`: named compact traces plus `manifest.json`.
- `src/simulator/setup.py`: package data inclusion for scenario fixtures.
- `src/simulator/requirements-browser.txt`: optional Playwright dependency file for browser visual QA.
- `src/simulator/simulator/web_stream.py`: richer status snapshot, `/healthz`, strict JSON metadata merging, HTML-escaped control label, and compact browser dashboard rendering.
- `src/simulator/simulator/web_visual_check.py`: optional Playwright-backed browser screenshot and layout smoke check for the HTTP dashboard, with clean skip semantics when browser tooling is absent.
- `src/simulator/simulator/viewer.py`: replay, selected-record, and simulator-input metadata snapshot for web status.
- `src/simulator/simulator/quality.py`: simulator-only quality gate orchestration, including opt-in `--with-browser-visual`.
- `src/simulator/test/test_web_stream.py`: focused HTTP tests for status, health, frame, control endpoints, strict JSON serialization, metadata override prevention, HTML escaping, control-path redaction, and dashboard HTML structure.
- `src/simulator/test/test_web_visual_check.py`: unit-level coverage for missing-browser skip behavior, required-browser failure behavior, fake-browser screenshot capture, and layout-finding diagnostics.
- `src/simulator/test/test_trace_contract.py`: selected-record status payload and path-redaction coverage.
- `src/simulator/test/test_quality_cli.py`: quality-gate dry-run, unit-scene gate inclusion, optional browser visual gate inclusion, build inclusion, repo-root validation, text-whitespace, and failure propagation tests.
- `src/simulator/assets/`: normalized unit and armor art extracted into simulator-owned runtime assets.
- `src/simulator/simulator/assets.py`: manifest parser and unit sprite catalog.
- `src/simulator/simulator/unit_trace.py`: scene-to-trace UnitInfo evidence checker.
- `src/simulator/test/test_assets.py`: asset manifest, alias, and pygame decode/scale/blit coverage.
- `src/simulator/simulator/main.py`: `--smoke-screenshot` output for smoke-test visual QA frames.
- `src/simulator/test/test_unit_trace.py`: UnitInfo evidence report, JSON CLI, and mismatch failure coverage.
- `src/behavior_tree/src/DecisionTrace.cpp`: trace-only emission of detailed navigation reach, navigation status, velocity conversion, relative target frame, official chase metadata, and cached BulletInfo facts.
- `src/simulator/simulator/model.py`, `src/simulator/simulator/trace.py`: normalized formal navigation/reach/velocity/chase fields in `TraceRecord`.
- `src/simulator/simulator/foxglove_export.py`: full normalized decision frame and metrics export for formal navigation/reach/velocity/chase and BulletInfo fields.
- `src/simulator/simulator/validation.py`: consistency warnings for goal-reach mismatch, missing relative-target frame, invalid velocity scale, and missing should-rotate values.
- `src/simulator/simulator/control_bus.py`: shared command schema for match clock, self state, structures, and units.
- `src/simulator/simulator/interactive_inputs.py`: offline input-state model, unit-scene parsing, runtime self-state tracking, and JSON-safe status snapshots.
- `src/simulator/simulator/inputs_panel.py`: sprite-backed dense Inputs tab rendering and placed-unit HP controls.
- `src/simulator/simulator/mock_inputs.py`: offline ROS mock publishers for EventData, SentryInfo, RfidStatus, BuffData, gimbal state, BulletInfo, detector Armors, navigation state, self position, optional UWB self position, official target fallback, optional external aim, and dynamic command-bus updates.
- `src/simulator/simulator/mock_sequence.py`: timed scheduler for existing simulator control-bus commands.
- `src/simulator/simulator/offline_workflow.py`: guided offline decision workflow catalog for common regional debugging jobs, including BulletInfo resource snapshots and detector armor-list target rehearsals.
- `src/simulator/simulator/decision_input_coverage.py`: formal decision-input coverage catalog for mock/topic/trace/fixture/workflow evidence, including UWB self-position, BulletInfo, and detector armor-list coverage.
- `src/simulator/simulator/unit_scene.py`: bundled unit-scene catalog and JSON/text summary for mock Health/PositionData/sprite/decision-channel mapping.
- `src/simulator/simulator/viewer.py`: sprite-backed units, armor target preview, formal navigation/reach/velocity rows, interactive Layers tab, and scrollable right-panel body.
- `src/simulator/simulator/visual_asset_qa.py`: full-roster screenshot-to-sprite pixel visibility QA.
- `src/simulator/config/visual_asset_qa.yaml`: clean layer preset used only for automated sprite visibility checks.
- `src/simulator/simulator/panel_scroll.py`: isolated right-panel scroll state and clamping helper.
- `src/simulator/test/test_panel_scroll.py`: pure scroll-state regression tests.
- `src/simulator/simulator/start.py`: wrapper CLI forwarding for mock referee/resource/RFID/navigation/UWB/target inputs, named offline mock preset overlays, scripted mock sequence process orchestration, and bundled sequence/unit-scene catalog listing.
- `src/simulator/test/test_mock_inputs_cli.py`: parser and helper tests for mock decision-input knobs.
- `src/simulator/test/test_mock_sequence.py`: parser, validation, ordering, reset, and JSONL emission coverage for scripted mock sequences.
- `src/simulator/test/test_offline_workflow.py`: workflow catalog validation, generated-command coverage, CLI output, and entry-point packaging checks.
- `src/simulator/test/test_decision_input_coverage.py`: decision-input catalog validation, partial-gap reporting, JSON/detail CLI coverage, and entry-point packaging checks.
- `src/simulator/test/test_start_mock_command.py`: dry-run command coverage for wrapper-to-mock forwarding, preset expansion, explicit override precedence, preset listing, and sequence validation.
- `src/simulator/simulator/model.py`, `src/simulator/simulator/trace.py`, `src/simulator/simulator/viewer.py`: formal UnitInfo parsing and display.
- `src/simulator/sample/unit_scene.json`: richer multi-unit default scene including other unit art.
- `src/simulator/sample/unit_scenes/full_roster.json`: full roster visual QA scene for all packaged unit sprites.
- `src/simulator/sample/scenarios/multi_unit_decision_context.jsonl`: multi-unit decision-context fixture.
- `src/simulator/sample/scenarios/bullet_info_resource.jsonl`: BulletInfo resource-state fixture.
- `src/simulator/sample/scenarios/detector_armors_target_list.jsonl`: detector armor-list fixture for target distance and hitable-target evidence.
- `src/simulator/sample/scenarios/goal_id_output.jsonl`, `goal_pos_raw_bridge.jsonl`, `chase_goal_pos.jsonl`, `chase_goal_pos_raw_bridge.jsonl`: compact fixtures for missing output-kind coverage.
- `src/simulator/sample/mock_sequences/`: timed regional mock input scripts for general regional context, buff timeout/fallback, low-resource recovery exit, official-target fallback companion, and multi-unit target-priority rehearsal.
- `src/simulator/setup.py`: `simulator-quality`, `simulator-web-visual-check`, `simulator-mock-sequence`, `simulator-offline-workflows`, `simulator-decision-input-coverage`, `simulator-unit-scene`, and `simulator-unit-trace` console script entry points plus sequence/unit-scene/asset/browser-requirement sample packaging; package install is no longer marked zip-safe because pygame loads asset paths from the filesystem.
- `src/simulator/README.md`: source-tree operator documentation.
- `docs/sentry/internal/simulator.md`: maintained simulator architecture and workflow documentation.
- `docs/sentry/internal/simulator_offline_debug_roadmap.md`: current improvement roadmap and verification baseline.

## Verification

Commands run successfully:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_web_stream.py src/simulator/test/test_trace_contract.py src/simulator/test/test_validation_cli.py src/simulator/test/test_quality_cli.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_start_mock_command.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_mock_sequence.py src/simulator/test/test_start_mock_command.py -q
PYTHONPATH=src/simulator python3 -m simulator.start --list-mock-presets
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence --list-samples
PYTHONPATH=src/simulator python3 -m simulator.start --list-mock-sequences
PYTHONPATH=src/simulator python3 -m simulator.unit_scene --list-samples
PYTHONPATH=src/simulator python3 -m simulator.unit_scene src/simulator/sample/unit_scene.json --team red
PYTHONPATH=src/simulator python3 -m simulator.unit_scene src/simulator/sample/unit_scenes/full_roster.json --team red --json
PYTHONPATH=src/simulator python3 -m simulator.unit_trace src/simulator/sample/scenarios/multi_unit_decision_context.jsonl --unit-scene src/simulator/sample/unit_scenes/multi_unit_trace_contract.json
PYTHONPATH=src/simulator python3 -m simulator.start --list-unit-scenes
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mock-preset buff-ready --dry-run
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mock-preset uwb-fusion --dry-run
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mock-preset multi-unit-regional --dry-run
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mock-preset full-roster-regional --dry-run
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence src/simulator/sample/mock_sequences/regional_timed_context.json --control-file /tmp/test_sequence_control.jsonl --dry-run
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence src/simulator/sample/mock_sequences/buff_timeout_context.json --control-file /tmp/test_sequence_control.jsonl --dry-run
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence src/simulator/sample/mock_sequences/low_resource_recovery_exit.json --control-file /tmp/test_sequence_control.jsonl --dry-run
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence src/simulator/sample/mock_sequences/official_target_fallback_companion.json --control-file /tmp/test_sequence_control.jsonl --dry-run
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence src/simulator/sample/mock_sequences/multi_unit_target_priority_rehearsal.json --control-file /tmp/test_sequence_control.jsonl --dry-run
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mock-sequence src/simulator/sample/mock_sequences/regional_timed_context.json --dry-run
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_assets.py src/simulator/test/test_interactive_inputs.py src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_start_mock_command.py src/simulator/test/test_trace_contract.py src/simulator/test/test_validation.py src/simulator/test/test_scenarios.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_interactive_inputs.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_web_stream.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_web_visual_check.py src/simulator/test/test_quality_cli.py src/simulator/test/test_web_stream.py
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check --json
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_trace_contract.py
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_validation.py
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_foxglove_export.py
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_assets.py
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_unit_trace.py
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_mock_sequence.py
PYTHONPATH=src/simulator pytest -q src/simulator/test/test_scenarios.py
PYTHONPATH=src/simulator python3 -m simulator.main --validate-only
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/route_churn_warning.jsonl --validate-only --validate-format json
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/goal_id_output.jsonl --validate-only
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/chase_goal_pos_raw_bridge.jsonl --validate-only
python3 -m py_compile src/simulator/simulator/*.py
python3 -m py_compile src/simulator/simulator/mock_inputs.py src/simulator/simulator/interactive_inputs.py src/simulator/simulator/control_bus.py
python3 -m py_compile src/simulator/simulator/web_stream.py src/simulator/test/test_web_stream.py
PYTHONPATH=src/simulator python3 -m simulator.quality --check-text-whitespace
PYTHONPATH=src/simulator python3 -m simulator.quality --dry-run --with-browser-visual
PYTHONPATH=src/simulator python3 -m simulator.quality --dry-run --with-build
PYTHONPATH=src/simulator python3 -m simulator.quality --with-build
colcon build --packages-select simulator
colcon build --packages-select behavior_tree simulator
SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test --web-port 9021
PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy python3 -m simulator.main --smoke-test --web-port 9022 --unit-scene src/simulator/sample/unit_scenes/full_roster.json --smoke-screenshot /tmp/ly-simulator-full-roster-smoke.png
PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy python3 -m simulator.main --smoke-test --web-port 9023 --config src/simulator/config/visual_asset_qa.yaml --unit-scene src/simulator/sample/unit_scenes/full_roster.json --smoke-screenshot /tmp/ly-simulator-full-roster-visual-qa.png
PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy python3 -m simulator.visual_asset_qa --screenshot /tmp/ly-simulator-full-roster-visual-qa.png --unit-scene src/simulator/sample/unit_scenes/full_roster.json --config src/simulator/config/visual_asset_qa.yaml --team red --json
```

Observed environment note:

- Plain `pytest` currently auto-loads ROS launch-testing plugins and fails in this Python 3.13 shell unless plugin autoload is disabled or the missing `lark` dependency is installed.
- The normal full-roster headless smoke writes `/tmp/ly-simulator-full-roster-smoke.png` for manual UI/overlay QA. The clean full-roster visual asset QA writes `/tmp/ly-simulator-full-roster-visual-qa.png` and checks sprite pixels against the packaged art so normal overlays do not create false sprite failures.
- `simulator.web_visual_check --json` currently reports `status: skip` in this environment because the Python `playwright` package is not installed. The optional checker and failure/skip semantics are covered by unit tests; a real Chromium screenshot pass still requires installing `src/simulator/requirements-browser.txt` and running `playwright install chromium`.

## Risks

- The viewer remains a large pygame module, so adding UI polish directly inside it will increase maintenance cost unless future work splits layout, rendering, and interaction boundaries.
- Current diagnostics are threshold-based and intentionally warning-only; they improve review visibility but are not yet a full behavior regression oracle.
- Scenario fixtures now include one positive transition and one expected-WARN route churn case, but longer transitions, repeated event changes, and malformed negative traces remain open.
- Mock presets reduce command length for common input states, but preset names describe inputs only; actual behavior-tree outcomes still depend on the selected BT config and task gates.
- Scripted mock sequences now cover temporal control-bus inputs, including self HP, ammo, posture, self position, unit, structure, and match-clock changes. A future scoped mock-state command may still be needed for additional scalar referee fields if they become important to script over time.
- Optional external aim mock publishing depends on `sentry_msgs` Python imports being available from the sourced workspace; enabling it without those generated messages returns a clear startup error.
- Drone art is available for visual/offline context and publishes PositionData rows, but Drone has no formal `Health.msg` field and is not a current UnitInfo decision field. Keep this distinction when adding more unit-related logic.
- Infantry3 publishes `Health.msg.reserve` and PositionData in offline mocks, but current formal UnitInfo/RobotLists logic does not consume it as a decision unit.
- The asset source archive has unknown license/attribution status; keep redistribution local-project-only until provenance is confirmed.
- `/ly/navi/target_official` uses behavior-tree `ArmorType` IDs, while draggable simulator pieces use `UnitType` IDs; docs and tests now call out the Sentry/Drone ID collision risk.
- Map overlays are useful for review context, but they are not authoritative navigation geometry.
- The worktree contains existing simulator and behavior-tree changes outside this milestone. Future slices must continue to avoid reverting or masking unrelated user changes.
- The HTTP control endpoint is still unauthenticated by design; use loopback binding when LAN control is not needed.

## Remaining Issues

- The browser dashboard now has an optional Playwright visual checker, but the current environment lacks Playwright/Chromium, so desktop/narrow real-browser screenshots have not been captured here.
- Inputs tab sprite/dense rendering and right-panel scrolling are in place, but the pygame UI still needs stronger wrapping, keyboard focus rules, and a clearer health/status area for small windows.
- The Layers tab makes existing layer switches interactive, and dense right-panel content can now scroll; scroll state is extracted, while remaining panel rendering helpers still need smaller module boundaries.
- Unit-art and armor-preview rendering now has headless decode/scale/blit, normal full-frame screenshot evidence, and clean pixel-level sprite visibility QA; visual quality still needs a real-window pass at desktop and minimum-size layouts.
- The offline mock node publishes richer decision inputs, and `simulator.unit_trace` now checks fixture-level UnitInfo HP/position evidence. There is still not a launched offline-decision regression that asserts a freshly generated BT trace reflects every important mock knob.
- Fixture suite should add more multi-record transition scenarios for target-loss fallback, navigation unreachable transitions, and malformed trace failures.
- Validation thresholds should move into structured config once the diagnostics stabilize.
- Documentation should keep `simulator-quality` as the source of truth for simulator verification commands to reduce drift between `src/simulator/README.md` and `docs/sentry/internal/simulator.md`.

## Recommended Next Actions

1. Refactor pygame viewer UI in small slices: extract status/timeline/panel rendering helpers, then tighten wrapping and keyboard focus conventions.
2. Install optional browser tooling and run `PYTHONPATH=src/simulator python3 -m simulator.quality --with-browser-visual` to capture desktop/narrow HTTP dashboard screenshots.
3. Expand fixture scenarios with transition-heavy traces and at least one expected-WARN fixture.
4. Add more scripted sequence examples for target-loss fallback and navigation unreachable transitions after adding scoped control-bus commands for dynamic target/navigation state.
5. Keep running `PYTHONPATH=src/simulator python3 -m simulator.quality --with-build` after simulator slices and reserve formal self-checks for runtime/link changes.
