# Simulator Offline Debug Roadmap

Updated: 2026-06-02

## Purpose

This document is the working roadmap for turning `src/simulator` into a mature offline debugging workbench for sentry behavior-tree decisions.
The simulator remains an offline/debug tool; it must not reroute or replace the formal ROS2 runtime chain.

## Current Architecture

- `behavior_tree` writes opt-in JSONL traces through `src/behavior_tree/src/DecisionTrace.cpp`.
- `src/simulator/simulator/trace.py` normalizes raw JSONL rows into the stable `TraceRecord` model in `model.py`.
- `src/simulator/simulator/viewer.py` renders replay/live-follow state with pygame.
- `src/simulator/simulator/panel_scroll.py` owns right-panel scroll offsets, body/content heights, and clamping semantics.
- `simulator.start` can run offline decision mode with mock publishers while the viewer follows a growing trace.
- `interactive_inputs.py`, `inputs_panel.py`, `mock_inputs.py`, and `control_bus.py` provide file-based match/input commands for offline tests.
- `visual_asset_qa.py` compares clean full-roster smoke screenshots against packaged unit sprites for automated sprite visibility checks.
- `web_stream.py` mirrors the pygame frame and exposes HTTP control buttons.
- `validation.py` performs offline consistency checks before replay, export, or regression comparison.
- `foxglove_export.py` exports normalized records to MCAP only when explicitly invoked.

## Stable Boundary

The stable simulator-facing boundary is:

```text
raw JSONL trace -> simulator.trace.normalize_record -> simulator.model.TraceRecord
```

Behavior-tree debug-only fields may change without simulator work.
Simulator changes are required only when `TraceRecord` semantics, viewer-visible fields, validation checks, map assumptions, mock input commands, or exported Foxglove fields change.

## Engineering Direction

The target product is a dense robotics debug dashboard:

- deterministic trace replay first, live ROS/offline decision controls second;
- structured validation before visual inspection;
- explicit field coordinates and topic/output semantics;
- compact UI panels optimized for repeated engineering use;
- test fixtures that exercise decision output, events, posture, target state, map points, and mock inputs;
- docs updated in the same slice as trace, UI, or mock-input changes.

## Completed Slices

### 2026-06-02: Validation Report Foundation

- Added stable validation issue codes and per-issue suggestions.
- Upgraded `--validate-only` output into a readable health report with PASS/WARN/FAIL status.
- Added summary counters for schema versions and output kinds.
- Added focused tests for validation codes, suggestions, clean reports, and failed reports.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Scenario Fixture Suite

- Added `src/simulator/sample/scenarios/manifest.json` as the fixture contract.
- Added named compact fixtures for startup home hold, target acquisition, buff activation, relative target bridge, outpost attack, and low-resource recovery.
- Added manifest-driven tests that load every fixture, run validation, and verify schema/output/intent/event coverage.
- Added scenario fixture packaging so installed simulator packages include the suite.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Validation Diagnostics

- Added warning-level scenario diagnostics for route churn, stale chase targets, posture lag, match-time jumps, and missing referee resource state.
- Grouped validation report issues by domain so `schema`, `trace`, `output`, `unit`, `runtime`, `referee`, and `scenario` problems are easier to scan.
- Added focused tests that prove each diagnostic code fires under a compact synthetic trace.
- Kept existing sample and scenario fixtures PASS, so diagnostics improve operator visibility without blocking valid replay.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Web Stream Health API

- Enriched `/status.json` with readiness, frame age, control-file availability, host/port, FPS, and JPEG quality.
- Added `/healthz` so automation can distinguish "server process is up" from "first pygame frame is available".
- Updated the compact browser page to show live readiness and frame age from the same status payload.
- Added focused HTTP tests for status, health, frame JPEG output, and command API error handling.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Web Stream Replay Metadata

- Added `trace`, `validation`, `replay`, and `current_record` metadata to `/status.json`.
- Reused the shared `ly_simulator_validation_report_v1` payload so CLI JSON validation and HTTP automation see the same validation state.
- Kept trace and control paths basename-only in HTTP responses to avoid exposing resolved local workspace paths.
- Made `/status.json` strict JSON by converting non-finite floats to `null` and dumping with `allow_nan=False`.
- Escaped the control-file basename before rendering it in the browser page.
- Rejected non-finite web FPS values at both CLI and web stream construction boundaries.
- Added focused tests for strict JSON metadata, selected-record summaries, metadata override prevention, HTML escaping, and path redaction.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Start-Gate Transition Fixture

- Added `start_gate_lifecycle`, the first multi-record scenario fixture.
- Added optional manifest sequence assertions for event order, goal order, intent-reason order, publish-allowed state, and match time.
- Covered reset re-arm, Home hold, and first active regional patrol goal selection without changing runtime behavior.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Expected-WARN Route Churn Fixture

- Added `route_churn_warning`, an expected-WARN fixture that exercises `scenario.route_churn`.
- Added manifest-level warning-code assertions so scenario fixtures can document operator-facing warnings without converting them into hard failures.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Validation JSON Report

- Added `validation_report()` as the shared machine-readable validation summary.
- Added `--validate-format json` for CI and scripts while preserving the existing text format as the default.
- Added CLI tests for PASS/WARN/FAIL JSON behavior and invalid option usage.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Simulator Quality Gate

- Added `simulator.quality` and the `simulator-quality` console script as the standard simulator-only verification entrypoint.
- The default gate runs the simulator pytest suite, Python bytecode compilation, sample trace validation, route-churn JSON validation, a headless web smoke test, and simulator text whitespace checks, including untracked simulator-related files.
- `--with-build` adds `colcon build --packages-select simulator` for package integration verification.
- `--dry-run` prints the exact command plan for review or CI wiring.
- Added focused tests for dry-run output, build-gate inclusion, invalid repo root handling, failing-step return codes, and keep-going behavior.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Unit Art, UnitInfo, And Mock Decision Inputs

- Normalized the provided unit art into `src/simulator/assets/` and added a manifest-backed unit sprite catalog.
- Added red/blue Hero, Engineer, Infantry, Sentry, and Drone sprites for trace units, offline input units, and drag previews, with circle-marker fallback.
- Added UnitInfo parsing, status payloads, viewer Events-tab summaries, and validation warnings for missing/out-of-field UnitInfo positions and reliable enemy-position evidence gaps.
- Added `multi_unit_decision_context`, a compact fixture for HP, position, UnitInfo, area/source evidence, RFID, team buff, and resource-state context.
- Extended `simulator.mock_inputs` to publish existing formal decision inputs for EventData, SentryInfo, RfidStatus, BuffData, navigation status, self position, and official target fallback.
- Extended `simulator.start --offline-decision` with wrapper flags for those mock inputs and added focused CLI/command tests.
- Documented the `UnitType` vs `ArmorType` ID distinction so Drone visual assets do not get confused with formal Sentry armor ID `6`.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Offline Mock Presets

- Added `--mock-preset` overlays in `simulator.start` for common static input states: buff-ready, outpost-dead, nav-unreachable, official-target-sentry, uwb-fusion, multi-unit-regional, full-roster-regional, and low-resource.
- Presets expand only to existing wrapper/mock-input fields and require `--offline-decision`, so formal stack runs cannot be silently changed.
- Preset values fill only fields the operator did not pass explicitly; direct CLI flags remain the source of truth.
- Added `--list-mock-presets` with expanded overlay flags for command review before launch.
- Documented that preset names describe input state, not guaranteed behavior-tree outcome, because task gates and selected BT config still decide runtime behavior.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Scripted Mock Sequences

- Added `simulator.mock_sequence`, a timed scheduler that appends existing simulator control-bus JSONL commands.
- Added `simulator.start --offline-decision --mock-sequence` process orchestration and dry-run validation.
- Added `src/simulator/sample/mock_sequences/regional_timed_context.json` for a timed regional multi-unit/resource transition.
- Added `buff_timeout_context.json` and `low_resource_recovery_exit.json` for buff timeout/fallback rehearsal and low-resource recovery exit rehearsal.
- Packaged mock sequence examples and exposed `simulator-mock-sequence` as a console script.
- Kept sequence actions on the simulator command bus so `simulator.mock_inputs` remains the only ROS topic publisher in this offline path.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Input-State Snapshot And Expanded Mock Publishers

- Added `SimulatorInputState.snapshot()` and exposed it through `/status.json` as `simulator_inputs` with runtime self HP/ammo/posture/self-position, structure HP, placed units, palette, and low-HP summaries.
- Improved the Inputs tab with sprite-backed palette chips, placed-unit rows, and HP bars for denser multi-unit offline debugging.
- Extended mock publishers with existing formal topics for gimbal fire-code/chassis/capV, navigation velocity/lower-head, bullet info, and optional external aim (`/ly/aim/armor_targets`, `/ly/aim/result`).
- Added dynamic control-bus commands for `set_self_health`, `set_ammo`, `set_posture`, and `set_self_position`, and updated the timed regional sequence to exercise them.
- Fixed mock publisher startup ordering around posture initialization and made invalid/non-finite `set_self_position` payloads no-op instead of process-stopping errors.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Compact HTTP Debug Dashboard

- Replaced the minimal stream page with a compact local dashboard that keeps the pygame frame visible and renders trace, replay, current decision, simulator-input, placed-unit, and alert summaries from `/status.json`.
- Preserved `/frame.jpg`, `/status.json`, `/healthz`, and `/api/control` as low-dependency endpoints for scripts and external tools.
- Kept dashboard rendering in static HTML/CSS/JavaScript with `textContent` updates so status payload values are treated as data, not markup.
- Added focused web-stream tests for dashboard structure, simulator-input status metadata, disabled controls, path redaction, and escaped control labels.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Optional Browser Visual Dashboard QA

- Added `simulator.web_visual_check`, a simulator-only browser visual check for the HTTP dashboard.
- The check starts a temporary `SimulatorWebStream`, publishes a synthetic pygame frame and status payload, opens Chromium through Playwright, verifies desktop and narrow viewport layout/readiness/overflow signals, and writes screenshots to `/tmp/ly-simulator-web-visual`.
- Kept Playwright in `requirements-browser.txt` as an optional dependency because Playwright browser binaries are installed separately through its CLI.
- Added `simulator-quality --with-browser-visual` for environments where Playwright/Chromium is installed.
- Added unit coverage for skip/fail behavior when browser tooling is missing, fake-browser screenshot capture, and dashboard layout finding generation.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Mock Sequence Catalog

- Added `simulator.mock_sequence --list-samples` for bundled timed input scripts.
- Added `simulator.start --list-mock-sequences` so operators can discover sequence examples from the same launcher used for offline decision runs.
- The catalog reports file name, resolved path, description, and action count by parsing the same JSON/YAML sequence contract used by dry-run and launch validation.
- Added focused tests for catalog metadata and launcher-level listing.
### 2026-06-02: Unit Scene Catalog And Art QA

- Added `simulator.unit_scene --list-samples` and `simulator.start --list-unit-scenes` for bundled unit-scene discovery.
- Added `simulator.unit_scene <scene> --team red|blue` to summarize the exact mock decision inputs produced by a unit scene: `Health.msg` field mapping, `/ly/position/data` car ID/raw-y values, field-side mapping, sprite availability, and decision-channel badges.
- Added machine-readable `ly_simulator_unit_scene_summary_v1` JSON output for scripts and future CI checks.
- Added `src/simulator/sample/unit_scenes/full_roster.json`, a full Hero/Engineer/Infantry/Sentry/Drone red-blue roster for checking the provided unit art and mock input mapping.
- Extended the simulator quality gate with unit-scene catalog and full-roster summary checks.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Unit Decision-Channel Visibility

- Added per-unit decision-channel metadata for simulator input units, scene summaries, Inputs-tab rows, and map labels.
- Hero, Engineer, Infantry1, Infantry2, and Sentry are labeled `BT:HP,POS,UI` because the current formal UnitInfo path consumes their HP/position facts and can emit trace `unit_info`.
- Infantry3 is labeled `PUB:HP,POS noUI`: offline mocks publish `Health.msg.reserve` plus `/ly/position/data`, but current formal `RobotLists`/UnitInfo logic does not consume it as a decision unit.
- Drone is labeled `PUB:POS noUI`: it publishes PositionData and is drawn as tactical/visual context, but has no formal `Health.msg` field and no current UnitInfo output.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Formal Trace Coverage And Layers UI

- Added stable simulator model/parser coverage for detailed `goal_reach_state`, `navi_status`, `navi_velocity`, relative target `frame_id`, and official chase metadata.
- Extended the selected-record status payload, pygame Events/Runtime panels, validation, and Foxglove export so the same formal navigation/reach/velocity/chase facts are visible across offline tools.
- Added validation warnings for goal-reach/output mismatch, missing relative-target frame, invalid velocity scale, and should-rotate freshness/value inconsistency.
- Added compact fixtures for missing behavior-tree output kinds: `goal_id`, `goal_pos_raw_bridge`, `chase_goal_pos`, and `chase_goal_pos_raw_bridge`.
- Updated the scenario manifest so the fixture suite now covers `goal_id`, `goal_pos`, `goal_pos_raw_bridge`, `relative_target_bridge`, `chase_goal_pos`, and `chase_goal_pos_raw_bridge`.
- Added a pygame `Layers` tab for toggling existing map layers and reviewing asset catalog/provenance status without editing YAML.
- Recorded local `素材.zip` provenance in the asset manifest and docs, including SHA-256 and unknown license/local-project-only redistribution status.
- Kept runtime behavior unchanged except for trace-only observability additions in `DecisionTrace.cpp`.

### 2026-06-02: Asset Render Smoke And Screenshot QA

- Added pygame-level asset smoke coverage that decodes, scales, and blits every manifest unit and armor PNG.
- Added `--smoke-screenshot` for `simulator.main --smoke-test` so headless smoke runs can write a reviewable PNG frame.
- Wired the full-roster quality smoke to write `/tmp/ly-simulator-full-roster-smoke.png` for manual sprite/layout QA after automated checks.
- Added CLI error handling for invalid screenshot usage and unwritable screenshot targets.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Clean Full-Roster Visual Asset QA

- Added `simulator.visual_asset_qa`, a screenshot-to-sprite pixel checker for full-roster unit scenes.
- Added `src/simulator/config/visual_asset_qa.yaml`, a clean layer preset that disables non-essential overlays while leaving simulator inputs and units visible.
- Extended `simulator-quality` so it first writes the normal manual-review screenshot, then writes `/tmp/ly-simulator-full-roster-visual-qa.png` with the clean config, then runs pixel-level sprite visibility QA.
- Documented that normal full-roster smoke remains for manual UI/overlay review, while clean visual QA is the automated pass/fail artifact.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Right-Panel Scroll UX

- Added a fixed right-panel header with scrollable tab body content for Decision, Events, Runtime, Inputs, and Layers.
- Added per-tab scroll offsets, mouse-wheel scrolling, panel-local keyboard scrolling, content clipping, and a compact scrollbar.
- Preserved normal replay timeline shortcuts when the mouse is outside the right panel.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Panel Scroll State Extraction

- Extracted right-panel scroll bookkeeping into `PanelScrollState`.
- Added pure tests for offset clamping, missing/invalid values, per-tab state, and scroll-range semantics.
- Kept pygame rendering in `viewer.py` while moving state math out of the monolithic module.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: Offline Workflow And Decision Input Catalogs

- Added `simulator.offline_workflow` for common offline decision playbooks that pair preset, sequence, unit scene, expected evidence, preflight commands, and post-run checks.
- Added `simulator.decision_input_coverage` to map formal behavior-tree input groups to simulator mock flags, command-bus commands, trace fields, fixtures, workflows, viewer surfaces, and known gaps.
- Added both catalogs to the simulator quality gate so stale playbook references and decision-input coverage drift fail locally.
- Kept the formal ROS2 runtime chain untouched.

### 2026-06-02: BulletInfo Trace Evidence

- Added trace-only `bullet_info` emission for cached `/ly/game/bullet` state: receipt age, initial speed, shoot data, shooter metadata, projectile allowance, and remaining gold coin.
- Added normalized simulator model/parser/status/Foxglove/viewer support so the same BulletInfo facts are visible in JSONL replay, `/status.json.current_record.bullet_info`, the Runtime tab, and exported decision frames.
- Added the `bullet_info_resource` scenario fixture, `bullet-resource` mock preset, and `bullet-info-resource-snapshot` workflow to make BulletInfo evidence discoverable and regression-tested.
- Kept formal behavior-tree decision gates unchanged; current decisions still use the existing legacy ammo/speed path unless formal logic changes in a separate reviewed slice.

### 2026-06-02: Detector Armors Offline Input Coverage

- Added simulator-only `/ly/detector/armors` publishing through `simulator.mock_inputs`, with a single-target shortcut and repeatable `--armor TYPE:DISTANCE_M` entries.
- Added wrapper flags, early CLI validation, and the `detector-armors` preset in `simulator.start`.
- Added `detector-armors-target-list` workflow and `detector_armors_target_list` scenario fixture for target distance, hitable-target, and outpost-window evidence.
- Kept detector disabled in offline-decision launch defaults; the mock node publishes the existing formal topic without changing subscribers, message definitions, or runtime launch wiring.

## Current Issues

- The pygame UI is functional but still monolithic; panel layout and dense text rendering should be split into smaller view components.
- The Layers tab exposes existing switches, and dense right-panel content now scrolls. Scroll state is extracted, but most panel rendering still lives inside the large viewer module.
- The HTTP stream now exposes machine-readable status, a compact browser dashboard, and an optional Playwright visual checker. Current local environment still lacks Playwright/Chromium, so the real screenshot pass is not captured here.
- Validation diagnostics are useful but still threshold-only; future work should make thresholds configurable and add richer transition-aware diagnostics for multi-record fixtures.
- The bundled scenario suite now covers all current output kinds and includes one multi-record start-gate transition, one expected-WARN route churn fixture, and one multi-unit context fixture, but most scenarios are still compact; future scenarios should add more transitions and negative fixtures rather than overloading `sample_trace.jsonl`.
- Static mock preset coverage is available for common input states including UWB position-fusion rehearsal, scripted mock sequences now cover temporal match-clock, self-state, unit, structure, buff-timeout rehearsal, low-resource recovery exit, and time-left transitions, and workflow playbooks now pair these inputs with expected evidence.
- Formal decision-input coverage is cataloged, including known partial areas for optional external aim trace fixtures and Drone/Infantry3 formal UnitInfo exclusions; UWB self-position, BulletInfo resource state, and detector armor-list input now have opt-in offline mock and trace evidence.
- Unit-scene, decision-channel, and asset-render tooling can prove what the offline mock path will publish and draw, but a launched offline-decision trace assertion is still needed to prove downstream behavior-tree traces receive every important HP/position fact.
- Clean full-roster visual asset QA proves packaged sprites are visible in a low-noise screenshot. It does not replace manual review of the normal UI screenshot, where overlays, labels, and current-goal markers are expected to interact with unit art.
- Time-varying scalar referee fields beyond current self HP/ammo/posture/self-position commands still need a deliberately scoped mock-state schema before being scripted.
- Map overlays are approximate and should not be used for precise path-cost or collision decisions until aligned with official field metadata.
- Direct `pytest` in the current shell auto-loads ROS launch-testing plugins and fails unless plugin autoload is disabled or the missing `lark` dependency is installed.

## Next Slices

### Slice 4: Pygame UI Refactor

- Split renderer code into focused modules for map layers, timeline, panel tabs, controls, and overlays.
- Add predictable text wrapping and keyboard focus conventions on top of the current panel-scroll baseline.
- Preserve the existing Layers tab contract while extracting layer controls and asset status into smaller testable helpers.
- Keep config-driven colors, point styles, layers, and dimensions.

### Slice 5: Remote Debug Dashboard Polish

- Preserve the existing HTTP frame endpoint for low-dependency usage.
- Install optional browser tooling and run `PYTHONPATH=src/simulator python3 -m simulator.quality --with-browser-visual` to capture desktop/narrow screenshots.
- Add deeper dashboard interactions only if they stay backed by the existing `/status.json` and `/api/control` contracts.

### Slice 6: Offline Decision Workflow Polish

- Add more workflow playbooks or sequence examples only when they cover a new decision gap, such as enemy outpost dead, navigation unreachable, or league-specific target behavior.
- Keep `simulator.offline_workflow` and `simulator.decision_input_coverage` as the source of truth for recommended preset/sequence/unit-scene pairings and formal decision-input coverage.
- Add a launched offline-decision regression for at least one unit scene that asserts the resulting trace contains expected UnitInfo HP/position facts.
- Include decision-channel assertions in that launched regression so Infantry3 and Drone stay explicit formal exclusions unless the behavior-tree contract changes.
- Extend the launched regression to assert at least one navigation-status fact (`should_rotate`, reached/reachable, or velocity) when the mock publisher controls it.
- Design a scoped mock-state command only if scalar referee fields need to change over time beyond existing self HP/ammo/posture/self-position command-bus coverage.
- Make generated temporary BT configs and trace paths explicit in startup logs.
- Add docs for regional/league recommended scenarios and expected decision transitions.

## Verification Baseline

Use the quality gate for simulator-only slices from the workspace root:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality
```

For package integration:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --with-build
```

For optional browser visual QA:

```bash
python3 -m pip install -r src/simulator/requirements-browser.txt
playwright install chromium
PYTHONPATH=src/simulator python3 -m simulator.quality --with-browser-visual
```

For formal runtime/link changes, also run the repo self-checks described in `AGENTS.md`.
Simulator-only UI, validation, fixture, and doc slices should not require changes to the formal ROS2 runtime chain.
