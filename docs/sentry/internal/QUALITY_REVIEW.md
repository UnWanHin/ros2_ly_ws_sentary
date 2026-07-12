# Simulator Offline Debug Quality Review

Updated: 2026-07-12

> Historical review. Internal detector-armors mock coverage described below was removed with the internal vision packages on 2026-07-12; current simulator input coverage uses external `/ly/aim/*` only.

## Strengths

- The simulator now has a clearer stable boundary: raw JSONL trace rows are normalized through `simulator.trace` into `TraceRecord` before validation, replay, or export.
- Validation reports are actionable for operators: issue codes are stable, suggestions explain likely fixes, and grouped domains make failures easier to triage.
- Scenario fixtures are no longer informal sample data; the manifest records expected schema, output, intent, event, goal, and runtime coverage.
- Current changes are simulator-only and preserve the formal ROS2 runtime chain.
- Tests are fast, deterministic, and can be run without launching ROS nodes.
- Web status output is strict JSON, path-redacted, and covered for non-finite float handling, HTML label escaping, simulator-input snapshot exposure, and dashboard HTML structure.
- The HTTP dashboard now has an optional Playwright visual checker that exercises desktop and narrow viewports against a temporary synthetic stream, while cleanly reporting SKIP when browser tooling is absent.
- `simulator-quality` now gives simulator-only work a single repeatable local gate with dry-run, keep-going, text-whitespace, smoke, validation, pytest, py_compile, optional browser visual QA, and optional package-build steps.
- Unit art is now runtime-owned under `src/simulator/assets/` with a manifest loader and fallback rendering, rather than depending on the local source archive.
- Offline decision mode now publishes a broader set of existing formal inputs: EventData, SentryInfo, RfidStatus, BuffData, gimbal state, BulletInfo, detector Armors, navigation state, self position, official target fallback, optional external aim, unit HP, and unit positions.
- BulletInfo is now traceable from `/ly/game/bullet` through `DecisionTrace.cpp`, the normalized simulator model, `/status.json.current_record.bullet_info`, the Runtime tab, and Foxglove export; this improves offline evidence without changing formal decision gates.
- Detector armor-list input is now covered by a simulator-only `/ly/detector/armors` mock path, a `detector-armors` preset, a workflow, and a compact fixture that asserts target distance and hitable-target evidence.
- UnitInfo is part of the normalized trace/view/validation contract when present, which makes reliable-position and area/source evidence visible during replay.
- Detailed formal navigation state is now part of the normalized trace/view/validation/export contract: `goal_reach_state`, `navi_status`, `navi_velocity`, relative target `frame_id`, and official chase metadata are no longer raw-JSON-only.
- Scenario fixtures now cover every traced output kind currently produced by the behavior-tree trace writer: `goal_id`, `goal_pos`, `goal_pos_raw_bridge`, `relative_target_bridge`, `chase_goal_pos`, and `chase_goal_pos_raw_bridge`.
- The pygame `Layers` tab exposes existing layer switches and asset provenance in the UI, reducing the need to edit YAML during visual debugging.
- Asset provenance is explicit in the manifest and docs, including unknown license status and local-project-only redistribution guidance.
- Asset QA now covers manifest parsing, PNG dimensions, pygame decode/scale/blit behavior, a normal full-roster headless screenshot artifact for manual review, and clean pixel-level sprite visibility checks for the full roster.
- Unit-scene samples are now discoverable and inspectable before launch; `simulator.unit_scene` exposes the same HP, PositionData, field-side, sprite, and decision-channel mapping that the offline viewer/mock-input path will use.
- Offline mock presets now make common input states reproducible from one command while preserving explicit CLI override precedence; `uwb-fusion` covers opt-in UWB self-position input rehearsal and `full-roster-regional` covers packaged unit art, formal health-unit HP mapping, and placed-unit PositionData mapping.
- Scripted mock sequence examples are now discoverable from both the sequence runner and launcher, including descriptions and action counts, so operators do not need to inspect sample directories by hand.
- Offline workflow playbooks now pair common presets, timed sequences, unit scenes, expected evidence, preflight checks, and post-run trace checks without changing the ROS2 runtime chain; `uwb-position-fusion` makes the dedicated UWB self-position path discoverable.
- Formal decision-input coverage is now cataloged and quality-gated, including explicit partial status for external aim, Drone/Infantry3 formal exclusions, covered BulletInfo trace evidence, detector armor-list coverage, and covered opt-in UWB self-position mocking.
- Unit decision-channel labels make formal-consumed facts visible in the UI and CLI: Hero/Engineer/Infantry1/Infantry2/Sentry are `BT:HP,POS,UI`, Infantry3 is `PUB:HP,POS noUI`, and Drone is `PUB:POS noUI`.
- The Inputs tab and `/status.json` now share the same simulator input state snapshot, so placed units, structure HP, runtime self state, and palette data can be checked without reading pixels.

## Weaknesses

- The pygame viewer still carries too many responsibilities in one module: map drawing, panel layout, event handling, replay state, input interaction, and web publishing orchestration.
- The new Layers tab improves control discoverability, and the right-panel body now scrolls; scroll state/clamping is isolated in a small tested helper, but panel rendering still lives inside the large viewer module.
- The web dashboard now renders the existing frame stream plus trace, replay, selected-record, simulator-input, placed-unit, and alert summaries from `/status.json`; automated DOM/layout checks exist, but real Chromium is not installed in this environment.
- Validation checks can identify suspicious states and now have one expected-WARN route-churn fixture, but they cannot yet explain richer multi-step decision transitions or compare full expected route plans over time.
- Fixture traces are intentionally compact, which keeps tests fast but still under-represents long-running match behavior; the suite now covers output kinds well, but still has only one real transition path, one expected-WARN path, and one multi-unit context row.
- Mock-input CLI coverage is broad, and named presets now cover the common static input states for daily use.
- Docs are accurate but duplicated across internal docs and package README; future edits may drift without a tighter doc ownership rule.

## Technical Debt

- Viewer layout logic should be split into small renderable panels with explicit state inputs and no hidden dependence on global viewer fields.
- Web stream status now uses a shared snapshot helper and strict JSON metadata merge, but the metadata dictionaries should eventually be replaced by a small explicit status model if more producers are added.
- Diagnostic thresholds are constants in `validation.py`; they should become config-backed once stable.
- `simulator.start` now has many `--mock-*` flags. Presets improve common-case ergonomics, but the wrapper still needs discipline as new mock inputs are added.
- BulletInfo evidence is visible in offline tooling, but current behavior-tree logic still uses legacy ammo/speed gates; changing formal decision behavior should be a separate reviewed slice.
- Detector armor-list coverage proves the formal topic and resulting trace surfaces, but launched offline-decision regression should still assert a freshly generated trace row before treating it as end-to-end runtime proof.
- The UnitType/ArmorType distinction is documented and tested at the CLI level, but future feature work must keep Drone visual context separate from formal UnitInfo/Health decision fields.
- Infantry3 remains a published-but-not-consumed case for current UnitInfo: it can publish `Health.msg.reserve` and PositionData in offline mocks, but must not be treated as a formal UnitInfo decision unit until the behavior-tree `RobotLists` contract changes.
- Asset license/attribution remains unresolved. The simulator treats `素材.zip` as local project source material only until provenance is confirmed.
- Unit-scene inspection currently reports what the simulator will publish into existing mock topics; it does not yet compare those inputs against expected downstream behavior-tree trace outputs.
- Some local generated artifacts such as `__pycache__` and `.pytest_cache` exist under `src/simulator`; they should remain uncommitted and may be cleaned separately if approved.
- Plain direct `pytest` still depends on disabling plugin autoload in this shell, but `simulator-quality` applies that environment setting for the simulator suite.

## Missing Polish

- Browser stream page now renders the core `/status.json` metadata and has an optional real-browser checker, but this environment still needs Playwright/Chromium installed before capturing desktop/narrow screenshots.
- Web control responses no longer disclose resolved local paths, the browser page escapes control labels, and the control endpoint remains unauthenticated for offline LAN use.
- Pygame panels now have body clipping and scroll affordances for dense regional/sentry traces, with scroll state covered by pure tests, but still need stronger text wrapping, focus rules, and extracted render helpers.
- The Layers tab exposes layer state and asset provenance, but it still uses text buttons rather than a richer compact icon/control language.
- Scripted mock sequences now cover temporal control-bus cases for match clock, self HP, ammo, posture, self position, unit, structure, buff-timeout rehearsal, and low-resource recovery exit; additional scalar referee/mock-state fields still need a deliberate schema extension if they must change over time.
- Mock sequence discoverability and workflow recommendation are now cataloged, but there is not yet a launched offline-decision regression that proves each playbook produces the expected downstream BT trace transitions.
- Full-roster unit-art QA is automated through manifest parsing, pygame asset rendering, a normal headless full-frame screenshot, and a clean pixel-level sprite visibility check, but still needs real-window visual review for sprite scale, overlap, and readability on the normal pygame map.
- Sprite rendering needs a real-window visual pass at default and minimum window sizes; current automated coverage proves loading, scaling, blitting, and headless smoke behavior, not final visual polish.
- The richer mock publisher needs a launched offline-decision regression that proves each important mock knob appears back in BT trace rows.
- HTTP status, health, frame, and control endpoints now have focused tests without opening a visible pygame window.

## Improvement Opportunities

1. Add transition fixtures for target loss, repeated route selection, buff timeout, navigation unreachable, and unit-resource recovery.
2. Install optional browser tooling and run `PYTHONPATH=src/simulator python3 -m simulator.quality --with-browser-visual` for desktop/narrow dashboard screenshots.
3. Extract viewer panel helpers gradually, starting with layer controls, decision-channel badges, and text wrapping helpers that are easy to test.
4. Add a launched offline-decision regression that proves generated BT traces contain expected UnitInfo and navigation-status facts from simulator inputs.
5. Wire `simulator-quality` into CI or PR checks once the project CI policy is decided.
6. Move stable diagnostic thresholds into config after more fixtures prove the defaults.
