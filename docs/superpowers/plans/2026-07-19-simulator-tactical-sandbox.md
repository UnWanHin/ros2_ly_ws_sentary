# Simulator Tactical Sandbox Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a data-driven, dual-client tactical sandbox that drives offline BT inputs safely and presents actual final behavior-tree decisions.

**Architecture:** `SceneCatalog` and `SceneState` become the single source of simulator facts.  Pygame and the browser submit the same validated scene commands.  A mode-gated mock ROS publisher projects the scene into formal inputs, while final BT outputs enter trace/monitor data only.

**Tech Stack:** Python 3, Pygame, standard-library HTTP server and JavaScript, YAML, ROS2 rclpy, C++17/nlohmann JSON, pytest, existing Foxglove MCAP exporter.

## Global Constraints

- Preserve all formal ROS topic/message contracts and leave the formal regional launch path unchanged.
- Use `mock` and `manual_ros` as mutually exclusive input ownership modes.
- Keep Pygame, browser, and ROS conversion free of duplicated roster/HP/topic mapping constants.
- Preserve replay of current decision-trace schema v2 input files.
- Use supplied red/blue PNG assets; retain circle fallback for missing assets.
- Update simulator trace adapters, validation, config, documentation, and graph artifacts for every trace/interface change.
- Do not commit generated `build/`, `install/`, `log/`, screenshots, cache, or credentials.

---

## File Structure

- Create `src/simulator/config/tactical_catalog.yaml`: canonical catalog for field entities, ROS projections, goal markers, asset keys, and styles.
- Create `src/simulator/simulator/tactical_catalog.py`: typed catalog loader and schema validation.
- Create `src/simulator/simulator/scene.py`: immutable/mutable scene types, command reducer, ROS projection, and snapshot builder.
- Create `src/simulator/simulator/tactical_web.py`: browser shell, status/render API payload helpers, and tactical command endpoint contract.
- Modify `interactive_inputs.py`: compatibility facade delegating catalog/state semantics to `scene.py`.
- Modify `mock_inputs.py`: mode-gated catalog-based formal ROS projection.
- Modify `control_bus.py`: typed scene command validation and ownership assertions.
- Modify `viewer.py`, `inputs_panel.py`, `web_stream.py`: dual tactical boards and shared interaction behavior.
- Modify `ros_topic_monitor.py`: observe final control topics and manual inputs.
- Modify `DecisionTrace.cpp`, `Application.hpp`, `trace.py`, `model.py`, `validation.py`, `foxglove_export.py`: trace v3 command/output evidence.
- Modify test files/add `test_tactical_catalog.py`, `test_scene.py`, `test_tactical_web.py`, `test_control_output_trace.py`.
- Modify `docs/sentry/internal/simulator.md`, `src/simulator/README.md`, and Understand Anything graph files.

## Task 1: Introduce a canonical tactical catalog

**Files:**
- Create: `src/simulator/config/tactical_catalog.yaml`
- Create: `src/simulator/simulator/tactical_catalog.py`
- Test: `src/simulator/test/test_tactical_catalog.py`

**Interfaces:**
- Produces `SceneCatalog.load(path: Path) -> SceneCatalog`.
- Produces `UnitArchetype`, `StructureArchetype`, `GoalMarker` immutable dataclasses.
- Produces `SceneCatalog.unit_by_key(key: str) -> UnitArchetype` and `validate_catalog_payload(raw: Mapping[str, Any]) -> None`.
- Consumed by scene, Pygame, browser, and mock ROS projection.

- [ ] **Step 1: Write failing catalog tests**

```python
def test_catalog_has_single_formal_mapping_per_projectable_unit(tmp_path: Path):
    catalog = SceneCatalog.load(default_catalog_path())
    assert catalog.unit_by_key("hero").health_field == "hero"
    assert catalog.unit_by_key("hero").position_car_id == 1
    assert catalog.unit_by_key("drone").project_to_ros is False

def test_catalog_rejects_duplicate_formal_position_ids(tmp_path: Path):
    with pytest.raises(ValueError, match="position_car_id"):
        SceneCatalog.from_mapping(duplicate_position_catalog())
```

- [ ] **Step 2: Run catalog tests and confirm the missing module failure**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_tactical_catalog.py`

Expected: collection fails because `simulator.tactical_catalog` does not exist.

- [ ] **Step 3: Implement typed YAML validation and the canonical YAML**

Implement explicit dataclasses with fields `key`, `label`, `asset_key`,
`default_hp`, `max_hp`, `health_field`, `position_car_id`,
`project_to_ros`, and `decision_consumed`.  Reject malformed coordinates,
non-positive max HP, duplicate unit keys, duplicate formal car IDs, and a
projectable unit without an associated formal mapping.  Put every currently
supported unit/structure mapping and default position in the YAML; remove
equivalent constants from Python only after downstream callers use the catalog.

- [ ] **Step 4: Run focused tests**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_tactical_catalog.py`

Expected: all catalog tests pass.

- [ ] **Step 5: Commit this independently testable catalog foundation**

```bash
git add src/simulator/config/tactical_catalog.yaml src/simulator/simulator/tactical_catalog.py src/simulator/test/test_tactical_catalog.py
git commit -m "simulator: add tactical catalog"
```

## Task 2: Establish a single scene model and command reducer

**Files:**
- Create: `src/simulator/simulator/scene.py`
- Modify: `src/simulator/simulator/control_bus.py`
- Modify: `src/simulator/simulator/interactive_inputs.py`
- Test: `src/simulator/test/test_scene.py`
- Test: `src/simulator/test/test_interactive_inputs.py`

**Interfaces:**
- Consumes `SceneCatalog` from Task 1.
- Produces `SceneState.from_catalog(catalog, team)`, `SceneState.apply(command)`, `SceneState.snapshot()`, and `SceneState.project_ros_inputs()`.
- Produces `normalize_scene_command(data, catalog) -> SceneCommand`.
- Maintains existing JSONL command compatibility for `set_unit`, `set_unit_hp`, `set_structure_health`, and clock controls.

- [ ] **Step 1: Add reducer tests before implementation**

```python
def test_scene_keeps_two_placed_hero_instances_and_selects_one():
    state = SceneState.from_catalog(catalog, "red")
    first = state.apply(SceneCommand.place_unit("enemy:hero:a", "enemy", "hero", 1200, 700))
    second = state.apply(SceneCommand.place_unit("enemy:hero:b", "enemy", "hero", 1300, 700))
    assert first.changed and second.changed
    assert len(state.units) == 2
    assert state.snapshot()["selected_entity_id"] == "enemy:hero:b"

def test_manual_mode_rejects_mutating_scene_commands():
    state = SceneState.from_catalog(catalog, "red", ownership_mode="manual_ros")
    result = state.apply(SceneCommand.set_structure_hp("friend:base", 3000))
    assert not result.changed
    assert result.reason == "manual_ros_observer_mode"
```

- [ ] **Step 2: Run the focused tests and confirm they fail**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_scene.py`

Expected: collection fails because `simulator.scene` does not exist.

- [ ] **Step 3: Implement state, result objects, compatibility adapter, and projection conflicts**

Use `SceneCommand` and `SceneApplyResult` dataclasses.  The reducer owns all
HP clamp, coordinate clamp, selection, placement, removal, and ownership-mode
checks.  Projection returns a structured `RosProjection` with `health`,
`positions`, `structures`, and `conflicts`; it must reject or visibly report
two movable instances that map to the same formal ROS unit instead of silently
overwriting.  Convert `SimulatorInputState` into a compatibility wrapper over
the scene model so existing command files and tests continue to work.

- [ ] **Step 4: Run scene and legacy input tests**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_scene.py src/simulator/test/test_interactive_inputs.py`

Expected: all pass.

- [ ] **Step 5: Commit the domain-model migration**

```bash
git add src/simulator/simulator/scene.py src/simulator/simulator/control_bus.py src/simulator/simulator/interactive_inputs.py src/simulator/test/test_scene.py src/simulator/test/test_interactive_inputs.py
git commit -m "simulator: centralize tactical scene state"
```

## Task 3: Make mock ROS input projection and manual ownership explicit

**Files:**
- Modify: `src/simulator/simulator/mock_inputs.py`
- Modify: `src/simulator/simulator/start.py`
- Modify: `src/simulator/simulator/ros_topic_monitor.py`
- Test: `src/simulator/test/test_mock_inputs_cli.py`
- Test: `src/simulator/test/test_scene.py`

**Interfaces:**
- Consumes `SceneState.project_ros_inputs()` from Task 2.
- Adds CLI argument `--input-owner {mock,manual_ros}` to offline start/mock input paths.
- Produces monitor state keys for `/ly/control/angles`, `/ly/control/firecode`, `/ly/control/trajectory`, `/ly/navi/should_rotate`, and `/ly/control/vel`.

- [ ] **Step 1: Write ownership and projection tests**

```python
def test_mock_mode_projects_catalog_health_and_position(monkeypatch):
    state = loaded_scene_with_enemy_hero()
    projection = state.project_ros_inputs()
    assert projection.health["enemy"]["hero"] == 300
    assert projection.positions["enemy"][101] == (1200, expected_raw_y)

def test_manual_mode_does_not_publish_scene_owned_inputs(monkeypatch):
    node = build_mock_node(input_owner="manual_ros")
    node.publish_all_once()
    assert node.published_scene_input_count == 0
```

- [ ] **Step 2: Run the tests to verify the new mode is unimplemented**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_scene.py`

Expected: fail on missing `input_owner`/projection behavior.

- [ ] **Step 3: Wire the scene projection into the existing publishers**

Only `mock` creates/publishes the formal simulator input topics.  `manual_ros`
starts an observer monitor and never opens the mock input publisher.  Reject
`--offline-decision --input-owner manual_ros` unless the user explicitly
supplies a separate external publisher, because without one the BT would have
no input source.  Preserve the current default as `mock`.

- [ ] **Step 4: Run tests and a help-contract smoke check**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_scene.py`

Run: `PYTHONPATH=src/simulator python3 -m simulator.start --help`

Expected: tests pass; help contains `--input-owner`.

- [ ] **Step 5: Commit explicit ownership behavior**

```bash
git add src/simulator/simulator/mock_inputs.py src/simulator/simulator/start.py src/simulator/simulator/ros_topic_monitor.py src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_scene.py
git commit -m "simulator: add explicit ROS input ownership"
```

## Task 4: Record final BT control output separately from feedback

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`
- Modify: `src/behavior_tree/src/SubscribeMessage.cpp`
- Modify: `src/behavior_tree/src/PublishMessage.cpp`
- Modify: `src/behavior_tree/src/RuntimeGuard.cpp`
- Modify: `src/simulator/simulator/model.py`
- Modify: `src/simulator/simulator/trace.py`
- Modify: `src/simulator/simulator/validation.py`
- Test: `src/behavior_tree/test/test_decision_trace_control_output.cpp`
- Test: `src/simulator/test/test_trace_contract.py`
- Test: `src/simulator/test/test_control_output_trace.py`

**Interfaces:**
- Adds trace fields `gimbal_feedback` and `control_output`.
- `gimbal_feedback` is a callback-only received snapshot with `available` and
  `age_ms`; legacy `gimbal.fire_code` remains unchanged for v2 readers.
- `control_output` is a sequenced snapshot captured after actual output-message
  construction.  It contains separate `angles`, `fire_code`, and `trajectory`
  subobjects, each with its own `published` state and trajectory reason.
- Simulator exposes immutable `ControlOutputState` on `TraceRecord`.
- This task intentionally does not infer a new `tactical` policy object from
  mutable GameLoop state.  A later source-driven trace extension may add
  tactical evidence after the actual-control snapshot contract is stable.

- [ ] **Step 1: Add expected trace fixtures and failing adapter tests**

```python
def test_v3_trace_keeps_feedback_and_final_command_distinct():
    record = normalize_trace_row(v3_row(feedback_rotate=0, final_rotate=3))
    assert record.gimbal_feedback.rotate == 0
    assert record.control_output.fire_code.rotate == 3

def test_v2_trace_replay_uses_empty_control_output():
    record = normalize_trace_row(v2_row())
    assert record.control_output.is_available is False
```

- [ ] **Step 2: Run Python trace tests to demonstrate the missing contract**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_trace_contract.py src/simulator/test/test_control_output_trace.py`

Expected: fail because `control_output` is unavailable.

- [ ] **Step 3: Add additive C++ trace serialization at the final owner boundary**

Copy lower-machine fire-code feedback to a dedicated received snapshot in the
subscriber before legacy `RecFireCode` can be locally flipped.  Capture a
control-output snapshot inside `PubGimbalControlData()` from the exact angle,
fire-code, and optional trajectory ROS messages that function constructs and
publishes.  Reuse the same `MakeGimbalTrajectory()` result for publication and
snapshotting; do not recompute it later from `externalAimData`.  Make
`PublishSafeControl()` capture its angle/fire-code publish with an explicitly
unpublished trajectory reason.  Guard snapshots so runtime safe-control and
trace reads cannot race.  Include sequence and age metadata, preserve old
`gimbal.fire_code` unchanged, and do not infer a new publish from a
`game_start` or `stop` trace event.

- [ ] **Step 4: Implement simulator adapter/validation compatibility**

Accept schema v2 and v3.  Treat absent v3 fields as unavailable rather than
fabricating values.  Make validation assert finite trajectory fields only when
`available:true`.

- [ ] **Step 5: Run C++ and Python checks**

Run: `colcon test --packages-select behavior_tree --event-handlers console_direct+`

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_trace_contract.py src/simulator/test/test_control_output_trace.py src/simulator/test/test_validation.py`

Expected: target build/tests pass when the sourced `sentry_msgs` provides AimResult dynamics; otherwise Python tests pass and the external message-schema blocker is reported verbatim.

- [ ] **Step 6: Commit trace evidence upgrade**

```bash
git add src/behavior_tree/include/Application.hpp src/behavior_tree/src/DecisionTrace.cpp src/behavior_tree/test/test_decision_trace_control_output.cpp src/simulator/simulator/model.py src/simulator/simulator/trace.py src/simulator/simulator/validation.py src/simulator/test/test_trace_contract.py src/simulator/test/test_control_output_trace.py
git commit -m "simulator: trace final BT control output"
```

## Task 5: Upgrade the Pygame tactical board

**Files:**
- Modify: `src/simulator/simulator/viewer.py`
- Modify: `src/simulator/simulator/inputs_panel.py`
- Modify: `src/simulator/config/default.yaml`
- Test: `src/simulator/test/test_visual_asset_qa.py`
- Test: `src/simulator/test/test_interactive_inputs.py`

**Interfaces:**
- Consumes `SceneState.snapshot()` and `TraceRecord.control_output` from Tasks 2 and 4.
- Emits only `SceneCommand` payloads via the shared command bus.
- Produces `web_status_metadata()["scene"]`, `...["decision"]`, and `...["control_output"]` for the browser.

- [ ] **Step 1: Write UI-state tests around the normalised snapshot**

```python
def test_scene_snapshot_prefers_editable_units_over_passive_trace_units():
    metadata = viewer.web_status_metadata()
    assert metadata["scene"]["show_trace_units"] is False
    assert metadata["scene"]["units"][0]["asset_key"] == "red.hero"

def test_goal_snapshot_exposes_reason_priority_and_route():
    status = viewer.web_status_metadata()
    assert status["decision"]["goal"]["id"] == 25
    assert status["decision"]["intent"]["reason"]
```

- [ ] **Step 2: Run the focused tests and confirm the new metadata is absent**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_interactive_inputs.py src/simulator/test/test_visual_asset_qa.py`

Expected: fail on the new tactical metadata assertions.

- [ ] **Step 3: Implement the board hierarchy and shared style tokens**

Draw active goal/route above terrain but below selected pieces.  Render scene
pieces once, with compact health indicators and selection feedback.  Render
trace units only when no editable scene is active or the explicit evidence
layer is enabled.  Add direct map-adjacent base/outpost health badges.  Add a
Control Output panel with final angles/fire/trajectory beside a distinct
feedback panel.  Keep font sizes fixed and apply clipping/ellipsis rules for
all labels.

- [ ] **Step 4: Run headless visual QA and targeted tests**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_visual_asset_qa.py src/simulator/test/test_interactive_inputs.py`

Expected: pass and write no repository artifacts.

- [ ] **Step 5: Capture a manual screenshot only in `/tmp` and inspect it**

Run: `PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy python3 -m simulator.visual_asset_qa --output /tmp/simulator-tactical-board.png`

Expected: a nonblank screenshot with red/blue assets, a selected goal marker, and unobscured HP bars.

- [ ] **Step 6: Commit the Pygame tactical board**

```bash
git add src/simulator/simulator/viewer.py src/simulator/simulator/inputs_panel.py src/simulator/config/default.yaml src/simulator/test/test_visual_asset_qa.py src/simulator/test/test_interactive_inputs.py
git commit -m "simulator: improve pygame tactical board"
```

## Task 6: Build a full browser tactical board on the same API

**Files:**
- Create: `src/simulator/simulator/tactical_web.py`
- Modify: `src/simulator/simulator/web_stream.py`
- Modify: `src/simulator/simulator/viewer.py`
- Modify: `src/simulator/requirements-browser.txt`
- Test: `src/simulator/test/test_tactical_web.py`
- Test: `src/simulator/test/test_web_stream.py`
- Test: `src/simulator/test/test_web_visual_check.py`

**Interfaces:**
- Serves `GET /tactical`, `GET /api/tactical-state`, and `POST /api/control`.
- Consumes `web_status_metadata()` and `normalize_scene_command()`.
- Browser posts field-centimetre coordinates, never pixels.

- [ ] **Step 1: Write HTTP contract tests**

```python
def test_tactical_page_and_state_endpoint_are_available(client):
    assert client.get("/tactical").status == 200
    state = client.get_json("/api/tactical-state")
    assert state["scene"]["ownership_mode"] == "mock"
    assert state["field"]["width_cm"] == 2800

def test_tactical_control_rejects_manual_ros_mutation(client):
    client.set_mode("manual_ros")
    response = client.post_json("/api/control", {"command": "set_unit_hp", "entity_id": "enemy:hero:a", "hp": 1})
    assert response.status == 409
```

- [ ] **Step 2: Run tests and confirm endpoints do not exist**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_tactical_web.py src/simulator/test/test_web_stream.py`

Expected: fail because `/tactical` and tactical state are not registered.

- [ ] **Step 3: Implement an accessible dependency-free map editor**

Serve semantic HTML with a responsive board, a palette, selected-piece detail
drawer, structure HP controls, timeline, decision/control strip, and live
status.  Use pointer events to map browser pixels to field centimetres using
metadata from the server.  Send only JSON commands; redraw from server state
after every result.  Use keyboard alternatives for select, delete, and HP
step actions.  Hide/disable controls in `manual_ros`, including the HTTP
server rejecting mutation so UI disabling is not the only guard.

- [ ] **Step 4: Add browser-level test when a compatible runtime exists**

Install no production frontend dependency.  If Playwright is available,
exercise `/tactical`, select an enemy piece, drag it, change base HP, and
assert command payload has `x_cm`/`y_cm`.  If unavailable, make the pytest
HTTP/API tests mandatory and record the unavailable runtime in the final
verification result.

- [ ] **Step 5: Run web tests**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_tactical_web.py src/simulator/test/test_web_stream.py src/simulator/test/test_web_visual_check.py`

Expected: all HTTP/state tests pass.

- [ ] **Step 6: Commit browser editor support**

```bash
git add src/simulator/simulator/tactical_web.py src/simulator/simulator/web_stream.py src/simulator/simulator/viewer.py src/simulator/requirements-browser.txt src/simulator/test/test_tactical_web.py src/simulator/test/test_web_stream.py src/simulator/test/test_web_visual_check.py
git commit -m "simulator: add browser tactical board"
```

## Task 7: Extend Foxglove evidence and tactical scenario fixtures

**Files:**
- Modify: `src/simulator/simulator/foxglove_export.py`
- Modify: `src/simulator/simulator/decision_input_coverage.py`
- Create: `src/simulator/sample/scenarios/tactical_protect_castle.jsonl`
- Create: `src/simulator/sample/scenarios/tactical_follow_rotate.jsonl`
- Create: `src/simulator/sample/unit_scenes/tactical_board.yaml`
- Create: `src/simulator/sample/mock_sequences/tactical_protection.json`
- Test: `src/simulator/test/test_foxglove_export.py`
- Test: `src/simulator/test/test_decision_input_coverage.py`
- Test: `src/simulator/test/test_scenarios.py`

**Interfaces:**
- Adds MCAP channels `decision/control_output`, `decision/tactical`, and `simulator/scene`.
- Exposes workflow evidence fields for final rotate/follow and ProtectCastle source.

- [ ] **Step 1: Write failing export/scenario tests**

```python
def test_export_contains_final_control_and_tactical_channels(tmp_path: Path):
    channels = export_and_list_channels(v3_trace_path, tmp_path / "out.mcap")
    assert "decision/control_output" in channels
    assert "decision/tactical" in channels

def test_tactical_protect_castle_scenario_has_explicit_goal_evidence():
    report = validate_trace_file(protect_castle_scenario_path())
    assert report.ok
    assert report.records[-1].tactical.protect_castle_enemy_pos_active
```

- [ ] **Step 2: Run tests to confirm new channels/fixtures are absent**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_foxglove_export.py src/simulator/test/test_scenarios.py`

Expected: fail on missing channels or fixtures.

- [ ] **Step 3: Serialize normalised evidence, not raw private state**

Map `ControlOutputState` and `TacticalDecisionState` to versioned JSON MCAP
schemas.  Add scene snapshots as independent observation channels.  Create
the two fixtures from stable expected evidence only: ProtectCastle source
activation and FollowMode forcing final rotate to zero.  Register them in the
scenario manifest and decision-input coverage catalog.

- [ ] **Step 4: Run export/scenario tests**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_foxglove_export.py src/simulator/test/test_decision_input_coverage.py src/simulator/test/test_scenarios.py`

Expected: all pass.

- [ ] **Step 5: Commit observable tactical scenarios**

```bash
git add src/simulator/simulator/foxglove_export.py src/simulator/simulator/decision_input_coverage.py src/simulator/sample/scenarios/tactical_protect_castle.jsonl src/simulator/sample/scenarios/tactical_follow_rotate.jsonl src/simulator/sample/unit_scenes/tactical_board.yaml src/simulator/sample/mock_sequences/tactical_protection.json src/simulator/test/test_foxglove_export.py src/simulator/test/test_decision_input_coverage.py src/simulator/test/test_scenarios.py
git commit -m "simulator: export tactical control evidence"
```

## Task 8: Document, update graph, and perform end-to-end acceptance

**Files:**
- Modify: `docs/sentry/internal/simulator.md`
- Modify: `src/simulator/README.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`
- Modify: `.understand-anything/intermediate/scan-result.json` when regenerated inventory differs
- Test: `src/simulator/test/test_quality_cli.py`

**Interfaces:**
- Documents `--input-owner`, `/tactical`, scene catalog, final control trace, Foxglove ownership boundary, and acceptance command.

- [ ] **Step 1: Write documentation/quality assertions**

```python
def test_simulator_docs_describe_both_input_ownership_modes():
    text = Path("docs/sentry/internal/simulator.md").read_text(encoding="utf-8")
    assert "manual_ros" in text
    assert "mock" in text
    assert "/tactical" in text
```

- [ ] **Step 2: Run the test and verify it fails before docs are updated**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_quality_cli.py`

Expected: fail on missing tactical mode documentation.

- [ ] **Step 3: Update source-backed docs and graph**

Document exact launch commands for local Pygame, browser board, and manual
Foxglove observer mode.  Update the graph to show `SceneCatalog`, Pygame,
browser, mock publisher, final-control monitor, and trace/MCAP edges.  Update
all graph generated/checked metadata with current HEAD and current working
tree facts; state fallback graph mode if the Understand Anything plugin remains unavailable.

- [ ] **Step 4: Run mandatory verification**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test`

Run: `python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null`

Run: `python3 -m json.tool .understand-anything/meta.json >/dev/null`

Run: `git diff --check`

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: simulator suite passes, JSON parses, no whitespace errors, and static self-check passes or external `sentry_msgs` schema failure is captured exactly.

- [ ] **Step 5: Run a launch-level smoke acceptance when ROS dependencies exist**

Run: `PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --input-owner mock --live-view --unit-scene src/simulator/sample/unit_scenes/tactical_board.yaml`

Expected: Pygame board and `http://127.0.0.1:9000/tactical` both show the same scene; changing a structure HP in either client appears in `/status.json`, then in trace referee fields.  Stop the smoke process normally after inspection.

- [ ] **Step 6: Commit docs, graph, and verification tests**

```bash
git add docs/sentry/internal/simulator.md src/simulator/README.md .understand-anything/knowledge-graph.json .understand-anything/project-knowledge-graph.md .understand-anything/meta.json .understand-anything/intermediate/scan-result.json src/simulator/test/test_quality_cli.py
git commit -m "docs: document tactical simulator"
```

## Plan Self-Review

- Every required feature maps to a task: catalog/anti-hardcode (1-2), safe ROS/Foxglove boundary (3), actual BT command evidence (4), Pygame UI (5), browser UI (6), Foxglove/scenarios (7), and documentation/acceptance (8).
- The only intentionally conditional check is real browser automation; HTTP/API verification remains mandatory if Playwright is not installed.
- Trace v2 compatibility and separate command-vs-feedback meanings are explicit in Tasks 4 and 7.
- Formal runtime behavior remains outside the change boundary in every task.
