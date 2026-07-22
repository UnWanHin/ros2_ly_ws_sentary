# Simulator Workspace Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the pygame and browser tactical prototype layouts with matching, map-first dockable workspaces while retaining every existing simulator control and data contract.

**Architecture:** Add a presentation-only workspace model for viewport math, selection, dock state, and local layout preferences. Keep status/scene/command data in the existing simulator sources; render the model natively in pygame and dependency-free HTML/CSS/JS in the browser. The web and pygame clients use the same viewport commands and selection semantics, but neither gains a second scene or backend.

**Tech Stack:** Python 3, pygame, stdlib HTTP server, dependency-free HTML/CSS/JavaScript, pytest, optional Playwright screenshots.

## Global Constraints

- Do not alter BT, ROS topics, simulator rules, `/api/tactical-state`, `/api/control`, `SceneCatalog`, `SimulatorInputState`, or control-bus command payloads.
- Preserve `mock` edit ownership and `manual_ros` observer-only behavior.
- Do not add a database, graph, duplicate scene state, web build step, or browser dependency.
- Dock panels only to left, right, and bottom; never use floating panels over the battlefield.
- Keep all user-facing type at least 14px; use 40px minimum buttons/inputs in the browser.
- Preserve official centimeter coordinate conversion independent of visual zoom.
- Test each increment and commit it independently. Do not stage unrelated workspace changes.

---

### Task 1: Create the presentation-only workspace and viewport model

**Files:**
- Create: `src/simulator/simulator/workspace.py`
- Create: `src/simulator/test/test_workspace.py`
- Modify: `src/simulator/simulator/viewer.py: Viewer.__init__, map_image_rect, reset_map_view, adjust_map_zoom`

**Interfaces:**
- Consumes: `FieldGeometry`, viewport pixel rectangles, and existing selected scene IDs.
- Produces: `WorkspaceLayout`, `ViewportState`, `Selection`, `fit_viewport`, `zoom_at_screen_point`, `pan_viewport`, and `zoom_to_field_bounds`.
- Must not consume or mutate scene/control state.

- [ ] **Step 1: Write failing viewport math and layout-state tests**

```python
from simulator.workspace import Rect, ViewportState, WorkspaceLayout

def test_zoom_keeps_pointer_field_coordinate_fixed():
    state = ViewportState.fit(field_width=2800, field_height=1500, viewport=Rect(0, 0, 1120, 700))
    before = state.screen_to_field(725, 340)
    state.zoom_at(725, 340, factor=1.2)
    after = state.screen_to_field(725, 340)
    assert after == pytest.approx(before)

def test_workspace_layout_keeps_center_map_primary_when_inspector_is_open():
    layout = WorkspaceLayout.desktop(1440, 900, inspector_width=340, shelf_collapsed=True)
    assert layout.viewport.width / layout.content_width >= 0.76
    assert layout.inspector.zone == "right"

def test_zoom_to_selection_centers_selection_inside_viewport():
    state = ViewportState.fit(field_width=2800, field_height=1500, viewport=Rect(0, 0, 1000, 700))
    state.zoom_to_field_bounds(Rect(1880, 780, 160, 160))
    cx, cy = state.screen_to_field(500, 350)
    assert (cx, cy) == pytest.approx((1960, 860), abs=2)
```

- [ ] **Step 2: Run the new tests and verify the expected failure**

Run: `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_workspace.py`

Expected: collection fails because `simulator.workspace` does not exist.

- [ ] **Step 3: Implement immutable workspace primitives**

Create `workspace.py` with no pygame imports. The implementation must expose typed dataclasses equivalent to:

```python
@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    width: float
    height: float

@dataclass
class ViewportState:
    field_width: float
    field_height: float
    viewport: Rect
    scale: float
    center_x: float
    center_y: float

    def screen_to_field(self, x: float, y: float) -> tuple[float, float]: ...
    def field_to_screen(self, x: float, y: float) -> tuple[float, float]: ...
    def zoom_at(self, x: float, y: float, factor: float) -> None: ...
    def pan_by(self, dx: float, dy: float) -> None: ...
    def fit(self, padding_px: float = 24.0) -> None: ...
    def actual_size(self, map_pixel_width: float) -> None: ...
    def zoom_to_field_bounds(self, bounds: Rect, padding_px: float = 96.0) -> None: ...
```

Implement `zoom_at` by reading the field coordinate before scale changes, clamping scale to `[0.5, 4.0]`, then solving the new center so the same coordinate maps to the original screen point. `WorkspaceLayout.desktop` must reserve the compact activity rail, Inspector, command bar, and optional shelf before calculating the viewport rectangle.

- [ ] **Step 4: Adapt pygame’s map geometry without changing commands**

Replace the independent `map_zoom`/`map_pan` calculations in `Viewer` with one `ViewportState`. Convert its existing `map_image_rect`, `field_to_screen`, `screen_to_field`, Fit, 1:1, wheel, and middle-pan paths to delegate to that state. Keep the existing control-bus and structure/unit command calls untouched.

- [ ] **Step 5: Run focused tests and source checks**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_workspace.py \
  src/simulator/test/test_scene.py \
  src/simulator/test/test_interactive_inputs.py
git diff --check
```

Expected: all tests pass and no whitespace error is reported.

- [ ] **Step 6: Commit the isolated foundation**

```bash
git add src/simulator/simulator/workspace.py src/simulator/simulator/viewer.py src/simulator/test/test_workspace.py
git commit -m "simulator: add shared workspace viewport model"
```

### Task 2: Rebuild the browser tactical workspace shell

**Files:**
- Modify: `src/simulator/simulator/tactical_web.py: build_tactical_html`
- Modify: `src/simulator/test/test_tactical_web.py`
- Modify: `src/simulator/simulator/web_visual_check.py: inspect_tactical`
- Modify: `src/simulator/test/test_web_visual_check.py`

**Interfaces:**
- Consumes: unchanged `TACTICAL_STATE_SCHEMA` payload from `tactical_state_from_status`.
- Produces: the existing `/tactical` page and existing `/api/control` commands only.
- Preserves: `mapViewport`, `fieldBoard`, scene edit ownership, command behavior, and local browser preference persistence.

- [ ] **Step 1: Write failing browser markup and interaction tests**

Add assertions that the response contains semantic workspace regions and controls:

```python
assert 'id="workspaceShell"' in body
assert 'id="activityRail"' in body
assert 'id="operationsShelf"' in body
assert 'id="viewportZoomSelection"' in body
assert 'data-selection-kind="overview"' in body
assert 'id="inspectorDockHandle"' in body
assert 'id="shelfResizeHandle"' in body
```

When Playwright is available, add a test that selects a Base, reads `#inspectorTitle`, collapses the shelf, drags the Inspector splitter, uses the mouse wheel over the map, and confirms the browser emits no console errors.

- [ ] **Step 2: Run tests and verify failure**

Run: `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_tactical_web.py src/simulator/test/test_web_visual_check.py`

Expected: markup assertions fail because the workspace shell has not been rendered.

- [ ] **Step 3: Replace the fixed web shell with semantic dock regions**

Within `build_tactical_html`, replace the old fixed `.shell` structure with these semantic regions:

```html
<main id="workspaceShell" data-inspector-dock="right">
  <nav id="activityRail" aria-label="Workspace surfaces">…</nav>
  <section id="battlefieldWorkspace">
    <div id="viewportToolbar">…</div>
    <div id="mapViewport"><div id="mapCanvas"><div id="fieldBoard">…</div></div></div>
  </section>
  <aside id="inspector" aria-label="Inspector"><div id="inspectorDockHandle"></div>…</aside>
  <section id="operationsShelf"><div id="shelfResizeHandle"></div>…</section>
</main>
```

Use CSS Grid variables for Inspector width and shelf height. The layout must keep the activity rail compact, dock regions non-overlapping, field minimum size valid, and keyboard focus visible. Replace fixed, dense tab rows with labeled activity rail buttons and collapsible cards that show only one expanded card by default.

- [ ] **Step 4: Implement browser viewport and dock behavior**

Implement browser-local workspace preferences using the existing `localStorage` mechanism. Preserve only presentation values: Inspector dock/width/collapse, shelf height/collapse, active activity surface, compact/focus/debug flags, perspective, zoom, and pan.

Implement the same coordinate-preserving zoom algorithm as Task 1 in JavaScript: calculate the official field coordinate under the pointer, apply clamped zoom, and solve pan for that coordinate. Add Fit, 1:1, Zoom to Selection, focus, Escape/fullscreen, pointer pan, and resize behavior. Map click selection must update only UI selection; unit/structure mutations must continue to invoke existing `command({...})` payloads.

- [ ] **Step 5: Preserve and contextualize every existing surface**

Render inspectors by selection kind using the existing state fields. Keep Overview expanded initially and expose existing controls as follows:

- Base/Outpost: `set_structure_health`, small step buttons, Apply, Restore, Destroy, Undo.
- Robot: existing `set_unit_hp`, drag placement, and current decision/navigation values.
- Area: catalog label and current visible navigation/decision context only.
- Overview: existing match state, goal, decision, and final control snapshot.

Move existing Decision, Events, Runtime, Control, Inputs, and Layers renderers behind Activity rail selections; do not remove any data path. Keep Runtime content behind Debug Mode.

- [ ] **Step 6: Verify browser visual contracts**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_tactical_web.py \
  src/simulator/test/test_web_visual_check.py
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check \
  --output-dir /tmp/ly-simulator-web-workspace-check
```

Expected: tests pass; when Playwright is installed, desktop and narrow screenshots are written and no layout finding is reported. If Playwright is unavailable, record its explicit skip reason and retain the unit tests.

- [ ] **Step 7: Commit the web workspace**

```bash
git add src/simulator/simulator/tactical_web.py src/simulator/simulator/web_visual_check.py \
  src/simulator/test/test_tactical_web.py src/simulator/test/test_web_visual_check.py
git commit -m "simulator: rebuild tactical web workspace"
```

### Task 3: Rebuild pygame into the same docked workspace

**Files:**
- Modify: `src/simulator/simulator/viewer.py: Viewer.draw, input handlers, panel drawing`
- Modify: `src/simulator/simulator/inputs_panel.py`
- Modify: `src/simulator/config/default.yaml: window presentation defaults only`
- Modify: `src/simulator/test/test_trace_contract.py`
- Modify: `src/simulator/test/test_validation_cli.py`

**Interfaces:**
- Consumes: `WorkspaceLayout` and `ViewportState` from Task 1 plus current `Viewer` trace/scene state.
- Produces: the existing pygame viewer and current file-control commands.
- Preserves: `record_status_payload`, trace rendering, `InputsPanel` commands, scene ownership, and default map/catalog assumptions.

- [ ] **Step 1: Write failing pygame layout and selection tests**

Add renderer-independent assertions by constructing a viewer with the sample tactical scene:

```python
def test_viewer_workspace_selection_switches_between_overview_unit_and_structure(viewer):
    viewer.select_overview()
    assert viewer.workspace_selection.kind == "overview"
    viewer.select_unit("enemy:hero:demo")
    assert viewer.workspace_selection.kind == "unit"
    viewer.select_structure("friend_base")
    assert viewer.workspace_selection.kind == "structure"

def test_viewer_workspace_resize_keeps_field_center(viewer):
    before = viewer.map_viewport.screen_to_field(*viewer.map_viewport.viewport.center)
    viewer.resize_workspace(1720, 980)
    after = viewer.map_viewport.screen_to_field(*viewer.map_viewport.viewport.center)
    assert after == pytest.approx(before)
```

- [ ] **Step 2: Run tests and verify failure**

Run: `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_trace_contract.py src/simulator/test/test_validation_cli.py`

Expected: the new Viewer workspace methods are missing.

- [ ] **Step 3: Implement the pygame workspace shell**

Refactor drawing into explicit, testable methods inside `Viewer` or presentation-focused helper classes:

```python
def draw_command_bar(self, rect): ...
def draw_activity_rail(self, rect): ...
def draw_battlefield_viewport(self, rect): ...
def draw_contextual_inspector(self, rect): ...
def draw_operations_shelf(self, rect): ...
```

Compute every rectangle from `WorkspaceLayout`; remove direct reliance on fixed `panel_w` and `timeline_h` for the overall shell. The center viewport must use `ViewportState`, not bespoke zoom/pan fields. Retain all existing render data and buttons by moving their drawing ownership, not changing their content source.

- [ ] **Step 4: Implement pygame docking, splitter, cards, and contextual selection**

Add title-bar dock controls that move Inspector left/right and collapse it; add bottom shelf collapse and splitter hit targets. Add Reset Layout and preserve fullscreen/Escape behavior. Use selection methods for empty map, unit, structure, and area. The contextual Inspector must offer the same existing unit/structure commands as the browser and default to one expanded Overview card.

Use map drawing order so route/goal/object highlights remain visible, and preserve the existing `send_sim_command` payloads. Selecting an area must not generate a simulator mutation.

- [ ] **Step 5: Capture pygame visual smoke checks**

Run:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/tactical_protect_castle.jsonl \
  --smoke-test --smoke-screenshot /tmp/ly-simulator-workspace-pygame.png
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_trace_contract.py \
  src/simulator/test/test_validation_cli.py
```

Expected: the screenshot file exists, trace and validation tests pass, and visual inspection confirms no overlap/cutoff at the default desktop geometry.

- [ ] **Step 6: Commit the pygame workspace**

```bash
git add src/simulator/simulator/viewer.py src/simulator/simulator/inputs_panel.py \
  src/simulator/config/default.yaml src/simulator/test/test_trace_contract.py \
  src/simulator/test/test_validation_cli.py
git commit -m "simulator: rebuild pygame workspace"
```

### Task 4: Integrate validation, documentation, and final quality gate

**Files:**
- Modify: `docs/sentry/internal/simulator.md`
- Modify: `src/simulator/README.md`
- Modify: `src/simulator/simulator/web_visual_check.py`
- Modify: `src/simulator/test/test_quality_cli.py`

**Interfaces:**
- Consumes: unchanged launch/command instructions and the new presentation controls.
- Produces: current simulator documentation and visual acceptance checks.

- [ ] **Step 1: Write failing quality/documentation tests**

Add exact assertions to `test_quality_cli.py` for current documentation terms:

```python
assert "Zoom to Selection" in text
assert "Activity rail" in text
assert "Operations shelf" in text
assert "manual_ros" in text
assert "official centimeter" in text
```

Add visual-check expectations for `#workspaceShell`, visible field dominance, no horizontal overflow, and absence of console errors when Playwright is available.

- [ ] **Step 2: Run tests and verify failure**

Run: `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_quality_cli.py src/simulator/test/test_web_visual_check.py`

Expected: current documentation and visual inspection do not yet contain the new workspace guarantees.

- [ ] **Step 3: Update current simulator documentation**

Update `docs/sentry/internal/simulator.md` and `src/simulator/README.md` with the actual controls, dock behavior, Inspector selection rules, observer-mode constraint, both launch paths, and an updated `Updated: 2026-07-22` line. State explicitly that the browser and pygame use the same catalog-backed scene and command bus, while layout preferences are presentation-only.

- [ ] **Step 4: Run the complete targeted suite and static checks**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check \
  --output-dir /tmp/ly-simulator-workspace-final
./scripts/selfcheck.sh sentry --static-only
git diff --check
```

Expected: all available simulator tests pass; the browser check passes or explicitly reports unavailable optional Playwright; static self-check passes; no whitespace error remains.

- [ ] **Step 5: Review screenshots and verify contracts manually**

Inspect the browser desktop/narrow/fullscreen captures and pygame smoke capture. Confirm: battlefield is dominant, docked panels do not overlap it, each Inspector mode is contextual, cards are readable, no text is clipped, controls are at least 40px in browser, selection and viewport feedback are visible, and no debug content appears unless Debug is active.

- [ ] **Step 6: Commit docs and final validation changes**

```bash
git add docs/sentry/internal/simulator.md src/simulator/README.md \
  src/simulator/simulator/web_visual_check.py src/simulator/test/test_quality_cli.py \
  src/simulator/test/test_web_visual_check.py
git commit -m "docs: document simulator workspace controls"
```

## Plan self-review

- Spec coverage: Tasks 1–3 cover docked workspace, contextual selection, pointer-preserving viewport behavior, browser/pygame parity, and unchanged control contracts. Task 4 covers docs and verification.
- No-placeholder scan: no `TBD`, deferred implementation text, or unspecified test command remains.
- Interface consistency: both renderer tasks consume `WorkspaceLayout`/`ViewportState`; no task changes scene payloads or control endpoints.
