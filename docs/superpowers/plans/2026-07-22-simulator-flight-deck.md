# Simulator Flight Deck Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply the approved Sentinel Flight Deck visual language to pygame and `/tactical` without changing simulator data, commands, ROS links, or interaction semantics.

**Architecture:** Keep `Viewer` and `build_tactical_html()` as native renderers. Use existing `default.yaml` palette/config for pygame and matching dependency-free CSS tokens for browser. All scene, selection, command bus, and API state remains authoritative where it is.

**Tech Stack:** Python 3, pygame, YAML, dependency-free HTML/CSS/JavaScript, pytest, existing Playwright visual checker.

## Global Constraints

- Do not change `SceneCatalog`, `SimulatorInputState`, trace schema, ROS topics, `/api/tactical-state`, `/api/control`, command payloads, or ownership semantics.
- Do not add a framework, build step, database, second state source, or network-loaded icon dependency.
- Browser icons are a local minimal Lucide-compatible SVG subset; pygame uses native equivalents plus visible text labels.
- Use only the exact Flight Deck palette in `docs/superpowers/specs/2026-07-22-simulator-flight-deck-design.md`.
- Browser uses only 18 px title, 15 px content, 13 px caption; browser inputs/buttons remain at least 40 px high.
- Preserve Fit, 1:1, Zoom to Selection, pan, fullscreen, docking, shelf, HP, drag/drop, timeline, undo, mock edit, and `manual_ros` read-only paths.
- Stage only files named by each task; this workspace contains unrelated dirty runtime work.

---

### Task 1: Lock the Flight Deck token contract

**Files:**
- Modify: `src/simulator/config/default.yaml`
- Modify: `src/simulator/simulator/tactical_web.py`
- Modify: `src/simulator/test/test_pygame_workspace.py`
- Modify: `src/simulator/test/test_tactical_web.py`

**Interfaces:**
- Consumes: existing `colors` YAML and tactical HTML output.
- Produces: identical named Flight Deck values in both renderers, no API/interface change.

- [ ] **Step 1: Write failing token tests**

```python
def test_default_pygame_palette_uses_flight_deck_tokens() -> None:
    colors = load_config(None)["colors"]
    assert colors["bg"] == "#111418"
    assert colors["panel"] == "#1D232B"
    assert colors["panel2"] == "#202832"
    assert colors["text"] == "#F5F7FA"
    assert colors["muted"] == "#AEB7C2"
    assert colors["accent"] == "#4DB7FF"


def test_tactical_html_has_flight_deck_css_tokens() -> None:
    body = build_tactical_html(9011).decode("utf-8")
    for value in ("#111418", "#171C22", "#1D232B", "#202832", "#4DB7FF"):
        assert value in body
    assert "--fd-border:rgba(255,255,255,.06)" in body
```

- [ ] **Step 2: Verify the new tests fail**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_pygame_workspace.py \
  src/simulator/test/test_tactical_web.py
```

Expected: exact token assertions fail against the old colours.

- [ ] **Step 3: Apply the approved tokens without renaming public controls**

Set `default.yaml` values:

```yaml
colors:
  bg: "#111418"
  panel: "#1D232B"
  panel2: "#202832"
  text: "#F5F7FA"
  muted: "#AEB7C2"
  line: "#FFFFFF0F"
  accent: "#4DB7FF"
  friend: "#58D68D"
  enemy: "#FF6262"
  neutral: "#4DB7FF"
```

In browser CSS, declare and use:

```css
:root { --fd-root:#111418; --fd-raised:#171C22; --fd-panel:#1D232B;
  --fd-card:#202832; --fd-border:rgba(255,255,255,.06); --fd-text:#F5F7FA;
  --fd-muted:#AEB7C2; --fd-accent:#4DB7FF; --fd-warning:#FFB648;
  --fd-danger:#FF6262; --fd-success:#58D68D; }
```

Do not rename DOM IDs, CSS hooks used by tests, JavaScript command calls, or localStorage fields.

- [ ] **Step 4: Verify and commit the token foundation**

Run the Step 2 command; then:

```bash
git add src/simulator/config/default.yaml src/simulator/simulator/tactical_web.py \
  src/simulator/test/test_pygame_workspace.py src/simulator/test/test_tactical_web.py
git commit -m "simulator: add flight deck visual tokens"
```

### Task 2: Recompose `/tactical` as Sentinel Flight Deck

**Files:**
- Modify: `src/simulator/simulator/tactical_web.py`
- Modify: `src/simulator/simulator/web_visual_check.py`
- Modify: `src/simulator/test/test_tactical_web.py`
- Modify: `src/simulator/test/test_web_visual_check.py`

**Interfaces:**
- Consumes: existing workspace DOM (`workspaceShell`, `activityRail`, `inspector`, `operationsShelf`), current selection, and command calls.
- Produces: same functionality with Flight Deck cards/icons/type/motion.

- [ ] **Step 1: Write failing visual-semantic tests**

```python
def test_tactical_html_has_flight_deck_card_icon_and_motion_contract() -> None:
    body = build_tactical_html(9011).decode("utf-8")
    assert 'class="flight-deck-card"' in body
    assert 'data-lucide="crosshair"' in body
    assert '@media (prefers-reduced-motion: reduce)' in body
    assert 'transition:transform 150ms ease-out' in body
    assert 'border-radius:12px' in body
```

- [ ] **Step 2: Verify the visual semantic test fails**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_tactical_web.py \
  src/simulator/test/test_web_visual_check.py
```

Expected: card, icon, and motion assertions fail before the re-skin.

- [ ] **Step 3: Implement browser-only Flight Deck primitives**

Inside `tactical_web.py`:

```python
def inline_icon(name: str, label: str) -> str:
    """Return local Lucide-compatible SVG markup with an accessible label."""
```

Use it for the existing rail, command bar, map controls, Inspector headings, and shelf, keeping adjacent
visible labels. Apply one `flight-deck-card` grammar to cards/docks: 12 px radius, `--fd-card` translucent
material, 1 px subtle separator, low shadow, upper highlight, and 150 ms hover lift. Use only 18/15/13 px
text roles. Keep action/input heights at least 40 px. Add 250 ms card/dock expansion and a
`prefers-reduced-motion` override. Do not alter JavaScript interaction logic.

- [ ] **Step 4: Add measurable visual checks**

In `inspect_tactical`, keep existing interaction checks and add:

```python
assert page.locator("#fieldBoard").bounding_box()["width"] > page.locator("#inspector").bounding_box()["width"] * 2
assert page.locator(".flight-deck-card").count() >= 4
assert page.evaluate("document.documentElement.scrollWidth <= document.documentElement.clientWidth")
```

- [ ] **Step 5: Verify, inspect, and commit browser output**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_tactical_web.py \
  src/simulator/test/test_web_visual_check.py
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check \
  --output-dir /tmp/ly-simulator-flight-deck-web
```

Inspect desktop/narrow screenshots for map dominance, no clipping, no overflow, and only blue operation accent.

```bash
git add src/simulator/simulator/tactical_web.py src/simulator/simulator/web_visual_check.py \
  src/simulator/test/test_tactical_web.py src/simulator/test/test_web_visual_check.py
git commit -m "simulator: style tactical board as flight deck"
```

### Task 3: Apply the same grammar to pygame

**Files:**
- Modify: `src/simulator/simulator/viewer.py`
- Modify: `src/simulator/config/default.yaml`
- Modify: `src/simulator/test/test_pygame_workspace.py`
- Modify: `src/simulator/test/test_interactive_inputs.py`

**Interfaces:**
- Consumes: unchanged `WorkspaceLayout`, `ViewportState`, selection, and control-bus paths.
- Produces: identical pygame actions with Flight Deck visual hierarchy and native icon glyphs.

- [ ] **Step 1: Write failing pygame presentation/control tests**

```python
def test_viewer_uses_flight_deck_palette_and_three_text_roles(viewer) -> None:
    assert viewer.palette["bg"] == viewer.pg.Color("#111418")
    assert viewer.palette["accent"] == viewer.pg.Color("#4DB7FF")
    assert viewer.title_font.get_height() > viewer.font.get_height() > viewer.small_font.get_height()


def test_context_selection_keeps_existing_command_subjects(viewer) -> None:
    viewer.select_structure("friend_base")
    assert viewer.workspace_selection.kind == "structure"
    viewer.select_overview()
    assert viewer.workspace_selection.kind == "overview"
```

- [ ] **Step 2: Verify pygame presentation test fails**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_pygame_workspace.py \
  src/simulator/test/test_interactive_inputs.py
```

Expected: exact Flight Deck palette assertion fails; selection/control regression remains green.

- [ ] **Step 3: Add focused Viewer drawing helpers**

Add presentation-only helpers inside `Viewer`:

```python
def draw_flight_deck_card(self, rect: Any, *, active: bool = False) -> None:
    self.pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=12)

def draw_flight_deck_icon(self, name: str, center: tuple[int, int], *, active: bool = False) -> None:
    self.pg.draw.circle(self.screen, self.palette["accent"], center, 8, 1)

def draw_flight_deck_title(self, label: str, x: int, y: int, width: int) -> int:
    return self.draw_text(label, x, y, self.title_font, self.palette["text"], width)

def draw_flight_deck_caption(self, label: str, x: int, y: int, width: int) -> int:
    return self.draw_text(label, x, y, self.small_font, self.palette["muted"], width)
```

Apply them to command bar, Activity rail, map toolbar, Inspector/context cards, shelf, and controls. Native
icons use line/circle/rect primitives for the approved semantic names. Do not change hit rectangles,
commands, selection order, ownership gates, map coordinate transforms, or Viewer data methods.

- [ ] **Step 4: Apply visual hierarchy**

- Replace non-team yellow active/focus visuals with `#4DB7FF`.
- Use 12 px cards, subtle separators, a one-pass shadow/top highlight, and calm panel backgrounds.
- Keep team red/blue data and success/warning/danger health semantics separate from UI accent state.
- Keep contextual Inspector card first; active activity content is visually secondary but remains equally usable.
- Preserve F11/Esc, Fit, 1:1, Focus, wheel zoom, middle pan, dock/splitter, shelf, and Reset Layout.

- [ ] **Step 5: Verify screenshot and commit pygame output**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q \
  src/simulator/test/test_pygame_workspace.py \
  src/simulator/test/test_interactive_inputs.py \
  src/simulator/test/test_trace_contract.py \
  src/simulator/test/test_validation_cli.py
SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/tactical_protect_castle.jsonl \
  --smoke-test --smoke-screenshot /tmp/ly-simulator-flight-deck-pygame.png
```

Inspect the 1500×900 image for no blank region, clipped content, or map-obscuring panels; then:

```bash
git add src/simulator/simulator/viewer.py src/simulator/config/default.yaml \
  src/simulator/test/test_pygame_workspace.py src/simulator/test/test_interactive_inputs.py
git commit -m "simulator: style pygame viewer as flight deck"
```

### Task 4: Document and validate the completed design language

**Files:**
- Modify: `docs/sentry/internal/simulator.md`
- Modify: `src/simulator/README.md`
- Modify: `src/simulator/test/test_quality_cli.py`

**Interfaces:**
- Consumes: approved Flight Deck spec and current simulator docs.
- Produces: accurate presentation-only documentation; no runtime interface change.

- [ ] **Step 1: Write failing documentation acceptance test**

```python
def test_simulator_docs_describe_flight_deck_identity() -> None:
    for path in (SIMULATOR_DOC, SIMULATOR_README):
        text = path.read_text(encoding="utf-8")
        assert "Sentinel Flight Deck" in text
        assert "#4DB7FF" in text
        assert "presentation-only" in text
        assert "manual_ros" in text
        assert "official centimeter" in text
```

- [ ] **Step 2: Verify the documentation test fails before its update**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_quality_cli.py
```

Expected: Flight Deck identity/token text is absent.

- [ ] **Step 3: Document only the user-visible identity and invariants**

Update both docs with Sentinel Flight Deck, palette, Activity rail, contextual Inspector, Operations shelf,
viewport controls, map dominance, presentation-only boundary, `manual_ros` read-only rule, and official
centimeter coordinate invariant. Keep `Updated: 2026-07-22` current. Do not document CSS internals.

- [ ] **Step 4: Run final acceptance checks**

Run:

```bash
PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check \
  --output-dir /tmp/ly-simulator-flight-deck-final
SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/tactical_protect_castle.jsonl \
  --smoke-test --smoke-screenshot /tmp/ly-simulator-flight-deck-final.png
./scripts/selfcheck.sh sentry --static-only
git diff --check
```

Expected: tests/browser check/screenshots/whitespace pass. If selfcheck lacks external ROS or `sentry_msgs`,
record that exact environment failure; do not bypass it in simulator code.

- [ ] **Step 5: Inspect final outputs and commit docs**

Inspect:

```text
/tmp/ly-simulator-flight-deck-final/web-tactical-desktop.png
/tmp/ly-simulator-flight-deck-final/web-tactical-narrow.png
/tmp/ly-simulator-flight-deck-final.png
```

Then:

```bash
git add docs/sentry/internal/simulator.md src/simulator/README.md \
  src/simulator/test/test_quality_cli.py
git commit -m "docs: record simulator flight deck controls"
```

## Plan self-review

- Task 1 locks colours; Task 2 covers browser cards/icons/type/motion; Task 3 covers pygame equivalents;
  Task 4 covers current docs and full validation.
- No task changes simulator data, commands, backend/API, ROS, ownership, or decision logic.
- Every implementation slice starts with a concrete failing test, ends with explicit verification, and has an
  isolated commit.
