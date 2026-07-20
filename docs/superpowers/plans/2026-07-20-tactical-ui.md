# Tactical UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing simulator tactical board a polished map-first command interface without changing simulator or ROS contracts.

**Architecture:** Keep `build_tactical_html()` as the only frontend source and retain the existing command bus. Add browser-local state for perspective, viewport transform, accordion expansion, and debug visibility; command payloads remain unchanged.

**Tech Stack:** Python-generated dependency-free HTML/CSS/JavaScript, existing `SimulatorWebStream`, pytest, optional Playwright screenshots.

## Global Constraints

- Do not add modules, change backend APIs, simulator logic, ROS inputs, or feature names.
- Preserve all existing commands and make their controls at least as accessible as before.
- Use a dark professional UI with one amber interface accent; red/blue are team semantics only.
- Keep text at 14px or larger, consistent spacing, responsive geometry, and no clipped/overlapping controls.
- Hide diagnostics unless Debug is enabled; keep match controls and HP editing directly reachable.

---

### Task 1: Lock The Existing Tactical Contract

**Files:** `src/simulator/test/test_tactical_web.py`

- [ ] Write failing tactical HTML assertions for `viewSideRed`, `viewSideBlue`,
  `mapZoomIn`, `resetView`, `debugToggle`, `mapViewport`, and accordion
  `aria-expanded` controls.
- [ ] Run `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test/test_tactical_web.py` and verify the new contract fails.
- [ ] Commit with `test: define tactical command desk contract`.

### Task 2: Implement The Command Desk Presentation

**Files:** `src/simulator/simulator/tactical_web.py`; test
`src/simulator/test/test_tactical_web.py`.

- [ ] Add a map-first DOM/CSS layout: a stable top command bar, `mapViewport`
  around the existing field board, responsive inspector dock, matching button
  heights, and details/summary accordion cards.
- [ ] Retain `command(payload)` and every existing payload name. Render unit
  and structure HP with decrement, number input, increment, and reset-to-max
  actions using existing `set_unit_hp` and `set_structure_health` commands.
- [ ] Add local `{side, zoom, panX, panY, debug}` state. Apply it to the map,
  and apply its inverse in `readPoint()` before existing official-centimeter
  command payloads are sent.
- [ ] Run focused tactical and visual tests; commit with
  `simulator: polish tactical command desk`.

### Task 3: Document And Validate The UI

**Files:** `docs/sentry/internal/simulator.md`,
`.understand-anything/knowledge-graph.json`,
`.understand-anything/project-knowledge-graph.md`, and
`.understand-anything/meta.json`.

- [ ] Document Red/Blue perspective, zoom/pan/reset, accordion cards, and
  Debug as browser-local presentation controls with unchanged ROS semantics.
- [ ] Run `PYTHONPATH=src/simulator python3 -m pytest -q src/simulator/test`,
  `git diff --check`, and JSON parsing for graph files.
- [ ] Launch the local offline simulator on port 9011, inspect desktop and
  narrow screenshots, verify the map image loads, and verify an existing
  `/api/control` command remains accepted.
- [ ] Commit with `docs: describe tactical command desk`.
