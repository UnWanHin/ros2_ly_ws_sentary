# Simulator Workspace Redesign

Updated: 2026-07-22

## Goal

Replace the prototype pygame and browser tactical layouts with a professional, map-first engineering workspace. The redesign must preserve the current simulator, decision, ROS, catalog, scene, HTTP, and command-bus behavior exactly while making the same existing operations easier to understand and use.

## Scope and invariants

- `src/simulator` remains the only simulator package; no second simulator, graph, state store, or backend is introduced.
- `SceneCatalog`, `SimulatorInputState`, `interactive_inputs`, the existing control-bus command schema, `/api/tactical-state`, and `/api/control` remain authoritative and unchanged.
- Browser and pygame keep consuming the same catalog-backed scene state. Their interaction vocabulary is synchronized, but each renderer remains native to its platform.
- Existing operations remain available: match start/pause/reset, perspective, unit placement/dragging, unit HP, Base/Outpost HP, restore/destroy/undo, map layers, decision/events/runtime/control content, playback timeline, and manual-ROS read-only behavior.
- The redesign may restructure presentation code and create layout components. It must not alter BT, ROS topics, simulation rules, payloads, or decision logic.
- Docked panels never float over or obscure the battlefield. Debug-only facts remain hidden until Debug Mode is enabled.

## Current-state audit

| Severity | Finding | Consequence |
| --- | --- | --- |
| P0 | Browser is a fixed map-plus-inspector grid; pygame is a fixed-width right panel plus fixed timeline. | Neither client is a dockable, resizable workspace. |
| P0 | Browser presentation is a monolithic HTML/CSS/JS string with appended string-replacement patches; pygame `Viewer` owns rendering, layout, interaction, and inspector state. | Layout changes are brittle and cannot be verified or evolved as independent components. |
| P1 | Inspector mixes global state, selected object, controls, target preview, tabs, and diagnostics in one scrolling stack. | It reads as a debug console rather than a contextual Unity-style Inspector. |
| P1 | Viewport interactions use center-oriented zoom and have inconsistent Fit/1:1/reset/focus semantics. | A user loses context while inspecting or editing a map object. |
| P1 | Battlefield area depends on hard-coded panel dimensions rather than a map-first workspace policy. | Resolution changes compromise the main tactical view. |
| P2 | Map selection feedback and object inspection differ between structures, robots, areas, and empty field. | The mental model changes with each click. |
| P2 | Existing visual tests mostly prove routes and element identifiers. | Passing tests do not prove commercial visual quality, spacing, or interaction continuity. |

## Architecture options considered

### A. Restyle the existing two-column presentation

This keeps the current fixed layouts and only changes cards, spacing, and colors. It is fast but cannot meet docking, resizing, contextual inspection, or viewport requirements. Rejected.

### B. Shared workspace grammar with native renderers — selected

Define presentation-only workspace concepts: dock zone, panel state, selection, viewport command, and layout preference. Browser renders them with CSS Grid and pointer splitters; pygame renders the same concepts with native geometry and input handling. Scene and simulator state stay where they are.

This produces matching behavior without adding a web build system, a database, a duplicate scene, or a dependency that is inappropriate for the offline ROS workspace.

### C. React plus a third-party docking framework for the browser

This may improve web authoring ergonomics but introduces a build chain and an independent browser architecture while pygame still needs its own dock manager. It adds deployment and synchronization risk without improving simulator correctness. Rejected.

## Target workspace

```text
┌────────────────────────── Global Command Bar ──────────────────────────┐
│ Match controls · perspective · Fit · 1:1 · focus · fullscreen · status │
├─ Activity rail ─┬──────────── Battlefield viewport ────────────┬───────┤
│ Scene          │ map, pieces, structures, areas, goal, route   │       │
│ Units          │                                                │       │
│ Layers         │          world-coordinate interaction          │ Inspector
│ Runtime        │                                                │       │
├────────────────┴────────── Operations shelf ────────────────────┴───────┤
│ Existing timeline, replay and runtime/log content; collapsible          │
└────────────────────────────────────────────────────────────────────────┘
```

### Spatial policy

- At desktop size, the battlefield receives 76–84% of usable workspace width when the Inspector is visible and the Operations shelf is collapsed.
- The Activity rail is compact. It selects existing work surfaces; it is not a second wide sidebar.
- The Inspector may dock left or right, resize within safe limits, collapse to a narrow rail, and return via Reset Layout.
- The Operations shelf may dock at the bottom, resize, and collapse. It keeps existing timeline, replay, runtime, and log information rather than creating new simulator facts.
- Panels can move only among left, right, and bottom dock zones. Floating windows are deliberately excluded so the battlefield remains unobscured.
- Browser preferences use existing browser-local presentation storage. Pygame preferences are local presentation state only. Neither persists simulator facts.

## Component boundaries

### Presentation-only workspace model

The model holds only:

- selected object identity and selection kind;
- current dock positions, panel widths/heights, collapse state, focus/compact/debug flags;
- viewport transform and active viewport command.

It never stores copied unit, structure, decision, or ROS facts. Those continue to arrive from the current scene/status/trace sources.

### Battlefield viewport

Both renderers expose the same command semantics:

- wheel zoom is anchored to the pointer’s official field coordinate;
- middle button or Space-drag pans;
- Fit frames the full field with safe padding;
- 1:1 displays native map-pixel scale;
- Zoom to Selection centers and frames the selected robot, structure, area, goal, or route;
- double click zooms to the current selection; double-clicking empty field returns to Fit;
- window resize preserves the field coordinate under the viewport center;
- fullscreen retains the command bar and docked Inspector, and Escape exits.

All map pointer operations are converted through the existing official-centimeter coordinate conversion. Visual zoom never changes scene coordinates.

### Contextual Inspector

Clicking the battlefield produces exactly one Inspector context:

| Selection | Inspector content | Existing actions retained |
| --- | --- | --- |
| Empty field | Battlefield overview, current decision, current goal, current control summary | existing match/view actions |
| Robot | position, HP, heat/ammo where present, decision, target, BT/navigation status | existing drag/place and unit HP commands |
| Base / Outpost | team, coordinates, HP, alive/destroyed, recent known state, decision impact | existing step, Apply, Restore, Destroy, Undo commands |
| Area | label, coordinates/region, visible existing strategy and navigation data | no fabricated data or new decision action |

Each Inspector uses collapsible cards with icon, title, one-line summary, action area, and editable properties. Only Overview is expanded initially. Runtime diagnostics appear only under the existing Debug setting.

### Existing information surfaces

The existing Decision, Events, Runtime, Control, Inputs, Layers, timeline and replay information are retained. They move into the Activity rail-selected content or Operations shelf according to their existing role; no information is removed and no new simulator data is invented.

## Visual system

- Dark industrial interface: neutral graphite surfaces, white/gray typography, one blue interaction accent, restrained orange for objectives, red only for danger/destruction, and team tint only where team identity requires it.
- Typography never drops below 14px for user-facing text. Button and input height is 40px minimum.
- Use a fixed spacing scale: 8, 12, 16, 20, 24, 32 px. Outer workspace padding is 24 px; dock gap is 16 px; card padding is 20 px.
- Cards use restrained 10px radii, soft borders, and light elevation only where it establishes hierarchy. No rainbow palettes, decorative gradients, or debug-console density.
- State transitions, card expansion, panel collapse, selection, hover, and viewport focus use short, calm motion. Reduced-motion behavior must remain usable.

## Interaction and failure behavior

- `manual_ros` remains observer-only; editing controls retain the existing disabled reason and never issue a command.
- Failed control requests keep the current state intact and show an accessible, concise status indication using the current response reason.
- Layout reset affects only layout preferences, never simulator input or trace state.
- Fullscreen, compact, focus, dock, and resizing are presentation operations; their failure cannot alter scene ownership or control availability.

## Verification and acceptance criteria

1. Existing simulator tests continue to pass without changing external payload contracts.
2. Browser: test `/tactical`, `/api/tactical-state`, and `/api/control`; verify console is clean and inspect 1440×900, 1024×768, narrow, focus, and fullscreen screenshots.
3. Pygame: capture desktop and fullscreen screenshots; verify panel docking, resizing, collapse, reset, map selection, pointer-anchored zoom, pan, Fit, 1:1, and Zoom to Selection.
4. The same editable action in either client emits only the existing command-bus command and reaches the existing mock input owner.
5. Base, Outpost, Robot, Area, and empty-field selection each display the correct contextual Inspector without data duplication.
6. At normal desktop size, the field remains visibly dominant; no text or control overlaps, clips, or requires tiny type.
7. `git diff --check`, targeted simulator tests, browser visual checks, and the required static self-check pass before completion.

## Non-goals

- No simulator rule changes, fake telemetry, ROS integration changes, new HTTP APIs, or new data schemas.
- No separate graph, scene database, or duplicate documentation/state model.
- No new floating windows, no replacement of the decision system, and no change to the game-field coordinate convention.
