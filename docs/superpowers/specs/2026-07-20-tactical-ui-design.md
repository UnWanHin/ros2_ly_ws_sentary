# Tactical UI Design

Updated: 2026-07-20

## Goal

Redesign the existing `/tactical` page into a map-first offline sentry command
surface. It must preserve every current simulator control and HTTP contract,
while making scene editing and decision review usable at a glance.

## Chosen Direction

Three presentation directions were considered: map-first command desk,
inspector-first debug console, and full-screen map with modal inspectors. The
map-first command desk is chosen: unit dragging and reading the active goal are
the primary workflow, while a compact inspector dock keeps editing direct.

The visual language is a quiet charcoal command console with one restrained
amber interface accent. Red and blue remain team semantics, not general UI
accents.

## Scope

- Keep the existing `/api/tactical-state` and `/api/control` contracts.
- Keep scene drag/drop, unit HP, structure HP, palette placement, match
  Start/Pause/Reset, decision evidence, tactical evidence, control output and
  feedback visible through the existing page.
- Add browser-local view state only: Red/Blue field perspective, zoom,
  panning, reset view, debug visibility, and accordion expansion state.
- Do not change simulator logic, ROS projection, mock ownership rules, names,
  API payloads, or add a module.

## Layout And Interaction

- The top command bar holds the Red/Blue perspective selector, zoom controls,
  Reset view, Debug, and the existing match controls in two aligned groups.
- The field remains the dominant surface. Pointer-wheel zoom, drag-pan, and
  a red/blue 180-degree perspective flip act only on browser projection;
  pointer-to-field conversion remains official field centimeters.
- The right dock has consistent accordion cards: Units, Structures, Decision,
  Tactical, Control, and Diagnostics. Units, Structures, and Decision are open
  initially; Diagnostics is visible only when Debug is enabled.
- Unit and structure HP controls use aligned decrement, number input,
  increment, and reset-to-maximum controls, all writing existing command types.
- On narrow screens the dock becomes a full-width continuation after the field;
  no essential match or editing control moves into a menu.

## Verification

- Extend tactical HTML tests for the command bar, perspective selector, map
  viewport, accordion semantics, zoom controls, and retained command actions.
- Run the simulator tactical/browser tests and full simulator suite.
- Run a local offline launch with a full 14-piece scene, inspect `/frame.jpg`,
  `/tactical`, and `/api/tactical-state`, then capture desktop and narrow
  browser screenshots.
