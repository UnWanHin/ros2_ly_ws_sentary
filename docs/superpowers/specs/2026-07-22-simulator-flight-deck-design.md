# Simulator Flight Deck Visual Language

Updated: 2026-07-22

## Decision

Replace the simulator's current visual treatment with a shared **Flight Deck** design language. It is a
DJI-style tactical command surface: calm at rest, explicit when actionable, and visually centred on the
battlefield. The browser `/tactical` and native pygame viewer remain two renderers of the same simulator
state, catalog, command bus, and interaction model. This change only replaces presentation.

The product identity is **Sentinel Flight Deck**. It does not imitate a generic admin dashboard, Bootstrap,
or a game HUD. It uses a matte, cool-blue industrial surface with one primary accent and restrained warning
colours.

## Reference analysis

| Reference | Adopt | Do not adopt |
| --- | --- | --- |
| DJI Assistant / Ground Station | Reliable status hierarchy, restrained blue operation state, map-first control placement, direct controls | Dense labelled toolbars, warning-heavy yellow treatment |
| Unity 6 | Contextual Inspector and selection-driven detail | Editor-grey chrome, excessive pane borders |
| Unreal Engine 5 | Clear spatial hierarchy and selected-object focus | High visual density and aggressive panel ornament |
| Linear | Quiet surfaces, reliable spacing, concise action language | Issue-tracker/table visual idiom |
| Raycast / Arc | Compact command affordances, soft focus and motion | Consumer-app playfulness or colourful iconography |

## Visual tokens

These values are the single visual source of truth for both renderers. Pygame keeps equivalent RGB/alpha
values; browser CSS exposes them as custom properties.

| Token | Value | Use |
| --- | --- | --- |
| `surface.root` | `#111418` | window/page background |
| `surface.raised` | `#171C22` | workspace and command bar |
| `surface.panel` | `#1D232B` | docks, shelf, rail |
| `surface.card` | `#202832` | inspector and information cards |
| `line.subtle` | `rgba(255,255,255,0.06)` | separators only |
| `text.primary` | `#F5F7FA` | titles and selected values |
| `text.secondary` | `#AEB7C2` | labels and supporting values |
| `accent.operation` | `#4DB7FF` | selection, primary action, focus |
| `state.warning` | `#FFB648` | actionable caution only |
| `state.danger` | `#FF6262` | destruction / critical failure only |
| `state.success` | `#58D68D` | healthy / confirmed state only |

No other accent family is introduced. Red/Blue teams remain data colours on field pieces, never generic UI
accents.

### Typography

Only three type sizes appear in the interface:

| Role | Browser | Pygame equivalent | Use |
| --- | --- | --- | --- |
| Section title | 18 px / 600 | 20 px medium | workspace title, Inspector title |
| Content | 15 px / 400 | 16 px regular | fields, buttons, values |
| Caption | 13 px / 400 | 14 px regular | category label, status metadata |

Use system UI sans for normal text; use a system monospace face only for compact coordinate and sequence
values. There are no fourth-size micro labels. Line height is at least 1.45 for content.

### Material and depth

- Card radius is 12 px in browser and the nearest native pygame equivalent.
- Cards use `surface.card` with a 1 px `line.subtle` edge, a low-opacity blue-white upper highlight, and a
  soft shadow. They read as layered instrument housings, not outlined boxes.
- Docks use `surface.panel`; the battlefield retains a quieter `surface.raised` frame so the map dominates.
- Hover increases the upper highlight and lifts cards/buttons by 1 px. It never changes unrelated colour.
- Selection uses a single blue edge/glow and a 200 ms transition. Warning/danger never substitute for
  selection blue.

## Workspace composition

```text
┌ Sentinel Flight Deck / mission state / direct map controls ────────────────┐
│                                                                            │
├─ Activity rail ─┬──────────── battlefield viewport ─────────────┬─────────┤
│ scene surfaces  │ map, teams, route, target and tactical state  │ Inspector
│ one icon style  │                                                 │ contextual
│                │                                                 │ cards
├────────────────┴──────────── Operations shelf ──────────────────┴─────────┤
│ timeline / replay / selected active surface                               │
└───────────────────────────────────────────────────────────────────────────┘
```

- The battlefield remains 75–85% of the primary visual area when the Inspector is open.
- The command bar shows only mission status and frequent view/match controls.
- The Activity rail presents the pre-existing Decision, Events, Runtime, Control, Inputs, and Layers
  surfaces with a consistent modern outline icon and a caption. It does not add a new navigation model.
- Inspector and Operations shelf remain docked, resizable, and collapsible. They never float above the map.
- Empty field selects the Battlefield overview. Robot, Base, Outpost, and map area selections replace the
  Inspector content, not the simulator's scene/control state.

## Card grammar

Every information block uses one card grammar:

1. A title row: Lucide-style outline icon, title, one-line status summary, chevron/action at right.
2. A quiet content region: aligned labels and values with at least 12 px between controls.
3. Action region only when an existing mutation already exists (for example HP step, Restore, Destroy,
   Apply, Undo). Button dimensions and wording are preserved semantically.

Only the context card is expanded on entry. Secondary Inspector cards start collapsed. Existing operations
are not removed: they are grouped into the relevant current surface/card rather than duplicated.

## Icon policy

Browser uses Lucide outline SVGs (the maintained, permitted icon style) through a local, minimal icon subset;
no second icon library or external runtime fetch is introduced. Pygame uses matching line-drawn glyphs from
the same semantic set. Required semantic icons are: crosshair/scene, route, pulse/runtime, control sliders,
piece/inputs, layers, robot, base, outpost, health, target, dock, expand, fullscreen, reset, and warning.

Icons are labels' companions, never the only available name for a control. The browser keeps accessible text
labels and `aria-label` values.

## Motion

| Interaction | Duration | Curve | Behaviour |
| --- | --- | --- | --- |
| button/card hover | 150 ms | ease-out | highlight and 1 px lift |
| card expansion | 250 ms | cubic-bezier(0.2, 0.8, 0.2, 1) | height/opacity transition |
| dock/shelf resize or collapse | 250 ms | same | grid/panel interpolation |
| selection / focus | 200 ms | ease-out | blue focus ring; camera settles smoothly |
| map pan/zoom | 180–220 ms | ease-out | preserves official coordinate under pointer |

Browser honours `prefers-reduced-motion`; pygame uses the final state without motion when running at a low
frame rate. No looping decorative animation is introduced.

## Renderer boundaries

- Do not change `SceneCatalog`, `SimulatorInputState`, trace schema, ROS topics, `/api/tactical-state`,
  `/api/control`, simulator commands, or mock/manual ownership behaviour.
- Do not add a database, frontend framework, separate graph, or browser build step.
- Browser visual tokens live in `tactical_web.py`'s generated CSS; pygame token equivalents live in the
  existing viewer palette/config path. They must use the same values and semantic names.
- Browser keeps dependency-free delivery. Local Lucide SVG source is bundled as static inline path data,
  not a network dependency. Pygame draws semantic equivalents natively.

## Acceptance criteria

1. `/tactical` and pygame look recognisably like one Sentinel Flight Deck product.
2. All cards use the specified palette, 12 px radius, subtle separators, layered elevation, and consistent
   title/content/caption hierarchy.
3. Browser and pygame retain the existing workspace, selection, HP, drag, command, timeline, ownership,
   fullscreen, Fit, 1:1, and Zoom to Selection functionality.
4. The battlefield remains dominant; panels do not overlap it and no common action requires scrolling.
5. Browser controls stay at least 40 px high; text remains at least 13 px caption / 15 px content.
6. Tests verify the Flight Deck tokens and semantic workspace controls. Browser visual checks and pygame
   screenshots are manually reviewed at desktop and narrow sizes.
7. `mock` remains editable and `manual_ros` remains observer-only.
