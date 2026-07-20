# Simulator Tactical Sandbox Design

Updated: 2026-07-19

## Goal

Turn `src/simulator` into an offline tactical decision sandbox.  An operator
can place red and blue unit pieces, edit base/outpost health, advance match
time, and see the actual behavior-tree navigation/control decision.  The same
scene is usable through both the local Pygame client and a browser client.

The sandbox is explicitly an offline/test launch facility.  It must not alter
the formal `./scripts/start.sh gated --mode regional` chain or become a second
publisher in a real robot run.

## Non-negotiable boundaries

- Pygame and the browser are clients, not independent simulation engines.
- `simulator.mock_inputs` remains the only ROS publisher in `mock` mode.
- Foxglove/manual ROS mode is observer-only in the simulator and disables
  scene-edit commands, unless a later explicit launch is made for a dedicated
  isolated ROS test graph.
- All unit, structure, asset, position, HP, and goal metadata comes from one
  validated YAML catalog.  Python code may define types and validation, but
  must not duplicate the game roster or ROS mapping tables.
- The behavior tree continues to own all `/ly/control/*` outputs.  The
  simulator only observes those outputs and records them in traces.
- Existing trace v2 JSONL continues to replay.  New trace fields are additive
  and use a schema version bump rather than changing previous meanings.

## Architecture

```text
             tactical_catalog.yaml
                  |          |
                  v          v
           SceneState <-> CommandBus <- Browser Tactical Board
               |       ^
               |       +-------------- Pygame Tactical Board
               v
     MockRosPublisher (mock only)
               |
      formal BT input topics
               v
          behavior_tree
               |
    final navigation/control topics
               v
    ROS output monitor -> DecisionTrace v3 -> scene/status clients/Foxglove export
```

### Scene domain

`SceneCatalog` is immutable configuration loaded from
`src/simulator/config/tactical_catalog.yaml`.  It contains:

- field/team conventions and the red/blue asset manifest keys;
- unit archetypes, default/max HP, semantic role, formal `Health` field,
  `PositionData` car ID mapping, and whether the BT consumes that fact;
- friend/enemy base and outpost definitions, default/max HP, map position,
  and formal health topic mapping;
- navigation goals, label, position, tactical category, and marker style;
- visual tokens: colour, outline, asset scale, health bar and label rules.

`SceneState` contains mutable match data only: selected tool, match state,
structures, placed unit instances, selected entity, current BT target and
latest decision.  Unit identities are stable strings, so multiple same-class
pieces can be displayed without silently overwriting one another.  The ROS
adapter decides which instances can be projected onto the formal one-per-unit
topics and reports a visible conflict instead of picking an arbitrary winner.

`SceneCommand` is the only mutation interface.  Commands are JSON objects
with `command`, `entity_id`, and typed payload values.  They are validated by
the domain layer before appending to the existing JSONL command bus.  Both UI
clients call the same HTTP endpoint/command endpoint, so no browser-only or
Pygame-only business rules exist.

### Pygame client

The Pygame viewer remains the fastest local workbench.  Its map becomes a
clean tactical board rather than a trace overlay dump:

- a compact palette with red/blue piece thumbnails and a selected-piece state;
- draggable sprite pieces with team ring, selected outline, compact HP bar,
  role label, hover tooltip, and a decision-input badge;
- direct structure badges placed at base/outpost positions, with HP stepper
  controls and destroyed state;
- one prominent selected-goal marker, route history, goal name, decision
  layer/reason/priority, and a clear distinction between trace evidence and
  live ROS evidence;
- layer defaults that hide passive trace units when editable scene units are
  active, eliminating duplicate pieces;
- side panels for Inputs, Decision, Control Output, and event timeline.

### Browser client

The HTTP stream becomes a real tactical board rather than a JPEG dashboard.
It is dependency-free HTML/CSS/JavaScript served by the existing Python
server.  It receives a normalized `/status.json` scene snapshot and writes
validated commands through `/api/control`.

The browser board provides the same core interaction as Pygame: select a
piece from the palette, click/drag it onto the scaled map, select a placed
piece, alter HP, remove it, edit base/outpost HP, start/pause/reset, and see
the selected BT goal and route.  It does not duplicate coordinate conversion:
the server exposes the field dimensions and map transform metadata, while the
client sends field-centimetre positions.  Pygame remains available in parallel
for an operator who prefers the local desktop view.

The visual language is an operations console: dark neutral chrome, red/blue
team cues, warm gold for the active goal, restrained translucent tactical
overlays, and supplied robot art as the primary object.  It does not use
decorative gradients or separate floating-card-in-card layouts.

### Foxglove and manual ROS

There are two explicit ownership modes:

| Mode | Scene edits | Formal BT input publisher | Foxglove role |
| --- | --- | --- | --- |
| `mock` | enabled | `simulator.mock_inputs` | observe traces/topics; publish nothing competing |
| `manual_ros` | read-only | operator/Foxglove/CLI | inspect normalised state and final outputs |

`manual_ros` consumes formal topics through a dedicated monitor and reflects
them in the scene.  It never writes mock inputs.  The browser shows the mode
and disables controls server-side and client-side.  This makes a Foxglove
manual message test deterministic: the human owns input facts, while the
simulator observes BT outcome.

Foxglove export gains an additive scene/control channel and final control
output channel.  Live connection is through normal ROS topics and Foxglove
Bridge, not a custom second WebSocket protocol.

### Decision trace v3

The behavior tree will record two distinct objects.  Neither object is
reconstructed later from mutable decision state:

- `gimbal_feedback`: a timestamped snapshot copied only in the lower-machine
  `/ly/gimbal/firecode` callback.  It has explicit `available` and `age_ms`
  fields, so an all-zero never-received frame cannot be mistaken for a real
  lower-machine response.  The legacy `gimbal.fire_code` object remains
  unchanged for v2 consumers because `RecFireCode` is also used by legacy fire
  toggle logic and is therefore not reliable feedback evidence.
- `control_output`: a timestamped, sequenced snapshot captured at the actual
  output boundary after the outgoing ROS messages are built.  It has separate
  `angles`, `fire_code`, and `trajectory` payloads, each with its own
  `published` flag.  A safe-control publish records no trajectory with the
  reason `safe_control`; a missing publisher and invalid dynamics have their
  own reasons.  It represents the latest actual publish, not merely the
  mutable next command observed when a trace event happens.

This captures temporary FollowMode overrides and trajectory validity at the
same instant as publication.  Trace recording only copies the snapshot under
a small lock and never creates/publishes a ROS message itself.  Game-start,
stop, or other trace events may report the most recent output snapshot with
its sequence/age; they must not imply that a new command was published by the
event.

`tactical` stores effective `Tactical.yaml` outcomes required to explain
damage rotate and protection behavior: configured enable flags, damage ramp
gear, rotate before follow override, final rotate, follow override, Protect
Hero activation, ProtectCastle source activation, and textual decision
reason.  The simulator must display these as evidence, not reimplement the
BT policy.

### Acceptance scenarios

1. Drag a formal enemy Hero into MyBase in `mock` mode.  The emitted
   `PositionData` and `Health` reflect the catalog mapping; the live board
   shows the resulting Regional/ProtectCastle decision and selected goal.
2. Change friend base/outpost HP from either client.  The same state is shown
   in the other client, reaches mock ROS topics, and is reflected in trace
   referee fields.
3. Inject `/ly/aim/result` plus a target through the mock scene.  Trace and
   UI show final BT `/ly/control/angles`, `/ly/control/firecode`, and valid
   `/ly/control/trajectory`, separately from feedback.
4. In `manual_ros`, the browser/Pygame controls are disabled.  A manual
   Foxglove publish changes observed scene/BT result without later mock
   overwrite.
5. The default tactical board has no duplicated scene/trace pieces and has a
   visible active navigation goal with route/reason.

## Verification

- Python unit tests cover catalog validation, scene identity, command
  validation, ROS projection, ownership-mode gating, trace v2/v3 adapters,
  Foxglove export, and browser API responses.
- A headless browser smoke test exercises browser drag-to-field coordinate
  conversion and HP command submission when a browser test runtime is
  available; otherwise the HTTP contract tests remain mandatory and the
  limitation is documented.
- C++ tests assert the final control trace contains command data distinct from
  feedback and preserves non-finite trajectory suppression.
- Run the complete simulator test suite, target `behavior_tree` tests/build
  where external `sentry_msgs` permits it, `git diff --check`, JSON/YAML
  validation, and `./scripts/selfcheck.sh sentry --static-only`.
