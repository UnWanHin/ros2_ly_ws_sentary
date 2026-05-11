# Domain Docs

This repo is a single-context ROS2 sentry workspace. Skills should use the repo-level documentation before changing runtime links, messages, launch files, behavior-tree logic, or scripts.

## Primary Context

Read these first when they are relevant to the task:

- `AGENTS.md`
- `docs/architecture/`
- `docs/modules/`
- `docs/sentry/`
- `docs/record/`
- `docs/guides/`
- `docs/rules/` when RoboMaster referee, rules, or protocol semantics matter

## Optional Matt Pocock Docs

If these files exist, read them before deep design or architecture work:

- `CONTEXT.md`
- `CONTEXT-MAP.md`
- `docs/adr/`
- `src/*/docs/adr/`

If they do not exist, proceed silently. Do not create them just to satisfy a skill; create or update them only when the user asks for docs or when `$grill-with-docs` resolves new terminology or decisions.

## Domain Vocabulary

Use the existing project terms consistently:

- lower machine / `gimbal_driver` for serial downlink and referee uplink generation
- BT / `behavior_tree` for decision state, blackboard, strategy, and merged semantic state
- navi / `navi_tf_bridge` for map and official coordinate conversion
- decision trace for offline behavior-tree visualization
- sentry, regional, league, RFID, sentry info, posture, firecode, and sentry cmd with the meanings already used in `docs/sentry/` and `docs/record/`

When a change affects behavior-tree decision outputs, trace schema, navigation goal IDs/coordinates, posture fields, target fields, or unit state fields, follow the Decision Visualization Maintenance rules in `AGENTS.md`.
