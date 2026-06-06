# Simulator Stable Contract And Foxglove Export

Updated: 2026-05-29

## Decision

`src/simulator` treats `simulator.trace -> simulator.model.TraceRecord` as the stable simulator-facing contract.
The pygame viewer, validation, and Foxglove exporter consume the normalized model instead of behavior-tree internal trace fields.
`target_state` includes the active aim-source status (`external_aim_active`), current active aim freshness (`fresh_current_aim`), typed aim freshness, and compact target-set summaries.

Behavior-tree-only diagnostic fields may be added under optional names such as `debug` or `bt_debug` without requiring simulator changes.
Simulator updates are required only when a stable viewer-facing field changes, a new visible state is needed, or validation/map/mock-input assumptions change.

## Reason

The behavior tree is expected to keep changing while regional strategy is tuned.
Keeping the simulator bound to a stable decision-output contract avoids repeated viewer churn for BT node-local variables and keeps offline tooling focused on observable decision behavior.

## Runtime Impact

No robot runtime path is changed.
Foxglove support is an offline MCAP export path and runs only when `simulator.foxglove_export` or `simulator.main --export-foxglove` is invoked.
The optional `mcap` dependency is kept out of normal simulator requirements.
