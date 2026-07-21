# Decision Explain Detail Design

Updated: 2026-07-21

## Goal

Make final navigation logs explain Regional Recovery, Default area tasks, and
Regional idle patrol without changing strategy selection, navigation output,
or control behavior.

## Design

The existing `DecisionIntent` remains the only decision explanation source.
The existing final-output fingerprint remains the only log de-duplication
state. New metadata is recorded only when the corresponding navigation code
changes the final target point.

- Recovery gets a `DecisionReason::Recovery` in the Hard layer. Its detail
  identifies the stable recovery action (`enter_default`, a recovery probe, or
  a transition) and the point-change snapshot may include the current HP and
  ammo.
- A Default policy selection records the selected AreaManager area name. An
  active Regional area task records its area type and phase only when it sets a
  new final goal. The separate existing policy log continues to report score.
- Regional idle patrol records candidate index and configured hold seconds when
  it selects a new candidate.

All detail strings are stable for an unchanged point. A phase, score, HP, or
ammo update alone must not emit a new `[DecisionExplain][navi]` line.

## Verification

Pure tests cover the new reason mapping and stable-detail fingerprint behavior.
The `behavior_tree_node` build and existing Tactical/decision-trace tests prove
the metadata additions do not break the final control path.
