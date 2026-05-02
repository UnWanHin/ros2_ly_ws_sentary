# Behavior Tree Config Presets

Active presets in this directory are used by launch files, start scripts, or self-checks.

## Competition / runtime

- `regional_competition.json`: default regional competition config used by `sentry_all.launch.py`.
- `regional_simple_competition.json`: simplified regional patrol/chase config used by `mode:=regional_simple`.
- `league_competition.json`: league competition config used by `mode:=league`.
- `showcase_competition.json`: showcase/demo config used by showcase launch wrappers.

## Debug / feature presets

- `navi_debug_competition.json`: behavior tree config for navigation debug mode.
- `navi_debug_points.json`: point-plan data loaded by `NaviDebug.PlanFile`.
- `armor_only_test.json`: armor-only debug preset.
- `armor_patrol_test.json`: armor patrol preset without firing.
- `chase_only_competition.json`: chase-only behavior tree preset.
- `chase_internal_competition.json`: decision chase preset using internal goal output.
- `chase_tf_competition.json`: decision chase preset using TF goal bridge.

## Legacy presets

Old single-strategy presets are kept under `legacy/` for manual reference only. They are not selected by current launch wrappers by default.
