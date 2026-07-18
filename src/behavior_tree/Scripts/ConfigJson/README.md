# Behavior Tree Config Presets

Active presets in this directory are used by launch files, start scripts, or self-checks.

These JSON files should not carry patrol scan mode parameters. `PatrolScan.Mode`, mode curves, FaceMode fallback scan modes, and patrol pitch offsets are centralized in `src/behavior_tree/config/Patrol.yaml`; JSON debug presets may only add narrow `PatrolScan.TaskOverrides` when a test profile needs a deliberate exception.

## Root Competition Presets

- `regional_competition.json`: default regional competition config used by `sentry_all.launch.py`.
- `regional_simple_competition.json`: simplified regional patrol/chase config used by `mode:=regional_simple`.
- `league_competition.json`: league competition config used by `mode:=league`.

Only these three competition entry presets should stay in this directory root.

## League Presets

- `league/chase_only_competition.json`: chase-only behavior tree preset.
- `league/chase_internal_competition.json`: decision chase preset using internal goal output.
- `league/chase_tf_competition.json`: decision chase preset using TF goal bridge.

## Regional Presets

- `regional/debug/showcase_competition.json`: showcase/demo config used by showcase launch wrappers.
- `regional/debug/navi_debug_competition.json`: behavior tree config for navigation debug mode.
- `regional/debug/navi_debug_points.json`: point-plan data loaded by `NaviDebug.PlanFile`.
- `regional/debug/armor_only_test.json`: armor-only debug preset.
- `regional/debug/armor_patrol_test.json`: armor patrol preset without firing.
- `regional/test/regional_area_template.json`: canonical source for the four supported single-area test profiles.
  `scripts/areatest/regional_area_profile.py` creates a temporary JSON for `my_base`, `my_highland`,
  `my_ready_roadland`, or `common_central`; `--pure` adds only the route/hold-specific fire, recovery, and
  posture overrides. `regional_area_test.sh` and `navi_control_chain.sh` own the temporary file lifecycle.
- `regional/test/regional_area_my_pre_roadland.json`: retained separate fixture. Its shape intentionally
  differs and it is not a `regional_area_test.sh` CLI target.

## Legacy Presets

Old single-strategy presets were removed from runnable config directories. Keep legacy strategy notes in `docs/record/` only; do not keep launchable legacy JSON presets here.
