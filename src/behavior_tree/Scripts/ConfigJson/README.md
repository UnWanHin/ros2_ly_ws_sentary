# Behavior Tree Config Presets

Active presets in this directory are used by launch files, start scripts, or self-checks.

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
- `regional/test/regional_area_my_base.json`: single-area regional preset for `scripts/areatest/regional_base.sh`.
- `regional/test/regional_area_my_highland.json`: single-area regional preset for `scripts/areatest/regional_highland.sh`.
- `regional/test/regional_area_my_roadland.json`: single-area regional preset for `scripts/areatest/regional_roadland.sh`.
- `regional/test/regional_area_common_central.json`: single-area regional preset for `scripts/areatest/regional_central.sh`.
- `regional/test/regional_area_my_base_pure.json`: pure single-area preset for `scripts/areatest/regional_base.sh --pure`.
- `regional/test/regional_area_my_highland_pure.json`: pure single-area preset for `scripts/areatest/regional_highland.sh --pure`.
- `regional/test/regional_area_my_roadland_pure.json`: pure single-area preset for `scripts/areatest/regional_roadland.sh --pure`.
- `regional/test/regional_area_common_central_pure.json`: pure single-area preset for `scripts/areatest/regional_central.sh --pure`.

## Legacy Presets

Old single-strategy presets were removed from runnable config directories. Keep legacy strategy notes in `docs/record/` only; do not keep launchable legacy JSON presets here.
