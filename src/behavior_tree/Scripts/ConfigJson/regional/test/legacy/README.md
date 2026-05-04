# Legacy Behavior Tree Presets

These JSON files are old single-strategy presets kept for reference.

They are not used by current launch wrappers automatically. Use current scenario configs instead:

- `hero.json`: old default/HitHero-style preset; replaced by `../../../regional_competition.json` default behavior.
- `hit_sentry.json`: old forced HitSentry preset; replaced by regional strategy switching or a dedicated test preset if needed.
- `navi_test.json`: old fixed-time navigation test preset; replaced by `../../debug/navi_debug_competition.json` plus `../../debug/navi_debug_points.json`.
- `protect.json`: old forced Protected preset; replaced by regional low-resource Protected switching.

Manual use is still possible by passing an explicit path, for example:

```bash
ros2 launch behavior_tree sentry_all.launch.py bt_config_file:=Scripts/ConfigJson/regional/test/legacy/navi_test.json
```
