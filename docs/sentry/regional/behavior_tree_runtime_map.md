# Behavior Tree Runtime Map

Updated: 2026-07-21

> This page is the current inventory for every behavior-tree entry in this workspace. Update it whenever a launch starts `behavior_tree_node`, changes `bt_tree_file`, or changes a topic at the BT/bridge boundary. It is linked into the live documentation graph; it is not a second source of runtime configuration.

## Formal Runtime

The only formal entry is:

```bash
./scripts/start.sh gated --mode regional
```

It reaches `scripts/launch/start_sentry_all.sh`, then launches
`behavior_tree/sentry_all.launch.py` with `use_behavior_tree:=true`. The node is
`behavior_tree_node`, and its default tree is `src/behavior_tree/Scripts/main.xml`.
No alternate XML is selected unless an operator explicitly passes `bt_tree_file:=...`.

The wrapper also passes the source-tree
`src/behavior_tree/config/Chase.yaml` explicitly as `chase_config_file`. This makes
the formal launcher follow the editable source Chase policy instead of silently
using the installed package copy. The running node keeps the parameters it was
started with, so a Chase YAML change takes effect after the normal formal restart.

The active tree is:

```text
UpdateGlobalData -> EvaluateEvents -> SelectAimMode -> SelectStrategyMode
-> Hard -> Task -> PreprocessData -> SelectAimTarget -> Tactical
-> Special -> Default -> Finalizer -> SelectPosture -> PublishAll
```

The decision-layer semantics and navigation reach contract live in
[strategy_layers_and_navigation_reach.md](strategy_layers_and_navigation_reach.md).
The regional task and tactical meaning live in
[2026-07-12_regional_decision_graph.md](2026-07-12_regional_decision_graph.md) and
[decision_framework.md](decision_framework.md).
The source-frame, Chase, exact enemy-position fallback, and `/goal_pose` boundaries live in
[bt_aim_navi_coordinate_chain.md](bt_aim_navi_coordinate_chain.md).

## Aim, Chase, And FaceMode Boundaries

There are two different TF paths. Do not treat a camera-frame change in one path
as a change to the other.

| Flow | BT responsibility | Transform owner | Output |
|---|---|---|---|
| Normal aim | Subscribe `/ly/aim/result`; validate freshness and fire gate; select final gimbal control | External `sentry.aim` already owns target-to-angle solving | `/ly/control/angles`, `/ly/control/trajectory`, `/ly/control/firecode` |
| Chase | Select `targetArmor`; apply chase/area authorization; preserve selected target's position and frame in `/ly/navi/target_rel` | `navi_tf_bridge/target_rel_to_goal_pos_node` transforms `target_rel.header.frame_id` to `map` | `/goal_pose` |
| Official enemy-position fallback | Keep per-enemy official position evidence for RegionalDefense and official-position Chase fallback | `target_rel_to_goal_pos_node` transforms every valid `/ly/aim/armor_targets` point to `map`, then applies the inverse official-map matrix | `/ly/navi/target_official` -> BT `enemyRobots` |
| Fixed-point FaceMode | Decide whether a Regional/Buff/Outpost request owns the gimbal; publish official point | `map_aim_point_node`; formal camera primary is `gx_camera_0`, fallback `gx_camera_1` | `/ly/face_mode/angles`, then BT may publish `/ly/control/angles` |
| Fixed official navigation point | Choose an official cm goal | `target_rel_to_goal_pos_node` converts `/ly/navi/goal_pos_raw` to map | `/goal_pose` |

For Chase, the frame is carried from the selected external `AimTarget`:

1. `target.header.frame_id`
2. the enclosing `/ly/aim/armor_targets` header frame
3. `ExternalAim.TargetDefaultFrame`, currently `gimbal_world`

Therefore an aim provider that publishes camera-relative target coordinates must set
the target or array `frame_id` to that camera frame, such as `gx_camera_0`. A missing
frame is not implicitly repaired by the FaceMode `gx_camera_0` setting.

`/ly/navi/target_official` is a per-armor-type fallback, not a navigation command.
For the same enemy ID, BT retains a fresh non-zero lower/referee
`/ly/position/data` position first. Only when that source is absent or older than
`Chase.OfficialPositionFreshMs` may the bridge-derived position update
`enemyRobots`; `/ly/enemy/info.position_source` then reports `navi_target_official`.
This applies to every valid target in the aim array, even when no Chase target is
currently authorized.

ProtectCastle uses the same Chase path only during `PerimeterDefense`: the current BT-selected
target must have a fresh exact `MyBase` official position, and `ChasePolicy.MyBase` must be enabled.
`ApproachCastle` and `HoldCastle` keep their fixed Castle point. A composite navigation
`Unreachable` result drops that tick's Chase authorization and returns to the selected perimeter point.

The bridge contracts are documented in
[../../modules/2026-05-04_navi_tf_bridge.md](../../modules/2026-05-04_navi_tf_bridge.md),
the complete coordinate chain in [bt_aim_navi_coordinate_chain.md](bt_aim_navi_coordinate_chain.md),
and aim/task ownership in [vision_task_patrol_modes.md](vision_task_patrol_modes.md).

## Runtime Ownership

| Topic family | Formal active publisher | Notes |
|---|---|---|
| `/ly/control/angles`, `/ly/control/trajectory`, `/ly/control/firecode`, `/ly/control/vel`, `/ly/control/posture`, `/ly/control/sentry_cmd` | `behavior_tree_node` | The final formal control owner. `gimbal_driver` consumes and serializes these messages. |
| `/ly/face_mode/angles` | `map_aim_point_node` | Intermediate input only; it does not publish final control FireCode in formal launch. |
| `/ly/navi/target_rel`, `/ly/navi/goal_pos_raw`, `/ly/navi/goal`, `/ly/navi/speed_level` | `behavior_tree_node` | Decision intent inputs for navigation. |
| `/goal_pose` | `target_rel_to_goal_pos_node` | The bridge is the formal owner. BT's direct manual-Outpost pose is a test-only mode and disables bridge goal-pose output. |

The broader owner map is maintained in
[../../architecture/2026-07-12_project_link_graph.md](../../architecture/2026-07-12_project_link_graph.md).

## Other BT Launches

These files can launch the same `behavior_tree_node` executable, but none is entered
by `./scripts/start.sh gated --mode regional`:

| Entry | Role | Formal use |
|---|---|---|
| `behavior_tree.launch.py` | Node-only wrapper; external dependencies must already be running | No |
| `competition_autoaim.launch.py` | Isolated compatibility/auto-aim test | No |
| `chase_only.launch.py` and `scripts/launch/start_sentry_decision_chase.sh` | Chase-chain test | No |
| `navi_debug.launch.py` and `scripts/debug/navi_debug.sh` | Navigation debug plan | No |
| `outpost_regional_test.launch.py` | Outpost/Regional integration test | No |
| `armor_patrol_test.launch.py` | Armor patrol test wrapper | No |
| `showcase.launch.py` and `scripts/start/showcase.sh` | Demonstration profile | No |

Do not run one of these alongside the formal stack unless its `use_behavior_tree:=false`
or all overlapping control publishers are disabled. Two BT processes would both publish
the final `/ly/control/*` topics.

## Maintenance Checklist

When changing decision behavior, update this page together with the closest linked
regional document:

- A new XML node or a changed tick order: update this page and
  [strategy_layers_and_navigation_reach.md](strategy_layers_and_navigation_reach.md).
- A new Tactical/Default navigation owner: update this page and
  [2026-07-12_regional_decision_graph.md](2026-07-12_regional_decision_graph.md).
- An aim target frame, TF bridge, or `/goal_pose` change: update this page,
  [bt_aim_navi_coordinate_chain.md](bt_aim_navi_coordinate_chain.md),
  [vision_task_patrol_modes.md](vision_task_patrol_modes.md), and
  [../../modules/2026-05-04_navi_tf_bridge.md](../../modules/2026-05-04_navi_tf_bridge.md).
- A control topic owner change: update this page and
  [../../architecture/2026-07-12_project_link_graph.md](../../architecture/2026-07-12_project_link_graph.md).

`docs/**/*.md` remains the only graph source. The documentation website re-reads it on
each API request, so no graph file or regeneration step is allowed.
