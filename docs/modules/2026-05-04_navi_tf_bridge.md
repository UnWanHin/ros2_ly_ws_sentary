# navi_tf_bridge — 导航点位 / TF 转换 / FaceMode 固定点朝向

## 概述

`navi_tf_bridge` 负责把决策侧的导航点或相对目标转换成导航侧最终使用的 `geometry_msgs/PoseStamped /goal_pose`，并提供固定地图点朝向的 FaceMode 测试节点。

它不负责定位，也不维护 TF 树；gimbal TF 由外部 `sentry_tf` 发布，导航／定位 TF 由外部导航或定位节点发布，`navi_tf_bridge` 只查询和使用。

## 当前职责拆分

| 组件 | 作用 |
|---|---|
| `ChasePointer` | 处理 `/ly/navi/target_rel`，按 TF 从来源 frame 转到 map-frame 目标 |
| `MapPointer` | 处理 `/ly/navi/goal_pos_raw`，按 `tf_config.yaml` 的 raw-goal 4x4 矩阵把官方地图点转成 map-frame |
| `GoalOutput` | 统一发布 `/goal_pose`，可选发布 legacy `/ly/navi/goal_pos` 和 debug target map |
| `PointerSolver` | FaceMode 固定点朝向的 yaw/pitch 解算 |
| `PointerDebug` | debug/export 辅助 |

## 主要节点 / launch

| 入口 | 作用 |
|---|---|
| `target_rel_to_goal_pos_node` | 同时处理追击相对目标和静态点位 bridge |
| `navitomap.launch.py` | 手动官方地图点 `/ly/navi/goal_pos_raw -> /goal_pose` |
| `decision_chase.launch.py` | 追击决策测试时把 `/ly/navi/target_rel` 转导航目标 |
| `target_rel_to_goal_pos.launch.py` | bridge 节点通用 launch |
| `map_aim_point.launch.py` | FaceMode 固定点朝向测试 |
| `map_aim_point_node` | FaceMode 固定点朝向 solver；可独立发布 `/ly/control/angles`，正式 BT 链路发布 `/ly/face_mode/angles` |

## Topic

| 方向 | Topic | 说明 |
|---|---|---|
| 订阅 | `/ly/navi/target_rel` | 追击目标点，默认 frame 为 `gimbal_world` |
| 订阅 | `/ly/aim/armor_targets` | 外部 aim 全量候选；每个 target point 会按 frame 转成 map，再反算为 official-map fallback |
| 订阅 | `/ly/navi/goal_pos_raw` | 官方地图二维点，单位 cm |
| 订阅 | `/ly/face_mode/target_raw` | FaceMode 动态目标，`[official_map_x, official_map_y, map_z]` cm |
| 发布 | `/goal_pose` | 导航最终目标，`geometry_msgs/PoseStamped` |
| 发布 | `/ly/navi/goal_pos` | legacy/direct-XY 兼容输出，默认关闭 |
| 发布 | `/ly/navi/target_map` | debug target map，按配置可开关 |
| 发布 | `/ly/navi/target_official` | 有效 `/ly/navi/target_rel` 或 `/ly/aim/armor_targets` 里的 target point 反算回 official-map cm，`data=[official_x_cm, official_y_cm, armor_type]`，给 BT 敌方位置 fallback 使用 |
| 发布 | `/ly/navi/position` | `map_frame <- base_frame` 反解出的自身位置；`data=[official_x_cm, official_y_cm]` 保持官方地图 cm，`header.stamp` 使用 TF source stamp，`map_point` 保留 map 系 m 坐标并附带 `map_frame/source_frame` |
| 发布 | `/ly/control/angles` | FaceMode 独立测试/直接控制输出 |
| 发布 | `/ly/face_mode/angles` | 正式 BT 链路的 FaceMode solver 输出 |
| 发布 | `/ly/control/firecode` | FaceMode 可选 firecode 输出 |

## 关键参数

配置文件：

- `src/navi_tf_bridge/config/tf_config.yaml`

重点参数：

- `target_rel_default_frame: gimbal_world`
  - `/ly/navi/target_rel` 没带 `header.frame_id` 时使用。
- `input_armor_targets_topic: /ly/aim/armor_targets`
  - 每次收到外部 aim 候选 array 时，逐个 target 做 TF 到 `map`，再用同一套 raw-goal static calibration 反算回 official-map cm。
- `publish_target_official: true`
  - 发布 `/ly/navi/target_official`；BT 会把它作为敌方位置 fallback，且新鲜 `/ly/position/data` 优先。
- `map_frame: map`
  - 输出导航目标所在 frame。
- `use_raw_goal_static_calibration: true`
  - 开启官方地图点到 map-frame 的静态转换。
- `raw_goal_calibration_model: matrix`
  - 当前默认直接使用 4x4 矩阵。
- `raw_goal_transform_matrix`
  - row-major 4x4，输入点先从 cm 解码到 m，再应用矩阵。
- `Chase.AreaLimit.Enable: true`
  - 正式 BT 链路从 behavior_tree 的 `bt_config_file` 读取 `Chase.AreaLimit` 并传给 bridge；`tf_config.yaml` 不重复保存策略开关。
  - 追击 `/ly/navi/target_rel -> /goal_pose` 时，先把自身和候选追击点反算到官方地图 cm 坐标；`ChaseEnableCrossArea=false` 时候选点跨出当前所在大区域会被截到边界内侧，`true` 时可落到已启用的大区域内。
  - 官方坐标 fallback 追击走 `/ly/navi/goal_pos_raw`，由 BT 在发布前按同一个 `Chase.AreaLimit` 限制；bridge 不会把该限制套到普通固定点位 `/ly/navi/goal_pos_raw`。
  - 大区域边界从 `src/behavior_tree/module/Area.hpp` 的 `RedMainArea*Points` / `BlueMainArea*Points` / `CommonMainAreaCentralPoints` 解析，避免在 YAML 里重复维护点位。
  - `BoundaryMarginCm` 控制离边界保留的安全距离；自身不在任何已知大区域时会保持当前点。
  - `ChaseEnableCrossArea=false` 时，单次追击只允许在当前所在大区域内，目标越界会夹到边界内侧。
  - `ChaseEnableCrossArea=true` 时，追击目标可落在 `DecisionAutonomy.NaviGoal` 已开启的大区域内；未开启的大区域仍不允许作为追击目标。
  - 该限制只约束底盘追击目标，不关闭云台跟踪和开火；目标丢失或 Chase 退出后仍回到 BT 原决策链路。

FaceMode launch 必填：

- `official_map_x`
- `official_map_y`
- `map_z`

`map_aim_point_node` 也支持 `require_initial_target=false`：节点启动后不发布角度，直到收到 `/ly/face_mode/target_raw`。`sentry_all.launch.py` 的正式 BT 链路使用这个模式，由 BT 动态下发 `[official_map_x, official_map_y, map_z]`。

FaceMode 默认：

- `yaw_sign=-1.0`
- `solve_mode=camera_projection`
- `aim_frame=gimbal_world`
- `camera_frame=gx_camera`

正式 `sentry_all` 中的 BT FaceMode solver 使用 `solve_mode=relative_geometry`、`solve_frame=gimbal_barrel_joint`、`yaw_sign=1.0`，即按 TF 相对几何直接算朝向，不依赖相机投影看到目标。

## 脚本入口

| 脚本 | 作用 |
|---|---|
| `scripts/navi/navitomap.sh` | 手动发官方地图点并预览转换结果 |
| `scripts/navi/OfficialToNavi.sh` | 纯静态换算 official map -> navi/map，默认 official 输入 cm、navi 输出 m |
| `scripts/navi/NaviToOfficial.sh` | 纯静态反向换算 navi/map -> official map，默认 navi 输入 m、official 输出 cm |
| `scripts/navi/facemode.sh` | FaceMode 简短入口：位置参数为 `official_map_x official_map_y map_z`，单位 cm；`--bt-output` 输出到 `/ly/face_mode/angles` 给 BT 使用 |
| `scripts/navi/map_aim_point_test.sh` | 拉起 FaceMode 测试栈，可选拉 `gimbal_driver`，要求外部 `sentry_tf`；输入默认 cm，可用 `--unit m` |
| `scripts/navi/map_aim_point_attach.sh` | 已有 stack 上只附加 FaceMode 节点 |
| `scripts/debug/goal_pos_test.sh` | 稳定 debug wrapper，实际转到 `scripts/navi/navitomap.sh` |

`kabsch_calib` / `affine_calib` / `navi_calib_simple` 標定工具默認單位鏈路是 `official_map(m) -> map(m) -> raw_goal_transform_matrix(m)`。如果使用官方地圖 cm 點位，要在 YAML 或命令行顯式寫 `source_unit: cm` / `--source-unit cm`；工具內部會先把 source/target 換到 `output_unit` 後再解矩陣。目前給 `tf_config.yaml` 使用時 `output_unit` 應保持 `m`。工具都會在 `--help` 和互動輸入時顯示 `source_unit -> target_unit -> output_unit` 鏈路。

`kabsch_calib` 和 `affine_calib` 的默認可編輯點表都是 `src/navi_tf_bridge/config/navi_calib.yaml`。直接運行 `python3 src/navi_tf_bridge/script/kabsch_calib.py` 或 `python3 src/navi_tf_bridge/script/affine_calib.py` 後，在 `pair[1]` 直接回車會使用這份 YAML；輸入任意點則會用新輸入的點重新標定。`kabsch_calib.py` 默認不回寫點表；需要把本次點表和 `last_result.raw_goal_transform_matrix` 保存回 `navi_calib.yaml` 時，加 `--save-input`。

## 维护注意

- 旧的 `tf_point_pairs.yaml` 不再作为静态配置维护；点对导出是运行时 debug/export 行为。
- `navitomap` 是官方地图点到 `/goal_pose` 的工具，不是追击测试。
- FaceMode 直接控制云台，不发布底盘速度；和正式 `behavior_tree` 同时跑时要确认 `/ly/control/angles` 只有预期发布者。
- `sentry_all.launch.py` 默认已拉起 BT 用的 FaceMode solver，输出到 `/ly/face_mode/angles`，不会直接抢 `/ly/control/angles`。
- 单独调试区域任务 FaceMode 时，运行 `scripts/navi/facemode.sh --bt-output ...`，避免和 BT 同时抢 `/ly/control/angles`。
- 区域任务需要切换固定朝向点时，由 BT 发布 `/ly/face_mode/target_raw`；格式为 `[official_map_x, official_map_y, map_z]` cm。
- `/ly/navi/position` 依赖 raw-goal static calibration；如果 `tf_config.yaml` 的 4x4 没准备好，节点不会发布这个补充位置。BT 会把 `data[0:2]` 作为 `AreaManager.SentryPositionFusion` 的 Navi 源；`map_point` 只提供 map 系原始位置给调试或新订阅者。
- FaceMode 默认 `yaw_sign=-1.0`；`camera_projection` 下目标在 `gx_camera` 后方时会用几何 yaw/pitch fallback 先转向正面，再继续投影微调。
- 详细变更记录见 [navi_tf_bridge / FaceMode / scripts 入口整理记录](../record/2026-05-03_navi_tf_bridge_facemode_and_script_layout.md)。
