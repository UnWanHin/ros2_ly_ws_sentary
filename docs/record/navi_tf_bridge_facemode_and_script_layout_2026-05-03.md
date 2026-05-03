# navi_tf_bridge / FaceMode / scripts 入口整理记录

日期：2026-05-03

## 背景

从 `ff31d389c89ac0baa9947ed727aa13c3d8e9d8ed` 之后，导航点位转换和固定点朝向链路做了几件会影响联调入口的调整：

1. `navi_tf_bridge` 从一个大节点拆成若干职责文件，方便维护。
2. 手动官方地图点位转 `/goal_pose` 的工具从 `goal_pos_test/manual_goal` 语义改成 `navitomap`。
3. 新增 FaceMode 固定地图点朝向测试链路。
4. `scripts/launch/` 只保留完整/决策 stack，其它 wrapper 按 `aim/navi/tools` 分类。

## 当前脚本入口

完整/决策 stack 仍在 `scripts/launch/`：

- `scripts/launch/start_sentry_all.sh`
- `scripts/launch/start_sentry_all_nogate.sh`
- `scripts/launch/start_sentry_chase_only.sh`
- `scripts/launch/start_sentry_decision_chase.sh`
- `scripts/launch/start_sentry_navi_debug.sh`
- `scripts/launch/start_sentry_showcase.sh`

辅瞄/识别测试在 `scripts/aim/`：

- `armor_test.sh`
- `armor_only_test.sh`
- `armor_patrol_test.sh`
- `buff_test.sh`
- `outpost_test.sh`

导航/TF/固定朝向工具在 `scripts/navi/`：

- `navitomap.sh`
- `facemode.sh`
- `map_aim_point_test.sh`
- `map_aim_point_attach.sh`

标定和工具 wrapper 在 `scripts/tools/`：

- `shooting_table_calib.sh`
- `buff_shooting_table_calib.sh`

`scripts/debug/` 是有意暴露出来的稳定调试接口；即使里面是 wrapper，也保留给人直接找命令用。

## navi_tf_bridge 当前职责拆分

包内 C++ 拆分后的职责：

- `ChasePointer`：处理 `/ly/navi/target_rel`，从消息 `frame_id` 或 `target_rel_default_frame` 查询到地图系。
- `MapPointer`：处理 `/ly/navi/goal_pos_raw`，把官方/原始二维点转换成输出地图点。
- `PointerSolver`：负责官方二维地图点到输出地图系的 4x4 matrix / rigid / affine 解算。
- `GoalOutput`：统一发布 `/goal_pose`、legacy `/ly/navi/goal_pos`、以及可选 `/ly/navi/target_map`。
- `PointerDebug`：按 `Area.hpp` 导出调试点对。
- `map_aim_point_node`：FaceMode 固定点朝向节点。

安装后的脚本/工具名：

- `navitomap_input_node`
- `tf_matrix`
- `navi_calib`
- `mock_gimbal_state_node`

旧名 `manual_goal_input_node`、`solve_static_tf_from_points`、`fit_static_tf_kabsch` 不再作为新的入口使用。

## navitomap 链路

入口：

```bash
./scripts/navi/navitomap.sh
```

行为：

1. 启动 `navi_tf_bridge/navitomap.launch.py`。
2. 终端输入官方/原始 `x y`。
3. 发布 `/ly/navi/goal_pos_raw`。
4. bridge 按 `src/navi_tf_bridge/config/tf_config.yaml` 的 `raw_goal_transform_matrix` 转换。
5. 默认先发布 preview `/ly/navi/goal_pose_preview`。
6. 用户确认后发布最终 `geometry_msgs/PoseStamped /goal_pose`。

默认输入单位是 `cm`，可用 `--input-unit m` 改成米。

## FaceMode 固定点朝向链路

测试入口：

```bash
./scripts/navi/facemode.sh 1093 366 100 --with-tf-tree
```

底层测试入口：

```bash
OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./scripts/navi/map_aim_point_test.sh --with-tf-tree
```

附加到已运行 stack：

```bash
OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./scripts/navi/map_aim_point_attach.sh
```

关键语义：

- `OFFICIAL_MAP_X/OFFICIAL_MAP_Y/MAP_Z` 都按 `cm` 传入。
- X/Y 是官方二维地图坐标，会走 `tf_config.yaml` 的 `raw_goal_transform_matrix`。
- Z 不参与官方二维 X/Y 转换，只作为目标在输出地图系里的高度使用。
- `yaw_sign` 默认是 `-1.0`。
- 默认 `solve_mode=camera_projection`：把目标点转换到 `gx_camera`，按相机投影误差给 `/ly/control/angles`。
- `solve_mode=base_link` 可走目标在 `solve_frame` 下的几何角度解算。
- 节点只发云台角和可选 aim firecode，不发底盘速度。
- 如果目标在 `gx_camera` 后方，当前不会再直接跳过；会先用同一个 camera-frame 向量做几何 yaw/pitch fallback，让云台先转到正面，随后继续用 camera projection 微调。

默认输出：

- `/ly/control/angles`
- `/ly/control/firecode`，只更新 `FIELD_FIRE_STATUS | FIELD_AIM_MODE`，`fire_status=0`，`aim_mode=true`

`facemode.sh` 是给现场使用的简短入口；`map_aim_point_test.sh` 保留更完整的测试/bench 参数。

## tf_point_pairs.yaml 变化

旧的 `src/navi_tf_bridge/config/tf_point_pairs.yaml` 已移除，不再作为静态配置维护。

当前调试点对由运行时导出：

- 默认输出：`log/navi_tf_bridge/tf_point_pairs.yaml`
- 控制参数：`debug_export_point_pairs`、`debug_area_header_file`、`debug_point_pairs_output_file`
- `navitomap.sh` 默认关闭导出，避免手动测试时产生额外文件。

## 验证方法

```bash
bash -n scripts/launch/*.sh scripts/aim/*.sh scripts/navi/*.sh scripts/tools/*.sh scripts/debug/*.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select navi_tf_bridge --parallel-workers 1
source install/setup.bash
ros2 pkg executables navi_tf_bridge
ros2 launch navi_tf_bridge navitomap.launch.py --show-args
ros2 launch navi_tf_bridge map_aim_point.launch.py --show-args
```

## 当前约束

- FaceMode 现在主要作为独立节点直接发 `/ly/control/angles` 使用。
- `behavior_tree` 已预留 `/ly/face_mode/angles` 订阅和 `AimMode::FaceMode` 枚举，但主决策尚未把 FaceMode 接成一个完整 BT 模式。
- `tf_tree` 只提供车体到云台/相机链路；地图/里程计到车体的定位 TF 仍由导航/定位系统提供。
