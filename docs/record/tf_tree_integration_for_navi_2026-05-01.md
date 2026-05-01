# tf_tree 合并与导航链路接入记录

日期：2026-05-01

## 背景

当前追击/静态目标转换由 `navi_tf_bridge` 负责，核心依赖是 `map <- base_link` 的 TF 查询。  
项目里已有 `navi_tf_bridge`，但 `base_link` 到云台/相机的 TF 广播逻辑原先在外部仓库 `~/sentry.common`，主工作区启动链路默认不会拉起这部分 TF 节点。

## 根因/判断

1. `~/sentry.common` 当前只包含 `sentry_tf` 包（本仓已重命名为 `tf_tree`），不是导航包。  
2. 该包提供的是 `base_link -> gimbal_* -> camera` 链，不提供 `map -> base_link`。  
3. 若不把该包并入主工作区并接入主启动，现场调试时 TF 依赖容易“看起来有配置、实际没启动”。

## 改动内容

### 1) 合并包到主工作区

将 `~/sentry.common/src/sentry_tf` 合并并重命名到：

- `src/tf_tree/`

保留原包结构：

- `src/tf_tree/src/tf_node.cpp`
- `src/tf_tree/launch/tf_tree.launch.py`
- `src/tf_tree/config/tf_tree.yaml`
- `src/tf_tree/CMakeLists.txt`
- `src/tf_tree/package.xml`

### 2) 接入主启动链路（sentry_all）

在 `behavior_tree/sentry_all.launch.py` 增加 `tf_tree` 启动接入，默认开启并支持参数覆盖：

- 新增 launch 参数：
  - `use_tf_tree`（默认 `true`）
  - `tf_tree_params_file`（默认空，空时回落到包内默认 yaml）
- 新增内部解析参数：
  - `resolved_tf_tree_params_file`
- 当 `use_tf_tree=true` 时，自动 include：
  - `tf_tree/launch/tf_tree.launch.py`

对应文件：

- `src/behavior_tree/launch/sentry_all.launch.py`

### 3) 追击 TF 变换链修正（source -> map）

追击点转换使用 `RelativeTarget.header.frame_id`（若为空则用 `target_rel_default_frame`）作为来源，  
直接做单段 TF 查询 `map <- source_frame` 后发布 `/ly/navi/goal_pos`。

新增参数：

- `target_rel_default_frame`（默认 `gimbal_world`）
  - 当 `RelativeTarget.header.frame_id` 为空时，使用该 frame 作为追击点来源。
  - 若 `use_msg_frame_id=true` 且消息携带 `frame_id`，优先使用消息内 frame。

涉及文件：

- `src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp`
- `src/navi_tf_bridge/config/tf_config.yaml`
- `src/navi_tf_bridge/launch/target_rel_to_goal_pos.launch.py`
- `src/navi_tf_bridge/launch/decision_chase.launch.py`

## 关键参数/阈值

1. `use_tf_tree`
   - 文件：`src/behavior_tree/launch/sentry_all.launch.py`
   - 默认：`true`
   - 作用：是否在主链路中自动拉起 `tf_tree`

2. `tf_tree_params_file`
   - 文件：`src/behavior_tree/launch/sentry_all.launch.py`
   - 默认：`""`（自动回落到 `src/tf_tree/config/tf_tree.yaml`）
   - 作用：覆盖 TF 包参数文件，便于赛场快速切换标定

3. `tf_tree_node.big_yaw_offset_deg` / `barrel_offset_z`
   - 文件：`src/tf_tree/config/tf_tree.yaml`
   - 作用：TF 姿态微调入口

4. `target_rel_default_frame`
   - 文件：`src/navi_tf_bridge/config/tf_config.yaml`
   - 默认：`gimbal_world`
   - 作用：追击 `target_rel` 缺失 `frame_id` 时的默认来源坐标系（直接用于 `map <- source_frame` 查询）

## 微调建议

1. `tf_tree` 只负责车体到云台/相机链路，不要把 `map->base_link` 人为塞到此包。  
2. 导航联调时，优先确认导航侧是否稳定发布 `map -> base_link`（或 `map -> odom -> base_link`）。  
3. 若现场需要临时禁用该包，直接在启动参数传：
   - `use_tf_tree:=false`

## 验证方法

1. 构建：
   - `colcon build --packages-select tf_tree behavior_tree`
2. 检查主启动参数是否出现：
   - `ros2 launch behavior_tree sentry_all.launch.py --show-args`
3. 运行后检查 TF 节点与链路：
   - `ros2 node list | rg tf_tree_node`
   - `ros2 topic echo /tf --once`
4. 导航联调重点检查：
   - `map <- base_link` 是否存在、时间戳是否连续

## 当前约束

本次改动未引入导航定位模块本体；`map -> base_link` 仍由导航/定位系统提供。  
若该链缺失，`navi_tf_bridge` 仍会在 `map <- base_link` 查询处失败，这不属于 `tf_tree` 回归问题。
