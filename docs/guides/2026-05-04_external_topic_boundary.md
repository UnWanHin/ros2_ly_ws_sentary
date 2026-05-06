# 外部 Topic 边界说明（导航/下位机）

本文档用于明确：哪些链路在本仓库内闭环，哪些链路依赖外部模块（导航上位机/下位机）。

## 1. 仓内闭环链路

- 打车主链：`detector -> tracker_solver -> predictor -> behavior_tree -> gimbal_driver`
- 打符链：`detector -> buff_hitter -> behavior_tree -> gimbal_driver`
- 前哨链：`detector -> outpost_hitter -> behavior_tree -> gimbal_driver`

以上链路的 ROS 节点都在本仓库内可见。

## 2. 外部依赖链路

### 导航接口（通常由外部导航模块消费/生产）

- BT 发布：
  - `/ly/navi/goal`
  - `/ly/navi/goal_pos_raw`（ToNavi=true 时的静态点位输入）
  - `/ly/navi/goal_pos`（legacy/direct-XY 兼容输出，默认不作为 bridge final）
  - `/goal_pose`（geometry_msgs/PoseStamped，bridge 最终导航目标）
  - `/ly/navi/speed_level`
  - `/ly/navi/target_rel`（追击相对目标点，x/y/z）
- BT 订阅：
  - `/ly/navi/vel`
  - `/ly/navi/lower_head`
  - `/ly/navi/reached`（`std_msgs/msg/Bool`，当前目标是否已到达）
  - `/ly/navi/reachable`（`std_msgs/msg/Bool`，当前目标是否有有效路径）

说明：本仓库内未包含完整导航执行节点；当前 TF bridge 链路的最终导航目标统一为 `/goal_pose`。外部导航状态 topic 只订阅不发布，且代码里的到达 topic 是 `/ly/navi/reached`，不是 `/ly/navi/reach`。

### 固定点位 / TF bridge 工具

- `scripts/navi/navitomap.sh`：手动把官方地图二维点 `/ly/navi/goal_pos_raw` 经 `navi_tf_bridge` 的 4x4 矩阵转成 `/goal_pose`。
- `scripts/navi/facemode.sh`：FaceMode 简短入口；位置参数为 `official_map_x official_map_y map_z`，单位 cm；`--bt-output` 输出到 `/ly/face_mode/angles` 给 BT 区域任务使用。
- `scripts/navi/map_aim_point_test.sh`：FaceMode 固定点朝向测试；给 `[official_map_x, official_map_y, map_z]`，X/Y 走 `tf_config.yaml` 的 raw-goal 矩阵，Z 直接按 map 高度使用。
- `scripts/navi/map_aim_point_attach.sh`：在已有 stack 上只附加 FaceMode/map_aim_point_node。

FaceMode 的独立测试节点默认直接发布 `/ly/control/angles`，可选发布 `/ly/control/firecode`，不发布底盘速度，默认 `yaw_sign=-1.0`。给 BT 使用时必须走 `/ly/face_mode/angles`，由 BT 统一发布 `/ly/control/angles` 和 firecode。BT 需要运行时切换固定朝向目标时，会发布 `/ly/face_mode/target_raw`，格式为 `[official_map_x, official_map_y, map_z]`，单位 cm。

`config/AreaManager.yaml` 放区域启用状态和区域任务参数；区域状态机内置的固定朝向目标通过 `/ly/face_mode/target_raw` 动态下发。`/ly/face_mode/angles` 本身仍然是角度 topic，不携带官方地图坐标。

`navi_tf_bridge` 还会按 `map_frame <- base_frame` 查询 TF，并用同一套 raw-goal 4x4 矩阵反解成官方地图厘米坐标发布 `/ly/navi/position`。BT 会把它作为 `/ly/me` 官方坐标之外的补充自身位置来源。

### 下位机串口接口（由 gimbal_driver 对接）

- 上位控制下发（经 `gimbal_driver` 串口写入）：
  - `/ly/control/angles`
  - `/ly/control/firecode`
  - `/ly/control/vel`
- 下位回传上行（由 `gimbal_driver` 发布）：
  - `/ly/gimbal/*`、`/ly/game/*`、`/ly/team/buff`、`/ly/position/data` 等

## 3. 关键兼容约定

- `behavior_tree` 已恢复速度桥接：`/ly/navi/vel -> /ly/control/vel`。  
  即：导航回传速度先进入 BT，再由 BT 转发给 gimbal_driver。
- 当 `Chase.ToNavi=true` 时，BT 不再执行本地追击速度闭环。追击输入有两源：
  `/ly/position/data` 官方坐标新鲜时发布 `/ly/navi/goal_pos_raw`，否则发布视觉 `/ly/navi/target_rel`。
- 当 `NaviSetting.ToNavi=true` 时，BT 发布的是 `/ly/navi/goal_pos_raw`，再由 `navi_tf_bridge` 转 `/goal_pose`；`false` 时不会走这条 4x4 静态转换链。
- 姿态 topic `/ly/control/posture` 已并入主控制幀字段 `GimbalControlData.Posture`（单通道下发）。
- 下发全量规格见：`docs/sentry/embedded/downlink_control_frame.md`。

## 4. 联调建议

- 联调前先确认 `/ly/control/vel` 是否只有预期发布者：  
  `ros2 topic info /ly/control/vel -v`
- 若缺少外部导航模块，不应将 `/ly/navi/*` 视为本仓缺件故障。  
