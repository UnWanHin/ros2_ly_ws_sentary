# ROS2 Topic Structure

Updated: 2026-05-08

本文记录当前哨兵上位机 ROS2 topic 结构，按接口边界分为：

- Internal：本仓内部节点之间的语义链路。
- External：外部导航/TF/导航状态提供方参与的接口。
- Embedded-facing：ROS topic 终点是 `gimbal_driver`，再由串口进入下位机或由下位机回传。

源码准入口：

- `src/behavior_tree/include/Topic.hpp`
- `src/gimbal_driver/main.cpp`
- `src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp`
- `src/navi_tf_bridge/src/pointer_solver_node.cpp`

## 1. 命名分层

| Prefix | 边界 | 说明 |
|---|---|---|
| `/ly/control/*` | Embedded-facing | 上位机控制输入，`gimbal_driver` 订阅后写入下行主控制帧。 |
| `/ly/gimbal/*` | Embedded-facing | 下位机/云台/底盘回读状态，由 `gimbal_driver` 发布。 |
| `/ly/game/*` | Embedded-facing | 裁判系统比赛状态、RFID、哨兵裁判信息和弹丸资源语义，由 `gimbal_driver` 从下位机上行拆出。 |
| `/ly/friend/*`, `/ly/enemy/*`, `/ly/team/*` | Embedded-facing | 我方/敌方血量、弹量、队伍增益等语义状态。 |
| `/ly/log/*` | Internal/Debug | 可选 raw 诊断 topic，默认关闭，不参与决策。 |
| `/ly/vision/*`, `/ly/bt/*`, `/ly/detector/*`, `/ly/predictor/*`, `/ly/buff/*`, `/ly/outpost/*` | Internal | 视觉、预测、任务模式和 BT 内部协作。 |
| `/ly/face_mode/*` | Internal | FaceMode 固定点朝向链路。 |
| `/ly/navi/*`, `/goal_pose` | External | 导航目标、导航桥、导航状态和 TF 导出的定位接口。 |

## 2. Embedded-Facing Control

这些 topic 是上位机向下位机控制的 ROS 入口。

| Topic | Type | Publisher | Subscriber | 结构/语义 |
|---|---|---|---|---|
| `/ly/control/angles` | `gimbal_driver/msg/GimbalAngles` | `behavior_tree` 或 FaceMode | `gimbal_driver` | `header`, `yaw`, `pitch`，云台目标角。 |
| `/ly/control/firecode` | `gimbal_driver/msg/FireCode` | `behavior_tree` | `gimbal_driver` | `field_mask`, `fire_status`, `cap_state`, `follow_mode`, `aim_mode`, `rotate`, `raw`。写入 1B `FireCode`。 |
| `/ly/control/vel` | `gimbal_driver/msg/ControlVelocity` | `behavior_tree` | `gimbal_driver` | `x_mps`, `y_mps`, `raw_x`, `raw_y`, `use_raw`。当前 BT 用 `use_raw=true`。 |
| `/ly/control/posture` | `gimbal_driver/msg/SentryCmd` | `behavior_tree` | `gimbal_driver` | 姿态专用入口，只读 `FIELD_POSTURE/posture`。写入 `SentryCmd bit21-22`。 |
| `/ly/control/sentry_cmd` | `gimbal_driver/msg/SentryCmd` | 手动工具/后续策略 | `gimbal_driver` | 完整哨兵裁判命令入口，用于复活、兑弹、远程回血、能量机关确认等。 |

当前 posture 测试命令：

```bash
ros2 topic pub /ly/control/posture gimbal_driver/msg/SentryCmd "{field_mask: 32, posture: 2}" -1
```

完整 `SentryCmd` 测试命令：

```bash
ros2 topic pub /ly/control/sentry_cmd gimbal_driver/msg/SentryCmd "{field_mask: 32, posture: 2}" -1
```

## 3. Embedded Feedback

这些 topic 由 `gimbal_driver` 从下位机串口上行或裁判系统数据拆出。

| Topic | Type | Consumer | 结构/语义 |
|---|---|---|---|
| `/ly/gimbal/angles` | `gimbal_driver/msg/GimbalAngles` | `behavior_tree`, FaceMode | 当前云台角 `yaw/pitch`。 |
| `/ly/gimbal/firecode` | `gimbal_driver/msg/FireCode` | `behavior_tree` | 下位机回读火控状态，`field_mask=FIELD_ALL`。 |
| `/ly/gimbal/vel` | `gimbal_driver/msg/Vel` | 调试/兼容 | `header`, `x`, `y`。 |
| `/ly/gimbal/chassis` | `gimbal_driver/msg/Chassis` | `behavior_tree` | `steer_angle`, `angular_velocity`, `velocity_x`, `velocity_y`。 |
| `/ly/gimbal/big_yaw_angles` | `std_msgs/msg/Float32` | 调试/可视化 | 大 yaw 角。 |
| `/ly/gimbal/posture` | `std_msgs/msg/UInt8` | `behavior_tree` | 下位机/裁判姿态回读，`1/2/3` 有效；TypeID 7 的 `/ly/game/sentry/info.posture` 会覆盖 TypeID 6 回读；不要镜像命令。 |
| `/ly/gimbal/capV` | `std_msgs/msg/UInt8` | `behavior_tree` | 电容电压/电容状态回读。 |
| `ly/gimbal/eventdata` | `std_msgs/msg/UInt32` | legacy 调试/兼容 | legacy 原始 event data，注意当前 gimbal 侧定义无前导 `/`；`behavior_tree` 不再订阅。 |
| `/ly/game/event_data` | `gimbal_driver/msg/EventData` | `behavior_tree` | 裁判 `0x0101 event_data` 语义拆字段。 |
| `/ly/game/all` | `gimbal_driver/msg/GameData` | `behavior_tree` | `gamecode`, `ammoleft`, `timeleft`, `selfhealth`, `exteventdata` 摘要。 |
| `/ly/game/is_start` | `std_msgs/msg/Bool` | `behavior_tree` | 比赛是否开始。 |
| `/ly/game/time_left` | `std_msgs/msg/UInt16` | `behavior_tree` | 剩余比赛时间。 |
| `/ly/friend/is_team_red` | `std_msgs/msg/Bool` | `behavior_tree` | 我方是否红方。 |
| `/ly/friend/is_at_home` | `std_msgs/msg/Bool` | `behavior_tree` | 是否回补/回家状态。 |
| `/ly/friend/is_precaution` | `std_msgs/msg/Bool` | `behavior_tree` | 英雄预警。 |
| `/ly/friend/hp` | `gimbal_driver/msg/Health` | `behavior_tree` | 我方各兵种血量。 |
| `/ly/friend/base_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 我方基地血量。 |
| `/ly/friend/op_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 我方前哨血量。 |
| `/ly/friend/ammo_left` | `std_msgs/msg/UInt16` | `behavior_tree` | 当前弹量。 |
| `/ly/friend/uwb_pos` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` | 自身官方坐标 `[x, y]`，来自下位机 TypeID 5。 |
| `/ly/friend/uwb_yaw` | `std_msgs/msg/UInt16` | 调试/兼容 | 自身 UWB yaw。 |
| `/ly/game/rfid` | `gimbal_driver/msg/RfidStatus` | `behavior_tree` | 裁判 `0x0209 rfid_status` 语义拆字段，TypeID 8 的 `rfid_status_2` 也合并在这里；BT 内部聚合为 `RfidMatchState`。 |
| `/ly/enemy/hp` | `gimbal_driver/msg/Health` | `behavior_tree` | 敌方各兵种血量。 |
| `/ly/enemy/base_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 敌方基地血量。 |
| `/ly/enemy/op_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 敌方前哨血量。 |
| `/ly/team/buff` | `gimbal_driver/msg/BuffData` | `behavior_tree` | 队伍增益与剩余能量。 |
| `/ly/position/data` | `gimbal_driver/msg/PositionData` | `behavior_tree` | 官方坐标系统中一组友方/敌方机器人位置。 |
| `/ly/bullet/speed` | `std_msgs/msg/Float32` | predictor/调试 | 旧弹速 topic，来自 TypeID 5。 |
| `/ly/game/sentry/info` | `gimbal_driver/msg/SentryInfo` | `behavior_tree`/调试 | 裁判 `0x020D sentry_info/sentry_info_2` 语义拆字段；其中有效 `posture` 会同步覆盖 `/ly/gimbal/posture`；BT 使用 `can_activate_energy_mechanism` 判斷打能量機關確認窗口。 |
| `/ly/game/bullet` | `gimbal_driver/msg/BulletInfo` | 后续策略/调试 | TypeID 7/8 合并出的弹速、发射事件、允许发弹量、金币；当前 BT 还未订阅，RFID2 不在这里。 |

## 3.1 Raw Debug Topics

这些 topic 由 `config/common.yaml` 的 `gimbal_raw.topic.enable` 控制，默认关闭，只用于在线观察或 rosbag 记录串口原始幀。

| Topic | Type | Consumer | 结构/语义 |
|---|---|---|---|
| `/ly/log/gimbal_raw_rx` | `gimbal_driver/msg/GimbalRawFrame` | 调试/rosbag | 下位机 -> 上位机 raw `TypedMessage`，`type_id=0..8`，`data` 是原始 bytes。 |
| `/ly/log/gimbal_raw_tx` | `gimbal_driver/msg/GimbalRawFrame` | 调试/rosbag | 上位机 -> 下位机 raw `GimbalControlData`，`type_id=255`，`data` 是 17B 主控制幀。 |

## 4. Internal Vision And Aim

| Topic | Type | Direction | 结构/语义 |
|---|---|---|---|
| `/ly/vision/mode` | `std_msgs/msg/UInt8` | `behavior_tree` -> detector/buff/outpost | `0=disabled`, `1=armor`, `2=buff`, `3=outpost`。 |
| `/ly/bt/target` | `std_msgs/msg/UInt8` | `behavior_tree` -> detector/predictor | 当前 BT 选择的装甲板目标类型。 |
| `/ly/detector/armors` | `auto_aim_common/msg/Armors` | detector -> tracker/predictor | `header`, `Armor[] armors`, `Car[] cars`, 当前 `yaw/pitch`，可用装甲板索引。 |
| `/ly/predictor/target` | `auto_aim_common/msg/Target` | predictor -> `behavior_tree` | `status`, `buff_follow`, `yaw`, `pitch`。普通辅瞄结果。 |
| `/ly/back_cam/target` | `auto_aim_common/msg/Target` | back camera -> `behavior_tree` | 后置相机目标结果。 |
| `/ly/buff/target` | `auto_aim_common/msg/Target` | buff_hitter -> `behavior_tree` | 打符目标角和可击打状态。 |
| `/ly/outpost/target` | `auto_aim_common/msg/Target` | outpost_hitter -> `behavior_tree` | 打前哨目标角和可击打状态。 |
| `/ly/face_mode/target_raw` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` -> FaceMode solver | `[official_map_x, official_map_y, map_z]`，x/y 为官方地图 cm，z 为 map 系高度。 |
| `/ly/face_mode/angles` | `gimbal_driver/msg/GimbalAngles` | FaceMode solver -> `behavior_tree` | 固定点朝向解算出的 yaw/pitch。BT 在 FaceMode 激活时转发到 `/ly/control/angles`。 |

## 5. External Navigation And TF Bridge

这些 topic 是本仓和外部导航栈的边界。BT 不直接发布 `/goal_pose`，正常由 `navi_tf_bridge` 输出。

| Topic | Type | Direction | 结构/语义 |
|---|---|---|---|
| `/ly/navi/goal` | `std_msgs/msg/UInt8` | `behavior_tree` -> navigation/兼容 | 点位 ID 模式。 |
| `/ly/navi/goal_pos_raw` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` -> `navi_tf_bridge` | 官方地图坐标 `[x_cm, y_cm]`；经 `tf_config.yaml` 静态矩阵转导航坐标。固定点位和官方坐标追击源都走这条链路。 |
| `/ly/navi/goal_pos` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` 或 bridge -> navigation/兼容 | 已处理坐标输出；`ToNavi=true` 时通常不作为最终导航目标。 |
| `/ly/navi/target_rel` | `auto_aim_common/msg/RelativeTarget` | `behavior_tree` -> `navi_tf_bridge` | 追击相对目标，默认 `gx_camera` frame，字段含 `x/y/z`, `distance_m`, `yaw_error_deg`, `pitch_error_deg`, `armor_type`, `aim_mode`。 |
| `/ly/navi/target_map` | `geometry_msgs/msg/PointStamped` | `navi_tf_bridge` -> debug | 追击目标转换到 map/导航 frame 后的点。 |
| `/ly/navi/position` | `std_msgs/msg/UInt16MultiArray` | `navi_tf_bridge` -> `behavior_tree` | TF 导出的自身位置，再逆变换为官方地图 cm `[x, y]`，用于区域判断辅助。 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `navi_tf_bridge` -> external navigation | 最终导航目标。 |
| `/ly/navi/reached` | `std_msgs/msg/Bool` | external navigation -> `behavior_tree` | 当前目标是否到达；true=到达，false=路上。 |
| `/ly/navi/reachable` | `std_msgs/msg/Bool` | external navigation -> `behavior_tree` | 当前目标是否有有效路径；true=可达，false=不可达。 |
| `/ly/navi/speed_level` | `std_msgs/msg/UInt8` | `behavior_tree` -> navigation/兼容 | 导航速度档位。 |
| `/ly/navi/lower_head` | `std_msgs/msg/UInt8` | navigation/兼容 -> `behavior_tree` | 低头/通过特定路径时的兼容状态。 |
| `/ly/navi/vel` | `gimbal_driver/msg/Vel` | 兼容/调试 | 当前 BT 代码保留 publisher，但主控制速度走 `/ly/control/vel`。 |

追击多源退化顺序：`Chase.ToNavi=true` 且 `/ly/position/data` 中目标敌方坐标和自身坐标都新鲜时，BT 优先发布 `/ly/navi/goal_pos_raw`；任一条件不满足时退回 `/ly/navi/target_rel`。两条链路不会在同一 tick 同时作为有效追击目标发布。

导航状态保护：

- `/ly/navi/reached` 和 `/ly/navi/reachable` 必须在当前 goal 发布后收到并保持新鲜。
- 状态缺失或超时时，BT 回退到自身位置和目标点距离判断。
- topic 名是 `/ly/navi/reached`，不是 `/ly/navi/reach`。

## 6. Key Message Structures

这里只列当前策略常用结构；完整字段以 `src/*/msg/*.msg` 为准。

| Type | Key fields | 用途 |
|---|---|---|
| `gimbal_driver/msg/GimbalAngles` | `header`, `yaw`, `pitch` | 云台角控制/回读/FaceMode 输出。 |
| `gimbal_driver/msg/FireCode` | `field_mask`, `fire_status`, `cap_state`, `follow_mode`, `aim_mode`, `rotate`, `raw` | 1B 火控语义。 |
| `gimbal_driver/msg/ControlVelocity` | `header`, `x_mps`, `y_mps`, `raw_x`, `raw_y`, `use_raw` | 下发底盘速度。 |
| `gimbal_driver/msg/SentryCmd` | `field_mask`, `confirm_free_revive`, `confirm_immediate_revive`, `exchange_projectile_allowance`, `remote_projectile_exchange_count`, `remote_hp_exchange_count`, `posture`, `confirm_energy_activate`, `raw` | 裁判 `0x0301/0x0120 sentry_cmd` 语义。 |
| `gimbal_driver/msg/Chassis` | `header`, `steer_angle`, `angular_velocity`, `velocity_x`, `velocity_y` | TypeID 6 底盘回读。 |
| `gimbal_driver/msg/GameData` | `gamecode`, `ammoleft`, `timeleft`, `selfhealth`, `exteventdata` | 比赛摘要。 |
| `gimbal_driver/msg/EventData` | `raw`, supply/energy/highland/dart/gain point fields | 裁判 `0x0101 event_data` 语义拆分。 |
| `gimbal_driver/msg/Health` | `hero`, `engineer`, `infantry1`, `infantry2`, `reserve`, `sentry` | 友方/敌方血量。 |
| `gimbal_driver/msg/RfidStatus` | `raw`, RFID gain/crossing bits, `has_rfid_status_2`, `rfid_status_2_raw` | 裁判 RFID 状态。 |
| `gimbal_driver/msg/SentryInfo` | `sentry_info_raw`, `sentry_info_2_raw`, exchange/revive/out_of_combat/posture/energy fields | 裁判 `0x020D` 哨兵状态。 |
| `gimbal_driver/msg/BulletInfo` | `initial_speed`, shoot data, projectile allowance, remaining coin | TypeID 7/8 弹丸与资源状态。 |
| `gimbal_driver/msg/GimbalRawFrame` | `header`, `direction`, `type_id`, `data`, `firecode_raw`, `sentry_cmd_raw` | 可选 raw 串口诊断 topic。 |
| `auto_aim_common/msg/Target` | `header`, `status`, `buff_follow`, `yaw`, `pitch` | predictor/buff/outpost 角度目标。 |
| `auto_aim_common/msg/RelativeTarget` | `header`, `valid`, `x`, `y`, `z`, `distance_m`, `yaw_error_deg`, `pitch_error_deg`, `armor_type`, `aim_mode` | 追击相对目标。 |
| `auto_aim_common/msg/Armors` | `header`, `Armor[] armors`, `Car[] cars`, `yaw`, `pitch`, predictor target index | 检测输出给跟踪/预测。 |

## 7. Maintenance Rules

- 改 `src/behavior_tree/include/Topic.hpp` 或 `src/gimbal_driver/main.cpp` 的 topic 名/类型时，同步更新本文档。
- 改 `gimbal_driver/msg/*.msg` 或 `auto_aim_common/msg/*.msg` 的字段时，同步更新本文档第 6 节。
- 外部导航 topic 行为变更时，同步更新 `docs/sentry/external/navi_status_topics.md`。
- 下位机串口帧、TypeID、`SentryCmd` 位定义变更时，同步更新 `docs/sentry/embedded/serial_data_mapping.md`。
