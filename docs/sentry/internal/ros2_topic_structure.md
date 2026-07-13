# ROS2 Topic Structure

> 当前正式辅瞄输入仅为外部 `/ly/aim/*`。本文件中的 `auto_aim_common` 旧消息定义是接口库存，不代表仍有内部 detector/tracker/predictor 节点或 topic。

Updated: 2026-07-12

本文记录当前哨兵上位机 ROS2 topic 结构，按接口边界分为：

- Internal：本仓内部节点之间的语义链路。
- External：外部 aim、TF、导航/导航状态提供方参与的接口。
- Embedded-facing：ROS topic 终点是 `gimbal_driver`，再由串口进入下位机或由下位机回传。

源码准入口：

- `src/behavior_tree/include/Topic.hpp`
- `src/gimbal_driver/main.cpp`
- `src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp`
- `src/navi_tf_bridge/src/pointer_solver_node.cpp`

## 1. 命名分层

| Prefix | 边界 | 说明 |
|---|---|---|
| `/ly/control/*` | Embedded-facing | 上位机控制输入，`gimbal_driver` 按 topic 写入 `DownlinkTypeID=0x00~0x03` 对应 frame。 |
| `/ly/gimbal/*` | Embedded-facing | 下位机/云台/底盘回读状态，由 `gimbal_driver` 发布。 |
| `/ly/game/*` | Embedded-facing | 裁判系统比赛状态、RFID、哨兵裁判信息和弹丸资源语义，由 `gimbal_driver` 从下位机上行拆出。 |
| `/ly/friend/*`, `/ly/enemy/*`, `/ly/team/*` | Embedded-facing | 我方/敌方血量、弹量、队伍增益等语义状态。 |
| `/ly/log/*` | Internal/Debug | 可选 raw 诊断 topic，默认关闭，不参与决策。 |
| `/ly/aim/*` | External | 外部 `sentry.aim` 和 BT 的正式辅瞄接口。 |
| `/tf`, `/tf_static` | External | 正式链路由外部 `sentry_tf` 发布 gimbal TF；本仓 `tf_tree` 只作 fallback，不能和外部 `sentry_tf` 同时发布同一套 frame。 |
| `/ly/vision/*`, `/ly/bt/*` | Internal | BT 模式和调试语义；`/ly/vision/mode` 只反映 AimMode，不再选择本仓视觉节点。 |
| `/ly/face_mode/*` | Internal | FaceMode 固定点朝向链路。 |
| `/ly/navi/*`, `/goal_pose` | External | 导航目标、导航桥、导航状态和 TF 导出的定位接口。 |

## 2. Embedded-Facing Control

这些 topic 是上位机向下位机控制的 ROS 入口。

| Topic | Type | Publisher | Subscriber | 结构/语义 |
|---|---|---|---|---|
| `/ly/control/angles` | `gimbal_driver/msg/GimbalAngles` | `behavior_tree` 或 FaceMode | `gimbal_driver` | `header`, `yaw`, `pitch`，云台目标角。 |
| `/ly/control/firecode` | `gimbal_driver/msg/FireCode` | `behavior_tree` | `gimbal_driver` | `field_mask`, `fire_status`, `cap_state`, `follow_mode`, `aim_mode`, `rotate`, `raw`。写入 1B `FireCode`。 |
| `/ly/control/vel` | `gimbal_driver/msg/ControlVelocity` | `behavior_tree` | `gimbal_driver` | `x_mps`, `y_mps`, `raw_x`, `raw_y`, `use_raw`。当前 BT 用 `use_raw=true`。 |
| `/ly/control/posture` | `gimbal_driver/msg/SentryCmd` | `behavior_tree` | `gimbal_driver` | 姿态专用入口，只读 `FIELD_POSTURE/posture`，写入独立 `DownlinkTypeID=0x01` 的 `SentryCmd bit21-23`；可取 `1~6`。 |
| `/ly/control/sentry_cmd` | `gimbal_driver/msg/SentryCmd` | 手动工具/后续策略 | `gimbal_driver` | 完整哨兵裁判命令入口，发独立 `DownlinkTypeID=0x01`，用于复活、兑弹、远程回血、能量机关确认等。 |
| `/ly/control/map_path` | `gimbal_driver/msg/MapPath` | 上位机路径策略/工具 | `gimbal_driver` | 一次下发 `DownlinkTypeID=0x02`，裁判 `0x0307 map_data_t` 语义。 |
| `/ly/control/custom_info` | `gimbal_driver/msg/CustomInfo` | 上位机工具 | `gimbal_driver` | 一次下发 `DownlinkTypeID=0x03`，裁判 `0x0308 custom_info_t`；携带完整 30B UTF-16 原始字节。 |
| `/ly/bt/sentry_position` | `geometry_msgs/msg/PointStamped` | `behavior_tree` | `gimbal_driver` | BT 融合后的哨兵自身位置，`frame_id=map`，单位 m；`gimbal_driver` 转 cm 后写入 `DownlinkTypeID=0x04` 坐标 frame。 |

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
| `/ly/gimbal/posture` | `std_msgs/msg/UInt8` | `behavior_tree` | 下位机/裁判姿态回读，`1/2/3` 有效；来源为 TypeID 7 的 `/ly/game/sentry/info.posture`；不要镜像命令。 |
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
| `/ly/friend/op_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 我方前哨血量；优先 TypeID 10 的 `0x0003 ally_outpost_HP` 精确值，TypeID 1 的 `GameCode.SelfOutpostHealth * 25` 只做 fallback。 |
| `/ly/friend/ammo_left` | `std_msgs/msg/UInt16` | `behavior_tree` | 当前弹量。 |
| `/ly/friend/uwb_pos` | `gimbal_driver/msg/StampedUInt16MultiArray` | `behavior_tree` | 自身官方坐标融合源，`data=[x, y]`，来自下位机 TypeID 5；`header.stamp` 为 `gimbal_driver` 发布时间。 |
| `/ly/friend/uwb_yaw` | `std_msgs/msg/UInt16` | 调试/兼容 | 自身 UWB yaw。 |
| `/ly/game/rfid` | `gimbal_driver/msg/RfidStatus` | `behavior_tree` | 裁判 `0x0209 rfid_status` 语义拆字段；TypeID 4 的低 32 bit 和 TypeID 8 的 `rfid_status_2` 在 `gimbal_driver` 内保留 shadow，任一侧更新都会合并发布完整消息；BT 内部聚合为 `RfidMatchState`。 |
| `/ly/enemy/hp` | `gimbal_driver/msg/Health` | `behavior_tree` | 敌方各兵种血量。 |
| `/ly/enemy/base_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 敌方基地血量。 |
| `/ly/enemy/op_hp` | `std_msgs/msg/UInt16` | `behavior_tree` | 敌方前哨血量；优先 TypeID 10 的 `0x0003 enemy_outpost_HP` 精确值，TypeID 1 的 `GameCode.EnemyOutpostHealth * 25` 只做 fallback。 |
| `/ly/team/buff` | `gimbal_driver/msg/BuffData` | `behavior_tree` | 队伍增益与剩余能量。 |
| `/ly/position/data` | `gimbal_driver/msg/PositionData` | `behavior_tree` | 官方坐标系统中一组友方/敌方机器人位置；`friendcarid == Sentry` 时作为自身坐标融合源。 |
| `/ly/bullet/speed` | `std_msgs/msg/Float32` | predictor/调试 | 旧弹速 topic，来自 TypeID 5。 |
| `/ly/game/sentry/info` | `gimbal_driver/msg/SentryInfo` | `behavior_tree`/调试 | 裁判 `0x020D sentry_info/sentry_info_2/sentry_info_3` 语义拆字段；其中有效 `posture` 会同步覆盖 `/ly/gimbal/posture`；BT 使用 `can_activate_energy_mechanism` 判斷打能量機關確認窗口。 |
| `/ly/game/bullet` | `gimbal_driver/msg/BulletInfo` | `behavior_tree`/调试 | TypeID 7/8 合并出的弹速、发射事件、允许发弹量、金币；BT 当前只订阅并缓存，暂不参与正式决策；RFID2 不在这里。 |
| `/ly/game/map_command` | `gimbal_driver/msg/MapCommand` | `behavior_tree`/调试 | TypeID 9 转出的裁判 `0x0303 map_command_t`；`header.stamp` 为 `gimbal_driver` 发布时间；BT 当前只订阅并缓存，不触发导航。 |
| `/ly/game/damage_difference` | `std_msgs/msg/Int16` | 调试/后续策略 | TypeID 6 转出的裁判 `0x0003 game_robot_HP_t` offset 8，`己方全队总伤害 - 对方全队总伤害`。 |

## 3.1 Raw Debug Topics

这些 topic 由 `config/common.yaml` 的 `gimbal_raw.topic.enable` 控制，默认关闭，只用于在线观察或 rosbag 记录串口原始幀。

| Topic | Type | Consumer | 结构/语义 |
|---|---|---|---|
| `/ly/log/gimbal_raw_rx` | `gimbal_driver/msg/GimbalRawFrame` | 调试/rosbag | 下位机 -> 上位机 raw `TypedMessage`，`type_id=0..9`，`data` 是原始 bytes。 |
| `/ly/log/gimbal_raw_tx` | `gimbal_driver/msg/GimbalRawFrame` | 调试/rosbag | 上位机 -> 下位机 raw downlink frame；`type_id=255` 表示 control 诊断 frame，`type_id=254` 表示 sentry coordinate 诊断 frame，`data` 是真实 17B 串口 bytes。 |

## 4. External Aim Topics

正式 `sentry_all` 只使用 `/ly/aim/*` 外部 aim 接口。内部视觉、追踪、预测、打符、前哨和射表标定包已经移除；BT 保留的旧消息声明不构成运行中的 ROS topic 链路。

| Topic | Type | Direction | 结构/语义 |
|---|---|---|---|
| `/ly/vision/mode` | `std_msgs/msg/UInt8` | `behavior_tree` -> debug/observer | `0=disabled`, `1=armor`, `2=buff`, `3=outpost`；只反映当前 AimMode。 |
| `/ly/bt/target` | `std_msgs/msg/UInt8` | `behavior_tree` -> debug/observer | 当前 BT 选择的目标类型；正式选目标同时发布 `/ly/aim/select_target`。 |
| `/ly/aim/armor_targets` | `sentry_msgs/msg/AimTargetArray` | external aim -> `behavior_tree` | 外部辅瞄输出的可打目标列表；数组元素是 `AimTarget.msg`，BT 用它生成 `hitableTargets`、目標距離和 Chase 相對 point。 |
| `/ly/aim/select_target` | `sentry_msgs/msg/AimTarget` | `behavior_tree` -> external aim | BT 选择的目标 `id`，`header.stamp` 为当前发布时间，`position` 尽量填最近一次 `/ly/aim/armor_targets` 中同 id 的位置。 |
| `/ly/aim/result` | `sentry_msgs/msg/AimResult` | external aim -> `behavior_tree` | 外部辅瞄 `follow`、最终 yaw/pitch 和 `fire` 门控；`follow=true` 时 BT 接管角度并转发 `/ly/control/angles`，`fire=true` 时翻转 `/ly/control/firecode`。 |
| `/ly/face_mode/target_raw` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` -> FaceMode solver | `[official_map_x, official_map_y, map_z]`，x/y 为官方地图 cm，z 为 map 系高度。 |
| `/ly/face_mode/angles` | `gimbal_driver/msg/GimbalAngles` | FaceMode solver -> `behavior_tree` | 固定点朝向解算出的 yaw/pitch。正式 `sentry_all` 默认由 `map_aim_point_node` 用 TF 相对几何输出，BT 在 FaceMode 激活时转发到 `/ly/control/angles`。 |

## 5. External Navigation And TF Bridge

这些 topic 是本仓和外部导航栈的边界。BT 不直接发布 `/goal_pose`，正常由 `navi_tf_bridge` 输出。

| Topic | Type | Direction | 结构/语义 |
|---|---|---|---|
| `/ly/navi/goal` | `std_msgs/msg/UInt8` | `behavior_tree` -> navigation/兼容 | 点位 ID 模式。 |
| `/ly/navi/goal_pos_raw` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` -> `navi_tf_bridge` | 官方地图坐标 `[x_cm, y_cm]`；经 `tf_config.yaml` 静态矩阵转导航坐标。固定点位和官方坐标追击源都走这条链路。 |
| `/ly/navi/goal_pos` | `std_msgs/msg/UInt16MultiArray` | `behavior_tree` 或 bridge -> navigation/兼容 | 已处理坐标输出；`ToNavi=true` 时通常不作为最终导航目标。 |
| `/ly/navi/target_rel` | `auto_aim_common/msg/RelativeTarget` | `behavior_tree` -> `navi_tf_bridge` | 追击目标点，默认 `gimbal_world` frame，字段含 `x/y/z`, `distance_m`, `yaw_error_deg`, `pitch_error_deg`, `armor_type`, `aim_mode`。 |
| `/ly/navi/target_map` | `geometry_msgs/msg/PointStamped` | `navi_tf_bridge` -> debug | 追击目标转换到 map/导航 frame 后的点。 |
| `/ly/navi/target_official` | `gimbal_driver/msg/StampedUInt16MultiArray` | `navi_tf_bridge` -> `behavior_tree` | 有效追击目标和 `/ly/aim/armor_targets` 中每个有效 target point 反算到 official-map cm，`data=[official_x_cm, official_y_cm, armor_type]`；BT 用作敌方位置 fallback。 |
| `/ly/navi/position` | `gimbal_driver/msg/StampedUInt16MultiArray` | `navi_tf_bridge` -> `behavior_tree` | TF 导出的自身位置；`data=[official_x_cm, official_y_cm]` 为逆变换后的官方地图 cm，作为 BT 自身坐标融合源；`header.stamp` 为 TF source stamp；`map_point` 为 map 系 m 坐标，附带 `map_frame/source_frame`。 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `navi_tf_bridge` -> external navigation | 最终导航目标。 |
| `/ly/navi/reached` | `std_msgs/msg/Bool` | external navigation -> `behavior_tree` | 外部导航到达源；true=导航端确认到达，false=导航端尚未确认到达。BT 内部最终 reached 还要结合自身融合坐标距离和保护逻辑。 |
| `/ly/navi/reachable` | `std_msgs/msg/Bool` | external navigation -> `behavior_tree` | 当前目标是否有有效路径；true=可达，false=不可达。 |
| `/ly/navi/reach_state` | `auto_aim_common/msg/GoalReach` | `behavior_tree` -> diagnostics/consumers | BT 内部 composite goal reach state；包含 status、reason、goal id/坐标、external reached/reachable freshness、融合自身坐标距离、grace 和 timeout。 |
| `/ly/navi/should_rotate` | `std_msgs/msg/Bool` | external navigation -> `behavior_tree`；可选 `gimbal_driver` debug | 区域兼容旋转控制；true=恢复 BT 正常小陀螺/巡逻，false=关闭小陀螺并请求 `FollowMode`。只有 `io_config.navigation_mode.enabled && should_rotate.enabled` 时 driver 才会为单独导航调试直接消费它。 |
| `/ly/navi/speed_level` | `std_msgs/msg/UInt8` | `behavior_tree` -> navigation/兼容 | 导航速度档位。 |
| `/ly/navi/lower_head` | `std_msgs/msg/UInt8` | navigation/兼容 -> `behavior_tree` | 低头/通过特定路径时的兼容状态。 |
| `/ly/navi/vel` | `gimbal_driver/msg/Vel` | 导航/兼容/调试 | 正式链路由 BT 接收后转 `/ly/control/vel`；`gimbal_driver` 仅在 legacy `io_config.navigation_test=true` 或 `io_config.navigation_mode.enabled && vel_chain=true` 时直接订阅，用于单独导航速度下发调试。 |

追击多源退化顺序：`Chase.ToNavi=true` 时，BT 优先使用 `/ly/aim/armor_targets` 里当前选中目标的 point 发布 `/ly/navi/target_rel`，消息携带来源 frame（默认 `gimbal_world`），由 `navi_tf_bridge` 转成 `/goal_pose`。bridge 也会直接订阅 `/ly/aim/armor_targets`，把 array 中每个有效 target point 反算成 `/ly/navi/target_official`；BT 仅在对应敌方没有新鲜非零 `/ly/position/data` 时把它写回 `enemyRobots` 和 `/ly/enemy/info`。`Chase.AreaLimit` 来自 BT JSON：`/ly/aim/armor_targets` 追击路径由 `navi_tf_bridge` 限制 `/goal_pose`，全量 `/ly/navi/target_official` 只作为敌方位置 fallback，不直接发布导航目标；官方坐标 fallback 追击路径由 BT 在发布 `/ly/navi/goal_pos_raw` 前限制目标点。`ChaseEnableCrossArea=false` 时限制在自身当前大区域边界内侧；`true` 时可追到 `DecisionAutonomy.NaviGoal` 已开启的大区域，未开启区域仍不允许。该限制不关闭云台跟踪/开火。只有 `/ly/aim/armor_targets` 追击点不可用时，才退化到 `/ly/position/data` 官方坐标源。两条追击链路不会在同一 tick 同时作为有效导航目标发布。

导航状态保护：

- `/ly/navi/reached` 和 `/ly/navi/reachable` 必须在当前 goal 发布后收到并保持新鲜，才能作为当前 goal 的外部状态源。
- `/ly/navi/reached=false` 不是最终未到达事实；goal-start grace 后，BT 可用自身位置和目标点距离生成 composite reached，并通过 `/ly/navi/reach_state` 暴露完整状态。
- 正式 BT 链中，`/ly/navi/should_rotate` 的新鲜度由 `NaviRotateControl.FreshTimeoutMs` 控制；新鲜 `true` 会按配置清掉外部置入的 `FollowMode`，但当前默认保留 regional 区域任务 FaceMode。默认超时后按允许旋转处理，但不继续保留旧 `false`。driver-only `navigation_mode` 是不做 freshness arbitration 的调试 bypass，仅在启用时采用最后一笔 `should_rotate`。
- topic 名是 `/ly/navi/reached`，不是 `/ly/navi/reach`。

## 6. Key Message Structures

这里只列当前策略常用结构；完整字段以 `src/*/msg/*.msg` 为准。

| Type | Key fields | 用途 |
|---|---|---|
| `gimbal_driver/msg/GimbalAngles` | `header`, `yaw`, `pitch` | 云台角控制/回读/FaceMode 输出。 |
| `gimbal_driver/msg/FireCode` | `field_mask`, `fire_status`, `cap_state`, `follow_mode`, `aim_mode`, `rotate`, `raw` | 1B 火控语义。 |
| `gimbal_driver/msg/ControlVelocity` | `header`, `x_mps`, `y_mps`, `raw_x`, `raw_y`, `use_raw` | 下发底盘速度。 |
| `gimbal_driver/msg/SentryCmd` | `field_mask`, `confirm_free_revive`, `confirm_immediate_revive`, `exchange_projectile_allowance`, `remote_projectile_exchange_count`, `remote_hp_exchange_count`, `posture`, `confirm_energy_activate`, `raw` | 裁判 `0x0301/0x0120 sentry_cmd` 语义。 |
| `gimbal_driver/msg/MapPath` | `intention`, `start_position_x_dm`, `start_position_y_dm`, `delta_x_dm[49]`, `delta_y_dm[49]`, `sender_id` | 裁判 `0x0307 map_data_t` 语义。 |
| `gimbal_driver/msg/CustomInfo` | `sender_id`, `receiver_id`, `user_data_utf16[30]` | 裁判 `0x0308 custom_info_t` 语义。 |
| `gimbal_driver/msg/Chassis` | `header`, `steer_angle`, `angular_velocity`, `velocity_x`, `velocity_y` | TypeID 6 底盘回读；同一 TypeID 还会额外发布 `/ly/game/damage_difference`。 |
| `gimbal_driver/msg/GameData` | `gamecode`, `ammoleft`, `timeleft`, `selfhealth`, `exteventdata` | 比赛摘要。 |
| `gimbal_driver/msg/EventData` | `raw`, supply/energy/highland/dart/gain point fields | 裁判 `0x0101 event_data` 语义拆分。 |
| `gimbal_driver/msg/Health` | `hero`, `engineer`, `infantry1`, `infantry2`, `reserve`, `sentry` | 友方/敌方血量。 |
| `gimbal_driver/msg/RfidStatus` | `raw`, RFID gain/crossing bits, `has_rfid_status_2`, `rfid_status_2_raw` | 裁判 RFID 状态。 |
| `gimbal_driver/msg/SentryInfo` | `sentry_info_raw`, `sentry_info_2_raw`, `sentry_info_3_raw`, exchange/revive/out_of_combat/posture/energy/enhanced posture/remaining seconds fields | 裁判 `0x020D` 哨兵状态。 |
| `gimbal_driver/msg/BulletInfo` | `initial_speed`, shoot data, projectile allowance, remaining coin | TypeID 7/8 弹丸与资源状态。 |
| `gimbal_driver/msg/MapCommand` | `header`, `has_target_position`, `target_position_x_m`, `target_position_y_m`, `has_target_robot`, `target_robot_id`, `cmd_keyboard`, `cmd_source` | TypeID 9 / 裁判 `0x0303` 小地图命令输入。 |
| `gimbal_driver/msg/GimbalRawFrame` | `header`, `direction`, `type_id`, `data`, `firecode_raw`, `sentry_cmd_raw` | 可选 raw 串口诊断 topic；TX `type_id=255/254/253/252/251` 依次为 control、sentry command、map path、custom info、sentry coordinate。 |
| `sentry_msgs/msg/AimTargetArray` | `header`, `aim_targets[]` | 外部 aim 可打目标列表；`/ly/aim/armor_targets` 使用 `SensorDataQoS`。 |
| `sentry_msgs/msg/AimTarget` | `header`, `position`, `id` | 外部 aim 候选目标和 BT 目标选择共用结构；`id` 对齐 `ArmorType`，`position` 为米制 point，`header.frame_id` 非空时才作为 Chase 真值点参与 TF 转换。 |
| `sentry_msgs/msg/AimResult` | `header`, `follow`, `fire`, `pitch`, `yaw` | 外部 aim 的角度接管、最终角度与开火门控。 |
| `auto_aim_common/msg/Target` | `header`, `status`, `buff_follow`, `yaw`, `pitch` | predictor/buff/outpost 角度目标。 |
| `auto_aim_common/msg/RelativeTarget` | `header`, `valid`, `x`, `y`, `z`, `distance_m`, `yaw_error_deg`, `pitch_error_deg`, `armor_type`, `aim_mode` | 追击相对目标。 |
| `auto_aim_common/msg/Armors` | `header`, `Armor[] armors`, `Car[] cars`, `yaw`, `pitch`, predictor target index | 检测输出给跟踪/预测。 |

## 7. Maintenance Rules

- 改 `src/behavior_tree/include/Topic.hpp` 或 `src/gimbal_driver/main.cpp` 的 topic 名/类型时，同步更新本文档。
- 改 `gimbal_driver/msg/*.msg` 或 `auto_aim_common/msg/*.msg` 的字段时，同步更新本文档第 6 节。
- 外部导航 topic 行为变更时，同步更新 `docs/sentry/external/navi_status_topics.md`。
- 下位机串口帧、TypeID、`SentryCmd` 位定义变更时，同步更新 `docs/sentry/embedded/serial_data_mapping.md`。
