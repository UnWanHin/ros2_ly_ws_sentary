# ROS2 Topic And Message Tree

Updated: 2026-06-06

这份文档用 tree 方式整理当前哨兵上位机 ROS2 topic 和消息结构，重点回答两个问题：

- `/ly/friend/*` 现在到底有多少 topic。
- RFID 不是 40 个 ROS topic，而是一个 `RfidStatus` 消息里承载裁判 `0x0209` 的 40 bit 状态。

源码核对入口：

- `src/behavior_tree/include/Topic.hpp`
- `src/gimbal_driver/main.cpp`
- `src/gimbal_driver/msg/*.msg`
- `src/auto_aim_common/msg/*.msg`
- `src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp`
- `src/navi_tf_bridge/src/pointer_solver_node.cpp`

边界标记：

- `[Internal]`：本仓内部 ROS 节点之间的数据流。
- `[External]`：外部导航、TF、相机驱动等非本仓决策模块提供或消费的数据流。
- `[Embedded]`：`gimbal_driver` 通过串口和下位机/裁判系统对接的数据流。

## Quick Answer: `/ly/friend`

当前 `/ly/friend/*` 共有 **9 个 topic**：

```text
/ly/friend
├── is_precaution  : std_msgs/msg/Bool              [Embedded] 英雄预警
├── is_at_home     : std_msgs/msg/Bool              [Embedded] 下位机/裁判回家状态
├── is_team_red    : std_msgs/msg/Bool              [Embedded] 我方是否红方
├── hp             : gimbal_driver/msg/Health       [Embedded] 我方各兵种血量
├── op_hp          : std_msgs/msg/UInt16            [Embedded] 我方前哨血量
├── base_hp        : std_msgs/msg/UInt16            [Embedded] 我方基地血量
├── ammo_left      : std_msgs/msg/UInt16            [Embedded] 当前弹量摘要
├── uwb_pos        : gimbal_driver/msg/StampedUInt16MultiArray [Embedded] 自身官方坐标 [x, y]，带 header.stamp
└── uwb_yaw        : std_msgs/msg/UInt16            [Embedded] UWB yaw 回读
```

`/ly/game/rfid` 是一个 topic，不是 40 个 topic。里面的 `RfidStatus` 由：

- TypeID 4 的 `RFIDStatus uint32` 提供低 32 bit。
- TypeID 8 的 `RfidStatus2 uint8` 提供额外 8 bit。

## Topic Tree

### `/ly/control` - 上位机下发控制入口

```text
/ly/control
├── angles     : gimbal_driver/msg/GimbalAngles     [Embedded] BT/FaceMode -> gimbal_driver -> 下位机
├── firecode   : gimbal_driver/msg/FireCode         [Embedded] BT -> gimbal_driver -> 下位机 FireCode 1B
├── vel        : gimbal_driver/msg/ControlVelocity  [Embedded] BT -> gimbal_driver -> 下位机速度 X/Y
├── posture    : gimbal_driver/msg/SentryCmd        [Embedded] BT 姿态主入口，只用 FIELD_POSTURE/posture
└── sentry_cmd : gimbal_driver/msg/SentryCmd        [Embedded] 完整 sentry_cmd 入口
```

### `/ly/gimbal` - 下位机/裁判回读状态

```text
/ly/gimbal
├── angles          : gimbal_driver/msg/GimbalAngles  [Embedded] 当前云台 yaw/pitch
├── firecode        : gimbal_driver/msg/FireCode      [Embedded] FireCode 回读
├── vel             : gimbal_driver/msg/Vel           [Embedded] 兼容速度回读
├── chassis         : gimbal_driver/msg/Chassis       [Embedded] 底盘舵角/角速度/速度
├── big_yaw_angles  : std_msgs/msg/Float32            [Embedded] 大 yaw/舵角回读
├── posture         : std_msgs/msg/UInt8              [Embedded] 下位机/裁判姿态回读
├── capV            : std_msgs/msg/UInt8              [Embedded] 电容状态/电压回读
└── eventdata       : std_msgs/msg/UInt32             [Embedded] legacy 原始 event data
```

注意：`gimbal_driver` 当前仍保留一个 legacy topic 名 `ly/gimbal/eventdata`，没有前导 `/`。`behavior_tree` 现在只订阅新版 `/ly/game/event_data`，不再从 legacy raw topic 更新 EventData。

### `/ly/log` - 可选 raw 调试 topic

```text
/ly/log
├── gimbal_raw_rx : gimbal_driver/msg/GimbalRawFrame  [Debug] 下位机 -> 上位机 raw TypeID 幀
└── gimbal_raw_tx : gimbal_driver/msg/GimbalRawFrame  [Debug] 上位机 -> 下位机 raw downlink frame
```

默认关闭，由 `config/common.yaml` 的 `gimbal_raw.topic.enable` 控制。`data` 是原始 bytes，不是 hex 字符串。

### `/ly/game`

```text
/ly/game
├── all         : gimbal_driver/msg/GameData    [Embedded] 比赛摘要 gamecode/ammo/time/selfhealth/event
├── is_start    : std_msgs/msg/Bool             [Embedded] 比赛开始标志
├── time_left   : std_msgs/msg/UInt16           [Embedded] 剩余时间
├── event_data  : gimbal_driver/msg/EventData   [Embedded] 裁判 0x0101 event_data 语义拆字段
├── rfid        : gimbal_driver/msg/RfidStatus  [Embedded] 裁判 0x0209 RFID bit 语义
├── sentry/info : gimbal_driver/msg/SentryInfo  [Embedded] 裁判 0x020D 哨兵状态
├── bullet      : gimbal_driver/msg/BulletInfo  [Embedded] 裁判发射/弹量/金币信息
└── map_command : gimbal_driver/msg/MapCommand  [Embedded] 裁判 0x0303 小地图命令；BT 当前只缓存
```

### `/ly/enemy`, `/ly/team`, `/ly/position`, `/ly/bullet`

```text
/ly/enemy
├── hp      : gimbal_driver/msg/Health  [Embedded] 敌方各兵种血量
├── op_hp   : std_msgs/msg/UInt16       [Embedded] 敌方前哨血量
└── base_hp : std_msgs/msg/UInt16       [Embedded] 敌方基地血量

/ly/team
└── buff : gimbal_driver/msg/BuffData   [Embedded] 我方增益与剩余能量

/ly/position
└── data : gimbal_driver/msg/PositionData  [Embedded] 官方坐标中一组友方/敌方机器人位置

/ly/bullet
└── speed : std_msgs/msg/Float32  [Embedded] 旧弹速 topic，来自 TypeID 5 BulletSpeed/100
```

### `/ly/vision`, `/ly/bt`, `/ly/detector`, `/ly/tracker`, `/ly/predictor`

```text
/ly/vision
└── mode : std_msgs/msg/UInt8  [Internal] BT -> detector/buff/outpost，0=disabled, 1=armor, 2=buff, 3=outpost

/ly/bt
├── target           : std_msgs/msg/UInt8              [Internal] BT -> detector/predictor，装甲板目标类型
└── sentry_position  : geometry_msgs/msg/PointStamped  [Embedded] BT -> gimbal_driver，融合后自身坐标，map frame，单位 m

/ly/detector
├── armors       : auto_aim_common/msg/Armors  [Internal] detector -> tracker/predictor
└── high_armors  : auto_aim_common/msg/Armors  [Internal] detector debug/高处装甲板输出

/ly/tracker
└── results : auto_aim_common/msg/Trackers  [Internal] tracker_solver -> predictor

/ly/predictor
├── target : auto_aim_common/msg/Target       [Internal] predictor -> BT，普通装甲板辅瞄角
├── debug  : auto_aim_common/msg/DebugFilter  [Internal] predictor debug
└── vis    : auto_aim_common/msg/PredictorVis [Internal] predictor 可视化
```

### `/ly/buff`, `/ly/outpost`, `/ly/face_mode`

```text
/ly/buff
├── target : auto_aim_common/msg/Target     [Internal] buff_hitter -> BT，打符角度/状态
└── debug  : auto_aim_common/msg/BuffDebug  [Internal] buff_hitter debug

/ly/outpost
├── armors : auto_aim_common/msg/Armors  [Internal] detector -> outpost_hitter
└── target : auto_aim_common/msg/Target  [Internal] outpost_hitter -> BT，前哨角度/状态

/ly/face_mode
├── target_raw : std_msgs/msg/UInt16MultiArray      [Internal] BT -> FaceMode solver，[official_map_x, official_map_y, map_z]
└── angles     : gimbal_driver/msg/GimbalAngles     [Internal] FaceMode solver -> BT/控制角

/ly/gimbal
└── facemode   : gimbal_driver/msg/FaceModeStatus   [Internal] FaceMode solver health/status，定位 target/gimbal/TF/solver 断点
```

当前 `pointer_solver_node` 也可以直接发布到 `/ly/control/angles`，并可选发布 `/ly/control/firecode`。正式 `sentry_all` 默认以 BT 模式启动 `map_aim_point_node`：等待 `/ly/face_mode/target_raw` 后，用 TF 相对几何输出 `/ly/face_mode/angles`，BT 激活 FaceMode 后再接管角度链路。`/ly/gimbal/facemode.function=true` 表示 solver 侧已具备 target、云台角、TF 和角度输出；它不等同于 BT 当前一定采用该角度，BT 是否采用仍由 FaceMode 请求状态和目标优先级决定。

### `/ly/navi` and `/goal_pose`

```text
/ly/navi
├── goal          : std_msgs/msg/UInt8              [External] BT -> 导航兼容点位 ID
├── goal_pos_raw  : std_msgs/msg/UInt16MultiArray   [External] BT -> navi_tf_bridge，官方地图 [x_cm, y_cm]
├── goal_pos      : std_msgs/msg/UInt16MultiArray   [External] 已处理导航点兼容输出
├── target_rel    : auto_aim_common/msg/RelativeTarget [External] BT -> navi_tf_bridge，相机系/目标相对点
├── target_map    : geometry_msgs/msg/PointStamped  [External] navi_tf_bridge debug，目标转换后的 map/odom 点
├── position      : gimbal_driver/msg/StampedUInt16MultiArray [External] navi_tf_bridge -> BT，自身官方坐标 data=[x, y]，另带 map_point/map_frame/source_frame
├── speed_level   : std_msgs/msg/UInt8              [External] BT -> 导航速度档
├── lower_head    : std_msgs/msg/UInt8              [External] 导航兼容低头状态
├── reached       : std_msgs/msg/Bool               [External] 导航 -> BT，当前 goal 是否到达
├── reachable     : std_msgs/msg/Bool               [External] 导航 -> BT，当前 goal 是否可达
├── should_rotate : std_msgs/msg/Bool               [External] 导航 -> BT，true 恢复正常巡逻，false 停小陀螺并请求 FollowMode
└── vel           : gimbal_driver/msg/Vel           [External] 兼容/调试速度链路

/goal_pose : geometry_msgs/msg/PoseStamped  [External] navi_tf_bridge -> 外部导航最终目标
```

追击多源链路：

```text
/ly/position/data
└── BT 匹配敌方官方坐标和自身官方坐标
    └── /ly/navi/goal_pos_raw
        └── navi_tf_bridge 读 tf_config.yaml 4x4
            └── /goal_pose

/ly/detector/armors -> /ly/tracker/results -> /ly/predictor/target
└── BT 构造 /ly/navi/target_rel
    └── navi_tf_bridge 查 TF
        └── /goal_pose
```

### `/ly/ra`, camera, image, TF

```text
/ly/ra
├── mode        : std_msgs/msg/UInt8              [Internal] buff_hitter 内部打符模式
├── image       : sensor_msgs/msg/Image           [Internal] buff_hitter debug image
└── angle_image : auto_aim_common/msg/AngleImage  [Internal] detector/buff debug angle image

/ly/compressed
└── image : sensor_msgs/msg/CompressedImage  [Internal] detector/buff 压缩图像调试

/ly/back_cam
└── target : auto_aim_common/msg/Target  [Internal] 后置相机目标结果，BT 消费

/ly/camera
└── image : sensor_msgs/msg/Image  [Internal] detector 中有 topic 常量，当前默认相机输出不走这里

/ly/backcamera
└── image : sensor_msgs/msg/Image  [Internal] detector 中有 topic 常量，当前未作为主链路使用

/camera_front
├── image_raw   : sensor_msgs/msg/Image       [External] detector 默认相机图像输出
└── camera_info : sensor_msgs/msg/CameraInfo  [External] detector 默认相机参数输出

/tf        : tf2_msgs/msg/TFMessage  [External] TF 动态变换
/tf_static : tf2_msgs/msg/TFMessage  [External] TF 静态变换
```

## Message Tree

### `gimbal_driver/msg`

```text
gimbal_driver/msg
├── GimbalAngles
│   ├── float32 yaw
│   ├── float32 pitch
│   └── std_msgs/Header header
├── Vel
│   ├── std_msgs/Header header
│   ├── float32 x
│   └── float32 y
├── UWBPos
│   ├── int16 x
│   └── int16 y
├── GimbalYaw
│   ├── int16 yaw_vel
│   ├── int16 yaw_angle
│   ├── float32 yaw_vel_deg_s
│   └── float32 yaw_angle_deg
├── FireCode
│   ├── constants FIELD_FIRE_STATUS/FIELD_CAP_STATE/FIELD_FOLLOW_MODE/FIELD_AIM_MODE/FIELD_ROTATE/FIELD_ALL
│   ├── uint8 field_mask
│   ├── uint8 fire_status
│   ├── uint8 cap_state
│   ├── bool follow_mode
│   ├── bool aim_mode
│   ├── uint8 rotate
│   └── uint8 raw
├── ControlVelocity
│   ├── std_msgs/Header header
│   ├── float32 x_mps
│   ├── float32 y_mps
│   ├── int8 raw_x
│   ├── int8 raw_y
│   └── bool use_raw
├── SentryCmd
│   ├── constants FIELD_CONFIRM_FREE_REVIVE/FIELD_CONFIRM_IMMEDIATE_REVIVE/FIELD_EXCHANGE_PROJECTILE_ALLOWANCE
│   ├── constants FIELD_REMOTE_PROJECTILE_EXCHANGE_COUNT/FIELD_REMOTE_HP_EXCHANGE_COUNT/FIELD_POSTURE/FIELD_CONFIRM_ENERGY_ACTIVATE/FIELD_ALL
│   ├── bool confirm_free_revive
│   ├── bool confirm_immediate_revive
│   ├── uint16 exchange_projectile_allowance
│   ├── uint8 remote_projectile_exchange_count
│   ├── uint8 remote_hp_exchange_count
│   ├── uint8 posture
│   ├── bool confirm_energy_activate
│   └── uint32 raw
├── GameData
│   ├── uint16 gamecode
│   ├── uint16 ammoleft
│   ├── uint16 timeleft
│   ├── uint16 selfhealth
│   └── uint32 exteventdata
├── EventData
│   ├── uint32 raw
│   ├── uint8 self_supply_status
│   ├── bool self_supply_occupied
│   ├── bool self_rmul_supply_occupied
│   ├── uint8 self_small_energy_status
│   ├── uint8 self_large_energy_status
│   ├── uint8 self_central_highland_status
│   ├── uint8 self_trapezoid_highland_status
│   ├── uint16 enemy_last_dart_hit_time
│   ├── uint8 enemy_last_dart_hit_target
│   ├── uint8 center_gain_point_status
│   ├── uint8 self_fortress_gain_point_status
│   ├── uint8 self_outpost_gain_point_status
│   └── bool self_base_gain_point_status
├── Health
│   ├── uint16 hero
│   ├── uint16 engineer
│   ├── uint16 infantry1
│   ├── uint16 infantry2
│   ├── uint16 reserve
│   └── uint16 sentry
├── BuffData
│   ├── uint8 recoverybuff/coolingbuff/defencebuff/vulnerabilitybuff
│   ├── uint16 attackbuff
│   └── uint8 remainingenergy
├── PositionData
│   ├── uint8 friendcarid
│   ├── int16 friendx
│   ├── int16 friendy
│   ├── uint8 enemycarid
│   ├── int16 enemyx
│   └── int16 enemyy
├── Chassis
│   ├── float32 steer_angle
│   ├── float32 angular_velocity
│   ├── float32 velocity_x
│   └── float32 velocity_y
├── SentryInfo
│   ├── uint32 sentry_info_raw
│   ├── uint16 sentry_info_2_raw
│   ├── uint16 exchanged_projectile_allowance
│   ├── uint8 remote_projectile_exchange_count
│   ├── uint8 remote_hp_exchange_count
│   ├── bool can_confirm_free_revive
│   ├── bool can_exchange_immediate_revive
│   ├── uint16 immediate_revive_cost
│   ├── bool out_of_combat
│   ├── uint16 remaining_exchangeable_17mm
│   ├── uint8 posture
│   └── bool can_activate_energy_mechanism
├── BulletInfo
│   ├── bool has_initial_speed
│   ├── float32 initial_speed
│   ├── bool has_shoot_data
│   ├── uint8 bullet_type/shooter_number/launching_frequency
│   ├── bool has_projectile_allowance
│   ├── uint16 projectile_allowance_17mm
│   ├── uint16 projectile_allowance_42mm
│   ├── uint16 remaining_gold_coin
│   └── uint16 projectile_allowance_fortress_17mm
└── RfidStatus
    └── see next section
```

### `RfidStatus` 40 bit tree

```text
gimbal_driver/msg/RfidStatus
├── raw : uint32  # low 32 bits from TypeID 4 / referee 0x0209 rfid_status
│   ├── bit00 friend_base
│   ├── bit01 friend_central
│   ├── bit02 enemy_central
│   ├── bit03 friend_highland
│   ├── bit04 enemy_highland
│   ├── bit05 friend_flyroad_front
│   ├── bit06 friend_flyroad_back
│   ├── bit07 enemy_flyroad_front
│   ├── bit08 enemy_flyroad_back
│   ├── bit09 friend_central_under
│   ├── bit10 friend_central_high
│   ├── bit11 enemy_central_under
│   ├── bit12 enemy_central_high
│   ├── bit13 friend_roadland_under
│   ├── bit14 friend_roadland_high
│   ├── bit15 enemy_roadland_under
│   ├── bit16 enemy_roadland_high
│   ├── bit17 friend_bastion
│   ├── bit18 friend_outpost
│   ├── bit19 friend_supply_noremix
│   ├── bit20 friend_supply_remix
│   ├── bit21 friend_armor
│   ├── bit22 enemy_armor
│   ├── bit23 central_rmul
│   ├── bit24 enemy_bastion
│   ├── bit25 enemy_outpost
│   ├── bit26 friend_tunnel_roadland_down
│   ├── bit27 friend_tunnel_roadland_mid
│   ├── bit28 friend_tunnel_roadland_up
│   ├── bit29 friend_tunnel_highland_low
│   ├── bit30 friend_tunnel_highland_mid
│   └── bit31 friend_tunnel_highland_high
└── rfid_status_2_raw : uint8  # high extra 8 bits from TypeID 8 / referee 0x0209 rfid_status_2
    ├── bit00 enemy_tunnel_roadland_down
    ├── bit01 enemy_tunnel_roadland_mid
    ├── bit02 enemy_tunnel_roadland_up
    ├── bit03 enemy_tunnel_highland_low
    ├── bit04 enemy_tunnel_highland_mid
    ├── bit05 enemy_tunnel_highland_high
    └── bit06-07 rfid_status_2_reserved
```

BT 内部会把 `/ly/game/rfid` 再聚合成 `RfidMatchState`，这是 blackboard 状态，不是新的 ROS topic：

```text
RfidMatchState
├── Fresh / Any / Raw / HasRfidStatus2 / RfidStatus2Raw
├── SelfSupply / SelfBaseGainPoint / SelfHighlandGainPoint / SelfRoadCrossing / SelfTunnel
├── EnemyHighlandGainPoint / EnemyRoadCrossing / EnemyTunnel
├── CenterGainPoint / SelfFortressGainPoint / EnemyFortressGainPoint
├── SelfOutpostGainPoint / EnemyOutpostGainPoint
├── SelfAssemblyGainPoint / EnemyAssemblyGainPoint
├── SelfFlyRamp / EnemyFlyRamp
└── OnSelfSideRfid / OnEnemySideRfid
```

对应 blackboard keys：

```text
RfidMatch
RfidFresh
RfidAny
RfidSelfSupply
RfidSelfBaseGainPoint
RfidSelfHighlandGainPoint
RfidEnemyHighlandGainPoint
RfidSelfRoadCrossing
RfidEnemyRoadCrossing
RfidSelfTunnel
RfidEnemyTunnel
RfidTunnel
RfidCenterGainPoint
RfidOnSelfSide
RfidOnEnemySide
```

### `auto_aim_common/msg`

```text
auto_aim_common/msg
├── Armor
│   ├── int32 type/color
│   ├── float32 distance
│   ├── float32 distance_to_image_center
│   ├── float32[] corners_x/corners_y
│   ├── float32[] rotation
│   └── float32[] translation
├── Armors
│   ├── std_msgs/Header header
│   ├── Armor[] armors
│   ├── Car[] cars
│   ├── float32 yaw/pitch
│   ├── bool is_available_armor_for_predictor
│   └── int16 target_armor_index_for_predictor
├── Car
│   ├── Rect bounding_rect
│   └── int32 car_id
├── Rect
│   ├── float32 x
│   ├── float32 y
│   ├── float32 width
│   └── float32 height
├── CarTracker
│   ├── int32 car_id
│   └── Rect bounding_rect
├── ArmorTracker
│   ├── float64 x/y/z
│   ├── float64 yaw
│   ├── int32 armor_id
│   └── int32 car_id
├── Trackers
│   ├── std_msgs/Header header
│   ├── CarTracker[] car_trackers
│   ├── ArmorTracker[] armor_trackers
│   └── float32 yaw/pitch
├── Target
│   ├── std_msgs/Header header
│   ├── bool status
│   ├── bool buff_follow
│   ├── float32 yaw
│   └── float32 pitch
├── RelativeTarget
│   ├── std_msgs/Header header
│   ├── bool valid
│   ├── float32 x/y/z
│   ├── float32 distance_m
│   ├── float32 yaw_error_deg
│   ├── float32 pitch_error_deg
│   ├── uint8 armor_type
│   └── uint8 aim_mode
├── DebugFilter
│   ├── bool tracking
│   ├── geometry_msgs/Point position
│   ├── float32 yaw
│   ├── geometry_msgs/Vector3 velocity
│   └── float32 v_yaw/radius_1/radius_2/z_2
├── PredictorVis
│   ├── bool has_predictions
│   ├── int32 aimed_car_id/aimed_armor_id
│   └── PredictorCarVis[] cars
├── PredictorCarVis
│   ├── int32 car_id
│   ├── bool stable
│   ├── geometry_msgs/Point center
│   └── PredictorArmorVis[] armors
├── PredictorArmorVis
│   ├── constants STATUS_NONEXIST/STATUS_UNSEEN/STATUS_AVAILABLE
│   ├── int32 id/status
│   ├── geometry_msgs/Point center
│   └── float32 yaw/theta
├── BuffDebug
│   ├── bool status
│   ├── uint8 mode
│   ├── float32 target_yaw/target_pitch
│   ├── float32 rotation_angle
│   └── float32 distance_m/height_m/target_x_m/target_y_m/target_z_m
└── AngleImage
    ├── sensor_msgs/Image image
    ├── float32 yaw
    └── float32 pitch
```

Standard messages used directly in topic contracts:

```text
std_msgs/msg/Bool
std_msgs/msg/UInt8
std_msgs/msg/UInt16
std_msgs/msg/UInt32
std_msgs/msg/Float32
std_msgs/msg/UInt16MultiArray
gimbal_driver/msg/StampedUInt16MultiArray
geometry_msgs/msg/PointStamped
geometry_msgs/msg/PoseStamped
sensor_msgs/msg/Image
sensor_msgs/msg/CompressedImage
sensor_msgs/msg/CameraInfo
tf2_msgs/msg/TFMessage
```

## Serial Packet Tree

### 上行：下位机 -> `gimbal_driver` -> ROS

当前上行统一是 `TypedMessage<12B payload>`：

```text
TypedMessage
├── HeadFlag : uint8  # '!'
├── TypeID   : uint8
├── Data     : 12B
└── Tail     : uint8
```

TypeID 分发：

```text
TypeID 0 GimbalData
├── GimbalAngles -> /ly/gimbal/angles
├── FireCode     -> /ly/gimbal/firecode
└── CapV         -> /ly/gimbal/capV

TypeID 1 GameData
├── GameCode     -> /ly/game/is_start, /ly/friend/is_team_red, /ly/friend/is_at_home, /ly/friend/is_precaution
├── AmmoLeft     -> /ly/friend/ammo_left
├── TimeLeft     -> /ly/game/time_left
├── SelfHealth   -> /ly/game/all.selfhealth
├── Outpost HP   -> /ly/friend/op_hp, /ly/enemy/op_hp
└── ExtEventData -> ly/gimbal/eventdata, /ly/game/event_data

TypeID 2 HealthMyselfData
├── Health fields -> /ly/friend/hp
└── BaseMyself    -> /ly/friend/base_hp

TypeID 3 HealthEnemyData
├── Health fields -> /ly/enemy/hp
└── BaseEnemy     -> /ly/enemy/base_hp

TypeID 4 RFIDAndBuffData
├── BuffStatus -> /ly/team/buff
└── RFIDStatus -> /ly/game/rfid.raw low32

TypeID 5 PositionData
├── Friend/Enemy position -> /ly/position/data
├── Friend CarId==7       -> /ly/friend/uwb_pos
└── BulletSpeed / 100     -> /ly/bullet/speed

TypeID 6 ChassisData
├── UWBAngleYaw           -> /ly/friend/uwb_yaw
├── ChassisPacked1/2      -> /ly/gimbal/chassis, /ly/gimbal/big_yaw_angles, /ly/gimbal/vel
└── valid Posture         -> /ly/gimbal/posture

TypeID 7 SentryData
├── SentryInfo/SentryInfo2 -> /ly/game/sentry/info
├── SentryInfo2.posture    -> /ly/gimbal/posture when valid
└── BulletInitialSpeed     -> /ly/game/bullet.initial_speed

TypeID 8 BulletDataAndRfid2
├── BulletType/Shooter/Frequency -> /ly/game/bullet
├── ProjectileAllowance/Gold     -> /ly/game/bullet
└── RfidStatus2                  -> /ly/game/rfid.rfid_status_2_raw
```

若 `gimbal_raw.topic.enable=true`，通过 `gimbal_raw.topic.type_ids` 过滤后的上行 raw 幀会同时发布到 `/ly/log/gimbal_raw_rx`。

坐标注意：

- `PositionType.X/Y` 在底层注释里写的是“乘了100”，当前 ROS `PositionData` 直接保留 `int16`。
- `behavior_tree` 消费 `/ly/position/data` 时会做 `Y = 1500 - raw_y` 的官方地图方向转换。
- `/ly/friend/uwb_pos` 和 `/ly/navi/position` 使用 `gimbal_driver/msg/StampedUInt16MultiArray`，`data=[x, y]`，`header.stamp` 是各自发布节点打的源时间戳。
- `behavior_tree` 通过 `AreaManager.SentryPositionFusion` 融合 `/ly/friend/uwb_pos`、`/ly/navi/position` 和 `/ly/position/data` 的 sentry friend slot，再写入 `friendRobots[Sentry].position_`；默认 priority 顺序是 UWB、Navi、PositionData。
- `/ly/navi/position` 是 `navi_tf_bridge` 从 TF 算出位置后再逆变换成官方地图 cm 的 `data=[x, y]`，给区域判断辅助用；同包的 `map_point` 保留 map 系 m 坐标，BT 当前不消费它。

### 下行：ROS -> `gimbal_driver` -> 下位机

当前下行是 17B `DownlinkTypeID` 分型 frame。上行 `TypeID` 和下行 `DownlinkTypeID` 是独立编号空间：

```text
GimbalControlFrame (DownlinkTypeID=0x00)
├── HeadFlag       : uint8   # '!'
├── DownlinkTypeID : uint8   # 0x00
├── Velocity.X     : int8    # /ly/control/vel
├── Velocity.Y     : int8    # /ly/control/vel
├── GimbalAngles   : 8B      # /ly/control/angles yaw/pitch float32
├── FireCode       : 1B      # /ly/control/firecode
└── SentryCmd      : 4B      # /ly/control/posture 或 /ly/control/sentry_cmd

SentryCoordinateFrame (DownlinkTypeID=0x01)
├── HeadFlag       : uint8   # '!'
├── DownlinkTypeID : uint8   # 0x01
├── X_cm           : int16   # /ly/bt/sentry_position x, m -> cm
├── Y_cm           : int16   # /ly/bt/sentry_position y, m -> cm
├── Reserved       : 10B     # 0
└── CRC8           : uint8   # byte0-15, poly=0x31 init=0xFF
```

若 `gimbal_raw.topic.enable=true` 且 `gimbal_raw.topic.downlink=true`，下行 raw frame 会同时发布到 `/ly/log/gimbal_raw_tx`，其中诊断 `type_id=255` 表示 control frame，`type_id=254` 表示 sentry coordinate frame。

`SentryCmd` bit tree：

```text
SentryCmd uint32
├── bit00     confirm_free_revive
├── bit01     confirm_immediate_revive
├── bit02-12  exchange_projectile_allowance
├── bit13-16  remote_projectile_exchange_count
├── bit17-20  remote_hp_exchange_count
├── bit21-22  posture  # 0=保留, 1=进攻, 2=防御, 3=移动
├── bit23     confirm_energy_activate
└── bit24-31  reserved
```

## Maintenance

- 新增或改名 topic：同步改 `src/behavior_tree/include/Topic.hpp`、对应节点源码和本文档。
- 改 `gimbal_driver/msg/*.msg` 或 `auto_aim_common/msg/*.msg` 字段：同步改本文 `Message Tree`。
- 改串口 TypeID 或下行主帧：同步更新 `docs/sentry/embedded/serial_data_mapping.md` 和 `docs/sentry/embedded/downlink_control_frame.md`。
- 改外部导航 topic：同步更新 `docs/sentry/external/navi_status_topics.md`。
