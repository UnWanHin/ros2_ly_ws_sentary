# 裁判系统串口对接清单（给下位机）

Updated: 2026-07-11

本文只写下位机需要和裁判系统串口对接的项目，以及这些项目如何映射到本仓库上下位机串口。

## 1. 总链路

```text
裁判系统串口
  -> 下位机解析裁判 cmd_id
  -> 下位机打包 TypeID 7/8/9 等上发给上位机
  -> gimbal_driver 发布 ROS 语义 topic
  -> behavior_tree 决策
  -> /ly/control/posture（姿态 SentryCmd）或 /ly/control/sentry_cmd（完整哨兵裁判命令）
  -> gimbal_driver `0x01 SentryCommandFrame`
  -> 下位机封装裁判 0x0301 / 0x0120
  -> 裁判系统串口
```

## 2. 下位机 -> 上位机：裁判状态读取

| 裁判 cmd_id | 裁判结构 | 下位机上发 TypeID | ROS topic | 说明 |
|---|---|---|---|---|
| `0x0207` | `shoot_data_t` | TypeID 7/8 | `/ly/game/bullet` | 初速度放 TypeID 7，弹丸类型/发射机构/射频放 TypeID 8 |
| `0x0003` | `game_robot_HP_t` | TypeID 6/10 | `/ly/game/damage_difference`, `/ly/friend/op_hp`, `/ly/enemy/op_hp` | TypeID 6 放 offset 8 `damage_difference`；TypeID 10 byte8~9 放 offset 12 `ally_outpost_HP`，byte10~11 放 offset 16 `enemy_outpost_HP` |
| `0x0208` | `projectile_allowance_t` | TypeID 8 | `/ly/game/bullet` | 17mm、42mm、剩余金币、堡垒储备 17mm |
| `0x0209` | `rfid_status_t` | TypeID 4/8 | `/ly/game/rfid` | 低 32 bit 仍走 TypeID 4，`rfid_status_2` 走 TypeID 8 |
| `0x020D` | `sentry_info_t` | TypeID 7/10 | `/ly/game/sentry/info` | `sentry_info/sentry_info_2` 放 TypeID 7；`sentry_info_3` 放 TypeID 10 byte0~7 |
| `0x0303` | `map_command_t` | TypeID 9 | `/ly/game/map_command` | 云台手/操作手小地图坐标、目标机器人 ID、按键值；重复包由消费端去重 |

TypeID 7/8/9/10 的具体 12B 布局见：

- `docs/record/2026-05-06_referee_uplink_typeid7_8.md`
- `docs/sentry/embedded/serial_data_mapping.md`
- `docs/sentry/embedded/map_command_typeid9.md`

## 3. 上位机 -> 下位机：下行分包

下位机必须先读取 byte `1` 的 `DownlinkTypeID`，再按对应长度解析：

| DownlinkTypeID | frame | 总长度 | 用途 |
|---|---|---:|---|
| `0x00` | `GimbalControlFrame` | 13B | 速度、云台角、FireCode |
| `0x01` | `SentryCommandFrame` | 6B | 裁判 `0x0120 sentry_cmd` |
| `0x02` | `MapPathFragmentFrame` | 64B x2 | CRC/sequence 重组后裁判 `0x0307 map_data_t` |
| `0x03` | `CustomInfoFrame` | 36B | 裁判 `0x0308 custom_info_t` |
| `0x04` | `SentryCoordinateFrame` | 17B | 哨兵自身坐标 |

`0x00 GimbalControlFrame` 的布局为 13B：

| byte offset | 字段 | 类型 | 下位机用途 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x00` |
| 2 | `Velocity.X` | `int8` | 底盘 x 速度 |
| 3 | `Velocity.Y` | `int8` | 底盘 y 速度 |
| 4-7 | `Yaw` | `float32` | 云台 yaw |
| 8-11 | `Pitch` | `float32` | 云台 pitch |
| 12 | `FireCode` | `uint8` | 开火、电容、FollowMode、AimMode、小陀螺 |

`0x01 SentryCommandFrame` 为 `[0x21, 0x01, SentryCmd uint32 little-endian]`，总长度 6B。
`SentryCmd` 已从 `0x00` 主控制包拆出；下位机不得再从 `0x00` 尾部读取命令字或 posture。

## 4. 下位机 -> 裁判系统：`0x0301 / 0x0120`

上位机下发的 `SentryCmd` 对应裁判 `0x0301 + data_cmd_id=0x0120 sentry_cmd`。
下位机需要按裁判系统协议封装完整帧，包括裁判帧头、`cmd_id=0x0301`、交互数据头和 CRC。

`sentry_cmd` 32-bit 字段如下：

| bit | 字段 | 意义 | 下位机处理 |
|---|---|---|---|
| 0 | `confirm_free_revive` | 确认免费复活 | 写入 `sentry_cmd bit0` |
| 1 | `confirm_immediate_revive` | 确认兑换立即复活 | 写入 `sentry_cmd bit1` |
| 2-12 | `exchange_projectile_allowance` | 非远程兑换允许发弹量累计值 | 必须单调递增 |
| 13-16 | `remote_projectile_exchange_count` | 远程兑换发弹量请求次数 | 每次请求只加 1 |
| 17-20 | `remote_hp_exchange_count` | 远程兑换血量请求次数 | 每次请求只加 1 |
| 21-23 | `posture` | `1` 进攻，`2` 防御，`3` 移动，`4` 强化进攻，`5` 强化防御，`6` 强化移动 | 直接映射姿态 |
| 24 | `confirm_energy_activate` | 确认能量机关进入正在激活状态 | 写入 `sentry_cmd bit24` |
| 25-31 | reserved | 保留 | 填 0 |

## 5. 下位机 shadow 建议

下位机建议维护两个 shadow：

| shadow | 来源 | 用途 |
|---|---|---|
| `upper_sentry_cmd_shadow` | 上位机 `0x01 SentryCommandFrame` byte `2-5` | 保存上位机最近一次请求 |
| `referee_sentry_cmd_shadow` | 下位机本地裁判发送模块 | 真正封装进裁判 `0x0301/0x0120` |

如果下位机没有本地人工命令，`referee_sentry_cmd_shadow = upper_sentry_cmd_shadow` 即可。
如果下位机有本地保护或人工命令，需要明确优先级，不能因为姿态更新清掉兑换、复活或能量机关确认位。

确认类 bit（`bit0`、`bit1`、`bit24`）不建议永久保持为 1。后续联调时需要约定：

- 上位机保持几帧
- 下位机发送裁判帧后是否自动清零
- 裁判上行状态变化后是否清零

## 6. 当前上位机实际会发什么

当前 BT 姿态主链路使用 `/ly/control/posture`，消息类型为 `gimbal_driver/msg/SentryCmd`：

```text
field_mask = FIELD_POSTURE
posture = 1/2/3
```

`gimbal_driver` 会只取 posture 字段并转写到 `SentryCmd bit21-23`。`/ly/control/sentry_cmd`
保留为完整 `SentryCmd` 命令入口，用于复活、兑弹、远程回血、能量机关确认等字段。

也就是说，现阶段实车最先需要验证的是：

1. 下位机按 byte1 分支解析下行 frame：`0x00=13B`、`0x01=6B`、`0x02=64B fragment x2`、`0x03=36B`、`0x04=17B`、`0x05=26B`；所有实际串口写入不超过 64B。
2. 下位机将 `0x01 SentryCommandFrame` 的 4B 命令字封装为裁判 `0x0301 + 0x0120`；姿态读取 bit21-23（`1~6`），能量机关确认读取 bit24。
3. 下位机只在同一 sequence 的两段 `0x02 MapPathFragmentFrame` CRC 都正确后重组 107B / 50 点 `map_data_t` 并封装裁判 `0x0307`；`0x03 CustomInfoFrame` 直接封装为 `0x0308 custom_info_t`。
4. 下位机通过 TypeID 7 透传 `0x020D sentry_info_2.posture`；有效姿态会发布到 `/ly/gimbal/posture`。TypeID 6 原姿态兼容字段已改为 `0x0003 damage_difference`。
5. 下位机通过 TypeID 10 透传 `0x020D sentry_info_3`，并按官方 `0x0003` 顺序打包前哨血量：byte8~9 己方，byte10~11 敌方。上位机 `/ly/friend/op_hp`、`/ly/enemy/op_hp` 会优先使用这两个精确值，旧 `GameCode * 25` 只做 fallback。若下位机暂时拿不到 `0x0003` 前哨血量，应暂停发 TypeID 10，而不是用 `0` 或默认值占位，因为 `0` 表示前哨已毁。

复活、兑弹、远程回血的上位机接口已经预留。能量机关确认已由 BT 打符链路主动使用：到达 `BuffOutpost`、`buff_hitter` 锁定且 `/ly/game/sentry/info.can_activate_energy_mechanism=true` 时，BT 会向 `/ly/control/sentry_cmd` 下发 `FIELD_CONFIRM_ENERGY_ACTIVATE` 脉冲。
