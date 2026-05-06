# 裁判系统串口对接清单（给下位机）

Updated: 2026-05-06

本文只写下位机需要和裁判系统串口对接的项目，以及这些项目如何映射到本仓库上下位机串口。

## 1. 总链路

```text
裁判系统串口
  -> 下位机解析裁判 cmd_id
  -> 下位机打包 TypeID 7/8 等上发给上位机
  -> gimbal_driver 发布 ROS 语义 topic
  -> behavior_tree 决策
  -> /ly/control/posture（姿态 SentryCmd）或 /ly/control/sentry_cmd（完整哨兵裁判命令）
  -> gimbal_driver 主控制幀 SentryCmd
  -> 下位机封装裁判 0x0301 / 0x0120
  -> 裁判系统串口
```

## 2. 下位机 -> 上位机：裁判状态读取

| 裁判 cmd_id | 裁判结构 | 下位机上发 TypeID | ROS topic | 说明 |
|---|---|---|---|---|
| `0x0207` | `shoot_data_t` | TypeID 7/8 | `/ly/game/bullet` | 初速度放 TypeID 7，弹丸类型/发射机构/射频放 TypeID 8 |
| `0x0208` | `projectile_allowance_t` | TypeID 8 | `/ly/game/bullet` | 17mm、42mm、剩余金币、堡垒储备 17mm |
| `0x0209` | `rfid_status_t` | TypeID 4/8 | `/ly/game/rfid` | 低 32 bit 仍走 TypeID 4，`rfid_status_2` 走 TypeID 8 |
| `0x020D` | `sentry_info_t` | TypeID 7 | `/ly/game/sentry/info` | 兑换成功次数、脱战、复活、姿态、能量机关可激活 |

TypeID 7/8 的具体 12B 布局见：

- `docs/record/2026-05-06_referee_uplink_typeid7_8.md`
- `docs/sentry/embedded/serial_data_mapping.md`

## 3. 上位机 -> 下位机：主控制幀

下位机需要把上位机主控制幀解析为 17B：

| byte offset | 字段 | 类型 | 下位机用途 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `Velocity.X` | `int8` | 底盘 x 速度 |
| 2 | `Velocity.Y` | `int8` | 底盘 y 速度 |
| 3-6 | `Yaw` | `float32` | 云台 yaw |
| 7-10 | `Pitch` | `float32` | 云台 pitch |
| 11 | `FireCode` | `uint8` | 开火、电容、FollowMode、AimMode、小陀螺 |
| 12-15 | `SentryCmd` | `uint32` | 裁判 `0x0120 sentry_cmd` 命令字 |
| 16 | `Tail` | `uint8` | 固定 `0x00` |

`SentryCmd` 使用 little-endian。下位机不要再按旧 14B 主幀从 byte `12` 读取单独 `posture`。

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
| 21-22 | `posture` | `1` 进攻，`2` 防御，`3` 移动 | 直接映射姿态 |
| 23 | `confirm_energy_activate` | 确认能量机关进入正在激活状态 | 写入 `sentry_cmd bit23` |
| 24-31 | reserved | 保留 | 填 0 |

## 5. 下位机 shadow 建议

下位机建议维护两个 shadow：

| shadow | 来源 | 用途 |
|---|---|---|
| `upper_sentry_cmd_shadow` | 上位机主控制幀 byte `12-15` | 保存上位机最近一次请求 |
| `referee_sentry_cmd_shadow` | 下位机本地裁判发送模块 | 真正封装进裁判 `0x0301/0x0120` |

如果下位机没有本地人工命令，`referee_sentry_cmd_shadow = upper_sentry_cmd_shadow` 即可。
如果下位机有本地保护或人工命令，需要明确优先级，不能因为姿态更新清掉兑换、复活或能量机关确认位。

确认类 bit（`bit0`、`bit1`、`bit23`）不建议永久保持为 1。后续联调时需要约定：

- 上位机保持几帧
- 下位机发送裁判帧后是否自动清零
- 裁判上行状态变化后是否清零

## 6. 当前上位机实际会发什么

当前 BT 姿态主链路使用 `/ly/control/posture`，消息类型为 `gimbal_driver/msg/SentryCmd`：

```text
field_mask = FIELD_POSTURE
posture = 1/2/3
```

`gimbal_driver` 会只取 posture 字段并转写到 `SentryCmd bit21-22`。`/ly/control/sentry_cmd`
保留为完整 `SentryCmd` 命令入口，用于复活、兑弹、远程回血、能量机关确认等字段。

也就是说，现阶段实车最先需要验证的是：

1. 下位机能解析 17B 主控制幀。
2. 下位机能从 `SentryCmd bit21-22` 取姿态。
3. 下位机能把姿态写进裁判 `0x0120 bit21-22`。
4. 下位机能通过 TypeID 6 回传实际姿态到 `/ly/gimbal/posture`，并通过 TypeID 7 透传 `0x020D sentry_info_2.posture`；TypeID 7 有效姿态会覆盖 `/ly/gimbal/posture`。

复活、兑弹、远程回血、能量机关确认的上位机接口已经预留，但 BT 自动策略还没有开始主动下发这些字段。
