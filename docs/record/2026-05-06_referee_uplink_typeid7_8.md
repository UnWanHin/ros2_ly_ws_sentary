# Referee Uplink TypeID 7/8

Updated: 2026-05-06

本文给下位机对接：下位机从裁判系统串口读取 `0x0207`、`0x0208`、`0x0209`、`0x020D` 后，如何打包发给上位机 `gimbal_driver`。

## 1. 总原则

- 方向：下位机 -> 上位机。
- 上位机串口帧仍使用现有 `TypedMessage<12>`：

```cpp
struct TypedMessage12 {
    uint8_t head_flag; // '!' / 0x21
    uint8_t type_id;
    uint8_t data[12];
    uint8_t tail;      // 0x00
};
```

- `data` 必须是 12B，结构体必须 `#pragma pack(1)`。
- 多字节字段按当前上下位机约定直接拷贝，小端。
- `float` 为 IEEE754 float32，直接从裁判系统 `shoot_data.initial_speed` 拷贝。
- TypeID 7 的 `float` 位于 payload offset 6，TypeID 8 的 `uint16_t` 位于 payload offset 3 后；若 MCU 不保证非对齐访问安全，请用 byte buffer + `memcpy` 填充，不要直接对非对齐地址解引用。
- TypeID 7/8 只做上行状态，不包含任何下发命令。`0x0120 sentry_cmd` 仍是后续单独设计。

## 2. 下位机需要读取的裁判系统数据

| 裁判 cmd_id | 结构 | 用途 | 上位机 TypeID |
|---|---|---|---|
| `0x0207` | `shoot_data_t` | 弹丸类型、发射机构、射频、初速度 | TypeID 7 + TypeID 8 |
| `0x0208` | `projectile_allowance_t` | 17mm/42mm 允许发弹量、剩余金币、堡垒储备 17mm 允许发弹量 | TypeID 8 |
| `0x0209` | `rfid_status_t` | `rfid_status_2` 扩展 8 bit | TypeID 8 |
| `0x020D` | `sentry_info_t` | 哨兵脱战、兑换、复活、姿态、能量机关可激活等状态 | TypeID 7 |

`0x0209 rfid_status` 的低 32 bit 已经由现有 TypeID 4 上发，不要重复放进 TypeID 8。TypeID 8 只放 offset 4 的 `rfid_status_2`。

## 3. TypeID 7 - SentryData

下位机发送结构：

```c
#pragma pack(push, 1)
typedef struct {
    uint32_t sentry_info;          // 0x020D offset 0
    uint16_t sentry_info_2;        // 0x020D offset 4
    float    bullet_initial_speed; // 0x0207 offset 3
    uint16_t reserved;             // 先填 0
} sentry_data_t;                   // sizeof == 12
#pragma pack(pop)
```

上位机行为：

- `sentry_info/sentry_info_2` 发布到 `/ly/referee/sentry_info`，消息类型 `gimbal_driver/msg/SentryInfo`。
- `bullet_initial_speed` 合并发布到 `/ly/referee/bullet_info.initial_speed`，消息类型 `gimbal_driver/msg/BulletInfo`。
- 不覆盖 `/ly/gimbal/posture`。`0x020D` 的姿态只放在 `/ly/referee/sentry_info.posture`。

`sentry_info` 拆字段：

| 位 | 语义 | ROS 字段 |
|---|---|---|
| `0-10` | 除远程兑换外成功兑换的允许发弹量 | `exchanged_projectile_allowance` |
| `11-14` | 成功远程兑换允许发弹量次数 | `remote_projectile_exchange_count` |
| `15-18` | 成功远程兑换血量次数 | `remote_hp_exchange_count` |
| `19` | 当前可确认免费复活 | `can_confirm_free_revive` |
| `20` | 当前可兑换立即复活 | `can_exchange_immediate_revive` |
| `21-30` | 立即复活所需金币 | `immediate_revive_cost` |
| `31` | 保留 | `sentry_info_reserved` |

`sentry_info_2` 拆字段：

| 位 | 语义 | ROS 字段 |
|---|---|---|
| `0` | 当前是否处于脱战状态 | `out_of_combat` |
| `1-11` | 队伍 17mm 允许发弹量剩余可兑换数 | `remaining_exchangeable_17mm` |
| `12-13` | 当前姿态，1 进攻、2 防御、3 移动 | `posture` |
| `14` | 己方能量机关当前可进入正在激活状态 | `can_activate_energy_mechanism` |
| `15` | 保留 | `sentry_info_2_reserved` |

## 4. TypeID 8 - BulletDataAndRfid2

下位机发送结构：

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  bullet_type;                      // 0x0207 offset 0
    uint8_t  shooter_number;                   // 0x0207 offset 1
    uint8_t  launching_frequency;              // 0x0207 offset 2
    uint16_t projectile_allowance_17mm;        // 0x0208 offset 0
    uint16_t projectile_allowance_42mm;        // 0x0208 offset 2
    uint16_t remaining_gold_coin;              // 0x0208 offset 4
    uint16_t projectile_allowance_fortress;    // 0x0208 offset 6
    uint8_t  rfid_status_2;                    // 0x0209 offset 4
} bullet_data_and_rfid2_t;                     // sizeof == 12
#pragma pack(pop)
```

注意：`uint8_t[3]` 后接 `uint16_t`，如果没有 `pack(1)`，编译器可能补 padding，导致不是 12B。

上位机行为：

- 发布 `/ly/referee/bullet_info`，消息类型 `gimbal_driver/msg/BulletInfo`。
- `rfid_status_2` 合并到现有 `/ly/me/rfid`，消息类型 `gimbal_driver/msg/RfidStatus`。
- 如果 TypeID 8 先到、TypeID 4 的低 32 bit `rfid_status` 还没到，上位机会先缓存 `rfid_status_2`；等 TypeID 4 到达后再发布完整 `/ly/me/rfid`。

## 5. 0x0207 同步要求

`0x0207 shoot_data_t` 被拆到两帧：

- TypeID 7：`initial_speed`
- TypeID 8：`bullet_type`、`shooter_number`、`launching_frequency`

下位机应维护一个 `last_shoot_data` shadow。收到一次 `0x0207` 时，同步更新四个字段，之后 TypeID 7/8 都从同一份 shadow 取最近值。否则上位机可能看到：

```text
bullet_type 是这一发
initial_speed 是上一发
```

上位机 `BulletInfo` 有 validity 字段：

| 字段 | 含义 |
|---|---|
| `has_initial_speed` | 已收到 TypeID 7 的 `initial_speed` |
| `has_shoot_data` | 已收到 TypeID 8 的 `bullet_type/shooter_number/launching_frequency` |
| `has_projectile_allowance` | 已收到 TypeID 8 的 `0x0208` |
| `has_rfid_status_2` | 已收到 TypeID 8 的 `rfid_status_2` |

## 6. 上位机 ROS 对接

新增 topic：

| Topic | Msg | 来源 |
|---|---|---|
| `/ly/referee/sentry_info` | `gimbal_driver/msg/SentryInfo` | TypeID 7, `0x020D` |
| `/ly/referee/bullet_info` | `gimbal_driver/msg/BulletInfo` | TypeID 7/8, `0x0207 + 0x0208 + rfid_status_2` |

保留旧 topic，不覆盖：

| 旧 topic | 仍然来源 | 说明 |
|---|---|---|
| `/ly/gimbal/posture` | TypeID 6 `ChassisData.Posture` | 不用 `0x020D posture` 覆盖 |
| `/ly/bullet/speed` | TypeID 5 `PositionData.BulletSpeed / 100` | 不用 `0x0207 initial_speed` 覆盖 |
| `/ly/me/ammo_left` | TypeID 1 `GameData.AmmoLeft` | 不用 `0x0208 projectile_allowance_17mm` 覆盖 |
| `/ly/me/rfid` | TypeID 4 low32 + TypeID 8 status2 | TypeID 8 只补 `rfid_status_2` |

## 7. 最小联调检查

上位机启动 `gimbal_driver` 后检查：

```bash
ros2 topic echo /ly/referee/sentry_info
ros2 topic echo /ly/referee/bullet_info
ros2 topic echo /ly/me/rfid
```

预期：

- TypeID 7 到达后，`/ly/referee/sentry_info` 有 `out_of_combat`、`can_activate_energy_mechanism` 等语义字段。
- TypeID 7 到达后，`/ly/referee/bullet_info.has_initial_speed=true`。
- TypeID 8 到达后，`/ly/referee/bullet_info.has_shoot_data=true`、`has_projectile_allowance=true`、`has_rfid_status_2=true`。
- TypeID 4 和 TypeID 8 都到达后，`/ly/me/rfid.has_rfid_status_2=true`。
