# TypeID 9 小地图命令对接（给下位机）

Updated: 2026-06-06

本文说明下位机如何从裁判系统串口读取 `0x0303 map_command_t`，并打包成
本仓库上下位机串口上行 `TypeID=9`，供 `gimbal_driver` 发布 `/ly/game/map_command`。

## 1. 下位机读取哪个串口

读取裁判系统常规链路，也就是：

```text
电源管理模块 User 串口 <-> 机器人下位机
```

通信协议 V1.3.0 的串口参数：

| 项目 | 值 |
|---|---|
| 物理接口 | 电源管理模块 User 串口 |
| 波特率 | `115200` |
| 数据位 | `8` |
| 停止位 | `1` |
| 校验 | 无 |
| 硬件流控 | 无 |

不要从图传链路或自定义客户端链路读取这个数据；`0x0303` 属于常规链路。

## 2. 裁判系统接收项

裁判系统完整帧仍按官方格式解析：

```text
frame_header 5B + cmd_id 2B + data N bytes + frame_tail CRC16 2B
```

只处理：

```text
cmd_id == 0x0303
```

官方说明：

```text
选手端点击 -> 服务器 -> 发送方选择的己方机器人
```

也就是说，云台手在选手端小地图触发后，服务器会通过常规链路把该帧发给被选择的己方机器人。

规则中的 50 金币消耗由裁判服务器结算，不是 `0x0303` 字段。下位机不要在 TypeID=9
中额外增加金币字段，也不要收到该消息后自行扣币。

## 3. 解析 `0x0303 map_command_t`

按通信协议 V1.3.0 `1.3.1 选手端下发数据` 的详细结构解析：

| data offset | 类型 | 字段 | 说明 |
|---:|---|---|---|
| `0` | `float` | `target_position_x` | 目标位置 x，单位 m；发送目标机器人 ID 时为 0 |
| `4` | `float` | `target_position_y` | 目标位置 y，单位 m；发送目标机器人 ID 时为 0 |
| `8` | `uint8_t` | `cmd_keyboard` | 云台手按键通用键值，无按键为 0 |
| `9` | `uint8_t` | `target_robot_id` | 坐标模式为 0；目标机器人模式为对方机器人 ID |
| `10` | `uint16_t` | `cmd_source` | 信息来源 ID |

下位机结构建议：

```c
#pragma pack(push, 1)
typedef struct {
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} map_command_t;
#pragma pack(pop)
```

`sizeof(map_command_t)` 应为 `12`。

注意：协议总表把 `0x0303` 数据段长度写为 `15`，但详细 `map_command_t` 字段合计为 `12B`。
下位机应按详细结构解析前 `12B`；如果实际 `data_length` 为 `15`，剩余 `3B` 先保留或记录日志，
不要当成坐标字段。

## 4. 重复包处理

`0x0303` 的发送机制会重复：

- 触发一次后，服务器以 `100ms` 间隔额外发送 4 次，共 5 次。
- 此后到下一次触发前，服务器以 `1Hz` 持续发送最近一次内容。

下位机可以原样上发每个 `0x0303` 包；如果下位机侧要直接触发动作，必须自行去重。
当前上位机只发布 `/ly/game/map_command`，BT 只缓存消息，不直接触发导航。

## 5. 上发给上位机的 TypeID=9

本仓库下位机 -> 上位机仍使用固定 15B 自定义上行帧：

| offset | 长度 | 字段 | 值 |
|---:|---:|---|---|
| `0` | `1` | `HeadFlag` | `'!'` / `0x21` |
| `1` | `1` | `TypeID` | `9` |
| `2` | `12` | `Data` | `MapCommandData` |
| `14` | `1` | `Tail` | `0x00` |

`Data` 的 12B 布局必须和 `map_command_t` 一致：

```c
#pragma pack(push, 1)
typedef struct {
    float target_position_x;   // offset 0
    float target_position_y;   // offset 4
    uint8_t cmd_keyboard;      // offset 8
    uint8_t target_robot_id;   // offset 9
    uint16_t cmd_source;       // offset 10
} upper_map_command_data_t;
#pragma pack(pop)
```

建议加静态检查：

```c
static_assert(sizeof(upper_map_command_data_t) == 12, "TypeID=9 payload must be 12B");
```

## 6. 上位机输出

上位机收到 `TypeID=9` 后发布：

```text
/ly/game/map_command
gimbal_driver/msg/MapCommand
```

ROS 字段：

| ROS 字段 | 来源 |
|---|---|
| `has_target_position` | `target_robot_id == 0` |
| `target_position_x_m` | `target_position_x` |
| `target_position_y_m` | `target_position_y` |
| `has_target_robot` | `target_robot_id != 0` |
| `target_robot_id` | `target_robot_id` |
| `cmd_keyboard` | `cmd_keyboard` |
| `cmd_source` | `cmd_source` |

本接口不改变 `/ly/control/posture`、`/ly/control/sentry_cmd`、`/ly/navi/reached`
或 `/ly/navi/reachable` 的语义。
