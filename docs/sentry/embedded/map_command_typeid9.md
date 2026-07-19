# TypeID 9 小地图命令对接（给下位机）

Updated: 2026-07-19

本文说明下位机如何从裁判系统串口读取 `0x0303 map_command_t`，并打包成
本仓库上下位机串口上行 `TypeID=9`，供 `gimbal_driver` 发布 `/ly/game/map_command`。

## 1. 下位机读取哪个串口

读取裁判系统常规链路，也就是：

```text
电源管理模块 User 串口 <-> 机器人下位机
```

RM2026 V2.0 的串口参数：

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

按 RM2026 V2.0 `0x0303 map_command_t` 的详细结构解析：

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

`map_command_t` 数据段为 `12B`。外层裁判帧的 `data_length` 以实际收到的官方帧为准。

官方 `0x0303` 与下位机上发的 `TypeID=9` 是两个不同的传输层：下位机先解析官方 float 米制
坐标，再转换成厘米定点数发送给上位机。不要把官方 `map_command_t` 的 12B 原样复制为
`TypeID=9`，否则上位机的定点解码会失效。

## 4. 重复包处理

`0x0303` 的发送机制会重复：

- 触发一次后，服务器以 `100ms` 间隔额外发送 4 次，共 5 次。
- 此后到下一次触发前，服务器以 `1Hz` 持续发送最近一次内容。

下位机可以原样上发每个 `0x0303` 包；如果下位机侧要直接触发动作，必须自行去重。
正式 BT 也会去重，不能依赖 1Hz 最新包来持续触发动作。

## 5. 上发给上位机的 TypeID=9

本仓库下位机 -> 上位机仍使用固定 15B 自定义上行帧：

| offset | 长度 | 字段 | 值 |
|---:|---:|---|---|
| `0` | `1` | `HeadFlag` | `'!'` / `0x21` |
| `1` | `1` | `TypeID` | `9` |
| `2` | `12` | `Data` | `MapCommandData` |
| `14` | `1` | `CRC8` | 前 14B 的 CRC8 |

`Data` 的前 8B 是下位机 CAN 命令的原样定点布局，后 4B 必须填 0：

```c
#pragma pack(push, 1)
typedef struct {
    int16_t target_position_x_100;  // offset 0, m * 100，little-endian
    int16_t target_position_y_100;  // offset 2, m * 100，little-endian
    uint8_t cmd_keyboard;           // offset 4
    uint8_t target_robot_id;        // offset 5
    uint16_t cmd_source;            // offset 6, little-endian
    uint32_t reserved;              // offset 8, 固定 0
} upper_map_command_data_t;
#pragma pack(pop)
```

建议加静态检查：

```c
static_assert(sizeof(upper_map_command_data_t) == 12, "TypeID=9 payload must be 12B");
```

例如 `21 09 87 03 36 02 00 00 06 01 00 00 00 00 82` 解码为
`x=903cm=9.03m`、`y=566cm=5.66m`、`cmd_source=0x0106=262`。其中 byte 14 是 CRC8，
不是固定尾字节。

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
| `target_position_x_m` | `target_position_x_100 / 100.0f` |
| `target_position_y_m` | `target_position_y_100 / 100.0f` |
| `has_target_robot` | `target_robot_id != 0` |
| `target_robot_id` | `target_robot_id` |
| `cmd_keyboard` | `cmd_keyboard` |
| `cmd_source` | `cmd_source` |

本接口不改变 `/ly/control/posture`、`/ly/control/sentry_cmd`、`/ly/navi/reached`
或 `/ly/navi/reachable` 的语义。

## 7. 正式 BT 坐标导航策略

`behavior_tree` 的 `Task.MapCommand` 消费本 topic：

```text
TypeID=9 -> /ly/game/map_command -> MapCommandTask
  -> /ly/navi/goal_pos_raw (official cm)
  -> navi_tf_bridge official_map -> map -> /goal_pose
```

完整数据链为：

```text
官方 0x0303 float(m)
  -> 下位机 int16 cm
  -> TypeID=9
  -> gimbal_driver /100.0f
  -> /ly/game/map_command float(m)
```

- 仅 `target_robot_id == 0` 的坐标模式可导航；目标机器人模式没有坐标，因此只保留为缓存信息。
- `(0,0)` 是下位机无命令默认值；接受范围为官方 `0..2800cm x 0..1500cm`。非有限值、负坐标、场地外坐标，或厘米取整后变成 `(0,0)` 的微小值一律忽略。
- 有效点击默认持有 `45s`，可由 `Task.MapCommand.HoldSec` 配置；首次点击立即发布，之后沿用 BT 的 2Hz 导航目标刷新率。
- 20cm 内的重复坐标不延长持有时间。到期后的相同 1Hz 重送也不会重新进入任务；协议没有 click sequence，只有坐标改变才能建立新任务。
- MapCommand 高于 Default、前哨、普通回防、Special 与 Chase；整个 `Hard` 层更高，包括 Recovery 和不可中断的 ReadyRoadland 穿越段。Recovery 每拍会吸收并取消当前坐标，因此恢复后不会自动回到被取消的点。
- 任意小地图点不是 AreaManager 的 BaseGoal。任务激活时会失效化旧 BaseGoal 的外部状态绑定，并暂停 `/ly/navi/reach_state` 发布，避免导航到小地图点时被误判为旧区域到点；它只按保持时间拥有导航输出。
