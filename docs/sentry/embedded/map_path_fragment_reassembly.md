# 0x02 地图路径分片重组与裁判系统发送（下位机对接）

Updated: 2026-07-18

本文是下位机固件实现 `DownlinkTypeID=0x02` 地图路径的交接规范。它只描述上位机到
下位机的两段 64B 串口帧，以及下位机重组成一份裁判系统 `0x0307 map_data_t` 后的发送。
上位机 ROS 链路、路径坐标转换和字段来源见
[downlink_control_frame.md](downlink_control_frame.md)。

## 1. 必须同步切换的协议

旧实现若把 `0x02` 当作一次 107B 串口读取，必须整体替换为本文件的两个固定 64B
fragment 接收器。不要让旧 107B parser 和新 parser 同时消费同一个 `0x02` 字节流，否则会
失去帧边界，不能保证裁判系统收到完整路径。

上位机仍生成原有的完整 50 点逻辑路径，**没有删减为 25 点，也没有拆成两条裁判路径**。
下位机只负责将它从两次物理串口写入中复原，随后向裁判系统发送一次完整的 `0x0307`。

```text
/Path_downsampled
  -> map_path_to_game_path_node
  -> /ly/game/path (完整 50 点 MapPath)
  -> gimbal_driver
  -> 0x02 fragment 0 (64B) + fragment 1 (64B)
  -> 下位机完整重组 105B map_data_t payload
  -> 裁判系统 0x0307 map_data_t (仅一次)
```

`gimbal_driver` 对同一条路径连续写出 fragment 0、fragment 1，不插入 sleep、重试或其他
`0x02` frame；每条新路径使用递增的 8-bit `Sequence`。

## 2. 固定 64B 物理帧

每次从上位机串口收到的 `0x02` 都恰好为 64B。整数采用 little-endian。

串口是字节流时，下位机接收器应先以 `0x21` 和 byte 1 识别下行 frame，再按 ID 选择固定长度。
本次固件改动的关键是：识别到 `0x21, 0x02` 后只累计 **64B** 即交给本协议校验，不能继续等待旧协议的
107B。候选 frame 校验失败时，丢弃该候选并从接收缓冲区重新寻找下一个完整 `0x21 + 已知 ID` frame；
不得把失败候选的 payload 当作地图路径使用。

| byte | 字段 | 固定值或范围 | 下位机处理 |
|---:|---|---|---|
| 0 | `HeadFlag` | `0x21` (`'!'`) | 不符则丢弃 |
| 1 | `DownlinkTypeID` | `0x02` | 不符则交给其他下行协议 |
| 2 | `Sequence` | `uint8_t` | 两段必须相同 |
| 3 | `FragmentIndex` | `0` 或 `1` | `0` 是首段，`1` 是尾段 |
| 4 | `FragmentCount` | 固定 `2` | 不符则丢弃 |
| 5 | `PayloadLength` | index 0 为 `56`；index 1 为 `49` | 不符则丢弃 |
| 6-61 | `Payload[56]` | 有效负载 | index 1 的最后 7B 必须忽略 |
| 62-63 | `CRC16` | little-endian `uint16_t` | 校验 byte 0-61 |

CRC16 算法为 reflected `0x1021`：初值 `0xFFFF`，每 bit 右移，最低位为 1 时异或
`0x8408`，无最终 xor。测试向量 ASCII `123456789` 的结果必须是 `0x6F91`。

## 3. 两段如何区分和拼接

这是**按字节切片**，不是按路径点数切片。

| fragment | `FragmentIndex` | 有效字节 | 写入重组 buffer 的范围 | 内容 |
|---|---:|---:|---|---|
| 首段 | `0` | 56 | `payload[0..55]` | `Intention`、起点 X/Y、全部 `DeltaX[49]`、`DeltaY[0..1]` |
| 尾段 | `1` | 49 | `payload[56..104]` | `DeltaY[2..48]`、`SenderId` |

重组 buffer 必须是 **105B**。仅当同一 `Sequence` 的首段和尾段均已通过全部校验，才允许
解析这 105B。禁止把 fragment 1 的 7B 填零尾部当成逻辑 payload 的一部分。

重组完成后的 105B 裁判 `map_data_t` payload 布局：

| payload byte | 字段 | 类型 / 数量 |
|---|---|---|
| 0 | `intention` | `uint8_t`，仅 `1/2/3` 有效 |
| 1-2 | `start_position_x` | `uint16_t` little-endian，dm |
| 3-4 | `start_position_y` | `uint16_t` little-endian，dm |
| 5-53 | `delta_x` | `int8_t[49]`，dm |
| 54-102 | `delta_y` | `int8_t[49]`，dm |
| 103-104 | `sender_id` | `uint16_t` little-endian |

`sender_id` 是机器人 ID，不是选手端 ID：红方哨兵为 `7`，蓝方哨兵为 `107`。

## 4. 下位机接收状态机

建议每个串口接收任务只维护一份 pending 路径；新首段天然覆盖旧路径，因为上位机的路径以
最新值为准。

```c
typedef struct {
    bool active;
    uint8_t sequence;
    bool has_fragment[2];
    uint8_t payload[105];
    uint32_t started_ms;
} map_path_pending_t;
```

按以下顺序处理每个完整 64B `0x02` frame：

1. 验证 `HeadFlag`、`DownlinkTypeID`、`FragmentCount`、`FragmentIndex`、`PayloadLength` 和 CRC16。
   任一失败，清空 pending buffer，且绝不向裁判系统发送数据。
2. `FragmentIndex=0`：清空旧 pending buffer，以该 `Sequence` 新建 pending，复制 56B 到
   `payload[0..55]`，标记首段已收到。
3. `FragmentIndex=1`：只有在 pending 存在、首段已收到且 `Sequence` 一致时才接受；复制 49B
   到 `payload[56..104]`，标记尾段已收到。没有首段、sequence 不同或重复尾段时清空 pending。
4. 两段都存在时，验证 `payload[0]` 是 `1/2/3`，一次性构造裁判 `map_data_t`，发送**一份**
   `cmd_id=0x0307`，然后清空 pending buffer。
5. pending 超时必须清空，防止旧首段和很久以后的尾段拼接。上位机连续发两段，因此可使用
   短超时；建议从首段开始计时 `100 ms`，实际值可按下位机串口任务调度设定，但不得无限等待。

参考伪代码：

```c
void on_upper_map_path_frame(const uint8_t frame[64], uint32_t now_ms) {
    if (!map_path_frame_is_valid(frame)) {  // SOF/ID/count/index/length/CRC 全部验证
        clear_pending();
        return;
    }

    const uint8_t sequence = frame[2];
    const uint8_t index = frame[3];
    const uint8_t length = frame[5];

    if (pending.active && now_ms - pending.started_ms > 100U) {
        clear_pending();
    }

    if (index == 0U) {
        clear_pending();
        pending.active = true;
        pending.sequence = sequence;
        pending.started_ms = now_ms;
        memcpy(&pending.payload[0], &frame[6], 56U);
        pending.has_fragment[0] = true;
        return;
    }

    if (!pending.active || !pending.has_fragment[0] ||
        pending.sequence != sequence || pending.has_fragment[1]) {
        clear_pending();
        return;
    }

    // 此处 index 必为 1，且 map_path_frame_is_valid 已确保 length == 49。
    memcpy(&pending.payload[56], &frame[6], length);
    pending.has_fragment[1] = true;

    if (!map_path_payload_is_valid(pending.payload)) {  // intention 为 1/2/3
        clear_pending();
        return;
    }

    map_data_t map = decode_map_data(pending.payload);  // 按第 3 节逐字段 little-endian 解码
    referee_send_map_data_0307(&map);                   // 仅在这里发送一次
    clear_pending();
}
```

`map_path_frame_is_valid()` 对 index 0 必须要求 `PayloadLength==56`，对 index 1 必须要求
`PayloadLength==49`；不能只检查 `PayloadLength<=56`。`decode_map_data()` 应从明确的字节数组
解码，避免因 MCU 编译器 struct packing/对齐与协议不一致。

## 5. 裁判系统发送

重组成功后，将第 3 节的完整 105B 作为官方 `map_data_t` 内容，通过下位机已有的裁判串口发送器
封装为 `cmd_id=0x0307`。该发送器仍应负责官方协议的帧头、长度、序号、CRC8、cmd_id 与 CRC16；
上位机这层的 64B fragment CRC16 不能替代裁判系统的 CRC。

必须遵守以下边界：

- 每个完整且有效的相同 sequence 只调用一次 `referee_send_map_data_0307()`。
- 只有一段、CRC 错误、sequence 不匹配、超时或无效 intention 时，**不得**发送 `0x0307`，也不得
  用旧路径或半份路径补发。
- 收到后续 sequence 的 fragment 0 时，立刻淘汰未完成的旧 sequence；最新路径优先。
- 这两个 64B frame 仅用于上位机到下位机；裁判系统看到的仍是一份标准完整 `0x0307 map_data_t`，
  不会看到 fragment index 或 sequence。

## 6. 联调验收

1. 用 ASCII `123456789` 验证 CRC16 结果为 `0x6F91`。
2. 从 `/ly/download/typeid0x02` 或 gimbal_driver raw log 抓取同一 sequence 的两帧，确认每帧正好
   64B，index 顺序 `0`、`1`，长度分别 `56`、`49`。
3. 下位机打印重组后的 `intention`、起点、49 个 `delta_x`、49 个 `delta_y`、`sender_id`，与抓包
   的 105B payload 按第 3 节逐字节一致。
4. 只输入首段、篡改 CRC、发送不同 sequence 的尾段、等待超时后再输入尾段，均确认裁判系统没有
   `0x0307` 输出。
5. 输入正确的同 sequence 两段，确认裁判串口只输出一次标准 `0x0307`，并在选手端小地图验证完整
   50 点路径。

## 7. 相关文件

- 上位机 frame 定义与 CRC：`src/gimbal_driver/module/BasicTypes.hpp`
- 上位机两段连续写入：`src/gimbal_driver/main.cpp` 的 `SendMapPath()`
- 上位机下行总协议：[downlink_control_frame.md](downlink_control_frame.md)
- 裁判协议与上下位机总对接：[referee_serial_integration.md](referee_serial_integration.md)
