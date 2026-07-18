# Embedded Docs

Updated: 2026-07-18

这里放当前有效的电控/下位机串口协议和上下行数据映射。

## 文件

| 文件 | 内容 |
|---|---|
| `serial_data_mapping.md` | 当前上发/下发串口结构、TypeID、ROS topic、裁判协议映射 |
| `downlink_control_frame.md` | 给下位机对接的上位机下发主控制幀说明 |
| `map_path_fragment_reassembly.md` | `0x02` 两个 64B 路径分片的下位机重组、校验及裁判 `0x0307` 发送实现 |
| `referee_serial_integration.md` | 给下位机对接裁判系统串口 cmd_id 与本仓库 TypeID/SentryCmd 的清单 |
| `map_command_typeid9.md` | 给下位机对接 `0x0303 map_command_t` 和上发 `TypeID=9` 的说明 |

## 维护重点

- `serial_data_mapping.md` 是上下位机通信总表，新增字段或裁判协议映射时优先更新它。
- 下位机只需要看下发主幀时，优先看 `downlink_control_frame.md`。
- 下位机实现地图路径 `0x02` 的分片接收与裁判转发时，看 `map_path_fragment_reassembly.md`。
- 下位机要同时对齐裁判系统串口时，看 `referee_serial_integration.md`。
- 下位机要接云台手小地图坐标/目标机器人输入时，看 `map_command_typeid9.md`。
- 外部导航 topic 不放这里，放到 `../external/`。
