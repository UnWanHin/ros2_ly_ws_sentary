# Embedded Docs

Updated: 2026-05-06

这里放当前有效的电控/下位机串口协议和上下行数据映射。

## 文件

| 文件 | 内容 |
|---|---|
| `serial_data_mapping.md` | 当前上发/下发串口结构、TypeID、ROS topic、裁判协议映射 |
| `downlink_control_frame.md` | 给下位机对接的上位机下发主控制幀说明 |

## 维护重点

- `serial_data_mapping.md` 是上下位机通信总表，新增字段或裁判协议映射时优先更新它。
- 下位机只需要看下发主幀时，优先看 `downlink_control_frame.md`。
- 外部导航 topic 不放这里，放到 `../external/`。
