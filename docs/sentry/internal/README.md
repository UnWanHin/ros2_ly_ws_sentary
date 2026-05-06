# Internal Docs

Updated: 2026-05-06

这里放本仓内部 ROS 链路、BT trace、离线工具和维护接口说明。

## 文件

| 文件 | 内容 |
|---|---|
| `ros2_topic_structure.md` | 当前 ROS2 topic 分层、内外部边界、关键消息结构 |
| `ros2_topic_tree.md` | Tree 方式整理当前 ROS2 topic、msg 包结构、RFID 40 bit 和上下行 TypeID |
| `decision_visualization.md` | 决策 trace 与离线 pygame viewer 维护说明 |

## 维护重点

- 只属于本仓内部的数据流、trace schema、调试/可视化工具写在这里。
- 外部导航接口放到 `../external/`。
- 电控/下位机串口接口放到 `../embedded/`。
