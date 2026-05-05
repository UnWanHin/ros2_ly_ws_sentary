# Regional Docs

Updated: 2026-05-06

这里放 Regional 模式当前有效的决策行为说明。

## 文件

| 文件 | 内容 |
|---|---|
| `current_behavior.md` | 当前 regional/BT 分层、任务触发、打断关系和运行行为 |
| `decision_framework.md` | Regional 大区域状态机框架和任务组织 |
| `zone_blocks.md` | 区域点位、区域块和底层区域任务记录 |
| `vision_mode_semantics.md` | `/ly/vision/mode`、task mode 等视觉/任务模式语义 |
| `vision_task_patrol_modes.md` | Vision、Task、Patrol 的当前流转关系 |

## 维护重点

- 改 Regional 的区域状态机、Default/Task/Tactical 分层、模式语义时，优先更新这里。
- 只属于 League 的差异写到 `../league/`。
- 只属于内部 trace 或工具维护的内容写到 `../internal/`。
