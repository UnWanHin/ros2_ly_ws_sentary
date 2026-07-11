# Regional Docs

Updated: 2026-07-08

这里放 Regional 模式当前有效的决策行为说明。

## 文件

| 文件 | 内容 |
|---|---|
| `current_behavior.md` | 当前 regional/BT 分层、任务触发、打断关系和运行行为 |
| `decision_framework.md` | Regional 大区域状态机框架和任务组织 |
| `patrol_scan_modes.md` | 雲台巡邏掃描 mode、Patrol.yaml 任務覆蓋和 pitch offset 鏈路 |
| `strategy_layers_and_navigation_reach.md` | Strategy 分层、`/ly/navi/reached`、坐标兜底和 watchdog 语义 |
| `zone_blocks.md` | 区域点位、区域块和底层区域任务记录 |
| `vision_mode_semantics.md` | `/ly/vision/mode`、task mode 等视觉/任务模式语义 |
| `vision_task_patrol_modes.md` | Vision、Task、Patrol 的当前流转关系 |
| `2026-07-12_regional_decision_graph.md` | Regional 每 tick 順序、分層優先級、導航閉環、姿態計時與下發的細節 Mermaid 圖 |

相关 debug 报告：

- `../../reports/2026-05-27_regional_reached_dataflow_debug.md`：regional reached 源不统一、bag 证据和 RCH issue 列表。

## 维护重点

- 改 Regional 的区域状态机、Default/Task/Tactical 分层、模式语义时，优先更新这里。
- 只属于 League 的差异写到 `../league/`。
- 只属于内部 trace 或工具维护的内容写到 `../internal/`。
