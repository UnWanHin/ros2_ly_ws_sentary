# External Interface Docs

Updated: 2026-05-06

这里放当前有效的外部包接口说明。外部包负责发布，本仓只订阅或消费的 topic 写在这里。

## 文件

| 文件 | 内容 |
|---|---|
| `navi_status_topics.md` | 导航侧 `/ly/navi/reached`、`/ly/navi/reachable` 等外部状态接口 |

## 维护重点

- 外部导航接口只说明本仓如何消费，不在这里描述导航包内部实现。
- 本仓内部发布/订阅链路放到 `../internal/`。
- 串口和电控接口放到 `../embedded/`。
