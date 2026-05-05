# 哨兵新姿态系统设计与使用说明

## 1. 目标与边界

本模块只负责**上位机决策层**姿态控制，目标是把姿态控制从“单次发值”升级为“可稳定运行的状态机”。

- 负责：
  - 姿态选择（进攻/防御/移动）
  - 5 秒切换冷却
  - 单姿态累计计时（3 分钟规则）
  - 防抖与防死锁（pending、超时、重试、降级）
- 不负责：
  - 串口协议细节
  - 裁判侧实际判定
  - 下位机执行闭环

---

## 2. 运行链路（上位机）

1. `UpdateGlobalData` 更新黑板（含 `IsFindTarget` 等）
2. `SelectAimMode / SelectStrategyMode / StrategyStack`
3. `PreprocessData / SelectAimTarget`
4. `SelectPosture`（新节点）
5. `PublishAll` 发布控制：
   - `/ly/control/angles`
   - `/ly/control/firecode`
   - `/ly/control/vel`
   - `/ly/control/posture`（新）

`/ly/gimbal/posture` 回读由订阅回调写入 `postureState`，供下一轮姿态状态机使用。

---

## 3. 文件分工

## 3.1 新增文件

- `src/behavior_tree/include/PostureTypes.hpp`
  - 姿态枚举、转换函数、运行时结构、决策输出结构。

- `src/behavior_tree/include/PostureManager.hpp`
- `src/behavior_tree/src/PostureManager.cpp`
  - 姿态状态机核心：
    - 冷却
    - 计时
    - pending / ack / retry
    - 防死锁
    - 3 分钟提前轮换

- `src/behavior_tree/src/PostureLogic.cpp`
  - 结合当前策略上下文生成 `desired posture`
  - 调用 `PostureManager::Tick` 更新 `postureCommand`

## 3.2 现有文件改动

- `include/Application.hpp`
  - 新增 `PostureManager` 成员与相关接口。
- `src/Configuration.cpp`
  - 新增 `Posture` 配置解析。
- `src/behavior_tree/include/BTNodes.hpp`
  - 新增 `SelectPostureNode`。
- `src/behavior_tree/src/BehaviorTree.cpp`
  - 注册 `SelectPosture` 节点。
- `Scripts/main.xml`
  - 主序列插入 `<SelectPosture/>`（在 `SelectAimTarget` 后、`PublishAll` 前）。
- `Scripts/config.json`
  - 新增 `Posture` 配置块。

---

## 4. 状态机要点

## 4.1 输入

- `desired_posture`（由决策上下文计算）
- `feedback_posture`（来自 `/ly/gimbal/posture`）
- `now`（steady clock）

## 4.2 输出

- `postureCommand`：
  - `0` 本轮不发
  - `1` 进攻
  - `2` 防御
  - `3` 移动

## 4.3 核心规则

1. 冷却：两次成功切换间隔必须 >= `SwitchCooldownSec`。
2. 计时：对当前姿态累计秒数，超过 `MaxSinglePostureSec` 标记降档风险。
3. 提前轮换：达到 `EarlyRotateSec` 且期望仍是当前姿态时，自动选择替代姿态。
4. pending：
   - 发出切换命令后进入 pending；
   - 在 `PendingAckTimeoutMs` 内等待回读确认；
   - 超时后按 `RetryIntervalMs` 重试，最多 `MaxRetryCount` 次。
5. 防死锁：
   - 重试耗尽后丢弃 pending；
   - 选择累计时间更低的姿态进行恢复性切换；
   - 防止长时间卡在“想切但切不动”的状态。

---

## 5. 决策逻辑（强化版）

当前不再是单纯 if-else，而是“**评分式决策 + 迟滞 + 时间窗**”：

1. 先按策略模式给基础分：
   - `Protected` 偏 `Defense`
   - `NaviTest` 偏 `Move`
   - `HitSentry/HitHero` 偏 `Attack`
2. 再叠加瞄准模式分：
   - `RotateScan` 偏 `Move`
   - `Buff/Outpost` 偏 `Attack`
3. 使用目标时间窗（`TargetKeepMs`）：
   - 最近目标可见则加大 `Attack` 权重
   - 避免目标瞬时丢失导致姿态抖动
4. 叠加风险分：
   - 低能量、低血、低弹、最近受击（`DamageKeepSec`）会提升 `Defense`/`Move`
5. 回读过期保护：
   - `feedback_stale` 时降低激进权重，偏保守姿态
6. 叠加 3 分钟规则惩罚：
   - 当前姿态累计接近或超过阈值时，惩罚该姿态得分
7. 加入切换迟滞（`ScoreHysteresis`）：
   - 如果新姿态分数领先不够大，则保持当前姿态，减少来回切换

这套逻辑在实战里比硬条件切换更稳定，尤其是目标抖动和链路抖动场景。

---

## 6. 配置项说明

位于 `src/behavior_tree/Scripts/config.json`：

```json
"Posture": {
  "Enable": true,
  "SwitchCooldownSec": 5,
  "MaxSinglePostureSec": 180,
  "EarlyRotateSec": 165,
  "MinHoldSec": 10,
  "PendingAckTimeoutMs": 600,
  "RetryIntervalMs": 300,
  "MaxRetryCount": 3,
  "OptimisticAck": true,
  "TargetKeepMs": 800,
  "DamageKeepSec": 4,
  "LowHealthThreshold": 120,
  "VeryLowHealthThreshold": 80,
  "LowAmmoThreshold": 30,
  "ScoreHysteresis": 2
}
```

- `Enable`: 姿态状态机总开关。
- `SwitchCooldownSec`: 姿态切换冷却。
- `MaxSinglePostureSec`: 单姿态累计阈值。
- `EarlyRotateSec`: 提前轮换阈值。
- `MinHoldSec`: 防抖最短保持时间。
- `PendingAckTimeoutMs`: 等待回读超时。
- `RetryIntervalMs`: 重试间隔。
- `MaxRetryCount`: 最大重试次数。
- `OptimisticAck`: 缺失回读时是否乐观确认（便于上位机单独联调）。
- `TargetKeepMs`: 目标短时丢失容忍窗口，控制攻击姿态稳定性。
- `DamageKeepSec`: 最近受击窗口，窗口内偏防御/机动。
- `LowHealthThreshold`: 低血阈值。
- `VeryLowHealthThreshold`: 极低血阈值（高优先级防守）。
- `LowAmmoThreshold`: 低弹阈值。
- `ScoreHysteresis`: 姿态切换分差迟滞，越大越稳但响应更慢。

---

## 7. 调试建议

1. 观察姿态命令：
   - `ros2 topic echo /ly/control/posture`
2. 观察姿态回读：
   - `ros2 topic echo /ly/gimbal/posture`
3. 对照行为树日志：
   - 关注 `[Posture] cmd=... desired=... current=... pending=... reason=...`，确认是否被 cooldown/pending/hysteresis 挡住。
4. 重点测试场景：
   - 高频目标进出（防抖）
   - 回读缺失（死锁恢复）
   - 长时间单姿态（3 分钟提前轮换）
5. 观察黑板可视化字段（已写入 Tick/Global Blackboard）：
   - `PostureCurrent / PostureDesired / PosturePending`
   - `PostureHasPending / PostureFeedbackStale`
   - `PostureAccumAttackSec / PostureAccumDefenseSec / PostureAccumMoveSec`
   - `PostureDegradedAttack / PostureDegradedDefense / PostureDegradedMove`
   - `PostureHasRecentTarget / PostureUnderFireRecent`

---

## 8. 后续建议

1. 回读来源稳定后，将 `OptimisticAck` 改为 `false`，严格依赖状态确认。
2. 把姿态运行态（累计时间、degraded 标志）做可视化输出，便于场上调参。
3. 若后续引入完整热量/底盘功率反馈，可把这些信号纳入 `SelectDesiredPosture` 的权重模型。

---

## 9. 变更记录

- 2026-03-04
  - 姿态决策升级为评分式模型（策略/瞄准/目标/资源/受击/回读综合评估）
  - 新增目标时间窗、受击时间窗、低血低弹阈值、分差迟滞参数
  - `SelectPosture` 增加姿态运行态写黑板，便于联调可观测
