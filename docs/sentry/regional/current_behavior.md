# 哨兵决策行为说明（纯行为版）

Updated: 2026-07-19

> 目的：只描述“机器人会怎么做”，不讲实现细节。

---

## 1. 开机后的行为

1. 先等云台姿态数据准备好。  
2. 再等裁判系统给出“比赛开始”。  
3. 比赛开始后才进入正常决策和火控。

> 结论：未开赛时不会按正式逻辑打人。

---

## 2. 比赛中的核心行为

- **有目标**：转向目标并进入瞄准，满足条件后开火。  
- **没目标**：进入搜索，不会盲目持续开火。  
- **目标丢失后**：优先继续搜索，找到后再回到打击。

---

## 3. 抬头 / 低头会在什么时候出现

- **常规搜索时**：会左右扫，同时做小幅抬头-低头扫描。  
- **特定打击任务（如高点/前哨搜索）**：会把视线抬高一些。  
- **收到“低头”指令时**：会强制压低云台俯仰，这个优先级最高。  

---

## 4. 什么时候会偏保守（你说的“逃跑”）

- 当血量或弹量进入低资源区间时，会从激进打击转向保守路线。  
- 保守阶段更重视生存与回补，不会一直顶在前面硬打。  

---

## 5. 区域过渡行为

- 进入我方高地、经高地兼容点、从我方高地离开时，会进入区域过渡状态。
- 区域过渡状态可以打开 `FollowMode` bit，但这个 bit 只随 `FireCode` 下发，不再让 BT 自动停小陀螺、停云台巡逻或停止新的开火翻转。是否停小陀螺看 `Rotate` 输出，是否接管云台/停火看 FaceMode 和显式停火控制。
- 从我方高地回我方基地侧目标时，会先走 `CastleLeft1`，到达后关闭 `FollowMode`，再继续原目标。
- `/ly/navi/reached` 和 `/ly/navi/reachable` 只是外部导航状态源；BT 内部到达由 `GoalReachState` 聚合这些状态和自身融合坐标距离。`/ly/navi/reached=true` 可优先确认到达，但 `false` 不能永久否决坐标兜底。

---

## 6. 姿态切换行为（进攻 / 防守 / 机动）

- 姿态会根据目标状态、受击情况、资源状态动态切换。  
- 切换不是每帧乱跳，带有保持时间和防抖机制。  
- 正式 Regional 的姿态切换必须等 `/ly/gimbal/posture` 回读确认；600 ms 未确认会按既有间隔重发，不能在未收到过回读时直接当作成功。
- 所以你看到的效果应是“有节奏地换姿态”，不是高频抖动切换。  

### 前哨强制交战锁

当外部 aim 新鲜选中 `7=前哨`、BT 也选中前哨，且官方 `/ly/enemy/op_hp` 新鲜且大于零时，BT 建立前哨交战锁。锁先保持 7 并使用普通进攻 `1`；敌方前哨血量在同一连续锁中出现新鲜下降时，若官方强攻剩余时间新鲜且大于零，会在普通进攻确认和 5 秒姿态冷却后申请强攻 `4`。

- 强攻 `4` 只在 `/ly/gimbal/posture=1` 和 `/ly/game/sentry/info.enhanced_posture=true` 都新鲜匹配时确认。
- 普通进攻 pending/确认冷却期间只在自身 HP `<=200` 时允许转火；强攻 pending/active 时门槛为 `<=250`。
- 敌前哨 HP 为零或过期、导航明确不可达时立即释放锁并取消该锁的 pending 姿态请求。
- 强攻 ACK 重试耗尽时保持已确认普通进攻和 7，不自动改发防御或移动。

---

## 7. League / Regional 分离

- `CompetitionProfile=league` 时只走 `LeagueSimple`。
- `CompetitionProfile=regional` 时固定进入 `Regional`，不再使用旧单策略点表。
- Regional 的无事件行为由 Default 大区域任务和 AreaManager 状态机决定；战术层只保留 RegionalDefense、Buff/Outpost 任务站位和导航 watchdog 这类明确 overlay。
- `/ly/game/event_data` 显示己方堡垒增益点状态为 `2` 或 `3` 时，RegionalDefense 不进 `Castle`，会在四个 Castle 边点里选最近点搜索敌方；默认继续边走边搜/打。若己方 Base 大区内新鲜敌方位置数达到 `RegionalDefense.FortressStandEnemyCountMin`，且当前普通装甲目标已锁定并允许开火，则原地停速度、小陀螺切最高档开火。
- 如果堡垒增益点长期保持 `2/3`，但连续 `RegionalDefense.FortressNoContactDegradeSec` 秒既没有己方 Base 大区敌方位置，也没有普通装甲视觉目标，则临时降级忽略该事件 `RegionalDefense.FortressDegradeCooldownSec` 秒，避免假占点事件一直拖住默认决策。
- `Tactical.ProtectCastle.Enable=false` 会同时关闭城堡 RFID 与 MyBase 敌方坐标两条来源。单独设 `RFID=false` 会关闭堡垒事件与站桩火控；单独设 `EnemyPos=false` 会忽略敌方实际进入 MyBase 的防守来源，但 Highland、道路和 Central 的普通 RegionalDefense 继续有效。`Tactical.ProtectHero.Enable=false` 会释放英雄保护并在同一 Tactical tick 继续尝试 RegionalDefense。
- 如果这些层都没有输出，Finalizer 只同步策略层 blackboard，不再做旧点表兜底。

---

## 8. 给联调同学的一句话版本

- 开赛前不打。  
- 有目标就打，没目标就搜。  
- 搜索时会有抬头/低头。  
- 收到低头指令会强制低头。  
- 低血低弹会转保守。  
- 区域过渡时 `FollowMode` 只改下发 bit；停旋转、固定点云台角、停火分别由 `Rotate`、FaceMode 和显式停火控制决定。
- Regional 不再落回旧单策略点表。
- 姿态会切，但不会无意义高频乱切。  
