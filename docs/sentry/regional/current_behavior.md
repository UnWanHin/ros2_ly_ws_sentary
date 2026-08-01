# 哨兵决策行为说明（纯行为版）

Updated: 2026-08-01

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

### 增强姿态与受击旋转

- 前哨锁定时，敌方前哨血量出现有效下降且 TypeID 10 的强攻额度新鲜且大于 0，先保持目标锁定并通过唯一的 `/ly/control/posture` 出口申请强攻 `4`。普通移动的 pending 请求不会消耗这次申请；只有回读到强攻 pending/active 后才算已接受，真正失败后才回退普通 Attack。
- ProtectHero 到点后先使用普通 Defense `2`。在配置的受击窗口内达到伤害阈值、强防额度新鲜且大于 0 时申请强防 `5`；已经形成强防意图后，不会再被通用受击分支降级成普通 Defense。额度不足、数据过期或矛盾隔离时仍安全回退普通 Defense。
- Recovery 行进期间，低于配置血线但仍为正血、且强移额度新鲜大于 0 时申请强移 `6`；`HP=0`、重生抑制、额度耗尽或 ACK 失败回退普通 Move `3`。Recovery 的姿态请求优先级最高，且不改变外部 `/ly/navi/should_rotate`、速度或轨迹话题。
- 受击旋转 ramp 仍按 `DamageRotate` 的 `0 -> 1 -> 2 -> 3` 输出到 `FireCode.Rotate`。Castle/ProtectCastle 本身不会压制受击旋转；只有显式 `StopRotate`、Highland 兼容、最新 `should_rotate=false` 或 FollowMode 才会最终压成 `0`。BT 日志会同时记录 `damage_detected`、`damage_gear`、`computed_gear`、`final_gear` 和 `suppressed_by`，可直接区分“没检测到受击”和“被外部控制压制”。

### CommonCentral 与 ProtectOutpost

- `AreaManager.yaml` 的 `RegionalAreaTask.CommonCentral` 属于 Default 区域巡逻开关；关闭时 Default 不会产生 Central 候选。`Tactical.yaml` 的 `Tactical.RegionalDefense.CommonCentral.Enable` 是另一条独立开关，默认开启，只控制敌方进入公共 Central 后的四点搜索。
- Tactical CommonCentral 的点序列是 `HoleRoad (17) -> CentralHigh (29) -> CentralLow (30) -> OutpostGuard (24)`；到达端点后反向走 `CentralLow -> CentralHigh -> HoleRoad`。红方坐标为 CentralHigh `(1000,1007)`、CentralLow `(989,496)`，蓝方坐标为 CentralHigh `(1800,493)`、CentralLow `(1811,1004)`。旧 `CentralLeft.A/B (26/27)` 保留给原有 Special Patrol。
- Tactical CommonCentral 旅行中不会因旧的短搜索计时器提前换点。它复用统一 `GoalReachState` 和导航 progress watchdog：只有到达后完成既有搜索驻留且未见目标，或外部明确不可达、14 秒内没有至少 80 cm 位移时，才切到另一点。
- ProtectOutpost 的固定防守点不是 `BuffOutpost`：红方使用 C3 `(1011,429)` cm，蓝方使用 C4 `(1789,1071)` cm。事件通过伤害阈值/时间窗触发，抵达后默认 SearchHold 30 秒；前哨为 0、不可达或保持结束时释放，重建后新的有效扣血可以再次触发。

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

### Hero 保护强防

ProtectHero 已到自己的守护点且仍拥有该导航 goal 时，BT 优先保持普通防御 `2`。仅当 `Tactical.ProtectHero.EnhancedDefense.Enable=true`，且 `DamageWindowMs` 内累计扣血达到 `DamageThresholdHp`，才检查 TypeID 10 回读的强防剩余时间；数据新鲜且大于零时，按照既有 5 秒姿态冷却申请强防 `5`。强防带来 99% 防御，但底盘功率为 1/2、散热为 1/3，因此行进期间不会使用。

- 未达到受击 burst、TypeID 10 过期或强防剩余时间为 0 时，保持普通防御 `2`，不猜测资源可用性。
- `5` 只有在 `/ly/gimbal/posture=2` 与 `enhanced_posture=true` 的新鲜匹配回读后才确认。
- 已确认且由本次 ProtectHero 姿态请求拥有的 `5`，只在本次姿态切换的 5 秒冷却内暂缓低血/低弹 Recovery，利用强防覆盖这段不可切换时间；冷却结束后，若仍满足原有 Recovery 阈值，立即按既有流程切到 Move `3` 并前往 Recovery。单独的下位机强防回读、回读失鲜或不再为 `5` 时都不会延后 Recovery。强防 ACK 重试耗尽后，本次 Hero 守护保持普通防御，直到任务退出或重新行进才允许新的申请，避免重复刷下发。

### Recovery 强化移动

只有 Regional 的低血量 Recovery 行进才会自动申请强化移动 `6`：尚未抵达回补点、HP 回读与 TypeID 10 都仍新鲜、HP 为 `1..80`、强移剩余时间大于 0，并且 YAML `Tactical.EnhancedPosture.RecoveryMove.Enable=true`。正常读条复活检测到自身 HP `0 -> 正数` 后的 30 秒内只用普通移动 `3`；抵达、HP 为 0、任一回读过期或强移 ACK 重试耗尽也回退到 `3`。这不使用 `out_of_combat` 作为复活判断。

`4/5/6` 下发前都会被 driver 以最新 TypeID 10 的对应正数额度二次核对，包含快取重发、其他 sentry command 欄位更新与串口重连；若 TypeID 7 的 `enhanced_posture=true` 与 TypeID 10 的对应额度 0 持续 500 ms，BT 隔离强化确认和新请求，等待普通姿态在既有 5 秒切换冷却后收敛。单帧不同步不会触发该隔离。

---

## 7. League / Regional 分离

- `CompetitionProfile=league` 时只走 `LeagueSimple`。
- `CompetitionProfile=regional` 时固定进入 `Regional`，不再使用旧单策略点表。
- Regional 的导航归属固定为 `Recovery > MapCommand > Task（Buff、开局/视觉前哨、区域过渡、watchdog）> Tactical > Special > Default`。Task 先于 Tactical；Tactical 只能在没有活动 Task 时接管。MapCommand 是 Task 中最高优先级，只有 Recovery 可以抢占它。
- 开局前哨、前哨 visual scout 与 Buff 都是 Task，不属于 Tactical。相机发现敌人进入 MyBase 可以触发 ProtectCastle，但不能取消活动中的开局前哨 Task；只有 Recovery、前哨毁灭/不可达、资源 gate 或该 Task 自己的结束条件可以释放它。
- Regional 的无事件行为由 Default 大区域任务和 AreaManager 状态机决定；Tactical 仲裁 ProtectCastle、ProtectOutpost、CommonCentral、ProtectHero 与 Chase，导航 watchdog 属于 Task。CommonCentral 是独立 Tactical 候选，当前默认优先级为 `3`，不再继承 ProtectCastle 的优先级。
- `/ly/game/event_data` 显示己方堡垒增益点状态为 `2` 或 `3` 时，`Tactical.ProtectCastle.StayWhenRfid=true` 会让 RegionalDefense 只去 `Castle`；在裁判事件仍新鲜期间，哨兵抵达后不再以导航追击或切点离开 Castle，但继续瞄准、旋转和开火。`StayWhenRfid=false` 保持原本在四个 Castle 边点搜索的行为。若己方 Base 大区内新鲜敌方位置数达到 `RegionalDefense.FortressStandEnemyCountMin`，且当前普通装甲目标已锁定并允许开火，则原地停速度、小陀螺切最高档开火。
- `StayWhenRfid=true` 时，新鲜原始 `2/3` 裁判事件不会进入 `FortressNoContactDegradeSec` 的无接触降级；事件过期或变为 `0/1` 后才释放 Castle 守点锁。关闭该开关时，原有降级保护不变。
- `Tactical.ProtectCastle.Enable=false` 会同时关闭城堡 RFID 与 MyBase 敌方坐标两条来源。单独设 `RFID=false` 会关闭堡垒事件与站桩火控；单独设 `EnemyPos=false` 会忽略敌方实际进入 MyBase 的防守来源，但 Highland、道路和 Central 的普通 RegionalDefense 继续有效。`StayWhenRfid` 只作用于 RFID `2/3` 来源，不改变 EnemyPos。`Tactical.ProtectHero` 完整管理英雄保护的开关、开局延时、保持/释放、新鲜度和守护目标；`Enable=false` 会释放英雄保护并在同一 Tactical tick 继续尝试 RegionalDefense。
- 如果这些层都没有输出，Finalizer 只同步策略层 blackboard，不再做旧点表兜底。
- 以后新增会接管导航的 Tactical 功能时，必须在同一次修改中使用明确的 `DecisionReason` 并写入可读 `detail`；最终 `[DecisionExplain][navi]` 日志会据此说明哨兵为何前往该点。
- 姿态 pending 也有来源优先级。普通评分请求是 `scored`，任务强制姿态是 `required`，Recovery、Buff 与新鲜 `navi_should_rotate=false` 的 Move 是 `safety`。新的更高优先级请求会以 `pending_superseded` 替换旧 pending，避免旧 Attack 重试覆盖当前必须的 Move。
- 普通 Transit 不再用固定 20 秒阈值消耗 Move。BT 以新鲜的哨兵官方地图位置、当前导航 goal 与新鲜 `/ly/navi/vel` 估算 ETA；速度失鲜时用 `Posture.TransitNominalSpeedMps`。ETA 乘安全系数、加到点缓冲后，限制在 `TransitMinReserveSec..TransitMaxReserveSec` 内。有效目标在 Transit 中仍允许 Attack；Recovery 始终优先 Move；非 Recovery 的短时间受击 burst 强制 Defense。

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

## 9. 决策日志

- 启动时会打印一次实际生效的 AreaManager 与 Tactical 开关。
- 哨兵最终换导航点时会打印原因、目标和坐标；同一点的重复下发不会刷屏。
- 回补显示 `recovery` 与当次 HP/弹量快照；默认区域任务显示区域和阶段；空闲巡逻显示巡逻序号与保持时间。分数、实时 HP 或弹量单独变化不会触发新的导航说明。
