# Regional 決策圖譜

Updated: 2026-08-01

> 範圍：`competition_profile:=regional` 的 `behavior_tree` 決策順序、優先級、導航輸出與姿態選擇。此圖描述 source 現有行為；任務級前哨交戰鎖可下發強攻 `4`，ProtectHero 到點受擊 burst 可下發強防 `5`，低血量 Recovery 行進可在額度/重生 gate 全部通過時下發強化移動 `6`。

正式入口、實際 XML、BT/bridge topic owner，以及所有測試用 BT launch 的邊界見
[behavior_tree_runtime_map.md](behavior_tree_runtime_map.md)。
外部 Aim 的 `frame_id`、BT Chase 授權、精確官方敵方點 fallback 與 `/goal_pose` 停距站位的
座標邊界見 [bt_aim_navi_coordinate_chain.md](bt_aim_navi_coordinate_chain.md)。

## 1. 每 tick 的實際順序

```mermaid
flowchart TD
  TICK[BT tick] --> GLOBAL[UpdateGlobalData\n訂閱資料、freshness、位置/血量/裁判狀態]
  GLOBAL --> EVENT[EvaluateEvents\nreached/reachable、受擊、補給等]
  EVENT --> AIM_MODE[SelectAimMode]
  AIM_MODE --> STRATEGY[SelectStrategyMode\nRegional]
  STRATEGY --> HARD[Hard]
  HARD --> TASK[Task]
  TASK --> PRE[PreprocessData]
  PRE --> TARGET[SelectAimTarget]
  TARGET --> LOCK[RefreshOutpostEngagementLock\n7 + fresh op_hp]
  LOCK --> TACTICAL[Tactical]
  TACTICAL --> SPECIAL[Special]
  SPECIAL --> DEFAULT[Default]
  DEFAULT --> FINAL[Finalizer]
  FINAL --> POSTURE[SelectPosture\nPostureManager composite ACK]
  POSTURE --> PUB[PublishAll]
```

`SelectPosture` 在導航/戰術目標決定後才執行：姿態會影響 `SentryCmd`，但不會回頭覆蓋該 tick 已選出的導航目標。

開局門控的 `StartGate.GimbalStrategy=face_mode_outpost` 同樣經 `FaceModeManager` 的唯一雲台仲裁：solver status 必須在
`FaceModeStatusFreshMs` 內、`function=true`、非 manual，且 target generation 超過本次請求前基線。最後一筆有效角度可在
`FaceMode.LostTargetHoldMs` 內持有，避免 callback 間隔造成 FaceMode/Patrol 閃爍；任何 status/TF 真失效仍回退 Patrol，
後續成功回讀會自動恢復 FaceMode。固定 FaceMode 與無目標 Patrol 的輸出則經 `PatrolScan.PassiveMotion` 限速；視覺 Aim 不限速。

## 2. 區域與任務優先級

```mermaid
flowchart TD
  HARD[Hard] --> RECOVERY{HP < 150\n或 ammo <= 30?}
  RECOVERY -->|是| HOME[Recovery / 回補\n持續到 HP >= 380 且 ammo > 30]
  RECOVERY -->|否| MAP_PREEMPT{有效 0x0303 坐標?}
  MAP_PREEMPT -->|是| MAP_GOAL[MapCommand\n45s raw official cm -> bridge]
  MAP_PREEMPT -->|否| TASK[Task]
  TASK --> OPENING[開局前哨 / visual scout]
  TASK --> BUFF[Buff 任務\nregional 預設關閉]
  TASK --> AREA_TRANSITION[導航區域轉換]
  TASK --> NAV_WATCHDOG[導航 watchdog / fallback]
  MAP_GOAL --> SPECIAL
  OPENING --> SPECIAL
  BUFF --> SPECIAL
  AREA_TRANSITION --> SPECIAL
  NAV_WATCHDOG --> SPECIAL

  TACTICAL[Tactical\n僅在沒有活動 Task 時] --> CASTLE[Protect Castle]
  TACTICAL --> OUTPOST_DEF[Protect Outpost\nfriend op_hp drop -> C3/C4]
  TACTICAL --> HERO[Protect Hero]
  TACTICAL --> CHASE[Chase\n同一 planned Default area 才追擊]

  TASK -->|無活動 Task| TACTICAL
  HERO --> SPECIAL
  CASTLE --> SPECIAL
  OUTPOST_DEF --> SPECIAL
  CHASE --> SPECIAL

  SPECIAL --> PATROL[Special Patrol\n預設關閉]
  PATROL --> DEFAULT

  DEFAULT[Default / AreaManager] --> MY_BASE[MyBase]
  DEFAULT --> MY_HIGH[MyHighland]
  DEFAULT --> MY_PRE[MyPreRoadland\nID 25: PreRoadland]
  DEFAULT --> MY_ROAD[MyReadyRoadland]
  DEFAULT --> CENTRAL[CommonCentral]
  DEFAULT --> ENEMY_AREA[敵方區域策略]
  DEFAULT --> SCORE[資源、距離、當前/上一區域懲罰\n選出 goal]
```

`PreRoadland` 與 `ReadyRoadland` 是正式同級 MainArea。`MyPreRoadland` 僅前往 ID 25，
到點後按 `GoalHoldSec` 結束；它可被更高優先級任務取消。`MyReadyRoadland` 保留原本的
`CentralToBase(ID 22) -> BaseToCentral(ID 21)` 強綁定穿越、安全返回、FollowMode 和
可選 FaceMode 行為。正式 `regional_competition.json` 已把兩區加入 `NaviGoal.MyArea`；
`MyReadyRoadland.UseFaceMode` baseline 為 `false`。ID 22 的正式座標為紅 `(515,100)`、藍
`(2285,1400)`，因此兩個穿越點都落在新 ReadyRoadland 邊界內。

導航所有權是 `Recovery > MapCommand > Task > Tactical > Special > Default`。MapCommand 雖在 Hard
pass 觀察，但 `DecisionIntent.Layer=Task`，可直接停止同 tick 內所有非 Recovery 導航輸出。Buff、開局前哨
與前哨 visual scout 都是 Task；開局 Task 活動時，camera 進入 MyBase 所觸發的 ProtectCastle 只能作為
威脅觀測，不能取消或改寫前哨導航。`navi_progress_watchdog` 也在 Task pass，只有沒有活動競賽 Task 時才
可能接管。`Tactical.Priority` 仲裁 ProtectCastle、ProtectOutpost、CommonCentral、ProtectHero、Chase（數字越小越高）。CommonCentral 是獨立 Tactical 候選，不再繼承 ProtectCastle 的順位。

`ProtectOutpost` 與敵方前哨 visual scout 是獨立任務。`/ly/friend/op_hp` 僅以 BT 本機收包時間
判新鮮，首次正血量只建立窗口基線；只有 `DamageWindowMs=2000` 內累積下降至少
`DamageThresholdHp=20` 才建立事件，小幅或跨窗口下降不會觸發。新鮮 `0 HP` 立即撤銷 C3/C4
並清空事件；重建後正血量會成為新基線，後續重新達門檻的掉血可再次觸發。事件使用官方厘米 C3（紅
`1011,429`）或 C4（藍 `1789,1071`）並經既有 `/ly/navi/goal_pos_raw -> navi_tf_bridge -> /goal_pose`
鏈路發出。抵達後保持 `SearchHoldSec=30` 秒，新的達門檻下降重置保持；不可達後沉默
`UnreachableCooldownSec=10` 秒，期間新的達門檻下降只排隊到冷卻結束。所有導航仍由 BT 的唯一最終
發佈出口發出。

`Tactical.RegionalDefense.CommonCentral.Enable` 只控制 Tactical 的公共 Central 威脅搜索，與
Default 的 `AreaManager.yaml: RegionalAreaTask.CommonCentral.Enable` 分開。其四點序列為
`HoleRoad (17) -> CentralHigh (29) -> CentralLow (30) -> OutpostGuard (24)`，到達
`OutpostGuard` 後反向為 `CentralLow -> CentralHigh -> HoleRoad`：行進中只在共享
`/ly/navi/reach_state` 明確不可達或同一個 `NaviProgressWatchdog` 判定 14 秒沒有 80 cm
進度時切點；已到達時才開始 `Tactical.RegionalDefense.CommonCentral.HoldSec`（預設 15 秒）
駐留，駐留完成且沒有新鮮視覺目標才切下一點。舊 `CentralLeft.A/B (26/27)` 不屬於這條路線。

ProtectHero 的全部有效參數由 `Tactical.yaml` 的 `Tactical.ProtectHero` 提供：開局時間、導航
保持、Hero 位置/血量新鮮度與目標 base goal 都可直接改 YAML。正式 Regional 預設
`ProactiveHoldWhenHeroInHighland=true`：Hero 位置新鮮、存活且位於 Highland 或 ProtectHero
子區域時，ProtectHero 直接駐守 Highland，不等待敵方座標，也不因無敵情釋放；它的 Tactical
priority `3` 仍低於 ProtectCastle `1`、ProtectOutpost `2`，且 Hard Recovery、MapCommand、既有
Outpost aim 分支都會暫時搶占，之後 Hero 條件仍成立便回守。設為 `false` 才恢復舊模式：同時有
我方 Base/Highland 敵情才啟動，連續 `NoEnemyReleaseSec` 秒無敵情後釋放。舊 JSON
`HeroProtection` 只在未提供 YAML 欄位時作相容基線；trace 的 `tactical.protect_hero` 會列出每個
gate，避免只看到 `active=false` 而不知道原因。

Chase 不會跨過 Default 的區域承諾：只有已啟動且可讓出的 Default 任務、敵方新鮮官方座標與
該任務完全相同的 `AreaKey`，並且 `Chase.yaml` 開啟該區時，Tactical 才發布追擊導航輸入。
拒絕後保留原區域任務的既有 goal；它不影響外部 aim 的瞄準或 fire。這條策略在 Tactical 內，
因此優先於目前預設關閉的 Special；未來若開 Special Patrol，可用 `SuppressChase` 主動禁用。

## 3. Regional 的輸入與導航閉環

```mermaid
flowchart LR
  HP[/ly/friend/hp\n/ly/enemy/hp] --> DATA[BT 全域資料]
  OP[/ly/friend/op_hp\n/ly/enemy/op_hp] --> DATA
  AMMO[/ly/game/bullet] --> DATA
  AIM[/ly/aim/armor_targets\n/ly/aim/result] --> DATA
  POS[/ly/friend/uwb_pos\n/ly/navi/position\n/ly/position/data] --> FUSION[SentryPositionFusion] --> DATA
  REACHED[/ly/navi/reached\n/ly/navi/reachable] --> REACH[GoalReachState] --> DATA

  MAP_CMD[/ly/game/map_command\nTypeID 9 / 0x0303] --> REGIONAL
  DATA --> REGIONAL[Regional 決策]
  REGIONAL --> GOAL[/ly/navi/goal]
  REGIONAL --> RAW[/ly/navi/goal_pos_raw]
  REGIONAL --> REL[/ly/navi/target_rel\n追擊]
  REGIONAL --> SPEED[/ly/navi/speed_level\nNavi.yaml 開關；0/1/2]
  RAW --> TF[navi_tf_bridge] --> POSE[/goal_pose]
  REL --> TF
  GOAL --> NAV[外部導航]
  POSE --> NAV
  SPEED --> NAV
  NAV --> REACHED

  TEAM[TypeID 1 GameCode\nIsMyTeamRed raw] --> TEAM_EFFECTIVE[gimbal_driver effective team\n唯一 /ly/friend/is_team_red publisher]
  TEAM_EFFECTIVE --> SENTRY_INFO[/ly/game/sentry/info\nSentryInfo.self_robot_id]
  TEAM_EFFECTIVE --> REGIONAL
  SENTRY_INFO --> PATH_BRIDGE
  NAV_PATH[/Path_downsampled\nnav_msgs/Path map/m] --> PATH_BRIDGE[map_path_to_game_path_node\nmap -> official inverse matrix]
  PATH_BRIDGE --> GAME_PATH[/ly/game/path\nMapPath dm + 原 header.stamp]
  GAME_PATH --> REF_PATH[0x02 -> map_data_t]
```

`MapCommand` 只接受官方場地內的坐標模式點，預設持有 45 秒、20cm 去重；5 次 100ms 和後續
1Hz 相同重送不續期。它不是 BaseGoal，任務持有期間會清除舊目標 external-status binding 並暫停
`/ly/navi/reach_state`，因此到達小地圖點不會被誤判為區域任務到點。除 Recovery 外，它會搶占
開局前哨、Protect、Buff、Default 和 ReadyRoadland 的所有導航。完整協議與邊界見
`docs/sentry/embedded/map_command_typeid9.md`。

`behavior_tree` 在等待開賽前會打印一次實際生效的 AreaManager/Tactical 配置快照（包括
ProtectOutpost 开关、新鲜度/掉血窗口/门槛、保持/冷却与五项 Tactical 优先级）；之後只在最終
可發布的導航決策 fingerprint 變化時打印 `[DecisionExplain][navi]`。該行由最終
`DecisionIntent` 與最後輸出的表示組成：區域點/小地圖命令使用官方 `cm`，手動前哨 pose 使用
`map` frame 的 `m`，相對追擊使用來源 frame 的 `rel_m`。相對追擊的位置持續更新不會每 tick
刷屏；它在決策、目標類別或 frame 切換時打印，不是每 tick 的除錯 trace。

Regional 的回補、Default 與閒置巡邏也使用同一最終輸出記錄。回補換點時為
`layer=hard reason=recovery`，detail 帶當次觸發動作和 HP/彈量快照；Default AreaTask 換點時
detail 帶 `area_task=<區域> phase=<階段>`；RegionalIdlePatrol 換點時帶
`index=<巡邏序號> hold_sec=<保持秒數>`。同一個最終 goal/坐標重發、HP/彈量或 Default score 的
單獨變化都不會另印一條導航決策說明。

### `speed_level` 與 `vel` 的分工

```mermaid
flowchart LR
  LEVEL[BT speedLevel\nUInt8 僅 0/1/2] --> TOPIC[/ly/navi/speed_level] --> EXT[外部導航\n檔位表不在本倉]
  RAW_VEL[BT naviVelocity.X/Y] --> SCALE[固定 raw * 0.025\n轉 x_mps/y_mps] --> CONTROL[/ly/control/vel] --> GD[gimbal_driver] --> LOWER[下位機]
```

- `Navi.yaml` 的 `Navi.Is_pub_navi_speed_level=true` 才啟用 `speed_level` 發布；它不經
  `gimbal_driver` 或下位機串口。`0=停`、`1=正常`、`2=高速`，其他內部值在發布邊界改為 `1`。
- Recovery 事件固定申請 `2`；其餘正式導航、追擊、小地圖命令與手動前哨導航為 `1`。`speed_level`
  不是 `/ly/control/vel` 的乘數，兩條鏈路在 BT 內互相獨立；外部導航的實際限速/倍率仍不在本倉斷言範圍。

## 4. 姿態選擇與計時來源

```mermaid
flowchart TD
  INFO3[TypeID 10\nsentry_info_3] --> INFO_TOPIC[/ly/game/sentry/info\nremaining seconds + age_ms]
  INFO_TOPIC --> FRESH{has info3 且\nage <= RefereeInfo3FreshMs?\n預設 1500ms}
  POSTURE_STATE[/ly/gimbal/posture\nUInt8 回讀] --> POSTURE_FRESH{本機收包 age <=\nFeedbackFreshMs?\n預設 1000ms}
  LOCAL[本地 AccumSec\n持續累積，永不停止] --> TIMER
  FRESH -->|是| TIMER[PostureManager 計時來源]
  FRESH -->|否| TIMER
  TIMER --> SCORE[SelectDesiredPosture]
  TIMER --> RECOVERY
  AREA[AreaManager\nTransit / ArrivedHold] --> TRANSIT[Transit override]
  HERO[ProtectHero 到守護點] --> HERO_READY{強防 YAML 開啟\n窗口內受擊達門檻\n且 TypeID 10 新鮮\n剩餘秒數 > 0?}

  RECOVERY[Regional Recovery 行進\nHP 1..80 + 強移額度新鮮\n且非讀條復活壓制?]
  SCORE --> OVERRIDE[硬規則\nRecovery/Buff -> Move\n前哨站狀態/受擊等]
  SCORE --> BASE[普通候選評分\nAttack / Defense / Move]
  FRESH --> REMAIN[普通或強化剩餘秒數]
  REMAIN --> PENALTY[0 秒強懲罰\n1..WarnSec 線性懲罰]
  REMAIN --> HOLD[當前為強化姿態時\n同類別保持 bonus]
  PENALTY --> BASE
  HOLD --> BASE
  OVERRIDE --> TRANSIT
  BASE --> TRANSIT
  TRANSIT --> HYST[ScoreHysteresis\n避免姿態抖動]
  HYST --> CMD[姿態命令\n1/2/3]
  HERO_READY -->|是| CMD5[強化防守 5\n保留既有 ACK/冷卻]
  HERO_READY -->|否| HERO_NORMAL[普通防守 2]
  RECOVERY -->|是| CMD6[強化移動 6]
  RECOVERY -->|否| OVERRIDE
  CMD5 --> TOPIC
  CMD6 --> TOPIC
  HERO_NORMAL --> TOPIC
  POSTURE_FRESH --> ACK[PostureManager\n僅新鮮且匹配才確認 pending]
  ACK --> CMD
  CMD --> TOPIC[/ly/control/posture]
  TOPIC --> SENTCMD[/ly/control/sentry_cmd]
  SENTCMD --> DL[Downlink 0x01\nRM2026 V2.0 0x0120]
```

### 姿態評分的現行配置

| 設定 | 預設 | 作用 |
|---|---:|---|
| `Posture.RefereeInfo3FreshMs` | 1500 ms | `sentry_info_3` 新鮮時才覆蓋官方剩餘秒數 |
| `Posture.FeedbackFreshMs` | 1000 ms | `/ly/gimbal/posture` 僅在本機實際收到回讀後的此窗口內可確認 pending；回讀接收時間還必須不早於該 pending 命令，逾時標記 stale |
| `Posture.RefereeRemainWarnSec` | 20 s | 剩餘秒數進入候選分數懲罰區間；不再作為 Transit Move 門檻 |
| `Posture.RefereeRemainPenalty` | 5 | 1..WarnSec 的最大候選扣分 |
| `Posture.RefereeZeroRemainPenalty` | 20 | 剩餘 0 秒的強扣分 |
| `Posture.EnhancedCurrentPostureBonus` | 3 | 已是強化姿態時，保持相同普通類別的加分 |
| `Posture.DynamicTransitReserveEnable` | true | 開啟以 ETA 計算 Transit Move 保留 |
| `Posture.TransitVelocityFreshMs` | 1500 ms | `/ly/navi/vel` 可用於 ETA 的新鮮窗口 |
| `Posture.TransitNominalSpeedMps` | 0.8 m/s | 速度失鮮時的保守 ETA 速度 |
| `Posture.TransitSafetyFactor` / `TransitArrivalBufferSec` | 1.5 / 8 s | ETA 安全係數與減速/到點確認緩衝 |
| `Posture.TransitMinReserveSec..TransitMaxReserveSec` | 30..120 s | 動態保留 Move 的上下界；無位置時使用 45 s fallback |

`/ly/gimbal/posture` 是無 header 的 `UInt8`，因此不能從協議上證明下位機採樣時間。BT 使用
`keep_last(1)` 限制積壓，並拒絕在命令前已被 BT 接收的回讀；若要完全保證下位機因果 ACK，必須由
下位機在未來協議中回顯命令序號或時間戳。

姿態採用一份內部 `TaskPostureIntent`，不建立第二份任務或導航資料。Default 的 `AreaManager` hint、
ProtectOutpost `Travel/SearchHold`、ProtectHero、固定區域防守和 SpecialPatrol 都只有在仍擁有目前
goal ID 與座標時才提供 intent。因此 goal 被 Chase 或上層任務替換時，舊 reached 不會保留到姿態仲裁。
SoftTransit 的 Move 保留由新鮮官方位置到目前 goal 的距離與 `/ly/navi/vel` ETA 決定：
`ceil(distance / speed * TransitSafetyFactor + TransitArrivalBufferSec)`，並限制在
`TransitMinReserveSec..TransitMaxReserveSec`；速度失鮮時用 `TransitNominalSpeedMps`，位置失鮮/無 goal 時用
`TransitFallbackReserveSec`。因此 60 秒 Move 面對短距離仍正常使用 Move，面對長距離會提前保留；不再以固定
1..20 秒才切換。有效目標且評分為 Attack 時，SoftTransit 允許 Attack；評分為 Defense 或非 Recovery 的短時
受擊 burst 則選 Defense。Recovery 仍是 HardMove，必定不被這個 ETA 規則覆蓋；Move 已為 0 時仍請求 Move，因為
它只表示弱化。
每個 pending 都記錄請求優先級與來源：普通評分為 `scored`、任務指定姿態為 `required`，Recovery、Buff 和
新鮮 `navi_should_rotate=false` 的 HardMove 為 `safety`。新的更高優先級姿態與 pending 不同時，
`PostureManager` 立即以 `pending_superseded` 發出新命令並重設 retry；因此待確認的 Attack 不會再重發並
覆蓋正在移動所需的 Move。`[Posture]` 日誌會同時輸出 `pending_priority` 與 `pending_source`。
ProtectHero 到達守護點固定提供普通防守 `2`。`Tactical.ProtectHero.EnhancedDefense.Enable=true` 時，只有 `DamageWindowMs` 內累積受擊達 `DamageThresholdHp`，且新鮮 TypeID 10 的強防剩餘秒數大於零才申請 `5`；其他情況保持 `2`。已確認的強防只在姿態 5 秒冷卻內暫緩 Regional Recovery；冷卻結束後若仍低血/低彈，既有 Move/Recovery 硬鏈路立即接管。強防回讀失鮮或解除時也不延後 Recovery。Recovery 尚在行進且 HP `1..80` 時，只有強移額度 TypeID 10 新鮮正數、`RecoveryMove.Enable=true`，並且未處於本機自身 HP `0 -> 正數` 後 30 秒壓制，才提供 `RecoveryEnhancedMove=6`；其他 Recovery 仍是 `HardMove=3`。強移 ACK 重試耗盡會在本次 Recovery 內鎖定普通 Move 回退。Buff、新鮮 `should_rotate=false` 是 `HardMove`；非 Recovery 的受擊 burst 是 `HardDefense`，會壓過前哨交戰的 Attack，Recovery 的 Move 仍最高。

當 Move 正餘額不高於 ETA 動態保留時，SoftTransit 會在 Attack/Defense 中選擇剩餘時間較多的一檔，平分時選
Defense。Transit/Hard request 禁止泛用提前輪換覆蓋；SoftArrived 保留輪換以平衡三種 180 秒、不恢復的普通姿態
預算。所有請求仍受 5 秒切換冷卻、10 秒最短保持與回讀 ACK 約束，故資料延遲或冷卻期間不能宣稱絕對不會
進入弱化。`4/5/6` 都要求最新 TypeID 10 的匹配強化額度正數，driver 也會二次拒絕過期/0 額度命令；普通 `0..3`
不受限。若 TypeID 7 的 `enhanced_posture=true` 與 TypeID 10 匹配額度 0 持續 `500ms`，BT 隔離強化確認與新請求，
取消強化 pending，並等普通姿態經既有冷卻收斂，避免短暫跨幀不同步誤動作。trace 的
`posture.task_intent/task_source/task_owns_current_goal` 與 `tactical.protect_hero.enhanced_defense_*` 是此仲裁的離線驗收輸出。

## 5. 發布與下發

```mermaid
flowchart TB
  REGIONAL[Regional 最終 intent] --> NAV_OUT[導航\ngoal / target_rel / speed_level]
  REGIONAL --> AIM_OUT[瞄準\n/ly/aim/select_target]
  REGIONAL --> FACE_REQ[FaceMode request\nRegional]
  AIM_OUT --> FACE_REQ2[FaceMode request\nBuff / Outpost]
  FACE_REQ --> FACE_MGR[FaceModeManager\n收集與統一仲裁]
  FACE_REQ2 --> FACE_MGR
  FACE_MGR --> FACE_OUT[/ly/face_mode/target_raw]
  FACE_OUT --> FACE_SOLVER[map_aim_point_node]
  FACE_SOLVER --> FACE_ANGLES[/ly/face_mode/angles]
  FACE_ANGLES --> FACE_MGR
  FACE_MGR --> GIMBAL_OUT[唯一最終仲裁\nangles / firecode]
  REGIONAL --> POSTURE_OUT[姿態\nposture / sentry_cmd]
  REGIONAL --> POS_OUT[融合自身座標\n/ly/bt/sentry_position]

  GIMBAL_OUT --> D0[0x00 GimbalControlFrame]
  POSTURE_OUT --> D1[0x01 SentryCommandFrame]
  POS_OUT --> D4[0x04 SentryCoordinateFrame]
  D0 --> LOWER[下位機]
  D1 --> LOWER
  D4 --> LOWER
```

## 6. Source of truth

- `src/behavior_tree/Scripts/main.xml`
- `src/behavior_tree/src/StrategyManager.cpp`、`src/behavior_tree/src/GameLoop.cpp`、`src/behavior_tree/src/FaceModeManager.cpp`、`src/behavior_tree/src/ChasePolicy.cpp`
- `src/behavior_tree/config/AreaManager.yaml`、`src/behavior_tree/config/Navi.yaml`、`src/behavior_tree/config/Task.yaml`、`src/behavior_tree/config/Chase.yaml`、`src/behavior_tree/config/Special.yaml`
- `src/behavior_tree/src/PostureLogic.cpp`、`src/behavior_tree/src/PostureManager.cpp`
- `src/behavior_tree/src/PublishMessage.cpp`
- `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
