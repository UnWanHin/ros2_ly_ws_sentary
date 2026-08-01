# Regional 決策框架說明

Updated: 2026-08-01

本文記錄目前 `behavior_tree` 裡 regional 決策的區域狀態機框架：它會做哪些任務、怎麼啟動、怎麼判斷到達、會輸出什麼控制，以及哪些階段會被高優先級邏輯打斷。

## 覆蓋範圍

目前已完成的基本區域狀態機：

- `MyArea.Base`
- `MyArea.Highland`
- `MyArea.PreRoadland`
- `MyArea.ReadyRoadland`
- `CommonArea.Central`

目前沒有寫 `EnemyArea.Base / EnemyArea.Highland / EnemyArea.PreRoadland / EnemyArea.ReadyRoadland`
的獨立狀態機，因為當前基本 regional 任務不需要它們。Central 任務裡會走到敵方側的中場點，
但它仍然屬於 `CommonArea.Central`，不是敵方大區域任務。

主要相關文件：

- `src/behavior_tree/config/AreaManager.yaml`
- `src/behavior_tree/config/Task.yaml`
- `src/behavior_tree/include/AreaManager.hpp`
- `src/behavior_tree/src/AreaManager.cpp`
- `src/behavior_tree/include/DefaultStrategyManager.hpp`
- `src/behavior_tree/src/DefaultStrategyManager.cpp`
- `src/behavior_tree/include/OutpostOpeningHold.hpp`
- `src/behavior_tree/include/EventManager.hpp`
- `src/behavior_tree/src/EventManager.cpp`
- `src/behavior_tree/include/StrategyManager.hpp`
- `src/behavior_tree/src/StrategyManager.cpp`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/module/Area.hpp`

## Regional 邏輯總覽

本節只列程式裡已存在的 regional 邏輯，不代表當前 YAML 一定全部開啟。

Regional 不是單一點表，而是分層策略：

- `Hard`：只有低血/低彈 `Recovery` 高於有效 `MapCommand`；ReadyRoadland 的強綁定穿越仍優先於一般策略，但可被有效小地圖指令打斷。
- `Task`：先處理裁判小地圖 `MapCommand`，再處理 Highland 兼容過渡和導航 watchdog 等支援任務，不擁有基本大區域狀態機。
- `Tactical`：處理 Buff、RegionalDefense（含 ProtectCastle）、ProtectOutpost、ProtectHero、Outpost 和 watchdog fallback。
- `Special`：可開關的專項巡察層，目前只包含兩點線段 Patrol，優先級低於 Tactical、高於 Default。
- `Default`：沒有事件、沒有任務、沒有 Buff/Outpost 時，按大區域候選分數選並持續 tick `MyBase / MyHighland / MyPreRoadland / MyReadyRoadland / CommonCentral`。
- `Finalizer`：只做策略層狀態同步，不再做舊點表 fallback。

Regional 目前已有的主要邏輯：

- 回補/回基地：低血或低彈優先去 `Recovery`；這層高於 RegionalDefense。非 league regional 下，已在 `Recovery` 且血量未回到門檻時會繼續守住 Recovery。
- Default 大區域任務：候選包含 `MyBase`、`MyHighland`、`MyPreRoadland`、`MyReadyRoadland`、`CommonCentral`；評分會看血量/彈量新鮮度、資源門檻、距離、目前區域、上一個區域、任務冷卻和失敗重試。JSON 的 `DecisionAutonomy.NaviGoal` 保留完整基線；正式 Regional 在讀完 `AreaManager.yaml` 後，會由 `RegionalAreaTask.Enable` 與五個 per-area `Enable` 重寫己方四區和 Central 的最終 scope，`EnemyArea` 維持 JSON。若有其他合格區域，剛選過的區域會排到本輪最後，避免原地重複。
- `MyBase` 任務：只在程式固定的四個 Castle 邊點中按自身距離、目前點與訪問新鮮度選點；到點保持 15 秒，完成 `MaxPatrolSteps` 後退出，交回 Default scorer 重新評估。`BuffOutpost`、`HoleRoad`、`OutpostGuard` 不屬 Default Base route。
- `MyHighland` 任務：`Highland` approach -> `Highland` hold -> `BuffShoot` -> `BuffShoot` hold -> `HoleRoad` 離開；approach/leave 仍是地形兼容階段，但正式配置下 Follow/Rotate 兼容交給 `/ly/navi/should_rotate`。
- `MyPreRoadland` 任務：只前往 ID `25`，到點後按 `GoalHoldSec` 完成；它可被更高優先級任務取消，不繼承後段的強制穿越控制。
- `MyReadyRoadland` 任務：`CentralToBase -> BaseToCentral -> BaseToCentral hold -> CentralToBase return`；穿越段仍是強綁定調度段，不能被一般策略直接打斷，但有效 MapCommand 會立即接管。`GuardHoldSec` 到時或資源不健康時會返回並完成任務。它會維持 FollowMode；是否請求 FaceMode 由 `MyReadyRoadland.UseFaceMode` 決定，baseline 為 `false`。
- `CommonCentral` 任務：中場巡邏路線是 `my OutpostArea -> my RightShoot -> my BuffAround2 -> my LeftShoot -> my OutpostShoot -> enemy RightShoot -> enemy OccupyArea -> enemy OutpostShoot`，啟動時也按自身位置選最近點；完成 `MaxPatrolSteps` 後退出，交回 Default scorer 重新評估下一個大區域。
- RegionalDefense：用官方敵方位置和 `event_data` 做戰術防守；道路前/後段分別統計後聚合為同一條 RoadCorridor 防守威脅，敵方進我方 Base/Highland/PreRoadland/ReadyRoadland/CommonCentral 或己方堡壘增益點 `2/3` 都可觸發。
- 己方堡壘增益點 `2/3`：`Tactical.ProtectCastle.StayWhenRfid=true` 時只去 `Castle`，抵達後禁止底盤導航追擊與切點離開；新鮮原始裁判事件持續期間不會無接觸退化。關閉時維持 `CastleLeft1 / CastleLeft2 / CastleRight1 / CastleRight2` 搜索和既有退化保護。此開關不影響 EnemyPos。
- ProtectOutpost：本機新鮮 `/ly/friend/op_hp` 在 `DamageWindowMs=2000` 內相對窗口基線累積下降至少 `DamageThresholdHp=20` 時，去紅 C3 `(1011,429)` 或藍 C4 `(1789,1071)` 官方厘米點；到達後固定搜索 30 秒。小幅下降或跨窗口的下降不觸發；`0 HP` 立即撤銷 C3/C4 並清空事件。重建後的正血量先成為新基線，之後再次達門檻的掉血可重新起任務；不可達則冷卻 10 秒，冷卻內的新達門檻掉血只排隊到期後重試。
- Recovery：Hard 層先回 `Recovery` 點；到達後若 3 秒內血量/彈量沒有回升，會在己方 `Recovery` 子區域內切換中心探測點，避免卡在補給區邊緣。
- MapCommand：裁判 `0x0303` 坐標模式的非零官方地圖點會成為 45 秒（`Task.MapCommand.HoldSec`）的 Task 層導航任務。除 Hard Recovery 外，它高於所有導航策略，包含開局 120 秒前哨、Protect、Buff、Default 與 ReadyRoadland。它直接走 `/ly/navi/goal_pos_raw -> navi_tf_bridge -> /goal_pose`，不偽裝成區域 Goal ID，也不進入 `GoalReachState`、AreaManager 或 watchdog。相同點的 5x/100ms 和後續 1Hz 重送不續期；目標機器人模式沒有座標，不導航。Hard Recovery 每拍取消這個任務並記住該點，恢復後不會自動續走。
- Buff：由能量機關裁判狀態、sentry info、timer、damage abort 和 timeout 決定是否進 `AimMode::Buff`；戰術站位使用 `BuffOutpost`，FaceMode 對己方目標側。
- Outpost：正式入口不依賴 `op_hp`，由血量/彈藥門檻、時間窗、damage abort、目標不可達狀態和 `Task.OutpostConfirm.VisualScoutWithoutHp` 決定是否去 `BuffOutpost` 偵查；Travel 階段保持選前哨，不讓遠距離普通車體接管 `/ly/aim/result`，但進入 `VisualScoutFaceDistanceCm` 前仍用普通裝甲視覺和 Move 姿態。進入該距離後開前哨視覺、敵方前哨 FaceMode 和 Attack 姿態，不再強依賴 `/ly/navi/reached=true`。120 秒時間窗內是高優先級任務，但己方 Base 有敵方時 RegionalDefense 可打斷；120 秒後按 `PostWindowScoutIntervalSec` 低優先級回 `BuffOutpost`，接近後用 `PostWindowScoutHoldMs` 短 FaceMode 偵查。普通裝甲目標若有效且不超過 `ArmorWarningDistanceCm`，會先打車；目標消失後若前哨 gate 仍允許，會按 `PostArmorFaceSearchMs` 回前哨 FaceMode 搜索。`op_hp` 接口保留，若它新鮮且為 0，可提前判定敵方前哨已毀並跳過任務。
- Navi progress watchdog：檢測 goal 不可達或長時間無位移，按當前目標區域選 fallback 點。
- Special Patrol：由 `src/behavior_tree/config/Special.yaml` 控制；目前 `Enable=false`，不參與正式鏈路。日後啟用時才會在 Tactical 無事件時巡己方 `CentralLeft` 線；`SuppressChase=true` 是它對 Tactical Chase 的明確 opt-out。
- Regional idle patrol：預留空閒巡邏，默認候選是 `HoleRoad / Castle / CastleRight2 / CastleRight1 / CastleLeft1 / CastleLeft2`。

Regional 裡常見控制語義：

- `AimMode::RotateScan`：普通裝甲搜索/打車。
- `AimMode::Buff`：打符視覺鏈路。
- `AimMode::Outpost`：前哨視覺鏈路。
- `FollowMode`：只表示下發到 `FireCode.FollowMode` 的語義位；BT 不再因這個 bit 自動停小陀螺、停巡邏或停開火。
- `FaceMode`：雲台朝固定區域/點接管，可按配置停火；它本身不等於停小陀螺。
- Chase：Regional 僅在一個可讓出的 Default `RegionalAreaTask` 已承諾區域時允許導航追擊。`Chase.yaml` 的 `MyBase`、`MyHighland`、`MyPreRoadland`、`MyReadyRoadland`、`CommonCentral` 只開啟各 planned area；選中的敵人必須有新鮮官方坐標，且精確落在同一個 `AreaKey`（不接受 nearest fallback）。拒絕追擊只清本拍追擊輸出，保留原 Default 任務與目標；瞄準和開火鏈不受此導航授權限制。League/Showcase 維持既有 area-scope 行為。

## Strategy 分層

`Scripts/main.xml` 的主決策入口現在先跑高優先級策略層，再整理目標，接著跑 Tactical 和 Special，最後由 Default 兜底。每 tick 依序執行：

```text
EvaluateEvents -> Hard -> Task -> PreprocessData -> SelectAimTarget -> Tactical -> Special -> Default -> Finalizer
```

各層的責任是：

- `EvaluateEvents`：語義整理層，只把裁判資料、視覺鎖定、受擊、導航狀態和 RegionalDefense 威脅收斂成 `EventSnapshot`。這層不發導航、不改火控、不接管輸出。
- `Hard`：最高優先級保護，處理 recovery/補血補彈和 ReadyRoadland 強綁定穿越段。ReadyRoadland 強綁定段在這層 hard lock，避免被戰術層中途搶走。
- `Task`：先接受有效 `0x0303` MapCommand 並壓過 Tactical/Special/Default，再處理 Highland 兼容過渡和導航 watchdog；不再 tick Highland/Base/PreRoadland/ReadyRoadland/Central 基本大區域狀態機。整個 Hard 層都高於 MapCommand：包含 Recovery 與不可中斷的 ReadyRoadland 穿越段。
- `PreprocessData / SelectAimTarget`：在 Tactical 前整理可打目標、官方坐標和本 tick `targetArmor`，讓戰術層使用最新目標資料。
- `Tactical`：regional 只保留明確戰術 overlay：`RegionalDefense`、ProtectOutpost、CommonCentral、ProtectHero、Buff/Outpost 任務站位、Chase 追擊和導航 watchdog；不再調用舊單策略點表。`Tactical.Priority` 以小數字優先排序 ProtectCastle、ProtectOutpost、CommonCentral、ProtectHero、Chase；Hard、Task 和既有 Buff/Outpost aim 分支仍高於此表。CommonCentral 只處理新鮮敵方官方坐標落在己方 Central 區域的四點搜索，不再繼承 ProtectCastle 的順位。`LeagueSimple` 只在 `CompetitionProfile=league` 時使用，Showcase 只在明確 showcase 配置時使用。
- `Special`：可選專項層；目前 `Special.Patrol.Enable=false`。日後啟用時才巡己方 `CentralLeft` 線；`PreRoadland` 已是 Default scope 內的正式 AreaTask。Special 在 Tactical 之後，若啟用且 `SuppressChase=true`，它會主動禁止該專項期間的 Chase。
- `Default`：無特別事件時的底層決策，只在已啟用的大區域中以程式內建的資源門檻、距離、目前區域、上次任務結果、冷卻和重試評分，再啟動 AreaManager 任務。每個 Default 區域任務都必須有完成/退出條件；任務完成後回到 scorer 重新評估，不寫死下一個大區域順序。沒有可用區域時不再 fallback 到任何舊點表。`RegionalIdlePatrol` 點表不再是正式 regional 的 Default 入口。
- `Finalizer`：只做本 tick 策略層完成標記和黑板同步；regional 不再 fallback 到舊點表。

分層狀態會寫入 BT blackboard：

- `EventSnapshot`
- `EventBuffCanActivate`
- `EventBuffActivating`
- `EventBuffActivated`
- `EventEnemyOutpostAlive`
- `EventRegionalDefenseActive`
- `EventRecentDamageOver30`
- `EventGoalReached`
- `EventGoalUnreachable`
- `EventSelfFortressGainPointStatus`
- `StrategyLayerHandled`
- `StrategyLayerHandledBy`
- `StrategyLayerHardLock`
- `StrategyLayerDefaultRequested`

這些字段只做監控和後續 Tactical 輸入，不改 ROS topic contract。

### Regional Chase 區域授權

`ChasePolicy` 是 Tactical 層的導航 overlay，不重新選目標、不改火控，也不重評 Default。
它只在當前有一個可讓出的 Default `RegionalAreaTask` 時工作：任務映射到
`MyBase`、`MyHighland`、`MyPreRoadland`、`MyReadyRoadland` 或 `CommonCentral` 的 planned
`AreaKey`；選中的敵人必須有新鮮官方場地坐標，並以 `AreaManager::ResolveAreaKeyForPoint()`
精確落入同一個 `AreaKey`。邊界外、過期座標、未知座標、nearest fallback、異 side/kind 或
`src/behavior_tree/config/Chase.yaml` 未開啟的 planned area 都拒絕 Chase。

拒絕時本拍不發布新的 `/ly/navi/target_rel` 或 official chase goal，仍由原 Default 任務持有
既有導航目標；這不會禁止 `/ly/aim/*` 的瞄準與開火。`Chase.AreaLimit` 仍是 bridge/goal 的
獨立幾何限制。`navi_tf_bridge` 會從 `Area.hpp` 解析 red/blue 的 Base、Highland、PreRoadland、
ReadyRoadland 加 CommonCentral，共 9 個正式主區；舊 `roadland` scope token 僅兼容映射為
`ready_roadland`。

## 數據來源

Regional 任務主要使用這些導航/定位輸入：

- `/ly/navi/reached`：外部導航對當前 goal 的到達來源；它不是 BT 內部最終 reached 事實。
- `/ly/navi/reachable`：當前 goal 是否有有效路徑，主不可達判斷。
- `/ly/navi/reach_state`：BT 對當前 goal 評估後發布的 composite `GoalReachState`，包含 status、reason、goal id/坐標、external freshness、融合自身坐標距離、grace 和 timeout。
- `/ly/navi/should_rotate`：外部導航區域兼容控制；`true` 恢復 BT 正常小陀螺/巡邏，`false` 關小陀螺並請求 `FollowMode`。
- `/ly/friend/uwb_pos`：雷達/UWB 推出的己方哨兵自身官方地圖坐標，單位 cm。
- `/ly/navi/position`：導航/TF 推出的自身官方地圖坐標，單位 cm。
- `/ly/position/data`：官方/雷達定位坐標。

自身哨兵坐標由 `AreaManager.SentryPositionFusion` 統一融合後寫入 `friendRobots[Sentry].position_`。默认 `Mode=priority`，優先級是 `/ly/friend/uwb_pos`、`/ly/navi/position`、`/ly/position/data` 裡 `friendcarid == Sentry`；也可以改成 `Mode=weighted`，按各 source 的 `Weight` 對新鮮坐標做加權平均。這個融合後坐標主要用於：

- 選最近的巡邏起點；
- 判斷自己目前在哪個大區域；
- 作為 BT 內部 composite reached 的坐標距離來源之一。

也就是說，`/ly/navi/reached`、`/ly/navi/reachable` 和自身融合坐標已一起進入 BT 的 goal-scoped `GoalReachState` 評估。`/ly/navi/reached=true` 可作為高優先級正向來源；`/ly/navi/reached=false` 不能永久否決坐標距離兜底。

## `/ly/navi/should_rotate` 的作用

`/ly/navi/should_rotate` 是外部導航給 BT 的地形兼容控制信號，類型是 `std_msgs/msg/Bool`。這裡的 `Rotate` 指 `FireCode.Rotate`，也就是下發給下位機的小陀螺檔位/rotate level，不是雲台 yaw 角速度。它只管本輪火控裡的 `FollowMode` 和小陀螺 `Rotate`；目前配置不以它釋放 regional 區域任務 FaceMode，也不負責選導航點、啟動、取消或推進任何 AreaManager task。

配置入口是 `src/behavior_tree/config/Navi.yaml`：

- `Enable=true` 時啟用這條外部控制鏈。
- `Is_pub_navi_speed_level=true` 時，BT 對每個固定導航輸出同步發布
  `/ly/navi/speed_level`。它是外部導航 ROS topic，不經 `gimbal_driver` 或任何串口
  TypeID；僅定義 `0=停、1=正常、2=高速`，其他內部值發出前一律收斂為 `1`。Recovery
  事件使用 `2`；普通任務、追擊、小地圖命令與手動前哨導航使用 `1`。
- `FreshTimeoutMs=500`，超過這個時間沒有新消息時按 `DefaultIsRotate=true` 處理，避免舊的 `false` 長時間卡住底盤。
- `DefaultIsRotate=true` 表示沒有新鮮信號時回到 BT 正常小陀螺/雲台巡邏策略。
- `ForceFollowModeWhenFalse=true`：收到新鮮 `false` 時，只在最終下發 `FireCode` 前臨時合併 `FollowMode=1`；這個 bit 不會觸發 BT 停火/停巡航分支。
- `StopRotateWhenFalse=true`：收到新鮮 `false` 時，本輪強制 `Rotate=0`。
- `ClearFollowModeWhenTrue=true`：收到新鮮 `true` 時，釋放 `FollowMode`，回到 BT 正常 rotate 策略。
- `ClearRegionalFaceModeWhenTrue=false`：目前保持關閉；收到新鮮 `true` 時仍保留 regional area-task 的 FaceMode，由區域任務本身決定是否固定朝向。Buff/Outpost 自己的固定朝向也不靠這個開關清。

語義可以理解成：

```text
should_rotate=false
  -> 外部導航認為當前處在不適合小陀螺的地形/區段
  -> 當 ForceFollowModeWhenFalse=true 時，最終下發 FireCode.FollowMode=1
  -> 當 StopRotateWhenFalse=true 時，最終下發 Rotate=0
  -> BT 仍按當前 AimMode 做雲台巡邏、鎖敵和開火

should_rotate=true
  -> 外部導航認為已離開兼容區段
  -> BT 不再合併外部 FollowMode bit；若 ClearFollowModeWhenTrue=true，會清除殘留 FollowMode
  -> 小陀螺回到 BT 自己的策略，可由任務/受擊決策為 0、1、2 或 3 檔
  -> regional FaceMode 仍由當前區域任務決定，不會因 should_rotate=true 被取消
```

所以正式當前配置下，`should_rotate=false` 表示「外部導航請求下位機 FollowMode bit + 關小陀螺檔位」。BT 仍會該巡航就巡航，該鎖敵開火就開火；`FollowMode` bit 本身只進最終 firecode 字段，不再觸發 BT 停火/停巡航分支。

它和 `MyReadyRoadland` 強鎖不是同一層：

- `MyReadyRoadland` 強鎖是任務調度保護：在 `ReadyRoadlandCrossToBaseToCentral`、`ReadyRoadlandCrossToCentralToBase` 兩個 phase 裡，Buff/Outpost、RegionalDefense、Recovery 不會直接取消這個穿越段，要等到達、不可達或超時。
- `/ly/navi/should_rotate` 是控制輸出兼容：告訴 BT 這一段要不要停小陀螺、開 FollowMode。它不會讓 ReadyRoadland task 進入或退出強鎖，也不會改變 ReadyRoadland phase。

因此現在的責任邊界是：AreaManager 決定「我要不要做 MyReadyRoadland，以及是不是處於不可讓出的 crossing phase」；外部導航通過 `/ly/navi/should_rotate` 決定「此刻底盤是否允許正常小陀螺」。這樣 Castle、ReadyRoadland、Highland 等地形細節可以放在導航側維護，BT 只保留任務優先級和戰術語義。

## FaceMode 統一仲裁

FaceMode 是雲台朝向任務，與 `Rotate`、`FollowMode` 分開。所有任務來源必須走 `FaceModeManager`，不能直接發布 `/ly/control/angles` 或在任務分支中直接改最終雲台輸出：

```text
Regional / Buff / Outpost 任務
  -> FaceModeManager request（本拍收集）
  -> 優先級：Buff / Outpost > Regional
  -> Resolve：視覺目標優先、導航釋放兼容、角度保持、巡邏 fallback、SuppressFire
  -> FaceModeDecision
  -> PublishTogether() 唯一發布 /ly/control/angles + /ly/control/firecode
```

- Regional、Buff、Outpost 的目標都只經 `/ly/face_mode/target_raw` 交給 `map_aim_point_node` 求解；求解器回傳 `/ly/face_mode/angles`，BT 只快取候選角度。
- `should_rotate=true` 在目前 `ClearRegionalFaceModeWhenTrue=false` 時不抑制 Regional FaceMode；若未來明確開啟該配置，抑制規則也只在 `FaceModeManager::Resolve()` 生效，不會插入任務分支。
- 視覺鎖敵優先時，FaceMode request 仍保留在本拍決策 trace，但不接管最終角度；下一拍視覺優先解除後可以重新仲裁。
- `/ly/face_mode/angles` 暫時無有效角度時，是否轉回 patrol scan 完全由 `PatrolScan.FaceModeFallbackEnable` 統一決定。

## 入口鏈路

上游決策選中導航點後，通常會走：

```text
TrySetScopedPositionByBaseGoal()
  -> Area scope 檢查
  -> TryStartRegionalAreaTaskForGoal()
  -> TryStartNaviAreaTransition()
  -> SetPositionByBaseGoal()
```

所以，上游策略不需要手動知道每個區域任務的細節。它只要選一個目標點，AreaManager 會先判斷這個點屬於哪個大區域，再決定是否啟動對應狀態機。

區域任務不會啟動的情況：

- 已經有 regional area task 在跑；
- Highland 兼容過渡正在跑；
- goal ID 不合法；
- goal 不能精確解析到一個大區域；
- 對應任務在 `AreaManager.yaml` 裡沒有開啟。

非 Central 的 MyArea 任務要求目標點屬於我方。`CommonArea.Central` 比較特殊，只要目標點解析到 Central，就會用我方作為任務 owner 來跑 Central 巡邏。

## 當前配置

### PreRoadland / ReadyRoadland 正式邊界

`PreRoadland` 與 `ReadyRoadland` 已是同級 `MainAreaKind`，都參與目標區域解析、Default scorer、Regional task 和正式導航輸出。`ReadyRoadland` 使用原 ReadyRoadLand 四邊形；共用邊界依 `AreaManager` 的主區解析順序歸屬 `ReadyRoadland`。

```text
PreRoadland Red:  (687,380) -> (758,235) -> (510,235) -> (510,205) -> (510,19) -> (389,15) -> (391,373)
PreRoadland Blue: (2113,1120) -> (2042,1265) -> (2290,1265) -> (2290,1295) -> (2290,1481) -> (2411,1485) -> (2409,1127)
ReadyRoadland Red:  (510,235) -> (510,19) -> (1251,17) -> (1333,221)
ReadyRoadland Blue: (2290,1265) -> (2290,1481) -> (1549,1483) -> (1467,1279)
```

`MyPreRoadland` 固定使用 ID 25，抵達後按自身 `GoalHoldSec` 結束；`MyReadyRoadland` 保留 ID 22 -> ID 21 的強綁定穿越、FollowMode 與 FaceMode。ID 22 目前是紅 `(515,100)`、藍 `(2285,1400)`，已落在新 ReadyRoadland 內。

目前 `src/behavior_tree/config/AreaManager.yaml` 的基本區域配置是：

```yaml
AreaManager:
  SwitchPoint: false
  RegionalAreaTask:
    Enable: true
    MyBase:
      Enable: true
    MyHighland:
      Enable: true
    MyPreRoadland:
      Enable: true
    MyReadyRoadland:
      Enable: true
    CommonCentral:
      Enable: true
  SentryPositionFusion:
    Enable: true
    Mode: priority
    FreshTimeoutMs: 2000
    Sources:
      Uwb:
        Enable: true
        Priority: 0
        Weight: 1.0
      PositionData:
        Enable: true
        Priority: 2
        Weight: 0.7
      Navi:
        Enable: true
        Priority: 1
        Weight: 0.8
```

正式 regional 和單區域 areatest 的 profile `DecisionAutonomy.NaviGoal` 保留完整地圖 scope 基線；`AreaManager.RegionalAreaTask.Enable` 與五個區域 `Enable` 是它們的最終、現場可改覆寫層：總開關為 `false` 時清空己方四區與 Central，個別為 `false` 時只從對應 scope 移除該區。`EnemyArea` 與其他 JSON 設定完全保留。設為 `false` 時，Default 不會產生該區域候選，正在跑的同類任務也不再輸出。`sentry_all` 也會把同一份 YAML 解析給 `navi_tf_bridge` 的 Chase area limit；使用 `./scripts/start.sh gated --mode regional` 時 wrapper 傳入 source YAML，改完重啟即可生效，不需要 `colcon build`。`AreaManager.yaml` 只保留 `SwitchPoint`、這六個核心 Enable 開關、`SentryPositionFusion` 與 `BuffOutpostCompat`。巡邏時序、共用評分、資源門檻、冷卻和重試都是程式內建預設；MyBase route 仍是程式固定的四 Castle 點，正式 launch 不再載入 `Base.yaml`。`Task.yaml` 只管這局是否允許 `Task.Buff / Task.Outpost`，會覆蓋 JSON 裡同名字段。

`SwitchPoint=true` 時只交換 `Area.hpp` 裡紅/藍官方點位和區域邊界查找結果，不交換 `team`、敵我語義或導航 goal ID。這是給導航零點/物理場地方向反了時使用的點位查找開關。

巡邏資源門檻與評分是 source-owned 的正式預設；ReadyRoadland 以健康門檻決定是否離開駐守點並安全返回，Central 則要求啟動與執行期間的血量/彈量新鮮且健康。若比賽調參確實需要變更，應以小範圍程式改動連同測試調整，不再從 AreaManager YAML 注入一整組策略參數。

DefaultPolicy 的當前選區規則：

- Area scope 是硬門檻：Regional 的 `MyArea / CommonArea` 最終由 `AreaManager.yaml` 的六個 Enable 決定；`EnemyArea` 保持 JSON。關掉的區域永遠不進 Default 候選。
- `MyBase` 需要新鮮血量/彈量並達到我方區域門檻，健康時才會啟動基地巡遊。
- `MyHighland` 需要新鮮血量/彈量並達到我方區域門檻。
- `MyReadyRoadland` 需要同時滿足 DefaultPolicy 我方門檻和 `MyReadyRoadland.Healthy*` 門檻。
- `CommonCentral` 需要同時滿足 DefaultPolicy Central 門檻和 `CommonCentral.Healthy*` 門檻。
- 候選分數會扣除距離、目前所在同區域、上次已選區域；Highland 任務正常完成後會臨時提高 MyBase/MyReadyRoadland 分數。
- `unreachable / timeout / unhealthy / canceled` 會進入 failure/unreachable cooldown，連續失敗數達到 `MaxRetry` 時使用更長的 unreachable cooldown。

## RegionalDefense

RegionalDefense 是事件驅動戰術層，優先級高於 Default。敵方位置判斷只使用 `/ly/position/data` 寫入的官方場地坐標，不使用 map/odom 坐標混判；AreaManager 用 `Area.hpp` 官方點位區域邊界判斷敵方是否進入我方 Base/Highland/PreRoadland/ReadyRoadland 或公共 Central，兩段道路會聚合為同一 RoadCorridor 防守威脅。城堡保護有兩條獨立來源：`Tactical.ProtectCastle.RFID` 控制 `/ly/game/event_data.self_fortress_gain_point_status` 的堡壘占領結果，`Tactical.ProtectCastle.EnemyPos` 控制敵方官方坐標進入我方 Base。`ProtectCastle.Enable=false` 會同時關閉兩條來源；`RFID=false` 只關事件和其站樁火控；`StayWhenRfid=true` 只在本車 RFID 支持的占領轉換被確認時站樁 Castle；`EnemyPos=false` 只忽略 MyBase 敵方坐標，Highland、兩段道路與 Central 的 RegionalDefense 不受影響。ProtectHero 的英雄保護條件會在 RegionalDefense 之前檢查；只有未命中英雄保護時，普通 RegionalDefense 才接管。

當前防守搜索規則：

- 敵方進入我方 Base：優先去 `Castle`，再 fallback 到左右 Castle 點。
- `/ly/game/event_data.self_fortress_gain_point_status` 是堡壘**占領結果**：`0=無人`、`1=我方`、`2=敵方`、`3=雙方`；`/ly/game/rfid.friend_bastion` 只表示本哨兵正在己方 Castle RFID 區，不能單獨判占領。若裁判新鮮狀態由 `0/2` 轉為 `1/3` 的當拍本哨兵 RFID 已有效，且前一個 `0/2` 狀態不超過 `Tactical.ProtectCastle.RfidCaptureTransitionWindowMs`（預設 3000 ms），才確認為「本哨兵完成占領」。該確認在 RFID 仍有效且狀態維持 `1/3` 時才令 `StayWhenRfid=true` 唯一導航到 `Castle` 並禁止 Chase/切點；隊友已占、我之後才進 Castle，或沒有此轉換證據時，`1/3` 只守 `CastleLeft1 / CastleLeft2 / CastleRight1 / CastleRight2` 外圍點。隊友官方位置是否在 Castle 只參與上述占領歸因與 decision trace，不是救援 gate；當下位機把新鮮裁判結果由 `1/3` 更新為 `0/2`（隊友離開後無人或敵方占領），BT 不等待 friend position，會立即把 ProtectCastle 導航切到 `Castle` 搶占。這是 V2.0.1 規則中「比賽滿 3 分鐘、己方前哨首次被毀後，敵方單車佔堡壘累計 20 秒會使己方基地護甲展開」的前置防守告警，不等同於已展開狀態：目前上行缺少單車計時與飛鏢命中結果。兩種模式都沿用普通裝甲模式邊走邊打；若己方 Base 大區的新鮮官方敵方位置數達到 `RegionalDefense.FortressStandEnemyCountMin`，且普通裝甲目標已鎖定並允許開火，則把底盤速度壓為 0、小陀螺覆蓋到最高檔站樁開火。
- 我方 Highland 和任一 RoadCorridor 段同時有敵方：優先去 `Castle`。
- 敵方進入我方 PreRoadland 或 ReadyRoadland：去 `CastleRight2 -> CastleRight1 -> Castle` 搜索。
- 敵方進入我方 Highland：去 `HoleRoad -> Highland -> Castle` 搜索，先利用 HoleRoad 視野，再進 Highland。
- 敵方在公共 Central：當 `Tactical.RegionalDefense.CommonCentral.Enable=true` 時，去 `HoleRoad (17) -> CentralHigh (29) -> CentralLow (30) -> OutpostGuard (24)` 搜索，抵達 OutpostGuard 後反向返回 `CentralLow -> CentralHigh -> HoleRoad`；它與 `AreaManager.yaml` 的 Default `RegionalAreaTask.CommonCentral.Enable` 完全獨立。中央點紅方坐標為 High `(1000,1007)`、Low `(989,496)`，藍方坐標為 High `(1800,493)`、Low `(1811,1004)`。

- 搜索點會尊重 area scope，但不啟動 Base/Highland/PreRoadland/ReadyRoadland 的 AreaManager 區域任務；它只做 scope 檢查、必要的 Highland transition，然後直接下導航點。`RegionalDefense.SearchHoldSec` 和 `RegionalDefense.SearchNoTargetSec` 控制其他 RegionalDefense 搜索；CommonCentral 到達後使用 `Tactical.RegionalDefense.CommonCentral.HoldSec`（預設 15 秒）駐留。Tactical CommonCentral 是例外的旅行保護：行進階段只復用 `GoalReachState` 與 `NaviProgressWatchdog`，不以駐留計時器換點；共享 watchdog 的 `MoveProgressCm=80`、`NoMoveTimeoutSec=14` 判定無進度時才沿四點序列前進。到達後才開始 15 秒駐留，且沒有新鮮視覺目標才切下一點。堡壘增益點事件在 `StayWhenRfid=false` 時保留退化保護：若連續 `RegionalDefense.FortressNoContactDegradeSec` 秒沒有己方 Base 大區官方敵方位置、也沒有普通裝甲視覺目標，會在 `RegionalDefense.FortressDegradeCooldownSec` 秒內暫時不把 `2/3` 當硬威脅；開啟 `StayWhenRfid` 時，新鮮原始 `2/3` 事件不退化。

## ProtectOutpost

`Tactical.ProtectOutpost` 防守己方前哨，与敌方前哨 visual scout 的
`Task.OutpostConfirm` 是两条独立链路。它只在 regional、开关启用、收到本机时间仍在
`HealthFreshMs` 内的 `/ly/friend/op_hp` 在 `DamageWindowMs` 内相对窗口基线累计下降至少
`DamageThresholdHp` 时建立事件；默认是 2 秒内 20 HP。第一帧、相同值、小于门槛的掉血、上涨
或过期回传都不会触发；新鲜 `0 HP` 会立即清空事件并撤销仍持有的 C3/C4。重建后的首笔正血量
只建立基线，之后再达到门槛才触发。事件经 `Area::ProtectOutpost` 走正常 raw 官方坐标链路：红方
C3 `(1011,429)`、蓝方 C4 `(1789,1071)`，最终仍由唯一的 BT 导航发布出口输出。

到达 C3/C4 后进入 `SearchHold` 并保持 `SearchHoldSec`（默认 30 秒）；保持中发生新的有效
达门槛的新掉血会重启计时。上层 Hard/Task 或更高 Tactical 项可以暂时抢占，但不会清掉同一事件状态，回到
本任务时仍复用同一目标。若该目标收到 `/ly/navi/reachable=false`，任务进入
`UnreachableCooldownSec`（默认 10 秒）冷却；冷却期间不重复发点，新的掉血只在冷却结束后排队为
一个新 Travel 事件。默认顺序由 `Tactical.Priority` 给出：ProtectCastle `1`、ProtectOutpost
`2`、CommonCentral `3`、ProtectHero `4`、Chase `5`，较小数字优先，相等时按此列出的固定顺序。

## ProtectHero

ProtectHero 是 Tactical 層的己方英雄保護點位。`Tactical.ProtectHero` 是正式 YAML 的唯一運行時權威；舊 JSON `HeroProtection` 僅在缺少對應 YAML 欄位時提供相容基線。比賽開始 `StartElapsedSec` 秒後，如果己方 Hero 的官方坐標在 `FriendPositionFreshMs` 內、血量不是 `FriendHealthFreshMs` 內已知的 0，且落在己方 Highland 大區或 `ProtectHero` 子區域內，BT 可下發 `GoalBaseId`，默認為 `Highland`，並用 `HoldSec` 控制駐守重發週期。

正式 Regional YAML 的 `ProactiveHoldWhenHeroInHighland=true` 為預防式保護：上述 Hero 條件成立即持續駐守，不等待敵方座標；只有 Hero 離開區域、位置過期或確認死亡才釋放。它目前是 Tactical priority `4`，會被 ProtectCastle、ProtectOutpost、CommonCentral、Hard Recovery 和有效 MapCommand 暫時搶占，搶占結束後 Hero 條件仍成立便回到 Highland。設為 `false` 才恢復舊敵情模式，必須同時有我方 Base 與 Highland 的新鮮敵方官方座標才啟動，並在連續 `NoEnemyReleaseSec` 秒無 RegionalDefense 敵情後釋放。Decision trace 的 `tactical.protect_hero` 記錄 `proactive_hold_when_hero_in_highland`、Hero gate、Base/Highland 敵方計數、舊 `threat_ready` 與 `mode_gate_ready`；後者只表示預防式/舊敵情的模式 gate，不代替其他 Hero 觸發條件。

## 各區域任務

### MyHighland

觸發條件：上游選中的 goal 精確屬於我方 Highland 大區域。

任務路線：

```text
Highland(FollowMode bit + optional FaceMode + explicit stop-fire)
  -> Highland 短暫巡邏/停留
  -> BuffShoot
  -> BuffShoot 駐守
  -> HoleRoad(FollowMode bit + optional FaceMode + explicit stop-fire)
  -> 完成
```

上面的 `FollowMode bit / FaceMode / stop-fire` 是三個獨立輸出。正式配置裡 `Navi.Enable=true` 時，進出 Highland 時實際是否停小陀螺主要由外部導航的 `/ly/navi/should_rotate` 決定。

進入 Highland 和離開 Highland 時：

- 若未啟用 `Navi.Enable`，AreaTask 可按結果寫 `FollowMode`；
- 若啟用 `Navi.Enable`，AreaTask 不直接寫 `FollowMode`，由 `/ly/navi/should_rotate=false` 觸發；
- 如果 `MyHighland.UseFaceMode=true`，才會發布 regional FaceMode 目標；
- 兼容 phase 仍會停火。

Highland 巡邏和 BuffShoot 駐守時：

- `FollowMode` 關閉；
- 雲台巡邏、小陀螺、開火回到普通 BT 控制。

### MyBase

觸發條件：上游選中的 goal 精確屬於我方 Base 大區域，並且當前不在我方 Highland 裡。

候選點：

```text
CastleLeft1 / CastleLeft2 / CastleRight2 / CastleRight1
```

每次啟動或切下一個巡邏點時，會按自身距離、目前點懲罰與訪問新鮮度評分，不按固定順序輪；拿不到自身坐標時按固定候選順序回退。每個點到達後保持 15 秒。Default 啟動 MyBase 前會先檢查血量/彈量是否新鮮且達到 `DefaultPolicy` 我方區域門檻；`BuffOutpost` 只由 Buff/Outpost tactical 發布。

MyBase 本身不開 `FollowMode`，也不開 `FaceMode`，就是普通基地巡遊狀態機。

### MyReadyRoadland

觸發條件：上游選中的 goal 精確屬於我方 ReadyRoadland 大區域，並且當前不在我方 Highland 裡。

任務路線：

```text
CentralToBase
  -> BaseToCentral(強綁定穿越)
  -> BaseToCentral 駐守
  -> CentralToBase(強綁定穿越)
  -> 完成
```

具體行為：

- 先正常去 `CentralToBase`。
- 到點、不可達或超時後，進入去 `BaseToCentral` 的強綁定穿越段。
- 強綁定穿越段中，任務調度 hard lock 生效；正式配置下是否 `FollowMode=1 / Rotate=0` 由 `/ly/navi/should_rotate=false` 決定。
- 是否穩定停火取決於獨立控制：active regional FaceMode 或任務結果明確 `SuppressFire` 才會壓住新的 `FireStatus` 翻轉；`FollowMode=1` 本身不再停火。
- 到 `BaseToCentral` 後，恢復普通巡邏、小陀螺、開火控制。
- 如果血量/彈量新鮮數據低於門檻，開始安全返回 `CentralToBase`。
- 如果非強綁定階段遇到更高優先級請求，也不是直接取消，而是請求安全返回。

如果 `MyReadyRoadland.UseFaceMode=true`，ReadyRoadland 的 FaceMode 目標就是當前穿越終點；正式配置目前是 `false`，地形朝向/跟隨主要交給外部導航的 `/ly/navi/should_rotate`：

- 往中場側穿越時，朝向 `BaseToCentral`；
- 返回基地側時，朝向 `CentralToBase`。

### CommonCentral

觸發條件：上游選中的 goal 精確屬於 Central，並且血量/彈量數據新鮮且健康。

巡邏路線：

```text
my OutpostArea
  -> my RightShoot
  -> my BuffAround2
  -> my LeftShoot
  -> my OutpostShoot
  -> enemy RightShoot
  -> enemy OccupyArea
  -> enemy OutpostShoot
  -> my OutpostArea
  -> repeat
```

起點會在整條巡邏路線裡按自身坐標選最近點。拿不到自身坐標時，從 `my OutpostArea` 開始。

CommonCentral 本身不開 `FollowMode`，也不開 `FaceMode`，就是普通中場巡遊狀態機。

任務中如果血量/彈量新鮮數據變成不健康，CommonCentral 會完成並釋放控制。

## Default 到點駐留

Default 區域任務的實際到點/guard phase 統一保持 15 秒：

- MyBase 的每個 Castle 點；
- MyHighland 的 HighlandPatrol 和 BuffShootHold；
- MyPreRoadland 的到點 hold；
- MyReadyRoadland 的 guard hold；
- CommonCentral 的每一個中央巡邏點。

Highland approach/leave、ReadyRoadland approach/cross/return 等純行進或安全穿越 phase 不加入
人為停留。不可達與 travel timeout 仍可跳過駐留並按各任務既有規則切點或完成。

### 任務到點與姿態

每個 tick 的姿態只讀一份內部 `TaskPostureIntent`。它不增加 YAML、ROS topic 或第二套導航計時；每個固定點
任務必須同時擁有當前輸出的 goal ID 與 goal 座標，才可以提供 intent。因此高優先級切點、動態 Chase 覆蓋
或舊的 `/ly/navi/reached` 都不會把已離開的點誤當作已到達。

- `SoftTransit`：Default regional、ProtectOutpost 的 `Travel`、ProtectHero、固定防守點、SpecialPatrol，
  在仍前往其自己 goal 時提供。正常請求 Move；原有評分已判為 Defense 時保留防禦優先，避免行進覆蓋受擊或
  資源安全策略。
- `SoftArrived`：Default 既有 hold、ProtectOutpost 的 `SearchHold`、固定防守點已到達、SpecialPatrol 的既有 hold。
  解除 Move 覆蓋後，沿用原有目標、受擊、血量、彈量評分選 Attack 或 Defense。
- `ProtectHeroDefenseHold`：ProtectHero 已到自己的守護目標時固定優先普通防守 `2`。
- `ProtectHeroEnhancedDefense`：只在 ProtectHero 駐留、`Tactical.ProtectHero.EnhancedDefense.Enable=true`，且 `DamageWindowMs` 內累積掉血達 `DamageThresholdHp` 時提供；TypeID 10 強防剩餘時間新鮮且大於 0 才申請 `5`，否則維持 `2`。行進仍是 `SoftTransit`，所以不會以半功率底盤趕路。已確認的 `5` 只在姿態 5 秒冷卻內暫緩 Recovery；冷卻結束仍低血/低彈時，或回讀不再確認強防時，既有 Move/Recovery 硬鏈路接管。
- `RecoveryEnhancedMove`：只在 Regional Recovery 尚在行進、己方 HP 回讀與強移額度 TypeID 10 都新鮮、HP `1..Tactical.EnhancedPosture.RecoveryMove.HealthThresholdHp`（預設 80）、強移額度大於 0、且沒有正常讀條復活 `0 -> 正數` 後的 `RespawnSuppressSec`（預設 30 秒）壓制時提供，才申請強化移動 `6`。其他 Recovery 情況都是普通 Move `3`；強移 ACK 重試耗盡會在本次 Recovery 內鎖定回退，離開 Recovery 才重置。
- `HardMove`：普通 Recovery、Buff、或新鮮 `/ly/navi/should_rotate=false`；不允許泛用資源輪換把它改走。
- `HardDefense`：受擊 burst；前哨 engagement lock 仍以既有 lock 優先。
- Buff/Outpost 保留原有專用語義；動態 Chase 不產生 arrived intent，避免移動目標的舊到點狀態造成停車。

當下位機 TypeID 10 的裁判 `0x020D sentry_info_3` 新鮮（`Posture.RefereeInfo3FreshMs=1500ms`）且
Move 剩餘時間介於 1 秒與 `RefereeRemainWarnSec=20s` 預警時，`SoftTransit` 會改用 Attack/Defense 中官方
剩餘時間較多的一檔，平分選 Defense；Move 已為 0 時仍維持 Move，因為 0 表示已弱化而非禁止使用，換成
Attack/Defense 會再降低移動能力。Transit 和 Hard 姿態會禁止 `PostureManager` 的泛用提前輪換覆蓋這個結果；
SoftArrived 則保留輪換，讓三種普通姿態的 180 秒非恢復預算仍能平衡。

賽規 5 秒是姿態切換冷卻；目前 `Posture.MinHoldSec=10` 是已切換姿態後的 manager 防抖，兩者都不是任務
導航停留時間。各任務仍完全沿用自己的既有 hold、切點與 preemption 規則。

任何 `4/5/6` 請求都需要新鮮 TypeID 10 的對應正數額度，driver 也在 topic 入口與實際串口 frame 組裝點以同一條件二次拒絕過期/0 額度的強化 frame（包含快取重發、同包其他欄位更新和重連），普通
`0..3` 不受影響。若 TypeID 7 的 `enhanced_posture=true` 與 TypeID 10 對應額度 0 連續超過
`Tactical.EnhancedPosture.ContradictionGraceMs=500ms`，BT 會隔離強化確認與新請求，取消強化 pending，並等待既有
5 秒姿態冷卻後由普通命令收斂；單幀不同步不觸發隔離。

decision trace 的 `posture.task_intent`、`task_source`、`task_owns_current_goal` 與
`tactical.protect_hero.enhanced_defense_*`（包括 `recovery_deferred`），以及 `[Posture]` 日誌中的 `scored`、`desired`、
`requested_enhanced`、`current_enhanced`、`enhanced_quarantined`、`recovery_enhanced_unavailable`、
`respawn_suppress_remaining_ms` 欄位可直接驗收本次仲裁。

## 到達與不可達

每個當前 goal 的到達/不可達已統一成 goal-scoped `GoalReachState`，任務層不應分別讀 raw topic。當前順序是：

1. `/ly/navi/reachable`
   - 新鮮 `false` 表示當前路徑不可達；
   - 狀態機會根據所在 phase 推進、切換或完成。
2. `/ly/navi/reached`
   - 新鮮 `true` 是外部導航到達來源。
   - 新鮮 `false` 只能表示外部導航還沒確認到達，不能作為 BT 永久未到達結論。
3. 自身坐標距離兜底
   - goal-start grace 之後，融合自身坐標首次連續進入到達半徑時，BT 先等待
     `DecisionAutonomy.NaviGoal.HighlandCompat.NearGoalConfirmWaitMs`；正式 regional 是 1500 ms。
   - 等待內新鮮 `/ly/navi/reached=true` 仍立即到達；等待到期且坐標仍在半徑內，才以坐標兜底
     判定 composite reached。離開半徑、坐標失鮮或切換 goal 都會清除本次等待。
4. timeout / watchdog
   - `GoalReachState.timeout` 用於顯示 goal timeout；watchdog 用於保護性 fallback。它們不應直接等同於物理 reached。

因此 `/ly/navi/position` 不替代 `/ly/navi/reached`，而是作為自身融合坐標來源進入同一個內部 reached contract。`EventGoalReached` / `EventGoalUnreachable` 與 watchdog 的到達/不可達 gate 都由 composite `GoalReachState` 生成；decision trace 同時記錄 `goal_reach_state` 和 raw `navi_status` 觀測欄位。

## 打斷規則

### 會被 Aim/Defense 直接打斷的任務

以下任務屬於低優先級任務，可以被 Buff/Outpost 模式或 regional defense 直接取消：

- MyBase
- MyPreRoadland
- CommonCentral

被取消時，BT 會清空當前 regional area task，重置 regional control override，並把 `FollowMode` 關掉。

Buff/Outpost、RegionalDefense 和 Special Patrol 的這類可恢復打斷會記為 `preempted`。它們釋放後，
Default 會先檢查原區域是否仍在 scope、啟用、資源健康且沒有 cooldown；全數成立才續走該區域，
若首次恢復選擇已不合格就立即丟棄恢復權、不重新按分數跳回先前區域。Recovery、timeout、unreachable 和 unhealthy 則是終止結果，不保留
Default 恢復權。

MyHighland 目前不在這個低優先級 aim/defense 取消集合裡。它仍然可能被 recovery 或其他外層明確清任務的鏈路清掉。

### PreRoadland

`MyPreRoadland` 是正式道路前段區域任務，取代已移除的 `Special.MiniRoadland`：

- 唯一導航入口為 `PreRoadland`，BaseGoalId 固定為 `25`；
- 到達後按程式預設保持 15 秒；
- 不繼承 ReadyRoadland 的強綁定穿越、FollowMode 或 FaceMode；
- 在 Default policy 中有獨立權重、當前/上一區懲罰與 retry/cooldown。

### ReadyRoadland 非強綁定階段

ReadyRoadland 非強綁定階段通常不直接取消，而是請求安全返回：

- Buff mode / Outpost mode 優先級更高；
- regional defense 發現威脅；
- recovery 請求；
- 血量/彈量數據低於門檻。

安全返回的意思是：狀態機切到返回 `CentralToBase` 的階段，而不是原地取消。

### ReadyRoadland 強綁定穿越段

這兩個 phase 是目前最強保護段：

- `ReadyRoadlandCrossToBaseToCentral`
- `ReadyRoadlandCrossToCentralToBase`

在這兩個階段：

- Buff/Outpost 模式不會取消它；
- regional defense 不會取消它；
- recovery 不會直接取消它；
- 任務會保持穿越控制權；
- `FollowMode / Rotate=0` 是否輸出由 `/ly/navi/should_rotate` 的新鮮值或任務本身的獨立字段決定；
- regional FaceMode 只有在對應 AreaTask `UseFaceMode=true` 時才會輸出；
- 普通裝甲鏈路是否巡航、鎖敵和開火不再由 `FollowMode` bit 決定。

強綁定穿越段只會因為以下條件結束：

- 到達穿越終點；
- 穿越終點不可達；
- 穿越超時。

## 前哨開局承諾

`Task.OutpostConfirm.OpeningHoldUntilWindowEnd=true` 且 `OpeningHoldSec>0` 時，
`IsOutpostOpeningHoldActive()` 是唯一開局 hard hold 判定：elapsed 在 `[0, OpeningHoldSec)` 內才生效，
因此預設 `120` 秒嚴格覆蓋 `0..119` 秒。有效期間 Outpost navigation ownership 固定為
`BuffOutpost`，Default、Regional idle patrol、Special 與 soft tactical 不可換點。這個承諾不壓過
前哨已毀、明確不可達、受擊 abort 或資源 gate；Hard Recovery 與己方 Base 的 RegionalDefense
硬威脅也保留既有接管權，避免為了駐守前哨犧牲回補或基地安全。hard hold 自身會取得開局 priority，
不依賴 `OpeningHighPriority`；該 YAML key 只控制不含 hard hold 的一般開局時間窗。

## 裝甲板選敵

新鲜 `/ly/aim/armor_targets` 是正式外部选靶的最高输入：BT 对未被 `AimTargetIgnore` 排除的
`ArmorType 0..7` 全部候选并选最近目标（距离相同时较小 ID 优先），不再用 `AimTargetPriority`
限制外部视觉数组可被选中的类型。正在生效的 Outpost engagement lock 仍保留前哨锁定，这是任务
安全语义；`AimTargetPriority` 只保留给没有新鲜外部视觉数组时的旧回退选择。
已选的新鲜目标进入 Attack 姿态评分，但 Recovery、Buff、受击 HardDefense、导航 HardMove、任务行进
与姿态切换冷却仍按既有优先级生效。

`DecisionAutonomy.AimTarget.Enable=true` 時，BT 會在這個優先級上疊加距離、低血量、當前目標保持、Hero/Sentry 偏置做打分。這不是舊的全局 utility strategy；`DecisionAutonomy.Enable=false` 時也可以只啟用這個局部選敵器。

臨時調試可使用 `src/behavior_tree/config/Aim.yaml`：預設
`Aim.Override.Enable=false`，開啟後只覆蓋明確提供的 `TargetPriority`/`TargetIgnore` 列表，
其餘欄位保留競賽 JSON。覆蓋順序是 JSON、其他模塊 YAML、Aim.yaml，最後才進行 Aim 合法性檢查。

延時和退化保護：

- 敵方血量只在 `HealthFreshTimeoutMs` 內用於低血量加分，避免官方裁判數據經串口/下位機延遲後把舊血量當新狀態。
- 敵方血量單次變成 0 不會立刻確認死亡；需要 0 血狀態持續到 `DeadHealthConfirmMs`，並且期間有 0 血包刷新，才會進 confirmed-dead hold。
- confirmed-dead hold 只保持 `DeadHealthHoldMs`，避免確認死亡後串口斷流導致永遠不打該目標。
- confirmed-dead 後只有在 `RespawnTransitionTimeoutMs` 內收到正血，才啟動復活無敵窗口；如果 0 血包中斷太久後才收到正血，視為資料鏈路延遲/恢復，不再補一段完整 30 秒無敵。
- 目標丟 1-2 幀時，在 `LostTargetHoldMs` 內保持當前目標，避免 detector 短暫掉包造成頻繁切換。
- 新目標分數未超過當前目標 `SwitchScoreMargin`，或還在 `MinSwitchIntervalMs` 內，保持當前目標。
- 打分器無候選或被關閉時，退回舊固定優先級和原有步兵 1/2 近距離/低血量判斷。

無敵排除：

- `ProcessData()` 先從 `/ly/detector/armors` 建 `hitableTargets`。
- 敵方血量從 0 恢復到非滿血時，BT 視為讀條復活，按配置的 `RespawnInvulnerableSec` 排除；Sentry 可用 `SentryRespawnInvulnerableSec` 單獨配置。
- 死亡確認耗掉的時間不會從復活無敵時間裡扣除；無敵計時從「confirmed dead 後 `RespawnTransitionTimeoutMs` 內第一次收到正血包」開始，因為上位機無法知道裁判端真實復活時刻。
- 2026 regional/league 正式配置默認 30 秒；如果要跑 2025 超級對抗賽哨兵規則，Sentry 應改成 60 秒。

## 控制輸出

Area task 使用既有 BT 控制鏈路輸出：

- 通過 `SetPositionByBaseGoal()` 發導航目標；
- `Tactical.yaml` 的 `DamageRotate` 管全域預設小陀螺檔位與受擊後 `0 -> 1 -> 2 -> 3` ramp；Castle/ProtectCastle 不會自行關閉受擊旋轉，最終壓制來源只會是顯式 `StopRotate`、Highland 兼容、最新 `/ly/navi/should_rotate=false` 或 FollowMode，日誌會記錄 `suppressed_by`。
- 通過 `FireCode.FollowMode` 控 FollowMode bit；
- 任一 BT 策略在本拍要求 `FollowMode=1` 時，最終都強制 `Rotate=0`，會壓過 Tactical ramp 與防守檔位；新鮮 `/ly/navi/should_rotate=false` 同樣以此規則輸出 Follow；目前不接管 regional FaceMode；
- 需要固定朝向時，通過 `/ly/face_mode/target_raw` 發 FaceMode 目標；
- 需要停火時，覆蓋本輪 fire 狀態。

FaceMode 目標格式：

```text
[official_map_x, official_map_y, map_z]  # cm
```

FaceMode 負責固定點朝向和接管雲台角度。FollowMode 只負責下發 firecode 語義位；小陀螺、雲台巡邏和停火分別由 `Rotate`、FaceMode、`SuppressFire`/開火邏輯控制。

## League / Regional 邊界

`CompetitionProfile=league` 和 `CompetitionProfile=regional` 已在策略入口分開：

- `league`：`SelectStrategyMode()` 固定為 `LeagueSimple`，Tactical 只會走 `SetPositionLeagueSimple()`。
- `regional`：`SelectStrategyMode()` 固定為 `Regional`，由 Default/AreaManager 產生正式區域任務；Tactical 不會再調用舊單策略點表。

因此正式 regional 的導航點來源應只來自：

- `Hard` recovery / ReadyRoadland hard lock；
- `Task` 的 Highland transition、watchdog；
- `Tactical` 的 `RegionalDefense`、ProtectHero、Buff/Outpost 任務站位或 Chase 追擊。
- `Default` 選中並執行的 AreaManager 大區域任務。

如果這些都沒有輸出，`Finalizer` 只同步策略層 blackboard，不再做舊點表兜底。歷史單策略邏輯已從 live code 移除；需要對照時只看 `docs/record/` 裡的歷史記錄。

## 當前完整性

以目前 regional 基本任務來看，框架已經完整：

- 我方 Base 巡遊由 MyBase 管；
- 我方 Highland 進入、駐守、離開由 MyHighland 管；
- 我方 PreRoadland 前段到點保持由 MyPreRoadland 管；
- 我方 ReadyRoadland 強綁定穿越、安全返回由 MyReadyRoadland 管；
- 中場健康巡邏由 CommonCentral 管。

敵方 Base/Highland/PreRoadland/ReadyRoadland 之後如果有明確任務，再單獨加狀態機即可。目前不寫不影響這套基本 regional 框架。
