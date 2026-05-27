# Regional 決策框架說明

Updated: 2026-05-27

本文記錄目前 `behavior_tree` 裡 regional 決策的區域狀態機框架：它會做哪些任務、怎麼啟動、怎麼判斷到達、會輸出什麼控制，以及哪些階段會被高優先級邏輯打斷。

## 覆蓋範圍

目前已完成的基本區域狀態機：

- `MyArea.Base`
- `MyArea.Highland`
- `MyArea.Roadland`
- `CommonArea.Central`

目前沒有寫 `EnemyArea.Base / EnemyArea.Highland / EnemyArea.Roadland` 的獨立狀態機，因為當前基本 regional 任務不需要它們。Central 任務裡會走到敵方側的中場點，但它仍然屬於 `CommonArea.Central`，不是敵方大區域任務。

主要相關文件：

- `src/behavior_tree/config/AreaManager.yaml`
- `src/behavior_tree/config/Base.yaml`
- `src/behavior_tree/config/Task.yaml`
- `src/behavior_tree/include/AreaManager.hpp`
- `src/behavior_tree/src/AreaManager.cpp`
- `src/behavior_tree/include/DefaultStrategyManager.hpp`
- `src/behavior_tree/src/DefaultStrategyManager.cpp`
- `src/behavior_tree/include/EventManager.hpp`
- `src/behavior_tree/src/EventManager.cpp`
- `src/behavior_tree/include/StrategyManager.hpp`
- `src/behavior_tree/src/StrategyManager.cpp`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/module/Area.hpp`

## Regional 邏輯總覽

本節只列程式裡已存在的 regional 邏輯，不代表當前 YAML 一定全部開啟。

Regional 不是單一點表，而是分層策略：

- `Hard`：最高優先級，先處理低血/低彈回 `Recovery`，以及 Roadland 強綁定穿越段。
- `Task`：處理 Highland 兼容過渡和導航 watchdog 等支援任務，不再擁有基本大區域狀態機。
- `Tactical`：處理 Buff、RegionalDefense、ProtectHero、Outpost 和 watchdog fallback。
- `Special`：可開關的專項巡察層，目前包含兩點線段 Patrol 和 `MiniRoadland` 偵察駐守，優先級低於 Tactical、高於 Default。
- `Default`：沒有事件、沒有任務、沒有 Buff/Outpost 時，按大區域候選分數選並持續 tick `MyBase / MyHighland / MyRoadland / CommonCentral`。
- `Finalizer`：只做策略層狀態同步，不再做舊點表 fallback。

Regional 目前已有的主要邏輯：

- 回補/回基地：低血或低彈優先去 `Recovery`；這層高於 RegionalDefense。非 league regional 下，已在 `Recovery` 且血量未回到門檻時會繼續守住 Recovery。
- Default 大區域任務：候選包含 `MyBase`、`MyHighland`、`MyRoadland`、`CommonCentral`；評分會看血量/彈量新鮮度、資源門檻、距離、目前區域、上一個區域、任務冷卻和失敗重試。
- `MyBase` 任務：在己方 Base 候選點中按 `Base.yaml` 權重和自身距離評估選點；候選包含四個 Castle 邊點、`HoleRoad`、`OutpostGuard`、`BuffOutpost`。完成 `MaxPatrolSteps` 後退出，交回 Default scorer 重新評估下一個大區域。
- `MyHighland` 任務：`Highland` approach -> `Highland` hold -> `BuffShoot` -> `BuffShoot` hold -> `HoleRoad` 離開；approach/leave 仍是地形兼容階段，但正式配置下 Follow/Rotate 兼容交給 `/ly/navi/should_rotate`。
- `MyRoadland` 任務：`CentralToBase -> BaseToCentral -> BaseToCentral hold -> CentralToBase return`；穿越段仍是強綁定調度段，不能被普通高優先級邏輯直接打斷。`GuardHoldSec` 到時或資源不健康時會返回並完成任務。正式配置下它不再靠 AreaTask 自己長時間開 `FollowMode / FaceMode` 做地形兼容，Follow/Rotate 由 `/ly/navi/should_rotate` 接管。
- `CommonCentral` 任務：中場巡邏路線是 `my OutpostArea -> my RightShoot -> my BuffAround2 -> my LeftShoot -> my OutpostShoot -> enemy RightShoot -> enemy OccupyArea -> enemy OutpostShoot`，啟動時也按自身位置選最近點；完成 `MaxPatrolSteps` 後退出，交回 Default scorer 重新評估下一個大區域。
- RegionalDefense：用官方敵方位置和 `event_data` 做戰術防守；敵方進我方 Base/Highland/Roadland/CommonCentral 或己方堡壘增益點 `2/3` 都可觸發。
- 己方堡壘增益點 `2/3`：不去 `Castle`，只在 `CastleLeft1 / CastleLeft2 / CastleRight1 / CastleRight2` 搜索；若 Base 大區敵方數達門檻且普通裝甲目標已鎖定並允許開火，才原地停車、最高小陀螺開火；長時間無官方敵方位置且無視覺目標會退化忽略一段時間。
- Recovery：Hard 層先回 `Recovery` 點；到達後若 3 秒內血量/彈量沒有回升，會在己方 `Recovery` 子區域內切換中心探測點，避免卡在補給區邊緣。
- Buff：由能量機關裁判狀態、sentry info、timer、damage abort 和 timeout 決定是否進 `AimMode::Buff`；戰術站位使用 `BuffOutpost`，FaceMode 對己方目標側。
- Outpost：正式入口不依賴 `op_hp`，由血量/彈藥門檻、時間窗、damage abort、目標不可達狀態和 `Task.OutpostConfirm.VisualScoutWithoutHp` 決定是否去 `BuffOutpost` 偵查；Travel 階段保持選前哨，不讓遠距離普通車體接管 `/ly/aim/result`，但進入 `VisualScoutFaceDistanceCm` 前仍用普通裝甲視覺和 Move 姿態。進入該距離後開前哨視覺、敵方前哨 FaceMode 和 Attack 姿態，不再強依賴 `/ly/navi/reached=true`。120 秒時間窗內是高優先級任務，但己方 Base 有敵方時 RegionalDefense 可打斷；120 秒後按 `PostWindowScoutIntervalSec` 低優先級回 `BuffOutpost`，接近後用 `PostWindowScoutHoldMs` 短 FaceMode 偵查。普通裝甲目標若有效且不超過 `ArmorWarningDistanceCm`，會先打車；目標消失後若前哨 gate 仍允許，會按 `PostArmorFaceSearchMs` 回前哨 FaceMode 搜索。`op_hp` 接口保留，若它新鮮且為 0，可提前判定敵方前哨已毀並跳過任務。
- Navi progress watchdog：檢測 goal 不可達或長時間無位移，按當前目標區域選 fallback 點。
- Special Patrol：由 `src/behavior_tree/config/Special.yaml` 控制；啟用後在 Tactical 無事件時巡己方 `CentralLeft` 線的 A/B 端點。默認 `GoalHoldSec=0`，到點即切下一端；`SuppressChase=true`，鎖到目標時不追擊、不邊走邊打，而是把導航目標壓到當前自身坐標。
- Special MiniRoadland：由 `src/behavior_tree/config/Special.yaml` 控制；啟用後在 Tactical 無事件時去己方 `MiniRoadland` 點偵察駐守，且不受 `Area.MyArea.Roadland=false` 影響。
- Regional idle patrol：預留空閒巡邏，默認候選是 `HoleRoad / Castle / CastleRight2 / CastleRight1 / CastleLeft1 / CastleLeft2`。

Regional 裡常見控制語義：

- `AimMode::RotateScan`：普通裝甲搜索/打車。
- `AimMode::Buff`：打符視覺鏈路。
- `AimMode::Outpost`：前哨視覺鏈路。
- `FollowMode`：只表示下發到 `FireCode.FollowMode` 的語義位；BT 不再因這個 bit 自動停小陀螺、停巡邏或停開火。
- `FaceMode`：雲台朝固定區域/點接管，可按配置停火；它本身不等於停小陀螺。
- Chase：鎖到目標後可發布追擊目標或速度；是否使用取決於配置，但鏈路已存在。

## Strategy 分層

`Scripts/main.xml` 的主決策入口現在先跑高優先級策略層，再整理目標，接著跑 Tactical 和 Special，最後由 Default 兜底。每 tick 依序執行：

```text
EvaluateEvents -> Hard -> Task -> PreprocessData -> SelectAimTarget -> Tactical -> Special -> Default -> Finalizer
```

各層的責任是：

- `EvaluateEvents`：語義整理層，只把裁判資料、視覺鎖定、受擊、導航狀態和 RegionalDefense 威脅收斂成 `EventSnapshot`。這層不發導航、不改火控、不接管輸出。
- `Hard`：最高優先級保護，處理 recovery/補血補彈和 Roadland 強綁定穿越段。Roadland 強綁定段在這層 hard lock，避免被戰術層中途搶走。
- `Task`：只保留 Highland 兼容過渡和導航 watchdog 這類支援任務；不再 tick Highland/Base/Roadland/Central 基本大區域狀態機。
- `PreprocessData / SelectAimTarget`：在 Tactical 前整理可打目標、官方坐標和本 tick `targetArmor`，讓戰術層使用最新目標資料。
- `Tactical`：regional 只保留明確戰術 overlay：`RegionalDefense`、ProtectHero、Buff/Outpost 任務站位、Chase 追擊和導航 watchdog；不再調用舊單策略點表。`LeagueSimple` 只在 `CompetitionProfile=league` 時使用，Showcase 只在明確 showcase 配置時使用。
- `Special`：可選專項層；`Special.Patrol.Enable=true` 時巡己方 `CentralLeft` 線，`Special.MiniRoadland.Enable=true` 時下發己方 `MiniRoadland` 偵察點。它直接走 base goal，不走 Default 的大區域 scope；但更高層回補、回防、前哨、打符仍會先接管。Special Patrol 默認抑制 Chase。
- `Default`：無特別事件時的底層決策，按 `AreaManager.DefaultPolicy` 對已啟用的大區域做資源門檻、距離、目前區域、上次任務結果、冷卻和重試評分，再啟動 AreaManager 任務。每個 Default 區域任務都必須有完成/退出條件；任務完成後回到 scorer 重新評估，不寫死下一個大區域順序。沒有可用區域時不再 fallback 到任何舊點表。`RegionalIdlePatrol` 點表不再是正式 regional 的 Default 入口。
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

## 數據來源

Regional 任務主要使用這些導航/定位輸入：

- `/ly/navi/reached`：外部導航對當前 goal 的到達來源；它不是 BT 內部最終 reached 事實。
- `/ly/navi/reachable`：當前 goal 是否有有效路徑，主不可達判斷。
- `/ly/navi/should_rotate`：外部導航區域兼容控制；`true` 恢復 BT 正常小陀螺/巡邏，`false` 關小陀螺並請求 `FollowMode`。
- `/ly/friend/uwb_pos`：雷達/UWB 推出的己方哨兵自身官方地圖坐標，單位 cm。
- `/ly/navi/position`：導航/TF 推出的自身官方地圖坐標，單位 cm。
- `/ly/position/data`：官方/雷達定位坐標。

自身哨兵坐標由 `AreaManager.SentryPositionFusion` 統一融合後寫入 `friendRobots[Sentry].position_`。默认 `Mode=priority`，優先級是 `/ly/friend/uwb_pos`、`/ly/navi/position`、`/ly/position/data` 裡 `friendcarid == Sentry`；也可以改成 `Mode=weighted`，按各 source 的 `Weight` 對新鮮坐標做加權平均。這個融合後坐標主要用於：

- 選最近的巡邏起點；
- 判斷自己目前在哪個大區域；
- 作為 BT 內部 composite reached 的坐標距離來源之一。

也就是說，`/ly/navi/reached`、`/ly/navi/reachable` 和自身融合坐標應該一起進入 BT 的 goal-scoped reached 評估。`/ly/navi/reached=true` 可作為高優先級正向來源；`/ly/navi/reached=false` 不能永久否決坐標距離兜底。

## `/ly/navi/should_rotate` 的作用

`/ly/navi/should_rotate` 是外部導航給 BT 的地形兼容控制信號，類型是 `std_msgs/msg/Bool`。這裡的 `Rotate` 指 `FireCode.Rotate`，也就是下發給下位機的小陀螺檔位/rotate level，不是雲台 yaw 角速度。它只管本輪火控裡的 `FollowMode`、小陀螺 `Rotate` 和 regional 區域兼容用的 FaceMode 釋放，不負責選導航點，也不負責啟動、取消或推進任何 AreaManager task。

配置入口是 `src/behavior_tree/config/NaviRotateControl.yaml`：

- `Enable=true` 時啟用這條外部控制鏈。
- `FreshTimeoutMs=500`，超過這個時間沒有新消息時按 `DefaultIsRotate=true` 處理，避免舊的 `false` 長時間卡住底盤。
- `DefaultIsRotate=true` 表示沒有新鮮信號時回到 BT 正常小陀螺/雲台巡邏策略。
- `ForceFollowModeWhenFalse=true`：收到新鮮 `false` 時，只在最終下發 `FireCode` 前臨時合併 `FollowMode=1`；這個 bit 不會觸發 BT 停火/停巡航分支。
- `StopRotateWhenFalse=true`：收到新鮮 `false` 時，本輪強制 `Rotate=0`。
- `ClearFollowModeWhenTrue=true`：收到新鮮 `true` 時，釋放 `FollowMode`，回到 BT 正常 rotate 策略。
- `ClearRegionalFaceModeWhenTrue=true`：收到新鮮 `true` 時，清掉 regional area-task 兼容用的 FaceMode；Buff/Outpost 自己的固定朝向不靠這個開關清。

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
  -> 小陀螺回到普通策略：平時 1 檔，受擊後可升檔
  -> 雲台巡邏/鎖敵/開火回到當前任務本來的狀態
```

所以正式當前配置下，`should_rotate=false` 表示「外部導航請求下位機 FollowMode bit + 關小陀螺檔位」。BT 仍會該巡航就巡航，該鎖敵開火就開火；`FollowMode` bit 本身只進最終 firecode 字段，不再觸發 BT 停火/停巡航分支。

它和 `MyRoadland` 強鎖不是同一層：

- `MyRoadland` 強鎖是任務調度保護：在 `RoadlandCrossToBaseToCentral`、`RoadlandCrossToCentralToBase` 兩個 phase 裡，Buff/Outpost、RegionalDefense、Recovery 不會直接取消這個穿越段，要等到達、不可達或超時。
- `/ly/navi/should_rotate` 是控制輸出兼容：告訴 BT 這一段要不要停小陀螺、開 FollowMode。它不會讓 Roadland task 進入或退出強鎖，也不會改變 Roadland phase。

因此現在的責任邊界是：AreaManager 決定「我要不要做 MyRoadland，以及是不是處於不可讓出的 crossing phase」；外部導航通過 `/ly/navi/should_rotate` 決定「此刻底盤是否允許正常小陀螺」。這樣 Castle、Roadland、Highland 等地形細節可以放在導航側維護，BT 只保留任務優先級和戰術語義。

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

目前 `src/behavior_tree/config/AreaManager.yaml` 的基本區域配置是：

```yaml
AreaManager:
  Switch_Point: false
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
        Priority: 1
        Weight: 0.7
      Navi:
        Enable: true
        Priority: 2
        Weight: 0.8
  RegionalAreaTask:
    Enable: true
    MyBase:
      Enable: true
    MyHighland:
      Enable: true
    MyRoadland:
      Enable: true
    CommonCentral:
      Enable: true
```

`src/behavior_tree/config/AreaManager.yaml` 不再默認寫 `Area.MyArea/EnemyArea/CommonArea` 的區域開關，避免它把不同 `bt_config_file` 裡的區域選擇全部覆蓋成同一套。正式 regional 和單區域 areatest 的「哪些區域可選」仍由對應 `ConfigJson` 裡的 `DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 控制；DefaultPolicy 只會在這些已啟用區域內挑候選，JSON 裡為 `false` 的區域不會因為血量健康或權重高而被選中。`AreaManager.yaml` 保留 `Switch_Point`、`SentryPositionFusion`、區域狀態機任務時序，以及 DefaultPolicy 的門檻、權重、冷卻和重試參數。`Base.yaml` 保留 MyBase patrol 的候選點權重和距離懲罰。`Task.yaml` 只管這局是否允許 `Task.Buff / Task.Outpost`，會覆蓋 JSON 裡同名字段。

`Switch_Point=true` 時只交換 `Area.hpp` 裡紅/藍官方點位和區域邊界查找結果，不交換 `team`、敵我語義或導航 goal ID。這是給導航零點/物理場地方向反了時使用的點位查找開關。

重要健康門檻：

- `DefaultPolicy.Health.MyAreaHpMin`：我方 Highland/Roadland 的底層選區 HP 門檻，默認 250。
- `DefaultPolicy.Health.CommonCentralHpMin`：Central 底層選區 HP 門檻，默認 300。
- `DefaultPolicy.Health.EnemyAreaHpMin`：敵方區域預留 HP 門檻，默認 350；目前敵方區域狀態機尚未接入 Default 候選。
- `DefaultPolicy.Ammo.*`：與 HP 對應的彈量門檻。
- `MyRoadland.HealthyHpMin`
- `MyRoadland.HealthyAmmoMin`
- `CommonCentral.HealthyHpMin`
- `CommonCentral.HealthyAmmoMin`

Roadland 用這些門檻決定是否離開駐守點並安全返回。Central 則要求啟動時血量/彈量數據新鮮且健康；任務中如果新鮮數據變成不健康，就完成並釋放控制。

DefaultPolicy 的當前選區規則：

- Area scope 是硬門檻：`MyArea / EnemyArea / CommonArea` 關掉的區域永遠不進候選。
- `MyBase` 需要新鮮血量/彈量並達到我方區域門檻，健康時才會啟動基地巡遊。
- `MyHighland` 需要新鮮血量/彈量並達到我方區域門檻。
- `MyRoadland` 需要同時滿足 DefaultPolicy 我方門檻和 `MyRoadland.Healthy*` 門檻。
- `CommonCentral` 需要同時滿足 DefaultPolicy Central 門檻和 `CommonCentral.Healthy*` 門檻。
- 候選分數會扣除距離、目前所在同區域、上次已選區域；Highland 任務正常完成後會臨時提高 MyBase/MyRoadland 分數。
- `unreachable / timeout / unhealthy / canceled` 會進入 failure/unreachable cooldown，連續失敗數達到 `MaxRetry` 時使用更長的 unreachable cooldown。

## RegionalDefense

RegionalDefense 是事件驅動戰術層，優先級高於 Default。敵方位置判斷只使用 `/ly/position/data` 寫入的官方場地坐標，不使用 map/odom 坐標混判；AreaManager 用 `Area.hpp` 官方點位區域邊界判斷敵方是否進入我方 Base/Highland/Roadland 或公共 Central。另有一個裁判事件來源：`/ly/game/event_data.self_fortress_gain_point_status == 2/3` 時，視為己方堡壘增益點有敵方占領，進入硬防守搜索。ProtectHero 的英雄保護條件會在 RegionalDefense 之前檢查；只有未命中英雄保護時，普通 RegionalDefense 才接管。

當前防守搜索規則：

- 敵方進入我方 Base：優先去 `Castle`，再 fallback 到左右 Castle 點。
- `/ly/game/event_data` 顯示己方堡壘增益點被對方或雙方占領：不進 `Castle`，只在 `CastleLeft1 / CastleLeft2 / CastleRight1 / CastleRight2` 裡按自身位置選最近點搜索。默認仍沿用普通裝甲模式邊走邊打；若己方 Base 大區的新鮮官方敵方位置數達到 `RegionalDefense.FortressStandEnemyCountMin`，且普通裝甲目標已鎖定並允許開火，則把底盤速度壓為 0、小陀螺覆蓋到最高檔站樁開火。
- 我方 Highland 和 Roadland 同時有敵方：優先去 `Castle`。
- 敵方進入我方 Roadland：去 `CastleRight2 -> CastleRight1 -> Castle` 搜索。
- 敵方進入我方 Highland：去 `HoleRoad -> Highland -> Castle` 搜索，先利用 HoleRoad 視野，再進 Highland。
- 敵方在公共 Central：去 `HoleRoad -> Castle` 搜索。

搜索點會尊重 area scope，但不啟動 Base/Highland/Roadland 的 AreaManager 區域任務；它只做 scope 檢查、必要的 Highland transition，然後直接下導航點。`RegionalDefense.SearchHoldSec` 和 `RegionalDefense.SearchNoTargetSec` 控制「一直找不到」後切下一個搜索點；找不到的判斷使用 autoaim 最近有效目標時間，不混用 buff/outpost 目標。堡壘增益點事件還有退化保護：若連續 `RegionalDefense.FortressNoContactDegradeSec` 秒沒有己方 Base 大區官方敵方位置、也沒有普通裝甲視覺目標，會在 `RegionalDefense.FortressDegradeCooldownSec` 秒內暫時不把 `2/3` 當硬威脅。

## ProtectHero

ProtectHero 是 Tactical 層的己方英雄保護點位。比賽開始 `HeroProtection.StartElapsedSec` 秒後，如果己方 Hero 的官方坐標新鮮、血量不是已知 0，且落在己方 Highland 大區或 `ProtectHero` 子區域內，BT 會評估 RegionalDefense 威脅。當我方 Base 和我方 Highland 同時有新鮮敵方官方坐標時，ProtectHero 會優先於普通 RegionalDefense 下發 `HeroProtection.GoalBaseId`，默認為 `Highland`，並用 `HeroProtection.HoldSec` 控制駐守重發週期。

ProtectHero 觸發後會保持保護狀態；只要仍有 RegionalDefense 敵情就刷新保護保持時間。當連續 `HeroProtection.NoEnemyReleaseSec` 秒沒有 RegionalDefense 敵情時，保護狀態釋放，後續 tick 交回普通 RegionalDefense、Buff/Outpost、watchdog 或 Default 區域任務。

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

上面的 `FollowMode bit / FaceMode / stop-fire` 是三個獨立輸出。正式配置裡 `NaviRotateControl.Enable=true` 時，進出 Highland 時實際是否停小陀螺主要由外部導航的 `/ly/navi/should_rotate` 決定。

進入 Highland 和離開 Highland 時：

- 若未啟用 `NaviRotateControl`，AreaTask 可按結果寫 `FollowMode`；
- 若啟用 `NaviRotateControl`，AreaTask 不直接寫 `FollowMode`，由 `/ly/navi/should_rotate=false` 觸發；
- 如果 `MyHighland.UseFaceMode=true`，才會發布 regional FaceMode 目標；
- 兼容 phase 仍會停火。

Highland 巡邏和 BuffShoot 駐守時：

- `FollowMode` 關閉；
- 雲台巡邏、小陀螺、開火回到普通 BT 控制。

### MyBase

觸發條件：上游選中的 goal 精確屬於我方 Base 大區域，並且當前不在我方 Highland 裡。

候選點：

```text
CastleLeft1 / CastleLeft2 / CastleRight2 / CastleRight1 / HoleRoad / OutpostGuard / BuffOutpost
```

每次啟動或切下一個巡邏點時，會按 `src/behavior_tree/config/Base.yaml` 裡的權重和自身距離評分，不按固定順序輪。拿不到自身坐標時只用權重評估；全部候選不可用時，保守回到 `CastleLeft2`。Default 啟動 MyBase 前會先檢查血量/彈量是否新鮮且達到 `DefaultPolicy` 我方區域門檻。

MyBase 本身不開 `FollowMode`，也不開 `FaceMode`，就是普通基地巡遊狀態機。

### MyRoadland

觸發條件：上游選中的 goal 精確屬於我方 Roadland 大區域，並且當前不在我方 Highland 裡。

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

如果 `MyRoadland.UseFaceMode=true`，Roadland 的 FaceMode 目標就是當前穿越終點；正式配置目前是 `false`，地形朝向/跟隨主要交給外部導航的 `/ly/navi/should_rotate`：

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

## 到達與不可達

每個當前 goal 的到達/不可達應統一成 goal-scoped state，而不是讓各個任務分別讀 raw topic。當前較完整的順序是：

1. `/ly/navi/reachable`
   - 新鮮 `false` 表示當前路徑不可達；
   - 狀態機會根據所在 phase 推進、切換或完成。
2. `/ly/navi/reached`
   - 新鮮 `true` 是外部導航到達來源。
   - 新鮮 `false` 只能表示外部導航還沒確認到達，不能作為 BT 永久未到達結論。
3. 自身坐標距離兜底
   - goal-start grace 之後，如果融合自身坐標進入到達半徑，BT 可判定 composite reached。
4. timeout / watchdog
   - 用於任務保護性推進或 fallback；它是 `done/timeout`，不應直接等同於物理 reached。

因此 `/ly/navi/position` 不替代 `/ly/navi/reached`，但必須進入同一個內部 reached contract。後續應把 `EventGoalReached` / trace 裡的 ambiguous bool 改成 composite `GoalReachState`，至少記錄 status、reason、distance、external reached/reachable freshness。

## 打斷規則

### 會被 Aim/Defense 直接打斷的任務

以下任務屬於低優先級任務，可以被 Buff/Outpost 模式或 regional defense 直接取消：

- MyBase
- CommonCentral

被取消時，BT 會清空當前 regional area task，重置 regional control override，並把 `FollowMode` 關掉。

MyHighland 目前不在這個低優先級 aim/defense 取消集合裡。它仍然可能被 recovery 或其他外層明確清任務的鏈路清掉。

### Roadland 非強綁定階段

Roadland 非強綁定階段通常不直接取消，而是請求安全返回：

- Buff mode / Outpost mode 優先級更高；
- regional defense 發現威脅；
- recovery 請求；
- 血量/彈量數據低於門檻。

安全返回的意思是：狀態機切到返回 `CentralToBase` 的階段，而不是原地取消。

### Roadland 強綁定穿越段

這兩個 phase 是目前最強保護段：

- `RoadlandCrossToBaseToCentral`
- `RoadlandCrossToCentralToBase`

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

## 裝甲板選敵

普通裝甲板模式仍以 `AimTargetPriority` 作基礎優先級，目前正式配置為：

```text
Hero -> Infantry1 -> Infantry2 -> Sentry -> Engineer
```

`DecisionAutonomy.AimTarget.Enable=true` 時，BT 會在這個優先級上疊加距離、低血量、當前目標保持、Hero/Sentry 偏置做打分。這不是舊的全局 utility strategy；`DecisionAutonomy.Enable=false` 時也可以只啟用這個局部選敵器。

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
- 通過 `FireCode.FollowMode` 控 FollowMode bit；
- 若啟用 `NaviRotateControl.yaml`，新鮮 `/ly/navi/should_rotate` 會接管區域兼容用的 FollowMode/小陀螺/regional FaceMode 釋放；
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

- `Hard` recovery / Roadland hard lock；
- `Task` 的 Highland transition、watchdog；
- `Tactical` 的 `RegionalDefense`、ProtectHero、Buff/Outpost 任務站位或 Chase 追擊。
- `Default` 選中並執行的 AreaManager 大區域任務。

如果這些都沒有輸出，`Finalizer` 只同步策略層 blackboard，不再做舊點表兜底。歷史單策略邏輯已從 live code 移除；需要對照時只看 `docs/record/` 裡的歷史記錄。

## 當前完整性

以目前 regional 基本任務來看，框架已經完整：

- 我方 Base 巡遊由 MyBase 管；
- 我方 Highland 進入、駐守、離開由 MyHighland 管；
- 我方 Roadland 強綁定穿越、安全返回由 MyRoadland 管；
- 中場健康巡邏由 CommonCentral 管。

敵方 Base/Highland/Roadland 之後如果有明確任務，再單獨加狀態機即可。目前不寫不影響這套基本 regional 框架。
