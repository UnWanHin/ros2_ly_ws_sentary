# Strategy Layers And Navigation Reach

Updated: 2026-05-13

本文記錄 `behavior_tree` 裡 regional 策略分層、各層目前實際做的事情，以及導航到達判斷 `/ly/navi/reached`、坐標距離兜底和 progress watchdog 的關係。

## 結論

你的理想分層應該理解成：

```text
Hard -> Task -> Tactical -> Special -> Default
```

實際 XML 裡會插入資料準備節點，所以目前 live 順序是：

```text
UpdateGlobalData
-> EvaluateEvents
-> SelectAimMode
-> SelectStrategyMode
-> Hard
-> Task
-> PreprocessData
-> SelectAimTarget
-> Tactical
-> Special
-> Default
-> Finalizer
-> SelectPosture
-> PublishAll
```

`PreprocessData / SelectAimTarget` 不是策略優先級，它們是 Tactical 前的資料整理。因為 Tactical 裡 Chase、RegionalDefense、ProtectHero 等會用本 tick 的目標、坐標、敵情，所以它們必須在 Tactical 前。

現在大方向是對的：Hard 最高，Task 支援任務次之，Tactical 戰術事件再來，Special 是可開關的專項巡察層，Default 是兜底的大區域任務。

## 各層責任

### Hard

入口：`StrategyManager::RunHard()`

目前做兩類事情：

- Recovery：`CheckPositionRecovery()`，低血/低彈回 `Recovery`，且高於所有 regional/tactical 行為。Regional 模式下若已到達但 3 秒內血量/彈量沒有回升，會在己方 `Recovery` 子區域內切換中心探測點。
- Roadland 強綁定穿越段：如果 active regional task 是 `MyRoadland`，且當前 phase 不能讓出控制，會在 Hard 層繼續 `TickRegionalAreaTask()`。

Roadland 這裡看起來像 Default task，但它在不可讓出的穿越段會被提升到 Hard。這是地形/安全約束，不是普通巡邏優先級。

### Task

入口：`StrategyManager::RunTask()`

目前做支援型導航任務：

- Highland transition：`TickNaviAreaTransition()`。
- Navi progress watchdog：`TickNaviProgressWatchdog()`。

Task 現在不應該擁有 Highland/Base/Roadland/Central 這些普通大區域狀態機。這些已經搬到 Default。

要注意：`TickNaviProgressWatchdog()` 目前在 Task 和 Tactical 裡都會被嘗試一次。實際上先觸發的一層會 mark handled，後面的層不再接管。從架構清晰度看，watchdog 更像 Task/安全支援任務，後續可以考慮只保留一個 owner。

### Tactical

入口：`StrategyManager::RunTactical()`

目前做事件/戰術 overlay：

- `LeagueSimple` profile 的固定 league 行為。
- showcase patrol。
- Buff 模式站位。
- Outpost opening high priority。
- ProtectHero。
- RegionalDefense。
- Outpost visual scout travel。
- Outpost aim mode。
- Navi progress watchdog fallback。
- Chase tactical。

Tactical 應該高於 Default。也就是有敵情、防守、保護英雄、前哨、打符、追擊這類明確戰術事件時，不應該被 Base 巡邏、Highland 駐守、Roadland 駐守、Central 遊走頂掉。

### Special

入口：`StrategyManager::RunSpecial()`

目前做可開關的專項行為：

- 配置在 `src/behavior_tree/config/Special.yaml`。
- 優先級低於 Tactical，高於 Default。
- `Patrol`：在己方 `CentralLeft` 線的 A/B 端點之間巡邏；默認抑制 Chase，鎖到目標時停在當前坐標打，不邊走邊追。
- `MiniRoadland`：只去己方 `MiniRoadland` 點；不會因敵方 Roadland scope 變化而去敵方點。
- 這些任務直接下發 Special base goal，因此可無視對應 Default 大區域 scope；但不會無視更高層的 Recovery、RegionalDefense、ProtectHero、Buff/Outpost 等 Tactical/Task/Hard 行為。

### Default

入口：`StrategyManager::RunDefault()`

目前做普通 regional 兜底：

- 如果已經有 active regional area task，就繼續 `TickRegionalAreaTask()`。
- 如果沒有 active task 且 `IsDefaultRegionalDecisionReady()` 通過，就用 `TrySetDefaultRegionalGoal()` 按 DefaultPolicy 選一個大區域任務。

Default 擁有這些普通大區域行為：

- `MyHighland`：Highland 駐守/巡邏流程。
- `MyBase`：Base 城堡點巡邏。
- `MyRoadland`：Roadland 駐守/穿越流程，其中不可讓出的穿越段會臨時由 Hard 接管。
- `CommonCentral`：Central 遊走。

Default 是底層行為。它只應該在 Hard/Task/Tactical/Special 都沒接管時輸出導航目標。

### Finalizer

入口：`StrategyManager::RunFinalizer()`

只同步策略層狀態到 blackboard。它不應該再做舊點表 fallback。

## `/ly/navi/reached` 的可信度和使用順序

訂閱點：`SubscribeMessage.cpp` 裡的 `/ly/navi/reached` callback。

收到消息後只保存三件事：

- `naviReach = msg->data`
- `hasReceivedNaviReach_ = true`
- `lastNaviReachRxTime_ = now`

真正使用時會走 `IsBaseGoalArrived()`，順序是：

1. goal id 必須有效。
2. `/ly/navi/reachable` 如果對當前 goal 新鮮且為 `false`，直接判定未到達。
3. `/ly/navi/reached` 如果對當前 goal 新鮮，就完全相信它：`true` 就到達，`false` 就未到達。
4. 只有 `/ly/navi/reached` 沒有新鮮值，且新 goal 下發已超過 `DecisionAutonomy.NaviGoal.DistanceFallbackGraceMs` 後，才用自身融合坐標和 goal 坐標距離做兜底。默認 grace 是 3000 ms。

`/ly/navi/reached` 的新鮮條件不是單純 2 秒內收到就算，它還要求：

- BT 記錄的外部導航狀態 goal 已初始化。
- 狀態對應的 goal id 等於當前 `naviCommandGoal`。
- 狀態對應的 goal 坐標等於當前 goal 坐標。
- callback 時間晚於本次 goal start time。
- callback 距今不超過 `kNaviExternalStatusTimeoutMs`，目前是 2000 ms。

所以 `/ly/navi/reached` 是主判斷，而且是可信的。只要它新鮮，坐標距離不會覆蓋它。

## 20cm 坐標兜底

`IsBaseGoalArrived()` 的最後兜底是自身坐標距離：

- 需要自身哨兵坐標新鮮。
- 坐標不能是 0/0。
- 距離 goal 小於等於 `DecisionAutonomy.NaviGoal.HighlandCompat.ArriveDistanceCm`。
- regional competition 和 areatest 配置裡這個通常是 20 cm。

現在的行為是：

- 如果 `/ly/navi/reached` 新鮮且為 `false`，即使坐標進了 20 cm，也不會到達。
- 如果 `/ly/navi/reached` 缺失或超過 2 秒不新鮮，坐標一進 20 cm 會立即當作到達。

這裡和你的理想有一個差異：你希望 20 cm 只是兜底，但進 20 cm 後仍要等待一小段時間，看 `/ly/navi/reached` 能不能回來，而不是馬上切下一個行為。

建議語義應該是：

```text
fresh /ly/navi/reached true  -> 立即到達
fresh /ly/navi/reached false -> 未到達，繼續等導航
no fresh reached + 進入 20cm -> 開始 near-goal grace timer
grace 期間 reached 變 true -> 到達
grace 超時仍沒有 fresh reached -> 才用 20cm 坐標兜底到達
```

這樣可以避免定位誤差或短暫靠近 goal 時，BT 太早進入下一段 regional task。

## `IsBaseGoalWithinDistance()` 和到達判斷不同

`IsBaseGoalWithinDistance()` 是純距離工具：

- 不看 `/ly/navi/reached`。
- 不看 `/ly/navi/reachable`。
- 只看自身坐標是否新鮮，以及距離是否小於指定值。

它適合用於「接近某點就開視覺/切模式」這類輔助條件，例如前哨偵查接近 `BuffOutpost` 後開 FaceMode。它不應該替代普通路點狀態機的到達判斷。

普通 regional task 的階段切換應該以 `IsBaseGoalArrived()` 為主。

## Navi Progress Watchdog

入口：`TickNaviProgressWatchdog()`，核心實現在 `AreaManager::TickProgressWatchdog()`。

每次 `SetPositionByBaseGoal()` 下發新導航點時，會更新 watchdog 目標：

- goal id
- base goal
- goal team
- goal position
- goal start time
- last move time
- last recorded self position

watchdog 的判斷順序：

1. 未啟用、沒有 active goal、Highland transition 阻塞時，不處理。
2. `Home` / `Recovery` 不做 watchdog fallback。
3. `/ly/navi/reached` 新鮮且為 `true` 時，刷新 `LastMoveTime`，不 fallback。
4. `/ly/navi/reachable` 新鮮且為 `false` 時，允許進入 fallback 流程。
5. 如果沒有 unreachable：
   - 沒有自身坐標就不判斷。
   - 距 goal 小於等於 `NaviProgressWatchdog.ArriveDistanceCm`，只刷新 `LastMoveTime`，不 fallback。
   - 自己從上次記錄點移動超過 `NaviProgressWatchdog.MoveProgressCm`，刷新 `LastMoveTime`，不 fallback。
   - goal start 和 last move 都超過 `NoMoveTimeoutSec` 後，才 fallback。

正式 regional 配置裡目前是：

- `ArriveDistanceCm = 140`
- `MoveProgressCm = 80`
- `NoMoveTimeoutSec = 14`
- `FallbackHoldSec = 5`
- `FallbackCooldownSec = 12`

這裡的 `MoveProgressCm` 是「移動超過門檻」，不是「朝目標方向前進」。這個語義是合理的，因為導航繞路時坐標不一定一直朝 goal 變近，只要底盤一直在動，就不應該判成卡住。

`ArriveDistanceCm = 140` 也不是「到達後切下一個行為」。它只是在 watchdog 裡表示「已經很接近目標，不要因為短時間位移小就 fallback」。真正的狀態機到達仍然走 `IsBaseGoalArrived()`。

## 當前需要注意的差異

目前鏈路大體符合：

- `/ly/navi/reached` 新鮮時優先。
- watchdog 用移動距離刷新，不要求朝 goal 方向。
- watchdog 的 140 cm 只防止 fallback，不推進 regional task。
- Default 已經是 Highland/Base/Roadland/Central 的底層 owner。

目前還不完全符合的是：

- 20 cm 坐標兜底沒有 grace timer。`/ly/navi/reached` 一旦缺失或不新鮮，進 20 cm 會立即到達。
- watchdog 在 Task 和 Tactical 兩層都有入口，行為上不一定錯，但 owner 不夠乾淨。
- `RunTactical()` 裡仍保留「Default 已處理但未 command goal 時再嘗試 Chase」的舊分支；現在 Default 已經在 Tactical 後面，這段基本不可達，可以後續清理。

## 建議下一步

最小正確改動是只加一個 near-goal reached grace：

- 在 `Application` 或 `AreaManager` 裡記錄當前 goal 進入 20 cm 的時間。
- goal 改變時清空這個時間。
- `IsBaseGoalArrived()` 中，沒有 fresh `/ly/navi/reached` 且距離小於 20 cm 時，先等待一個配置時間。
- 等待超時後才用 20 cm 坐標兜底。

這個改動不需要改 ROS topic，也不需要改導航端協議，只是讓 BT 對導航 reached 的等待更穩。
