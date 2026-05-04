# 舊 Strategy 決策邏輯盤點

Updated: 2026-05-04

本文記錄 `behavior_tree` 裡在新分層策略之前已存在、並且目前仍被復用的決策邏輯。它的目的不是提出新方案，而是把舊的 `HitHero / HitSentry / Protected / NaviTest / LeagueSimple` 鏈路、共同前置判斷、追擊和姿態輸出整理清楚，方便後續往 `Hard / Default / Task / Tactical / Finalizer` 分層搬遷。

主要代碼位置：

- `src/behavior_tree/Scripts/main.xml`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/src/StrategyManager.cpp`
- `src/behavior_tree/src/PostureLogic.cpp`
- `src/behavior_tree/src/PublishMessage.cpp`
- `src/behavior_tree/module/BasicTypes.hpp`

## 主 Tick 流程

現在主 BT 每 tick 的大順序是：

```text
UpdateGlobalData
  -> SelectAimMode
  -> SelectStrategyMode
  -> StrategyStack
  -> PreprocessData
  -> SelectAimTarget
  -> SelectPosture
  -> PublishAll
```

其中 `StrategyStack` 是新的分層入口：

```text
Hard -> Default -> Task -> Tactical -> Finalizer
```

舊的 `StrategyDispatch` subtree 還留在 `main.xml` 裡作兼容記錄，但主鏈路已經不再直接執行它。舊的各個策略函數沒有刪，現在主要在 `Tactical` 層繼續被調用。

## 舊 GameStrategy 開關

舊配置裡的 `GameStrategy` 主要字段是：

```cpp
HitOutpost
HitBuff
TestNavi
HitSentry
Protected
```

原本 `SetPositionRepeat()` 的靜態優先級是：

```text
League profile
  -> Showcase patrol
  -> HitSentry
  -> NaviTest
  -> Protected
  -> HitHero
```

也就是說，如果沒有明確開 `HitSentry / TestNavi / Protected`，普通策略就落到 `HitHero`。現在主鏈路改為 `SelectStrategyMode()` 先產生 `StrategyMode`，再由 `StrategyStack` 按層處理；但舊函數內部的大部分選點邏輯仍然保留。

## StrategyMode 選擇

`SelectStrategyMode()` 的舊核心規則：

- 如果是 league profile，直接選 `LeagueSimple`。
- 如果 `NaviDebug` 開啟，直接選 `NaviTest`。
- 開局前 10 秒保持當前策略，避免頻繁抖動。
- 如果已經在 `NaviTest`，且時間小於 340 秒，保持 `NaviTest`。
- 如果 `DecisionAutonomy.strategy_mode` 開啟，就對候選策略打分。
- 如果 autonomy 沒開，走固定規則：
  - 低資源：`Protected`；
  - 前哨窗口：`HitSentry`；
  - 否則：`HitHero`。

低資源判定目前是：

```text
self_health < 100
or time_left <= 120
or RemainingEnergy == 0b10000
or RemainingEnergy == 0b00000
```

前哨窗口判定目前是：

```text
enemy_outpost_health > 0
and self_outpost_health > 100
and elapsed_time < 55
```

Autonomy 模式下，候選策略默認是：

```text
HitHero / HitSentry / Protected
```

打分會疊加：

- 對應策略 bias；
- `HitSentry` 的前哨窗口 bonus；
- `HitSentry` 在沒有前哨窗口時的 penalty；
- `Protected` 的低資源 bonus、時間壓力 bonus；
- 低資源時非 `Protected` 的 penalty；
- 當前策略 bonus，降低抖動。

## AimMode 選擇

`SetAimMode()` 和導航 strategy 是兩條不同語義：

- Strategy 決定去哪個導航點；
- AimMode 決定輔瞄/打符/打前哨/巡邏掃描。

AimMode 的舊規則：

- 如果 regional defense 正在壓制特殊瞄準，強制 `RotateScan`。
- 如果 `GameStrategy.HitBuff` 開啟：
  - 前 25 秒且 `buff_shoot_count <= 15` 時嘗試 `Buff`；
  - 7 秒後如果裁判系統顯示 buff 已激活，切回 `RotateScan`；
  - 超時或次數超限，切回 `RotateScan`。
- 如果 `GameStrategy.HitOutpost` 開啟：
  - 敵方前哨血量大於 0 且開局 90 秒內，使用 `Outpost`；
  - 否則切回 `RotateScan`。
- 其他普通情況使用 `RotateScan`。

## 舊策略函數的共同前置邏輯

`SetPositionProtect()`、`SetPositionNaviTest()`、`SetPositionHitSentry()`、`SetPositionHitHero()` 進入真正選點前，基本都會先跑同一組前置判斷：

```text
CheckPositionRecovery()
  -> TickRegionalAreaTask()
  -> TickNaviAreaTransition()
  -> TrySetRegionalDefenseGoal()
  -> TickNaviProgressWatchdog()
  -> naviCommandIntervalClock.trigger()
```

含義是：

- 先處理補血/補彈/回家；
- 已經啟動的區域任務要繼續 tick；
- Highland 兼容過渡或導航 watchdog 可以接管；
- regional defense 可以插入防守/回防點；
- 導航指令有發送間隔，避免高速刷 goal。

這也是為什麼舊的 `HitHero / HitSentry / Protected` 不是純粹的戰術函數，它們裡面混有 recovery、區域任務、防守和節流。

## Recovery / 補血補彈

`CheckPositionRecovery()` 是舊鏈路裡最高優先級之一，目前已搬到新 `Hard` 層。

通用開關：

- `RegionalAreaTask.IgnoreRecovery=true` 時直接跳過 recovery。

League profile 裡 recovery 是較完整的血量回補狀態機：

- 只使用裁判系統血量/彈藥作觸發源；
- 有 stale 檢查，避免舊數據誤觸發；
- 低血量進入 Recovery 點；
- 到達退出血量、血量平台期或超時後退出；
- 失敗後有 cooldown，避免反覆抖動；
- Recovery 期間會持續鎖定 Recovery goal，並縮短導航重發間隔。

非 league 裡 recovery 較簡單：

- 如果當前已經在 Recovery 且血量小於 380，持續 hold Recovery。
- 如果血量小於 150，回 Recovery。
- 如果彈量小於等於 30 且 recoveryClock 到期，回 Recovery。
- 如果已有區域任務：
  - Roadland 會請求安全返回，不是直接取消；
  - 其他區域任務會被清掉，並關閉 FollowMode/區域控制覆蓋。

因此補血/補彈不應放在底層 Default，它更像 Hard Safety。

## DecisionAutonomy 選導航點

`TrySetNaviGoalByAutonomy()` 是舊策略裡比較新的 utility 選點入口，支持：

```text
HitHero
HitSentry
Protected
```

啟用條件：

- `DecisionAutonomy.Enable=true`；
- `EnabledModules` 里開了 `navi_goal`、對應的 `navi_goal_hit_hero / navi_goal_hit_sentry / navi_goal_protect`，或 `all`；
- 沒有被 `HardRuleModules` 排除。

候選點來源：

- 配置裡的 custom candidates；
- 如果 custom 為空，使用內建候選點。

候選點會先過 area scope：

- `MyArea`；
- `EnemyArea`；
- `CommonArea`。

打分因子包括：

- candidate bias；
- 與自身坐標的距離；
- 敵方側 bonus；
- 當前 goal bonus；
- 低能量時我方側 bonus / 敵方側 penalty；
- 我方前哨低血量時我方側 bonus；
- 敵方英雄接近程度。

選中後會走：

```text
TrySetScopedPositionByBaseGoal()
  -> 可能啟動 AreaManager 任務
  -> 可能啟動 Highland transition
  -> SetPositionByBaseGoal()
```

所以 autonomy 選中的不是裸坐標，而是 `Area.hpp` 裡的 base goal，再交給後續鏈路解析隊伍偏移和區域狀態機。

## SetPositionLeagueSimple

League 模式是單獨 profile，舊邏輯如下：

- 優先跑 league route compat，處理特定 2/3 點位切換兼容。
- 優先跑 recovery，命中後不再巡航。
- 巡航 plan 由 `MainGoal + PatrolGoals` 去重組成。
- plan 空時 fallback 到 `OccupyArea`。
- 首次進入選 plan 第一個點。
- 後續按 `GoalHoldSec` 週期切換下一個點。
- 所有 goal 都走 team offset。

## SetPositionShowcasePatrol

Showcase 是展示/演示巡航：

- 使用 `ShowcasePatrol.Goals`。
- 可配置順序或 random。
- 可配置是否 `DisableTeamOffset`。
- 可配置是否 `IgnoreRecovery`。
- plan 空時 fallback 到 `OccupyArea`。

它不屬於主要比賽策略，但在主鏈路裡優先於普通 Tactical 策略。

## SetPositionNaviTest

NaviTest 有兩種形態：

- 如果 `NaviDebugSettings.Enable=true`，走 `SetPositionNaviDebugPlan()`：
  - 使用 `NaviDebug.Goals`；
  - 可 random；
  - 可 DisableTeamOffset；
  - 可設 `SpeedLevel`；
  - plan 空時 fallback 到 `OccupyArea`。a
- 如果 NaviDebug 沒開，走舊固定時間腳本：
  - `BuffShoot`
  - `LeftHighLand`
  - `CastleLeft1`
  - `CastleRight1`
  - `CastleRight2`
  - `FlyRoad`
  - `OutpostArea`
  - `MidShoot`
  - `LeftShoot`
  - `OutpostShoot`
  - 再切敵方側若干點
  - 最後回 `Castle`

這個模式主要是導航測試，不應作為正式底層決策來源。

## SetPositionProtect

Protected 是保守模式：

- 先跑共同前置邏輯。
- 如果 AimMode 是 `Buff`，去我方 `BuffShoot`。
- 如果 AimMode 是 `Outpost`，去我方 `OutpostShoot`。
- 普通情況：
  - 優先嘗試 `TrySetNaviGoalByAutonomy(Protected)`；
  - 如果 autonomy 沒選中，fallback 在我方基地側幾個點隨機：
    - `CastleLeft1`
    - `CastleLeft2`
    - `CastleRight1`
    - `CastleRight2`
    - `BuffShoot`
  - 如果選到 `BuffShoot`，hold 30 秒，其他點 hold 10 秒。
- 比賽時間超過 300 秒或底盤能量低時，`speedLevel=0`。

## SetPositionHitSentry

HitSentry 是打哨兵/前哨窗口偏進攻模式：

- 先跑共同前置邏輯。
- 如果 AimMode 是 `Buff`，去我方 `BuffShoot`。
- 如果 AimMode 是 `Outpost`，去我方 `OutpostShoot`。
- 普通情況：
  - 優先嘗試 `TrySetNaviGoalByAutonomy(HitSentry)`；
  - 如果 autonomy 沒選中：
    - 在我方前哨血量大於 100 且開局 55 秒內，從我方/敵方中場和射擊點中隨機；
    - 否則去敵方 `FlyRoad`。
  - 敵方 `MidShoot` 或 `LeftShoot` hold 8 秒，其餘一般 hold 10 秒。
- 底盤能量低時會嘗試回我方 `HoleRoad`，但函數末尾又固定 `speedLevel=1`，所以當前實際速度等級可能覆蓋前面的低能量 `speedLevel=0`。

舊代碼裡還有一段判斷敵方 infantry 是否在 central highland，但目前只是打 debug log，沒有直接改變選點。

## SetPositionHitHero

HitHero 是舊普通模式的主 fallback：

- 如果是 league profile，轉 `SetPositionLeagueSimple()`。
- 如果 showcase 開啟，轉 `SetPositionShowcasePatrol()`。
- 然後跑共同前置邏輯。
- 如果 AimMode 是 `Buff`，去我方 `BuffShoot`。
- 如果 AimMode 是 `Outpost`，去我方 `OutpostShoot`。
- 普通情況：
  - 現在會先嘗試新的底層入口 `TrySetDefaultRegionalGoal()`；
  - 如果 Default 沒選中，才走舊 HitHero fallback。

舊 HitHero fallback：

- 如果敵方英雄在 central highland，去我方 `BuffAround1`。
- 否則，如果我方前哨血量大於 200：
  - 用硬編碼紅方點 `(982, 1124)` 和敵方英雄坐標算距離；
  - 距離小於 100 時，去敵方 `HoleRoad`，hold 2 秒；
  - 否則在我方/敵方中場和射擊點中隨機。
- 如果我方前哨血量不大於 200：
  - 我方基地血量大於 2000 時，去敵方 `HoleRoad`，hold 2 秒；
  - 否則同樣走隨機中場/射擊點。
- 隨機點裡敵方 `MidShoot` 或 `LeftShoot` hold 8 秒，其餘一般 hold 10 秒。
- 低能量且不是 Default 選中時，會改選我方 `BuffAround1 / BuffAround2 / RightShoot`，但函數末尾也固定 `speedLevel=1`，所以同樣存在覆蓋低能量 `speedLevel=0` 的現狀。

注意：你之前說過「hithero 的點位不用搬」，所以目前新的 Default 只接新底層選點，不把這些舊 fallback 點表搬進 Default。

## RegionalDefense

RegionalDefense 是舊策略函數共同前置裡的一層戰術插入：

- 收集新鮮敵方坐標；
- 由 AreaManager 判斷 hard threat / soft enemy-side threat；
- 命中時可以設置防守/回防導航點；
- 也會啟動 aim suppress，使 `SetAimMode()` 不再進 Buff/Outpost 特殊瞄準。

它現在仍然屬於戰術疊加邏輯，不是底層 Default。

## Chase 追擊鏈路

Chase 不在 `SetPosition*` 裡選固定點，而是在 `ProcessData()` 和 `PublishMessageAll()` 這一段處理。

啟用條件：

- `Chase.Enable=true`；
- `Chase.FollowAimTarget=true`；
- 當前 AimMode 對應的 `EnableInAutoAim / EnableInRotateScan / EnableInOutpostMode / EnableInBuffMode` 允許；
- 當前不在 FollowMode。

有有效裝甲板目標時：

- 使用目標距離、雲台 yaw error、pitch error 算出相對目標：
  - `naviRelativeTargetX`
  - `naviRelativeTargetY`
  - `naviRelativeTargetZ`
  - distance / yaw error / pitch error / armor type
- 如果 `Chase.ToNavi=true`，發布 `/ly/navi/target_rel`，由 navi bridge 轉成導航目標。
- 如果 `Chase.ToNavi=false`，BT 直接算追擊速度：
  - 前後速度用距離誤差和 `DistanceKp`；
  - 側向速度可用 yaw error 和 `YawKp`；
  - 速度會被 max forward/backward/strafe 限制。

沒有有效目標時：

- `StopWhenNoTarget=true` 且非 ToNavi 模式時，速度置 0。
- `StopWhenNoTarget=true` 且 ToNavi 直發坐標模式時，目標點可回到自身坐標。

發布時有一個重要互斥：

- 如果 chase bridge active，`PublishMessageAll()` 會發布 `/ly/navi/target_rel`，並避免普通固定點 `/ly/navi/goal_pos_raw` 覆蓋追擊目標。
- 如果沒有有效 chase target，才繼續發布普通導航 goal。

## FaceMode / FollowMode 對雲台和開火的影響

這兩個模式不屬於舊 `GameStrategy`，但會影響策略輸出結果。

FaceMode active 時：

- 停雲台巡邏掃描；
- `FireCode.AimMode=0`；
- 默認停火；
- 如果 `/ly/face_mode/angles` 有新鮮角度，轉發到 `/ly/control/angles`；
- 如果角度丟失但還在 hold 時間內，沿用鎖存角度；
- 否則保持當前雲台角。

FollowMode active 時：

- 停雲台巡邏掃描；
- `FireCode.AimMode=0`；
- 停火；
- 雲台保持當前角；
- FireCode 的 FollowMode 位會下發給下位機。

所以 Roadland 強綁定段目前是更硬的控制；FaceMode 更偏「固定朝向」，FollowMode 更偏「下位機跟隨語義/停小陀螺/停巡邏/停火」。

## Posture 姿態選擇

`SelectPosture` 的實際邏輯在 `PostureLogic.cpp`：

- 先根據 `strategyMode` 給 Attack / Defense / Move 加基礎分；
- 再根據 AimMode 加分；
- 有目標時提高 Attack；
- 低能量、低血量、低彈量、受擊時提高 Defense / Move，降低 Attack；
- 爆發受擊會直接選 Defense；
- 姿態回讀 stale 時降低激進性；
- 同一姿態累計太久會被扣分，避免長時間不切換；
- pending 姿態有小幅加分，避免抖動；
- 最後用 hysteresis 保持當前姿態，防止分差很小時頻繁切。

這套姿態是輸出 finalizer 後的下游命令，不直接決定導航 goal，但會和 strategy/aim/chase 的結果一起發下去。

## 現在已搬到新分層的位置

目前新分層的對齊狀態：

- `Hard`
  - `CheckPositionRecovery()`
  - Roadland 強綁定穿越 hard lock
- `Default`
  - 新底層無事件決策入口
  - 只嘗試 `TrySetDefaultRegionalGoal()`
  - 內容是 `AreaManager.DefaultPolicy` 對啟用大區域做資源門檻、距離、目前區域、上次任務結果、冷卻和重試評分，無可用區域時 fallback 到 `DecisionAutonomy.NaviGoal(HitHero)`
  - 不包含舊 HitHero hardcoded fallback 點表
- `Task`
  - 已啟動的 AreaManager 任務繼續 tick
  - Highland transition
  - Navi progress watchdog
- `Tactical`
  - 舊的 `SetPositionLeagueSimple()`
  - 舊的 `SetPositionShowcasePatrol()`
  - 舊的 `SetPositionHitSentry()`
  - 舊的 `SetPositionProtect()`
  - 舊的 `SetPositionNaviTest()`
  - 舊的 `SetPositionHitHero()`
- `Finalizer`
  - 如果前面沒有任何層處理，fallback 執行 `SetPositionHitHero()`
  - 同步策略層 blackboard 監控字段

## 目前已知存量特性

這些是盤點時看到的舊邏輯現狀，本文只記錄，不在這次改動裡修：

- `HitSentry` 和 `HitHero` 低能量分支裡曾設 `speedLevel=0`，但函數末尾又設 `speedLevel=1`，可能覆蓋低能量速度限制。
- `HitSentry` 中檢查 infantry 是否在 highland 時，紅藍區域判斷看起來重複使用了 blue highland；目前只影響 debug log。
- `HitHero` fallback 裡有硬編碼點 `(982, 1124)` 用來和敵方英雄位置算距離。
- 舊策略函數混合了「安全」「區域任務」「防守」「戰術選點」「節流」多種責任，所以後續繼續重構時應優先把行為按層拆清楚，而不是直接把整個 `SetPositionHitHero()` 搬進 Default。
