# Regional 決策框架說明

Updated: 2026-05-04

本文記錄目前 `behavior_tree` 裡 regional 決策的區域狀態機框架：它會做哪些任務、怎麼啟動、怎麼判斷到達、會輸出什麼控制，以及哪些階段會被高優先級邏輯打斷。

## 覆蓋範圍

目前已完成的基本區域狀態機：

- `MyArea.Base`
- `MyArea.Highland`
- `MyArea.Roadland`
- `CommonArea.Central`

目前沒有寫 `EnemyArea.Base / EnemyArea.Highland / EnemyArea.Roadland` 的獨立狀態機，因為當前基本 regional 任務不需要它們。Central 任務裡會走到敵方側的中場點，但它仍然屬於 `CommonArea.Central`，不是敵方大區域任務。

主要相關文件：

- `config/AreaManager.yaml`
- `src/behavior_tree/include/AreaManager.hpp`
- `src/behavior_tree/src/AreaManager.cpp`
- `src/behavior_tree/include/StrategyManager.hpp`
- `src/behavior_tree/src/StrategyManager.cpp`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/module/Area.hpp`

## Strategy 分層

`Scripts/main.xml` 的主決策入口現在走 `StrategyStack`，每 tick 依序執行：

```text
Hard -> Default -> Task -> Tactical -> Finalizer
```

各層的責任是：

- `Hard`：最高優先級保護，處理 recovery/補血補彈和 Roadland 強綁定穿越段。Roadland 強綁定段在這層 hard lock，避免被戰術層中途搶走。
- `Default`：無特別事件時的底層決策，現在只承接新的底層選點入口：`DecisionAutonomy.NaviGoal(HitHero)` 和 `RegionalIdlePatrol`。舊 HitHero fallback 點表不在 Default 裡。
- `Task`：已啟動的 AreaManager 任務繼續 tick，包含 Highland/Base/Roadland/Central 任務、Highland 兼容過渡和導航 watchdog。
- `Tactical`：戰術疊加層，繼續使用原本的 `SetPositionLeagueSimple/HitSentry/HitHero/Protect/NaviTest/ShowcasePatrol` 鏈路。
- `Finalizer`：保證本 tick 有一個策略層完成，並把策略層狀態同步到黑板。

分層狀態會寫入 BT blackboard：

- `StrategyLayerHandled`
- `StrategyLayerHandledBy`
- `StrategyLayerHardLock`
- `StrategyLayerDefaultRequested`

這些字段只做監控，不改 ROS topic contract。

## 數據來源

Regional 任務主要使用這些導航/定位輸入：

- `/ly/navi/reached`：當前 goal 是否已到達，主到達判斷。
- `/ly/navi/reachable`：當前 goal 是否有有效路徑，主不可達判斷。
- `/ly/navi/position`：導航/TF 推出的自身官方地圖坐標，單位 cm。
- `/ly/position/data`：官方/雷達定位坐標。

`/ly/navi/position` 和 `/ly/position/data` 目前都會寫入 BT 裡同一份哨兵自身坐標欄位。這個坐標主要用於：

- 選最近的巡邏起點；
- 判斷自己目前在哪個大區域；
- 在 `/ly/navi/reached` 缺失或超時時，作為距離到達判斷的備用條件。

也就是說，只要 `/ly/navi/reached` 和 `/ly/navi/reachable` 是新鮮有效的，到沒到點還是以它們為主，不靠 `/ly/navi/position` 硬判。

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

目前 `config/AreaManager.yaml` 的基本區域配置是：

```yaml
AreaManager:
  Switch_Point: false
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

根層 `AreaManager.yaml` 不再默認寫 `Area.MyArea/EnemyArea/CommonArea` 的區域開關，避免它把不同 `bt_config_file` 裡的區域選擇全部覆蓋成同一套。正式 regional 和單區域 areatest 的「哪些區域可選」仍由對應 `ConfigJson` 裡的 `DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 控制；`AreaManager.yaml` 只保留 `Switch_Point` 和區域狀態機任務的時序、門檻、啟停參數。

`Switch_Point=true` 時只交換 `Area.hpp` 裡紅/藍官方點位和區域邊界查找結果，不交換 `team`、敵我語義或導航 goal ID。這是給導航零點/物理場地方向反了時使用的點位查找開關。

重要健康門檻：

- `MyRoadland.HealthyHpMin`
- `MyRoadland.HealthyAmmoMin`
- `CommonCentral.HealthyHpMin`
- `CommonCentral.HealthyAmmoMin`

Roadland 用這些門檻決定是否離開駐守點並安全返回。Central 則要求啟動時血量/彈量數據新鮮且健康；任務中如果新鮮數據變成不健康，就完成並釋放控制。

## 各區域任務

### MyHighland

觸發條件：上游選中的 goal 精確屬於我方 Highland 大區域。

任務路線：

```text
Highland(FollowMode + FaceMode，停火)
  -> Highland 短暫巡邏/停留
  -> BuffShoot
  -> BuffShoot 駐守
  -> HoleRoad(FollowMode + FaceMode，停火)
  -> 完成
```

進入 Highland 和離開 Highland 時：

- 開 `FollowMode`；
- 如果配置允許，開 `FaceMode`；
- 停火。

Highland 巡邏和 BuffShoot 駐守時：

- `FollowMode` 關閉；
- 雲台巡邏、小陀螺、開火回到普通 BT 控制。

### MyBase

觸發條件：上游選中的 goal 精確屬於我方 Base 大區域，並且當前不在我方 Highland 裡。

任務路線：

```text
CastleLeft1 -> CastleLeft2 -> CastleRight2 -> CastleRight1 -> repeat
```

起點會用自身坐標選最近點。拿不到自身坐標時，保守從 `CastleLeft2` 開始。

MyBase 本身不開 `FollowMode`，也不開 `FaceMode`，就是普通基地巡遊狀態機。

### MyRoadland

觸發條件：上游選中的 goal 精確屬於我方 Roadland 大區域，並且當前不在我方 Highland 裡。

任務路線：

```text
CentralToBase
  -> BaseToCentral(FollowMode + FaceMode，停火)
  -> BaseToCentral 駐守
  -> CentralToBase(FollowMode + FaceMode，停火)
  -> 完成
```

具體行為：

- 先正常去 `CentralToBase`。
- 到點、不可達或超時後，進入去 `BaseToCentral` 的強綁定穿越段。
- 強綁定穿越段中，`FollowMode + FaceMode` 生效，並停火。
- 到 `BaseToCentral` 後，恢復普通巡邏、小陀螺、開火控制。
- 如果血量/彈量新鮮數據低於門檻，開始安全返回 `CentralToBase`。
- 如果非強綁定階段遇到更高優先級請求，也不是直接取消，而是請求安全返回。

Roadland 的 FaceMode 目標就是當前穿越終點：

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

每個當前 goal 的到達/不可達判斷順序是：

1. `/ly/navi/reachable`
   - 新鮮 `false` 表示當前路徑不可達；
   - 狀態機會根據所在 phase 推進、切換或完成。
2. `/ly/navi/reached`
   - 新鮮 `true` 表示當前 goal 到達。
3. 自身坐標距離兜底
   - 只有外部導航狀態缺失或超時時才用。

因此 `/ly/navi/position` 不替代 `/ly/navi/reached`。它主要是區域判斷、最近點插入、外部狀態失效時的兜底。

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
- `FollowMode / FaceMode / 停火` 會保持。

強綁定穿越段只會因為以下條件結束：

- 到達穿越終點；
- 穿越終點不可達；
- 穿越超時。

## 控制輸出

Area task 使用既有 BT 控制鏈路輸出：

- 通過 `SetPositionByBaseGoal()` 發導航目標；
- 通過 `FireCode.FollowMode` 控 FollowMode；
- 需要固定朝向時，通過 `/ly/face_mode/target_raw` 發 FaceMode 目標；
- 需要停火時，覆蓋本輪 fire 狀態。

FaceMode 目標格式：

```text
[official_map_x, official_map_y, map_z]  # cm
```

FaceMode 負責固定點朝向和接管雲台角度。FollowMode 是更強的模式，會同時用 firecode 語義停底盤小陀螺、停雲台巡邏並停火。

## 當前完整性

以目前 regional 基本任務來看，框架已經完整：

- 我方 Base 巡遊由 MyBase 管；
- 我方 Highland 進入、駐守、離開由 MyHighland 管；
- 我方 Roadland 強綁定穿越、安全返回由 MyRoadland 管；
- 中場健康巡邏由 CommonCentral 管。

敵方 Base/Highland/Roadland 之後如果有明確任務，再單獨加狀態機即可。目前不寫不影響這套基本 regional 框架。
