# Regional Roadland 同級拆分設計

Updated: 2026-07-13

## 目的

取消舊有「`Roadland` 主區 + `MiniRoadland` Special 子區」模型，改為兩個正式、同級的
MainArea：

- `PreRoadland`：道路前段、較易到達區域。
- `Roadland`：道路後段，採用原先 `ReadyRoadLand` 的新邊界。

這是正式策略資料模型變更；完成後不再保留 `Special.MiniRoadland`、
`MiniRoadland` 面積或其 Tactical 插入式任務。

## 正式區域邊界

| MainArea | 紅方 | 藍方 |
| --- | --- | --- |
| `PreRoadland` | C15, C17, C19, C21, C23, C25, C27 | C16, C18, C20, C22, C24, C26, C28 |
| `Roadland` | C19, C23, C29, C33 | C20, C24, C30, C34 |

原 `RedMainAreaRoadlandPoints` / `BlueMainAreaRoadlandPoints` 將移除，正式
`MainAreaKind::Roadland` 改用上述新的 Roadland 邊界。`PreRoadland` 成為新增
`MainAreaKind`，不再是 query-only candidate。

## 點位與 ID 歸屬

每個導航基礎點只能屬於一個正式區域：

| BaseGoalId | 新名稱/既有名稱 | 紅方座標 | 藍方座標 | 正式所屬 |
| --- | --- | ---: | ---: | --- |
| 25 | `PreRoadland`（原 `MiniRoadland`） | (457, 72) | (2343, 1428) | `PreRoadland` |
| 21 | `BaseToCentral` | (1125, 155) | (1675, 1345) | `Roadland` |
| 22 | `CentralToBase` | (515, 100) | (2285, 1400) | `Roadland` |

ID 25 保持不變，避免導航端既有 goal ID 映射斷裂；只更改 C++ 名稱、配置鍵與 trace/doc
語義為 `PreRoadland`。ID 22 按已確認座標遷入新 `Roadland`，因此原 Roadland 的雙向
穿越階段仍可保留，不會跨越到 `PreRoadland`。

## 決策與 BT 設計

1. `MainAreaKind` 新增 `PreRoadland`；`Base`、`Highland`、`Roadland`、`Central` 保持。
2. AreaManager 正式解析順序納入 `PreRoadland` 和新的 `Roadland`。兩區共享 C19/C23
   （藍方 C20/C24）邊界線時，邊界點的唯一歸屬固定為 `Roadland`，避免同一點同時被
   Default 評分成兩個區域。
3. 原 `Special.MiniRoadland` 的 `Enable`、`GoalHoldSec`、`SpeedLevel` 遷移為
   `AreaManager.RegionalAreaTask.MyPreRoadland` 的正式設定，並由 `RegionalAreaTaskType`
   的 `MyPreRoadland` 狀態機驅動。
4. `MyPreRoadland` 的初始目標為 ID 25；到點、不可達或逾時依其區域設定完成，交回
   Default 選擇。它不繼承 Roadland 的強綁定穿越與 FaceMode，因為原 MiniRoadland
   Special 也沒有這些行為。
5. `MyRoadland` 繼續使用 ID 21 / 22 的強綁定穿越、安全返回、FollowMode、FaceMode
   和既有停止條件；只替換正式邊界與區域鍵。
6. Default scorer、新區域 scope JSON、區域任務 retry/cooldown、目前/上一區懲罰和
   DecisionTrace 都以 `PreRoadland`、`Roadland` 分別記錄。原 `WeightMyRoadland` 的
   值保留給新 Roadland；原 MiniRoadland 沒有 Default weight，新增
   `WeightMyPreRoadland`，預設採原 Special 啟用時的前置優先級（不搶占 Hard/Tactical）。
7. 先維持現有「正式狀態機只實作我方區」範圍：配置與 area scope 可以表達敵方
   `PreRoadland` / `Roadland`，但未新增敵方主區任務。這與現有敵方 Base/Highland/
   Roadland 尚無任務狀態機的行為一致。

## 刪除與介面遷移

將移除下列舊名稱與特例：

- `Area::RedMiniRoadlandPoints` / `BlueMiniRoadlandPoints` 及其 area 查詢 API。
- `LangYa::MiniRoadland` 名稱，改為 `LangYa::PreRoadland`，ID 維持 25。
- `SpecialMiniRoadlandSetting`、`SpecialSetting::MiniRoadland`、
  `TrySetSpecialMiniRoadlandGoal()`、`Special.yaml` 的 `MiniRoadland` 區段。
- PointManager 的 `MiniRoadland` 配置鍵，改為 `PreRoadland`。
- `area_calculator` 的 `mini_roadland` 輸出，改為正式 main-area 解析與
  `pre_roadland` 顯示。

不變的外部 ROS topic、lower-machine 協議、導航 goal ID（25）、FaceMode 出口與
`/ly/control/*` 發布鏈路，不會因本次區域重構改名。

## 驗證

- 新增/調整幾何測試，驗證兩區紅藍內點、外點、共用邊界唯一歸屬。
- 新增 AreaManager / Default scorer 測試：ID 25 只啟動 `MyPreRoadland`；ID 21/22
  只屬新 `Roadland`；舊 Mini special 不再可觸發。
- 驗證 `area_calculator`、Regional DecisionTrace、simulator trace schema。
- 執行 `colcon build --packages-select behavior_tree simulator`、相關 gtest、
  `./scripts/selfcheck.sh sentry --skip-hz`；實機未啟動時不宣稱導航實跑。
- 同步更新區域決策文檔、地圖工具可匯入點名與 Understand Anything graph。
