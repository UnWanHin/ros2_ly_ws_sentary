# Regional 道路區域正式命名收斂

Updated: 2026-07-18

## 決定

Regional 的道路主區只保留兩個正式名稱：

- `PreRoadland`：前段，唯一導航目標為 BaseGoalId `25`。
- `ReadyRoadland`：後段，使用原 `ReadyRoadLand` 四邊形，保留 BaseGoalId `22 -> 21` 的強綁定穿越。

原本為相容而保留的正式 `Roadland` 名稱已移除。這避免「Roadland 是整段道路、後段，還是穿越流程」三種含義同時存在。

## 配置遷移

- `NaviGoal.MyArea/EnemyArea.Roadland` 改為 `ReadyRoadland`。
- `AreaManager.RegionalAreaTask.MyRoadland` 改為 `MyReadyRoadland`。
- `WeightMyRoadland` 與 `AfterHighlandMyRoadlandBonus` 改為對應的 `ReadyRoadland` key。
- area-scope token 只接受 `ready_roadland` / `readyroadland`；舊 `roadland`、`road_land`、`road` 不再解析。

這是刻意的清理，不保留舊 key alias，避免新舊設定在同一配置中產生不明確的優先級。

## 不變量

- ROS topic、message、導航 BaseGoalId `21`、`22`、`25` 和下位機協議不變。
- `ReadyRoadland` 的 crossing phase、FollowMode、可選 FaceMode、安全返回與 cooldown 行為不變。
- RegionalDefense 分別統計前段與後段，再聚合為 RoadCorridor 威脅，保持舊道路範圍的防守覆蓋。
- 未使用的 `RoadlandFollowMode` area helper、AreaCalculator 輸出與 simulator overlay 已移除。
