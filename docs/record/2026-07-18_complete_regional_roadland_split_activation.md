# 完成 Regional Roadland 拆分的配置接入

Updated: 2026-07-18

## 背景

`PreRoadland` 與後段 `Roadland` 的 area、ID、AreaTask 和 Default scorer 已經存在，但正式
配置鏈未完整接入：`ApplyAreaManagerParameterOverrides()` 沒有讀取 `MyPreRoadland` 的 YAML
設定及 `WeightMyPreRoadland`，`regional_competition.json` 也把兩區排除於 `NaviGoal.MyArea`
之外。因此拆分只存在於資料模型與測試設定，正式 Regional Default policy 不會選中它們。

## 落地結果

- `AreaManager.yaml` 的 `MyPreRoadland` 設定現在完整讀入 runtime config：`Enable`、
  `TravelTimeoutSec`、`GoalHoldSec`、`CommandHoldSec`、`SpeedLevel`。
- `WeightMyPreRoadland` 現在可由同一組 `AreaManager.DefaultPolicy.Score` ROS 參數覆蓋。
- `regional_competition.json` 及直接啟動的 `Scripts/config.json` 都把 `PreRoadland`、
  `Roadland` 作為我方可選區域；`AreaManager.yaml` 也啟用 `MyRoadland`。
- `PreRoadland` 繼續固定使用 ID `25`，到點 hold 後可讓出控制；`Roadland` 保留 ID `22 -> 21`
  的強綁定穿越與安全返回。
- `MyRoadland.UseFaceMode` 保持既有 baseline `false`，本次不更動固定朝向策略。

## 不變量

- 不改 ROS topic、導航 goal ID、外部導航接口、FaceMode 出口或下位機協議。
- `regional_simple` 與專用區域測試 profiles 的道路範圍開關保持原值，不會因正式 Regional
  profile 啟用兩個道路候選而改變。
