# Default 區域巡邏收斂

Updated: 2026-07-18

## 決定

Regional Default 的區域 eligibility 只讀各 BT JSON 的
`DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 開關。多個區域合格時，剛選過的
area 會排到本輪最後，避免完成任務後立刻重複同一區域。

`MyBase` 的內部巡邏改為程式固定的 `CastleLeft1`、`CastleLeft2`、`CastleRight2`、
`CastleRight1` 四點；每點到達後保持 15 秒，一輪最多四點。`Base.yaml` 和
`base_strategy_config_file` 已移除，舊 JSON/ROS parameter 的 MyBase `Patrol.GoalWeights` 也不再
讀取。

## 邊界

- `BuffOutpost`、`HoleRoad`、`OutpostGuard` 不再由 Default/MyBase 發布。
- Buff/Outpost tactical 仍是 `BuffOutpost` 的唯一 owner；前哨存活與交戰繼續由新鮮
  `/ly/enemy/op_hp`、視覺偵查和 Outpost task gate 管理。
- ROS topics、navigation BaseGoal IDs、下位機協議及外部裁判/RFID 欄位不變。
