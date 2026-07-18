# Default 區域巡邏收斂

Updated: 2026-07-18

## 決定

Regional Default 的區域 eligibility 只讀各 BT JSON 的
`DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 開關。多個區域合格時，剛選過的
area 會排到本輪最後，避免完成任務後立刻重複同一區域。

若 Default 區域任務在行進或駐留期間被可恢復的 tactical overlay 搶占，BT 會把該區域記為
`preempted`，而不是視為 timeout/unreachable/canceled。當 tactical overlay 釋放後，只有該區域
仍啟用、仍在 JSON area scope、資源仍健康且未進 cooldown 時，下一次 Default 選點會優先續走它；
否則立即丟棄恢復權，照正常 eligibility 與輪換規則選下一區。這避免 `A -> B` 尚未完成時因短暫前哨、回防或
Special Patrol 接管而回到舊的 `A`。

`MyBase` 的內部巡邏改為程式固定的 `CastleLeft1`、`CastleLeft2`、`CastleRight2`、
`CastleRight1` 四點；每點到達後保持 15 秒，一輪最多四點。`Base.yaml` 和
`base_strategy_config_file` 已移除，舊 JSON/ROS parameter 的 MyBase `Patrol.GoalWeights` 也不再
讀取。

## 邊界

- `BuffOutpost`、`HoleRoad`、`OutpostGuard` 不再由 Default/MyBase 發布。
- Buff/Outpost tactical 仍是 `BuffOutpost` 的唯一 owner；前哨存活與交戰繼續由新鮮
  `/ly/enemy/op_hp`、視覺偵查和 Outpost task gate 管理。
- ROS topics、navigation BaseGoal IDs、下位機協議及外部裁判/RFID 欄位不變。
- `preempted` 只代表可恢復的 Buff/Outpost、RegionalDefense 或 Special Patrol 搶占；Recovery
  仍是終止取消，不會在回補後強制回到舊巡邏區。
- `Task.OutpostConfirm.OpeningHoldSec=120` 且 `OpeningHoldUntilWindowEnd=true` 時，開局
  `0..119` 秒、且前哨 safety gate 合格時，前哨導航 ownership 固定在 `BuffOutpost`；Default、普通
  巡邏和 soft tactical 不可換點。前哨已毀、明確不可達、受擊 abort、資源 gate、Hard Recovery 與
  己方 Base 硬防守仍按既有安全優先級接管。這個 hard hold 不依賴 `OpeningHighPriority`；後者只控制
  非 hard-hold 的一般開局優先。
- DecisionTrace 的 `decision_intent` 會把正式、開局與 scout travel 的 Outpost tactical reason
  統一標為 `aim_mode / regional_tactical_aim_mode`，但保留原始 reason 在 `detail`，可直接辨識
  前哨接管而非 Default 重選。
