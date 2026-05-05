# Vision / Task / Patrol Mode Flow

Updated: 2026-05-05

本文說明目前 `regional` 鏈路裡幾個容易混淆的「模式」：`/ly/vision/mode`、BT JSON 的 `Task`、雲台巡邏、區域巡邏、`FollowMode`、`FaceMode`。這些不是同一層東西，不能混着改。

## 分層結論

| 名稱 | 類型 | 誰處理 | 作用 |
|---|---|---|---|
| `/ly/vision/mode` | ROS2 topic | `behavior_tree` 發，`detector` / `buff_hitter` 收 | 選視覺 pipeline：裝甲板、打符、前哨 |
| `Task.Buff / Task.Outpost` | BT JSON 配置 | `behavior_tree` 讀 | 決定這局是否進打符/前哨任務 |
| `AimMode` | BT 內部狀態 | `behavior_tree` | 把任務轉成當前視覺模式、目標選擇、開火/角度鏈路 |
| 雲台巡邏掃描 | BT 內部控制 | `behavior_tree` | 沒看到目標時自己算 `/ly/control/angles` 掃描 |
| 區域巡邏 | BT 區域狀態機 | `AreaManager` | Base/Highland/Roadland/Central 導航任務 |
| `FollowMode` | firecode 語義位 | `behavior_tree` 發，下位機收 | 停小陀螺、停雲台巡邏、停火，保持跟隨/穿越語義 |
| `FaceMode` | BT + `navi_tf_bridge` | `behavior_tree` + `map_aim_point_node` | 朝向固定地圖點，輸出 `/ly/face_mode/angles` 再由 BT 轉 `/ly/control/angles` |

## `/ly/vision/mode`

`/ly/vision/mode` 是上位機 ROS2 topic，不直接下發給下位機。

類型：

```text
std_msgs/msg/UInt8
```

取值：

```text
0 = DISABLED
1 = ARMOR
2 = BUFF
3 = OUTPOST
```

發布方：

```text
behavior_tree
```

訂閱方：

```text
detector
buff_hitter
```

作用：

- `1 ARMOR`：開普通裝甲板識別。
- `2 BUFF`：開能量機關識別和 buff_hitter 解算。
- `3 OUTPOST`：開前哨識別鏈路。

它只選視覺 pipeline，不代表小陀螺、開火、導航或雲台巡邏模式。

## FireCode 與打彈

`gimbal_driver` 拆 FireCode 後，現在不要再把 `/ly/control/firecode` 當 `UInt8` 直接發 `99 / 96`。

現在 topic 是：

```text
/ly/control/firecode
gimbal_driver/msg/FireCode
```

消息字段：

| 字段 | 對應下位機 1-byte FireCode bit | 說明 |
|---|---|---|
| `fire_status` | bit0-1 | 開火翻轉位，BT 使用 `0b00 <-> 0b11` |
| `cap_state` | bit2-3 | 電容狀態，`0/1/2/3` |
| `follow_mode` | bit4 | 跟隨模式 |
| `aim_mode` | bit5 | 輔瞄模式 |
| `rotate` | bit6-7 | 小陀螺檔位，`0..3` |
| `raw` | 回讀/兼容字段 | 命令側不要依賴它，`gimbal_driver` 按語義字段寫控制 |

以前直接發：

```text
96 = 0b01100000 = fire_status=0, cap_state=0, follow_mode=0, aim_mode=1, rotate=1
99 = 0b01100011 = fire_status=3, cap_state=0, follow_mode=0, aim_mode=1, rotate=1
```

所以以前的 `96 <-> 99` 本質上就是把 `fire_status` 在 `0` 和 `3` 之間翻轉。現在要打一發，也是翻轉 `fire_status`，不是固定發某一個值。

BT 內部做法：

```text
收到有效目標
  -> 讀 /ly/gimbal/firecode 回讀到 RecFireCode
  -> RecFireCode.FlipFireStatus()  # 0 <-> 3
  -> 發 /ly/control/firecode
```

普通裝甲板/前哨：

- 有有效目標時按 `FireRate` 節流翻轉。
- `AimMode` 會設為輔瞄狀態。

打符：

- 只有 `buffAimData.FireStatus=true` 時立刻翻轉。
- 翻轉後清掉 `buffAimData.FireStatus`，避免同一個狀態重複觸發。

`behavior_tree` 發的是完整快照：

```text
field_mask = FIELD_ALL = 31
fire_status = 0 or 3
cap_state = current cap state
follow_mode = current follow mode
aim_mode = current aim mode
rotate = current rotate gear
raw = semantic fields packed back to one byte
```

如果手動測試，等價於舊 `96 / 99` 的新命令是：

```bash
# 等價舊 99：fire_status=3, aim_mode=true, rotate=1
ros2 topic pub --once /ly/control/firecode gimbal_driver/msg/FireCode \
  "{field_mask: 31, fire_status: 3, cap_state: 0, follow_mode: false, aim_mode: true, rotate: 1, raw: 99}"

# 等價舊 96：fire_status=0, aim_mode=true, rotate=1
ros2 topic pub --once /ly/control/firecode gimbal_driver/msg/FireCode \
  "{field_mask: 31, fire_status: 0, cap_state: 0, follow_mode: false, aim_mode: true, rotate: 1, raw: 96}"
```

注意：

- 打彈要靠 `fire_status` 相對上一次狀態翻轉，實機上建議先看 `/ly/gimbal/firecode` 的回讀，再發相反值。
- 不要同時讓外部腳本和 `behavior_tree` 搶 `/ly/control/firecode`。
- `field_mask=0` 或 `31` 都是完整快照；部分字段更新可用 mask，但未更新字段超過 `firecode_partial_hold_ms` 會回 0，默認 100ms。
- `/ly/gimbal/firecode` 是 `gimbal_driver` 回讀 topic，用來檢查實際進入主控制幀的語義字段。

## JSON `Task`

目前沒有 `/ly/task` ROS2 topic。現在的 `Task` 是 BT JSON 裡的配置字段：

```json
"Task": {
  "Buff": false,
  "Outpost": false
}
```

作用：

- `Task.Buff=true`：BT 允許進 `AimMode::Buff`。
- `Task.Outpost=true`：BT 允許進 `AimMode::Outpost`。
- 兩者都是 `false`：普通裝甲板模式，通常是 `AimMode::RotateScan`。

舊的 `GameStrategy.HitBuff / HitOutpost` 仍會先兼容填到 `TaskSettings`，但新語義應該看 `Task.Buff / Task.Outpost`。

## 打裝甲板

普通 regional 默認：

```json
"Task": {
  "Buff": false,
  "Outpost": false
}
```

因此 BT 內部會進普通 `AimMode::RotateScan`，並發布：

```text
/ly/vision/mode = 1
```

數據流：

```text
behavior_tree AimMode::RotateScan
  -> /ly/vision/mode=1
  -> detector publishes /ly/detector/armors
  -> predictor / BT 產生自瞄角度
  -> /ly/control/angles
  -> /ly/control/firecode
```

如果有目標，BT 用目標角度接管雲台，並按 `FireRate` 翻轉 firecode 開火。沒目標時，BT 進雲台巡邏掃描。

## 打符

啟用條件：

```json
"Task": {
  "Buff": true
}
```

主要邏輯：

- 開局 `25s` 內才打。
- `buff_shoot_count <= 15`。
- 開局 `7s` 後會看 `event_data`，如果己方能量機關已激活，就退出 Buff 回普通掃描。
- 未激活時保持 `AimMode::Buff`。

輸出：

```text
/ly/vision/mode = 2
```

導航與朝向：

- 導航去 `BuffOutpost` 點。
- FaceMode 朝向 `BuffPose`。
- 識別到符時，視覺角度優先；沒識別到時，FaceMode 提供粗朝向。

## EventData

`/ly/game/event_data` 是 `gimbal_driver` 從裁判系統 `0x0101 event_data` 拆出來的語義 topic。

類型：

```text
gimbal_driver/msg/EventData
```

同時還保留 raw 來源：

```text
/ly/game/all.exteventdata
ly/gimbal/eventdata
```

目前按 `RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）` 的 `0x0101` layout 拆。不要用 `RoboMaster 裁判系统串口协议附录 V1.9.0（20250703）` 的 `event_data` layout 混判。

字段：

| bit 範圍 | ROS 字段 | 類型 | 說明 |
|---|---|---|---|
| 0-31 | `raw` | `uint32` | 原始 `event_data` |
| 0-2 | `self_supply_status` | `uint8` | 己方補給區狀態，3 bit 合併值 |
| 0 | `self_supply_occupied` | `bool` | 己方補給區占用位 |
| 1 | `self_supply_reserved` | `bool` | 己方補給區保留位 |
| 2 | `self_rmul_supply_occupied` | `bool` | RMUL 補給區占用位 |
| 3-4 | `self_small_energy_status` | `uint8` | 己方小能量機關狀態 |
| 5-6 | `self_large_energy_status` | `uint8` | 己方大能量機關狀態 |
| 7-8 | `self_central_highland_status` | `uint8` | 己方中央高地狀態 |
| 9-10 | `self_trapezoid_highland_status` | `uint8` | 己方梯形高地狀態 |
| 11-19 | `enemy_last_dart_hit_time` | `uint16` | 對方飛鏢最後一次命中時間 |
| 20-22 | `enemy_last_dart_hit_target` | `uint8` | 對方飛鏢最後一次命中目標 |
| 23-24 | `center_gain_point_status` | `uint8` | 中心增益點狀態 |
| 25-26 | `self_fortress_gain_point_status` | `uint8` | 己方堡壘增益點狀態 |
| 27-28 | `self_outpost_gain_point_status` | `uint8` | 己方前哨站增益點狀態 |
| 29 | `self_base_gain_point_status` | `bool` | 己方基地增益點狀態 |
| 30-31 | `reserved` | `uint8` | 保留位 |

BT 目前實際使用：

- `self_small_energy_status`
- `self_large_energy_status`
- `raw` 會同步到 blackboard / trace

打符任務中，BT 只在 event data 新鮮時看：

```text
self_small_energy_status == 1
or
self_large_energy_status == 1
```

只要其中一個成立，就認為己方能量機關已激活，退出 `AimMode::Buff`，回普通裝甲板掃描。其他 EventData 字段目前主要是已拆好、可給後續決策用，還沒有大面積接入 regional 任務。

## 打前哨

啟用條件：

```json
"Task": {
  "Outpost": true
}
```

主要邏輯：

- `enemyOutpostHealth > 0` 才打。
- 開局 `90s` 內保持 `AimMode::Outpost`。
- 前哨血量歸零或超時後退回普通掃描。

輸出：

```text
/ly/vision/mode = 3
```

導航與朝向：

- 導航去 `BuffOutpost` 點。
- FaceMode 朝向敵方 `OutpostPose`。
- 識別到前哨時，前哨視覺角度優先；沒識別到時，FaceMode 提供粗朝向。

## 雲台巡邏掃描

雲台巡邏不是 detector 做的，是 `behavior_tree` 自己算角度後發：

```text
/ly/control/angles
```

觸發條件大致是：

```text
沒有識別到目標
不是 Buff 模式
StopScan=false
距離上次看到目標超過 2s
```

配置：

```json
"PatrolScan": {
  "Mode": 2
}
```

目前 `Mode=2` 是左右擺頭巡邏，yaw/pitch 都由 BT 在 GameLoop 裡計算。看到目標後，巡邏狀態會被重置，雲台改用目標角度。

## 區域巡邏

區域巡邏不是 `/ly/vision/mode` 管的，是 `AreaManager` 管的。

已寫好的 regional 區域狀態機：

- `MyBase`：`CastleLeft1 -> CastleLeft2 -> CastleRight2 -> CastleRight1` 循環。
- `MyHighland`：Highland 進入、BuffShoot 駐守、HoleRoad 離開。
- `MyRoadland`：CentralToBase / BaseToCentral 強綁定穿越與駐守。
- `CommonCentral`：中場巡邏路線。

是否會被正式 regional 選中，取決於 JSON：

```json
"DecisionAutonomy": {
  "NaviGoal": {
    "MyArea": {
      "Base": true,
      "Highland": true,
      "Roadland": false
    },
    "EnemyArea": {
      "Base": false,
      "Highland": false,
      "Roadland": false
    },
    "CommonArea": {
      "Central": false
    }
  }
}
```

`AreaManager.yaml` 管狀態機參數和 DefaultPolicy 權重；JSON scope 管哪些區域允許被選。JSON 裡是 `false` 的區域，即使血量彈量健康，也不會被 Default 選中。

## FollowMode

`FollowMode` 是 firecode 裡的語義位，會發到：

```text
/ly/control/firecode
```

目前語義：

- 停底盤小陀螺。
- 停雲台巡邏。
- 停火。
- 保持特殊穿越/跟隨控制語義。

典型使用：

- Highland 進入/離開階段。
- Roadland 強綁定穿越階段。

Roadland 強綁定穿越時，`FollowMode + FaceMode + 停火` 會保持到到達終點、不可達或超時，不會因普通目標/防守事件中途釋放。

## FaceMode

`FaceMode` 用於「看不到目標時也大致朝向一個地圖點」。

目標輸入：

```text
/ly/face_mode/target_raw
[official_map_x, official_map_y, map_z]  # cm
```

解算輸出：

```text
/ly/face_mode/angles
```

BT 再轉發到：

```text
/ly/control/angles
```

注意：

- FaceMode 只管雲台朝向。
- FaceMode 不會自己停小陀螺。
- FaceMode 可以停雲台巡邏，並按配置停火。
- 如果 Buff/Outpost 視覺已識別到目標，視覺角度優先於 FaceMode 粗朝向。

## 目前正式 regional 默認狀態

正式 `regional_competition.json` 目前：

```json
"Task": {
  "Buff": false,
  "Outpost": false
}
```

所以默認不是打符，也不是打前哨，而是普通裝甲板掃描：

```text
AimMode::RotateScan
  -> /ly/vision/mode=1
  -> 雲台巡邏 / 裝甲板識別 / 普通開火鏈路
```

同時正式 regional 的區域 scope 默認只開：

```text
MyArea.Base
MyArea.Highland
```

因此 Default 底層區域任務目前只會在我方 Base / Highland 裡選。Roadland / Central 狀態機已存在，但正式配置沒有打開。
