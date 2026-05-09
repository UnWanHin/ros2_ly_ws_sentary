# Vision / Task / Patrol Mode Flow

Updated: 2026-05-09

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

## Task YAML

目前沒有 `/ly/task` ROS2 topic。現在的 `Task` 是 BT 配置字段，正式入口集中在 `src/behavior_tree/config/Task.yaml`：

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

舊任務開關解析兼容已移除；新語義只看 `Task.Buff / Task.Outpost`。
如果 JSON 和 `Task.yaml` 同時寫了 `Task.Buff/Outpost`，`Task.yaml` 會覆蓋 JSON，用來決定這局是否打符或打前哨。

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

- 默認不走定時窗口，而是看 `/ly/game/sentry/info.can_activate_energy_mechanism`。
- 當 `can_activate_energy_mechanism=true`，或 `/ly/game/event_data` 顯示己方能量機關已在 `正在激活`，BT 進 `AimMode::Buff`。
- 若 `Task.BuffTimer.Enable=true`，則兼容定時窗口：窗口內允許進 Buff，窗口外退出。
- 當 `event_data` 顯示已激活，且 `sentry_info` 沒有新的可激活窗口時，退出 Buff 回普通掃描。
- 打符任務一旦鎖住，不會被 RegionalDefense 或視野裡的普通裝甲板打斷；只會在成功激活、任務超時退化、或近期單次扣血超過 `DamageAbortThreshold` 時退出。

輸出：

```text
/ly/vision/mode = 2
/ly/control/sentry_cmd.confirm_energy_activate = pulse
```

導航與朝向：

- 導航去 `BuffOutpost` 點。
- FaceMode 朝向 `BuffPose`。
- 識別到符時，視覺角度優先；沒識別到時，FaceMode 提供粗朝向。
- 只有同時滿足「到達 `BuffOutpost`」、「`buff_hitter` 識別並給出可擊打狀態」、「`can_activate_energy_mechanism=true`」時，BT 才會向 `/ly/control/sentry_cmd` 發 `FIELD_CONFIRM_ENERGY_ACTIVATE` 脈衝。
- 默認狀態模式下，`FireStatus` 會在 `/ly/game/event_data` 顯示己方小/大能量機關 `正在激活(2)` 後翻轉；同時確認脈衝後有 `PostConfirmGraceMs` 的延時保護，避免裁判回包從可激活切到正在激活時的延遲讓開火被卡死。
- 若打符過程近期單次扣血大於 `DamageAbortThreshold`，BT 會退出 Buff，保持普通裝甲板視覺模式 `DamageAbortHoldMs`，之後仍可在可激活/正在激活條件恢復時回到打符。
- 打符期間姿態固定選 `Move(3)`，不切 `Attack(1)`；扣血中斷回普通裝甲板後才恢復原姿態策略。

相關 `Task.yaml`：

```yaml
Task:
  Buff: false
  BuffTimer:
    Enable: false
    StartSec: 0
    EndSec: 25
    MaxShootCount: 15
  BuffConfirm:
    RefereeFreshTimeoutMs: 2000
    PulseMs: 500
    RetryIntervalMs: 2000
    PostConfirmGraceMs: 3000
    TaskHoldTimeoutMs: 30000
    DamageAbortThreshold: 30
    DamageAbortWindowMs: 1000
    DamageAbortHoldMs: 5000
```

規則/通信差異：

- 2026 超級對抗賽規則把哨兵接入了能量機關確認鏈路：裁判 `0x020D sentry_info_2.bit14` 告訴哨兵「己方能量機關是否能進入正在激活」，裁判 `0x0120 sentry_cmd.bit23` 是哨兵確認使其進入正在激活。
- 2025 通信協議裡 `sentry_info_2.bit14` 和 `sentry_cmd.bit23` 仍是保留位；舊邏輯只能按規則時間窗/客戶端狀態去打，沒有哨兵串口確認位。
- 因此新 BT 不再把「未激活」直接等同於「可以去打」，默認要等 `can_activate_energy_mechanism=true`，到點並鎖到模型後才下發確認脈衝。

## EventData

`/ly/game/event_data` 是 `gimbal_driver` 從裁判系統 `0x0101 event_data` 拆出來的語義 topic。

類型：

```text
gimbal_driver/msg/EventData
```

`gimbal_driver` 同時還保留 raw 來源作調試/兼容；`behavior_tree` 不再依賴它們：

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
- `self_fortress_gain_point_status`
- `self_outpost_gain_point_status`
- `self_base_gain_point_status`
- `raw` 會同步到 blackboard / trace
- `EventManager` 會把上述字段和其他裁判/視覺/導航狀態整理成 `EventSnapshot`，但 `EventSnapshot` 本身不直接發導航或火控。
- RegionalDefense 會在 event data 新鮮且 `self_fortress_gain_point_status == 2 / 3` 時，把己方堡壘增益點視為有敵方占領；此時不去 `Castle`，只在四個 Castle 邊點裡按自身位置選最近點搜索。若己方 Base 大區內新鮮敵方位置數達到 `RegionalDefense.FortressStandEnemyCountMin`，且普通裝甲目標已鎖定並允許開火，才原地停車並把小陀螺調到最高檔；沒有鎖到目標時繼續搜索，不站樁。

打符任務中，BT 在 event data 新鮮時看：

```text
self_small_energy_status == 1 / 2
self_large_energy_status == 1 / 2
```

`1` 表示已激活，`2` 表示正在激活。是否能把能量機關從未激活切到正在激活，不從 event_data 推斷，而是看 `/ly/game/sentry/info.can_activate_energy_mechanism`。

## 打前哨

啟用條件：

```json
"Task": {
  "Outpost": true,
  "OutpostConfirm": {
    "RefereeFreshTimeoutMs": 2000,
    "MaxGameTimeSec": 90,
    "MinSelfHp": 150,
    "MinAmmo": 30,
    "VisualScoutWithoutHp": true,
    "VisualScoutHoldMs": 8000,
    "VisualScoutCooldownMs": 15000,
    "VisualScoutFaceDistanceCm": 300,
    "ArmorInterruptMaxDistanceCm": 1000,
    "DamageAbortThreshold": 30,
    "DamageAbortWindowMs": 1000,
    "DamageAbortHoldMs": 3000
  }
}
```

主要邏輯：

- `Task.Outpost=true` 才允許進前哨任務。
- `/ly/enemy/op_hp` 不是正式 gate；接口保留，若它新鮮且為 0，BT 可提前判定敵方前哨已毀並跳過任務。
- 若 `VisualScoutWithoutHp=true`，且血量/彈量/時間窗/不可達 gate 都通過，會先以普通裝甲模式導航去 `BuffOutpost`；距 `BuffOutpost` 小於 `VisualScoutFaceDistanceCm` 後才切 `AimMode::Outpost`、開 `/ly/vision/mode=3` 和敵方前哨 FaceMode。
- 到達 `BuffOutpost` 後才開始計算 `VisualScoutHoldMs` no-target timeout；到點後仍沒有 `/ly/outpost/target`，則退出並按 `VisualScoutCooldownMs` 冷卻。
- 自身血量、彈量低於 `OutpostConfirm.MinSelfHp / MinAmmo` 時不主動進前哨任務，讓 Hard Recovery 優先處理。
- 默認只在開局 `OutpostConfirm.MaxGameTimeSec=90` 秒內主動打前哨；設 `0` 可關閉時間窗口。
- Roadland 強綁定穿越、RegionalDefense、受擊超過門檻、導航回報 `BuffOutpost` 不可達，都會退出前哨模式。
- 行進/接近過程中若普通裝甲目標有效且距離不超過 `ArmorInterruptMaxDistanceCm`，先保持普通自瞄打車；目標消失或太遠後，回到前哨偵查任務。
- 前哨血量接口回報歸零、視覺偵查超時、視覺偵查冷卻中，或沒有允許 visual scout/近期前哨視覺鎖定時，退回普通掃描。
- `/ly/outpost/target.status` 必須有效，BT 才會把前哨視覺角度視為可用並允許按火控頻率開火。

輸出：

```text
/ly/vision/mode = 3
```

導航與朝向：

- Travel 階段導航去 `BuffOutpost` 點，但保持普通裝甲視覺，不開前哨 FaceMode。
- Approach 階段距 `BuffOutpost` 300cm 左右才 FaceMode 朝向敵方 `OutpostPose`。
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

`src/behavior_tree/config/AreaManager.yaml` 管狀態機參數和 DefaultPolicy 權重；JSON scope 管哪些區域允許被選。JSON 裡是 `false` 的區域，即使血量彈量健康，也不會被 Default 選中。

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

正式 `sentry_all` 會啟動 `map_aim_point_node` 作為 solver；它等待 BT 發 `/ly/face_mode/target_raw` 後，用 `gimbal_barrel_joint` 的 TF 相對幾何算 yaw/pitch，不要求相機先看到目標。

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
