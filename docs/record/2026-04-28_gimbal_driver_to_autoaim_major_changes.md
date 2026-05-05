# Gimbal Driver 到輔瞄鏈路大變動總結

日期：2026-04-28

## 範圍

這份記錄總結從 `gimbal_driver` 語義化改動開始，到後續 `DamageOpenGate` 通用化與輔瞄 100ms 目標保持為止的大變動。

這段改動的核心是：把下位機仍然需要的 raw/bitfield 通訊保留在 `gimbal_driver` 內部，但讓 ROS topic 層看到更清楚的語義消息。

## 新增消息

本輪新增的 `gimbal_driver/msg` 消息有 4 個：

| Message | 主要用途 | 主要 topic |
| --- | --- | --- |
| `FireCode.msg` | 拆解下位機 1-byte firecode，包含開火、電容、吊射、aim mode、rotate 位 | `/ly/control/firecode`, `/ly/gimbal/firecode` |
| `ControlVelocity.msg` | 讓上游用 m/s 發底盤速度，也保留 raw int8 直通模式 | `/ly/control/vel` |
| `EventData.msg` | 按 RM2026 V1.3.0 拆解裁判系統 `0x0101 event_data` | `/ly/game/event_data` |
| `RfidStatus.msg` | 按 RM2026 V1.3.0 拆解 `0x0209 rfid_status` 低 32 位 | `/ly/me/rfid` |

既有消息繼續保留，例如 `GimbalAngles.msg`、`Vel.msg`、`GameData.msg`、`Chassis.msg`、`BuffData.msg`、`Health.msg`、`PositionData.msg` 等。

## Topic 與鏈路改動

`/ly/control/firecode` 從看不出語義的 `std_msgs/msg/UInt8` 改成 `gimbal_driver/msg/FireCode`。

`/ly/gimbal/firecode` 也同步改成 `gimbal_driver/msg/FireCode`，回傳時 `field_mask=FIELD_ALL` 並帶 `raw`，方便同時看語義位和下位機原值。

`/ly/control/vel` 改成 `gimbal_driver/msg/ControlVelocity`：

- `use_raw=false` 時使用 `x_mps/y_mps`
- `use_raw=true` 時直接使用 `raw_x/raw_y`
- 下位機串口主控制幀仍然是原來的 2 個 `int8` 速度欄位

`/ly/game/event_data` 新增語義化 event topic；原始 `GameData.exteventdata` 仍保留在 `/ly/game/all`。

`/ly/me/rfid` 改成語義化 `RfidStatus`。不再保留額外的 `/ly/me/rfid_status`，避免 topic 名字混亂。

## FireCode 行為

`FireCode.msg` 以 bit 語義拆出：

- `fire_status`: bit0-1
- `cap_state`: bit2-3
- `follow_mode`: bit4
- `aim_mode`: bit5
- `rotate`: bit6-7

`field_mask` 支援 partial update。只更新其中一個欄位時，其他欄位會在 `firecode_partial_hold_ms` 內保留舊值；超時後退回 0。

目前 `firecode_partial_hold_ms` 放在 `config/common.yaml`，默認 `100`，`scripts/launch/start_sentry_all.sh` 會讀它並傳進 launch。

## Velocity 行為

`velocity_raw_to_mps` 也放進 `config/common.yaml`，默認 `0.025`。

語義換算是：

- raw `100` 約等於 `2.5m/s`
- raw `-100` 約等於 `-2.5m/s`

這樣上游可以看實際 m/s，不用直接理解下位機 int8 raw 值。

## EventData / RFID 對齊

`EventData.msg` 以 `RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）` 為準，不使用 2025 附錄版的 `0x0101` layout。

`RfidStatus.msg` 目前只拆 `rfid_status` 低 32 位。協議裡額外的 8 位 `rfid_status_2` 暫時沒有進 current TypeID=4 payload；2026-05-05 起 ROS msg 預留了 `rfid_status_2` 欄位，發布端默認標記 `has_rfid_status_2=false`。

## Behavior Tree 同步

`behavior_tree` 已經同步使用新消息：

- 發 `/ly/control/firecode` 時使用 `gimbal_driver/msg/FireCode`
- 發 `/ly/control/vel` 時使用 `gimbal_driver/msg/ControlVelocity`
- 訂閱 `/ly/gimbal/firecode` 時讀語義欄位
- 訂閱 `/ly/game/event_data`、`/ly/me/rfid` 時讀語義消息，同時保留 raw 值進黑板

安全控制、feature test、debug script、selfcheck topic contract 也同步到新 topic type。

## DamageOpenGate 通用化

原本受擊掉血開門邏輯在 `LeagueStrategy` 裡，且 `WaitBeforeGame` 限制 league profile。

現在改成頂層通用配置：

```json
"DamageOpenGate": {
    "Enable": false,
    "HealthDropThreshold": 30
}
```

league、regional、debug/test JSON 都可以用，但默認全關。`gate` 和 `nogate` 仍共用同一套決策鏈路。

## 輔瞄目標保持

對比 `~/sentry.aim` 後，沒有直接搬它的 TF/controller 整套鏈路，而是先把低風險的 100ms target timeout 思路套到現有 BT 下發鏈路。

新增配置：

```json
"LatchedTargetHoldMs": 100
```

行為：

- fresh `/ly/predictor/target` 來時正常立即用新角度
- 沒新 target 時，最多保持上一個有效角度 100ms
- held target 不會額外觸發開火翻轉
- 超時後不再吃舊角度，並清 `FireCode.AimMode`

這只改 BT 下發保持邏輯，沒有把識別鏈路改成 armor-only。

## 明確未改的範圍

這批改動沒有改下位機串口協議。下位機看到的 firecode byte、速度 int8、主控制幀欄位保持原樣。

這批改動也沒有把 detector/tracker/predictor 改成只裝甲板識別：

- detector 仍會跑裝甲板和車框檢測
- car bbox 仍會進 `/ly/detector/armors`
- tracker_solver 仍可用 whole-car matcher
- predictor 仍沿用現有 prediction/controller 鏈路

如果之後要做 armor-only，那是另一個獨立改動，應該單獨記錄 detector、tracker_solver、predictor 的配置和鏈路變更。

## 相關記錄

- `docs/record/2026-04-28_firecode_velocity_event_rfid_semantic_topics.md`
- `docs/record/2026-04-28_damage_open_gate_common.md`
- `docs/record/2026-04-28_autoaim_latched_target_hold.md`
- `docs/record/2026-04-23_eventdata_0x0101_v1_3_0_alignment.md`
- `docs/record/2026-04-23_rfid_0x0209_status32_alignment.md`
