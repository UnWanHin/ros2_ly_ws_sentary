# RMUC V2.0 哨兵強化姿態規則與工程現況

Updated: 2026-07-27

本文只摘要會影響哨兵決策、裁判介面與下位機命令的規則。完整且唯一的規則依據仍是
`docs/rules/RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V2.0.1（20260629）.pdf`。

## 1. V2.0 相對 V1.4.2 的變更

| 項目 | V1.4.2 | V2.0.1 |
|---|---|---|
| 姿態種類 | 進攻、防禦、移動三種普通姿態 | 保留三種普通姿態，新增強化進攻、強化防禦、強化移動 |
| 強化姿態資源 | 無 | 每一種強化姿態每局最多累計 15 秒；三類計時彼此獨立，用盡後不能再進入該強化姿態 |
| 下行姿態值 | 普通姿態 | `1=進攻`、`2=防禦`、`3=移動`、`4=強化進攻`、`5=強化防禦`、`6=強化移動` |
| 裁判回讀 | 普通姿態與普通姿態剩餘時間 | 另新增「目前是否強化」與三類強化姿態剩餘秒數 |

來源：V1.4.2，PDF 第 116 頁（印刷頁 115）；V2.0.1，PDF 第 115 頁（印刷頁 114）。

## 2. 共同時間規則

- 開局預設為移動姿態。
- 任意姿態切換冷卻為 5 秒。
- 同一基礎姿態累計超過 3 分鐘後，該姿態效果下降。
- 強化姿態仍是其對應基礎姿態的時間風險的一部分：官方例子明示移動姿態已累計 2 分 50 秒後，再進入強化移動 15 秒，會自動回到已下降效果的移動姿態。
- 熱量冷卻與底盤功率上限的姿態係數，均在其他熱量冷卻或底盤功率計算完成後再乘算，並取整數。

## 3. 姿態效果

| 類別 | 強化效果 | 普通效果 | 普通姿態超過 3 分鐘後 |
|---|---|---|---|
| 進攻 | 當前熱量固定為 0；發彈不增加熱量；底盤功率為 1/2；承受傷害增加 25% | 熱量冷卻 3 倍；底盤功率為 1/2；承受傷害增加 25% | 熱量冷卻 2 倍；底盤功率為 1/2；承受傷害增加 25% |
| 防禦 | 99% 防禦增益；底盤功率為 1/2；熱量冷卻為 1/3 | 50% 防禦增益；底盤功率為 1/2；熱量冷卻為 1/3 | 25% 防禦增益；底盤功率為 1/2；熱量冷卻為 1/3 |
| 移動 | 底盤功率上限 200 W；承受傷害增加 25%；熱量冷卻為 1/3 | 底盤功率上限 1.5 倍；承受傷害增加 25%；熱量冷卻為 1/3 | 底盤功率上限 1.2 倍；承受傷害增加 25%；熱量冷卻為 1/3 |

來源：V2.0.1，PDF 第 115 頁（印刷頁 114），5.6.4「哨兵機器人特殊機制」。

## 4. 已落地的資料與命令鏈

```text
裁判 0x020D sentry_info_2 bit15 / sentry_info_3
  -> 下位機 TypeID 7 / 10
  -> /ly/game/sentry/info (gimbal_driver/SentryInfo)
  -> behavior_tree 的 PostureRefereeTimer

behavior_tree /ly/control/posture 或 /ly/control/sentry_cmd
  -> gimbal_driver SentryCommandFrame (DownlinkTypeID 0x01)
  -> 裁判 0x0301 + data_cmd_id 0x0120, bit21-23 posture
```

| 裁判資料 | ROS 字段 | 工程用途 |
|---|---|---|
| `sentry_info_2 bit12-13` | `posture` | 當前基礎類別 `1..3` |
| `sentry_info_2 bit15` | `enhanced_posture` | 當前是否正處於強化姿態 |
| `sentry_info_3 bit0-23` | `attack/defense/move_posture_remaining_s` | 三類普通姿態剩餘秒數 |
| `sentry_info_3 bit32-55` | `enhanced_*_posture_remaining_s` | 三類強化姿態剩餘秒數 |
| `sentry_cmd bit21-23` | `SentryCmd.posture` | 下行接受 `1..6` |

權威實作位置：

- `src/gimbal_driver/msg/SentryInfo.msg`
- `src/gimbal_driver/msg/SentryCmd.msg`
- `src/gimbal_driver/main.cpp`
- `src/behavior_tree/src/SubscribeMessage.cpp`
- `docs/sentry/embedded/serial_data_mapping.md`

## 5. 當前策略邊界

`gimbal_driver` 對姿態值 `4..6` 啟用統一額度 gate：只有最新 TypeID 10 `sentry_info_3` 仍在
`io_config.enhanced_posture_guard.sentry_info3_fresh_ms=1500` ms 內，且該命令對應的強化剩餘秒數大於 0，
才會下發。`4/5/6` 分別核對強攻/強防/強移額度；資料缺失、過期或額度為 0 都直接拒絕該強化命令，不會靜默降級成
普通姿態。普通 `0..3` 不受這個 gate 影響。這個核對同時位於語義 topic 入口與實際 `0x01` 串口 frame
組裝邊界：已快取的強化命令在重發、其他 sentry command 欄位更新或串口重連時若已失效，實際下行 `posture`
會清為 `0`，並取消該快取的強化重發；額度恢復後必須由 BT 再次提出請求，不會自動重送舊快取。

`debug_mode` 的 `/ly/download/typeid0x01` raw 整包直發是明確的串口聯調旁路，不經語義姿態 guard；它只在
raw-downlink test mode 啟用，且會停用正常控制 writer，不能作為正式競賽控制通道。

正式 BT 只在明確任務邊界自動消耗強化資源：連續鎖定敵前哨且其血量新鮮下降時可申請強化進攻 `4`；ProtectHero 到達守護點後維持普通防守 `2`，只有 `Tactical.ProtectHero.EnhancedDefense` 設定窗口內累積扣血達門檻時才可申請強化防守 `5`。強化移動 `6` 只屬於 Regional 的 Recovery 行進：`Tactical.EnhancedPosture.RecoveryMove.Enable=true`、仍未到 Recovery、**己方 HP 回讀與對應強移額度都新鮮**、HP 在 `1..HealthThresholdHp`（預設 `80`）且額度正數時才可申請；到點、血量 0、任一回讀過期、非 Regional 或本次強移 ACK 重試耗盡時保持普通 Move `3`。

ProtectHero 的 `5` 必須同時滿足：已到守護點、`DamageWindowMs` 內累積掉血達 `DamageThresholdHp`、裁判 TypeID 10 `sentry_info_3` 新鮮、強化防守剩餘秒數大於零、既有 5 秒姿態冷卻完成；下位機 `/ly/gimbal/posture` 與 `enhanced_posture` 的新鮮匹配回讀才確認。資料過期、資源用盡或未達 burst 時保持普通防守 `2`；強防 ACK 重試耗盡時，本次守護維持普通防守且不重複刷命令。

已確認且由本次 ProtectHero 姿態請求擁有的強防，只在本次姿態切換的 5 秒冷卻內暫緩 Regional Recovery，避免 99% 減傷生效時因短暫低血直接離開守護點；冷卻結束後若仍符合 Recovery 門檻，或強防結束、回讀失鮮、任務離開，既有 Recovery 立即重新接管，並以 Move `3` 進入既有 Recovery 導航鏈路。單獨的下位機強防回讀不會延後 Recovery。

正常讀條復活以本機新鮮自身血量的 `0 -> 正數` 轉換判定，預設 `30` 秒不自動申請 `6`，避免在官方最多 30 秒的虛弱/無敵階段消耗強移資源。這不是 `out_of_combat` 判斷；即時復活並不符合此 `0 -> 正數` 條件。此等待可由 `Tactical.EnhancedPosture.RecoveryMove.RespawnSuppressSec` 改動。

TypeID 7 的 `enhanced_posture=true` 與 TypeID 10 對應強化剩餘為 0 可能短暫跨幀不同步。BT 因此必須連續看到此矛盾超過 `Tactical.EnhancedPosture.ContradictionGraceMs=500` ms 才隔離強化確認/新請求；隔離後停止把強化當作已確認，並讓既有 5 秒切換冷卻後的普通姿態命令收斂，不會以單一 0 秒回讀立刻強制切姿態。

## 6. 任務級資源使用

前哨與英雄保護都是明確的戰術駐守邊界。強化姿態是短時、不可恢復的資源，不會因為看到一般目標、普通巡邏或行進而啟用。ProtectHero 在行進中保持原有 Transit 姿態；只有到守護點才使用強防，因為強防也會將底盤功率降至一半。
