# 新一輪通訊協議變更草案

Updated: 2026-07-11

## 1. 狀態

已開始落地。TypeID 6 和 TypeID 10 的 V2.0 變更已補到 `gimbal_driver`、串口映射文檔和下位機對接清單；後續如果還有新欄位，再在本文繼續追加。

## 2. 涉及範圍

- 下位機串口收發與裁判幀封裝
- 上位機 `gimbal_driver` 的上行 `TypeID` / 下行 `DownlinkTypeID`
- ROS topic、msg、參數與日誌
- 裁判系統 `0x0207`、`0x0208`、`0x0209`、`0x020D`、`0x0301/0x0120`、`0x0303`

## 3. 目前基線

現有對照文件先保留這幾份：

- [src/gimbal_driver/module/BasicTypes.hpp](../../src/gimbal_driver/module/BasicTypes.hpp)
- [docs/sentry/embedded/serial_data_mapping.md](../sentry/embedded/serial_data_mapping.md)
- [docs/sentry/embedded/downlink_control_frame.md](../sentry/embedded/downlink_control_frame.md)
- [docs/rules/](../rules/)

## 4. 需要你下一輪提供的內容

| 項目 | 需要確認的內容 |
|---|---|
| 下位機 | 新增/刪除的幀、字段、位寬、單位、兼容策略 |
| 上位機 | 需要改的 `TypeID`、`DownlinkTypeID`、ROS topic、msg、參數 |
| 裁判系統 | 要對接的裁判幀 ID、bit 位語義、是否升到 V2.0 |
| 兼容性 | 是否保留舊 1.3 行為、是否允許雙版本並存 |

## 5. 記錄模板

### 5.1 下位機

- 接收哪些上行幀：
- 發哪些下行幀：
- 與裁判系統對接的 ID：
- 需要回傳的狀態：
- 是否保留舊協議兼容：

### 5.2 上位機

- `gimbal_driver` 修改點：
- ROS topic 變更：
- 消息/結構體變更：
- 日誌與 raw frame 記錄變更：
- 是否需要同步 simulator / trace schema：

### 5.3 裁判系統

- 來源幀：
- 目標幀：
- 位語義：
- 變更風險：

## 6. 待定結論

- TypeID 6 原 `Posture` 2B 欄位改為 `DamageDifference`。
- `DamageDifference` 來源是 RM2026 通信協議 V2.0.0 `0x0003 game_robot_HP_t` byte offset `8`，型別 `int16_t`。
- 語義是 `己方全隊總傷害 - 對方全隊總傷害`，上位機發布到 `/ly/game/damage_difference` (`std_msgs/msg/Int16`)。
- 姿態回讀不再走 TypeID 6，只走 TypeID 7 的 `0x020D sentry_info_2.posture`，有效 `1/2/3` 發布到 `/ly/gimbal/posture`。
- 下位機需要改 TypeID 6 byte `2~3` 的填充來源；不要再把姿態寫到這兩個 byte。
- 目前未要求雙版本兼容；這是 TypeID 6 欄位語義變更。
- 新增 TypeID 10，12B payload：byte `0~7` 放 `0x020D sentry_info_3`，byte `8~9` 放 `0x0003 ally_outpost_HP`，byte `10~11` 放 `0x0003 enemy_outpost_HP`。
- TypeID 10 前哨血量按官方 `0x0003` 順序：己方/ally 在前，敵方/enemy 在後；不要沿用舊 `GameCodeType` 的 enemy-first bit-field 順序。
- `/ly/friend/op_hp` 和 `/ly/enemy/op_hp` 優先使用 TypeID 10 的精確 `uint16_t` 血量；TypeID 1 的 `GameCode * 25` 只在 TypeID 10 未收到或 1500ms 內沒有更新時 fallback。
- `0` 是合法前哨血量，表示前哨已毀；下位機若暫時拿不到 `0x0003` 前哨血量，應暫停發 TypeID 10，而不是用 `0` 或預設值占位，讓上位機自然 fallback 到 `GameCode * 25`。
- `/ly/game/sentry/info` 新增 `sentry_info_3_raw`、`has_sentry_info_3` 和三種普通/強化姿態剩餘秒數字段；`sentry_info_3` shadow 由 TypeID 10 更新，實際 topic 發布仍跟隨 TypeID 7，避免高頻 HP 包刷新 `sentry_info_2` 新鮮度。
- 下行協議改為 `DownlinkTypeID=0x00~0x04` 分包：`0x00` 13B 角度/速度/FireCode，`0x01` 6B `0x0120 sentry_cmd`，`0x02` 107B `0x0307 map_data_t`，`0x03` 36B `0x0308 custom_info_t`，`0x04` 17B 自身座標。
- `0x0120 sentry_cmd` 已按 RM2026 V2.0 修正：姿態從 bit21-22 的 `1~3` 擴為 bit21-23 的 `1~6`，能量機關確認從 bit23 移到 bit24；舊下行 frame 不保留兼容。
- 上游 BT 姿態計時：`/ly/game/sentry/info` 的 `sentry_info_3` 會帶 TypeID 10 實際接收 age；當 `has_sentry_info_3=true` 且 age 不超過 `Posture.RefereeInfo3FreshMs`（預設 1500ms）時，普通或強化姿態的裁判剩餘秒數優先決定提前輪換和弱化判定。內部 `AccumSec` 永遠繼續累積，裁判資料缺失或過期立即 fallback，避免下位機斷流卡住策略。
- 上游 BT 姿態選擇：裁判 timer 新鮮時，`sentry_info_3` 剩餘秒數為 `0` 的姿態候選會被強烈扣分；低於 `Posture.RefereeRemainWarnSec`（預設 20s）會按接近 0 的程度扣分。若回讀 `enhanced_posture=true`，只給當前攻/防/移類別 `EnhancedCurrentPostureBonus`（預設 +3）保持偏好；本輪不自動下發 `4/5/6` 強化姿態命令。

## 7. 補充說明

等你把具體改法發過來後，我會直接把這份草案補成正式記錄，並同步整理成下位機、上位機、裁判系統三層對照表。
