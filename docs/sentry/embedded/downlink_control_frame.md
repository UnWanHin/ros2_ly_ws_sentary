# 上位機下發協議總覽（給下位機）

Updated: 2026-07-12

> 配置归属：`io_config.game_path_fresh_timeout_ms`、自身坐标下发频率/时效与串口 raw
> 诊断都在 `src/gimbal_driver/config/gimbal_driver_config.yaml`。正式 `sentry_all` 与
> 单独 `gimbal_driver` launch 均会加载此 baseline。

> Raw 观测：开启 `io_config.serial_mode=true` 后，每个下行 `DownlinkTypeID` 单独发布到
> `/ly/download/typeid0xNN`，使用带 `header.stamp` 的 `gimbal_driver/msg/GimbalRawFrame`。
> 这只做观测，不改变控制 frame 或既有语义 topic。

## 1. 範圍

本文描述 `gimbal_driver -> 下位機` 的緊湊串口下發 frame。下位機收到後，依
`DownlinkTypeID` 執行底盤/雲台控制或封裝對應的 RM2026 V2.0 裁判幀。

上行 `TypeID` 與下行 `DownlinkTypeID` 是兩個獨立編號空間。所有下行 frame 的
byte 0 都是 `0x21` (`'!'`)，byte 1 是 `DownlinkTypeID`；之後的 frame 長度由 ID 決定。

## 2. Frame 總表

| DownlinkTypeID | 名稱 | 總長度 | 裁判對接 | ROS 來源 |
|---|---|---:|---|---|
| `0x00` | `GimbalControlFrame` | 13B | 無 | `/ly/control/angles`、`/ly/control/vel`、`/ly/control/firecode` |
| `0x01` | `SentryCommandFrame` | 6B | `0x0301 + data_cmd_id=0x0120` | `/ly/control/posture`、`/ly/control/sentry_cmd` |
| `0x02` | `MapPathFrame` | 107B | `0x0307 map_data_t` | `/ly/control/map_path` |
| `0x03` | `CustomInfoFrame` | 36B | `0x0308 custom_info_t` | `/ly/control/custom_info` |
| `0x04` | `SentryCoordinateFrame` | 17B | 下位機自身座標使用 | `/ly/bt/sentry_position` |

下位機必須先讀 byte 1，再按上表讀取剩餘字節；不可再把所有下發資料固定按 17B 解析。

## 3. `GimbalControlFrame`（`0x00`，13B）

| byte offset | 字段 | 類型 | 說明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x00` |
| 2 | `Velocity.X` | `int8` | 底盤 x 速度原始值 |
| 3 | `Velocity.Y` | `int8` | 底盤 y 速度原始值 |
| 4-7 | `GimbalAngles.Yaw` | `float32` | little-endian |
| 8-11 | `GimbalAngles.Pitch` | `float32` | little-endian |
| 12 | `FireCode` | `uint8` | 開火/電容/Follow/Aim/Rotate |

此包不再攜帶 `SentryCmd`。`/ly/control/angles`、`/ly/control/vel`、
`/ly/control/firecode` 任一更新，都會更新 shadow 並發一包 `0x00`。

## 4. `SentryCommandFrame`（`0x01`，6B）

| byte offset | 字段 | 類型 | 說明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x01` |
| 2-5 | `SentryCmd` | `uint32` | little-endian，RM2026 V2.0 `0x0120 sentry_cmd` 命令字 |

`/ly/control/posture` 只更新姿態字段，會立即發送並按既有參數重發；
`/ly/control/sentry_cmd` 按 `field_mask` 更新完整命令 shadow 後發送一包。

### 4.1 `SentryCmd` 位語義（RM2026 V2.0）

| bit | ROS 字段 | 說明 |
|---|---|---|
| 0 | `confirm_free_revive` | 確認免費復活 |
| 1 | `confirm_immediate_revive` | 確認兌換立即復活 |
| 2-12 | `exchange_projectile_allowance` | 非遠程兌換允許發彈量累計值，必須單調遞增 |
| 13-16 | `remote_projectile_exchange_count` | 遠程兌彈請求次數，每次只加 1 |
| 17-20 | `remote_hp_exchange_count` | 遠程回血請求次數，每次只加 1 |
| 21-23 | `posture` | `1` 進攻、`2` 防禦、`3` 移動、`4` 強化進攻、`5` 強化防禦、`6` 強化移動 |
| 24 | `confirm_energy_activate` | 確認能量機關進入正在激活狀態 |
| 25-31 | - | 保留，填 0 |

## 5. `MapPathFrame`（`0x02`，107B）

此 frame 的 byte 2-106 是裁判 `0x0307 map_data_t` 原始 payload。下位機以自身機器人
ID/裁判發送流程封裝 `0x0307`。

正式導航鏈路是 `/ly/navi/path`（`nav_msgs/Path`，`map` frame、m）經
`map_path_to_game_path_node` 用 `navi_tf_bridge` 校準矩陣反算成 official-map dm，發布
`/ly/game/path`（`gimbal_driver/msg/MapPath`）後由 `gimbal_driver` 下發。本 bridge 固定
`intention=3`，最多取 50 點，第一點寫 start，後 49 點寫相鄰 delta。輸入 `header.stamp`
會原樣保留到 `/ly/game/path.header.stamp` 供 ROS 觀察；`map_data_t` 本身沒有 timestamp 欄位，
所以串口 `0x02` 無法攜帶時間戳。

`gimbal_driver` 不會週期性重發已收的 path。它只在收到 topic 消息時嘗試下發；對正式
`/ly/game/path`，`header.stamp` 為 0 或距上位機 ROS 時間超過
`io_config.game_path_fresh_timeout_ms`（預設 5000ms）會拒絕下發。收到帶新 timestamp 的 path
才恢復發送。這避免導航停止更新或上游重播舊 path 時持續塗亂小地圖。

`SenderId` 使用裁判附錄二的自身**機器人 ID**：紅方哨兵為 `7`，藍方哨兵為 `107`。
`gimbal_driver` 將此身份封裝為 `/ly/game/sentry/info.self_robot_id`（隊色未知時為 `0`）；
`map_path_to_game_path_node` 只訂閱該 SentryInfo 欄位。`self_robot_id=0` 時不發布
`/ly/game/path`，因此不會用固定 `0` 或選手端 ID 下發。

`/ly/control/map_path` 保留為既有手動/測試相容入口；兩個 topic 都會下發同一種 `0x02` frame，
現場不可同時發布兩者，避免重複送路徑。

| byte offset | 字段 | 類型 | 說明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x02` |
| 2 | `Intention` | `uint8` | `1` 到點攻擊、`2` 到點防守、`3` 移動到點 |
| 3-4 | `StartPositionX_dm` | `uint16` | 小地圖起點 x，dm |
| 5-6 | `StartPositionY_dm` | `uint16` | 小地圖起點 y，dm |
| 7-55 | `DeltaX_dm[49]` | `int8[49]` | 相對上一點的 x 增量，dm |
| 56-104 | `DeltaY_dm[49]` | `int8[49]` | 相對上一點的 y 增量，dm |
| 105-106 | `SenderId` | `uint16` | 裁判發送者 ID |

上位機只接受 `intention=1/2/3`。`MapPath.msg` 的 `delta_x_dm`、`delta_y_dm` 必須各恰好 49 個元素。

## 6. `CustomInfoFrame`（`0x03`，36B）

此 frame 的 byte 2-35 是裁判 `0x0308 custom_info_t` 原始 payload。下位機以裁判發送流程封裝
`0x0308`，接收者僅可填己方允許的選手端 ID。

| byte offset | 字段 | 類型 | 說明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x03` |
| 2-3 | `SenderId` | `uint16` | 裁判發送者 ID |
| 4-5 | `ReceiverId` | `uint16` | 裁判接收選手端 ID |
| 6-35 | `UserDataUtf16` | `uint8[30]` | 完整 30B UTF-16 原始字節，按裁判規定字節序 |

`/ly/control/custom_info` 不做字串編碼轉換；上游必須已提供 30B UTF-16 原始資料。

## 7. `SentryCoordinateFrame`（`0x04`，17B）

| byte offset | 字段 | 類型 | 說明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x04` |
| 2-3 | `X_cm` | `int16` | official-map x，cm，little-endian |
| 4-5 | `Y_cm` | `int16` | official-map y，cm，little-endian |
| 6-15 | `Reserved` | `uint8[10]` | 填 0 |
| 16 | `CRC8` | `uint8` | 對 byte 0-15 計算 |

資料來源是 `/ly/bt/sentry_position`。上位機預設每 100ms 最多發一次；位置超過 2000ms 未刷新則停止發送。
CRC8 參數：poly `0x31`、init `0xFF`、非反射。

## 8. 舊協議遷移

| 舊布局 | 新布局 |
|---|---|
| `0x00` 固定 17B，最後 4B 為 `SentryCmd` | `0x00` 改 13B，只保留速度/角度/FireCode |
| `SentryCmd` 夾在控制包 byte 13-16 | 獨立為 `0x01` 6B frame |
| `0x01` 是自身座標 | 自身座標改為 `0x04`，內容與 CRC8 規則不變 |
| 無 `0x0307` / `0x0308` compact frame | 新增 `0x02` 107B 路徑、`0x03` 36B 自訂訊息 |

此變更不保留舊下行解析兼容。下位機與上位機必須同時切換，否則舊 `0x01` 座標包會被新固件當作
`SentryCommandFrame`，造成錯誤解析。
