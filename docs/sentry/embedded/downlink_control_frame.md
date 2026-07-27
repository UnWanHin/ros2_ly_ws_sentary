# 上位機下發協議總覽（給下位機）

Updated: 2026-07-27

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
byte 0 都是 `0x21` (`'!'`)，byte 1 是 `DownlinkTypeID`；之後的 frame 長度由 ID 決定。`0x02`
例外地以兩個固定 64B fragment 組成一份邏輯路徑。

## 2. Frame 總表

| DownlinkTypeID | 名稱 | 總長度 | 裁判對接 | ROS 來源 |
|---|---|---:|---|---|
| `0x00` | `GimbalControlFrame` | 13B | 無 | `/ly/control/angles`、`/ly/control/vel`、`/ly/control/firecode` |
| `0x01` | `SentryCommandFrame` | 6B | `0x0301 + data_cmd_id=0x0120` | `/ly/control/posture`、`/ly/control/sentry_cmd` |
| `0x02` | `MapPathFragmentFrame` | 64B x2 | 重組後 `0x0307 map_data_t` | `/ly/control/map_path` |
| `0x03` | `CustomInfoFrame` | 36B | `0x0308 custom_info_t` | `/ly/control/custom_info` |
| `0x04` | `SentryCoordinateFrame` | 17B | 下位機自身座標使用 | `/ly/bt/sentry_position` |
| `0x05` | `GimbalTrajectoryFrame` | 26B | MPC 云台原子轨迹 | `/ly/control/trajectory` (`gimbal_driver/msg/GimbalTrajectory`) |

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

## 5. `MapPathFragmentFrame`（`0x02`，64B x2）

完整的邏輯路徑仍是 107B：`!`、`0x02` 與 105B 裁判 `0x0307 map_data_t` payload。為符合
下位機「每次串口寫入最多 64B」限制，driver 把那 105B payload 分成兩個固定 64B fragment；
兩段到齊前，下位機不得封裝或發送裁判 `0x0307`。

正式導航路徑來源是 `/Path_downsampled`（`nav_msgs/Path`，`map` frame、m）經
`map_path_to_game_path_node` 用 `navi_tf_bridge` 校準矩陣反算成 official-map dm，發布
`/ly/game/path`（`gimbal_driver/msg/MapPath`）後由 `gimbal_driver` 下發。本 bridge 固定
`intention=3`，最多取 50 點，第一點寫 start，後 49 點寫相鄰 delta。輸入 `header.stamp`
會原樣保留到 `/ly/game/path.header.stamp` 供 ROS 觀察；`map_data_t` 本身沒有 timestamp 欄位，
所以串口 `0x02` 無法攜帶時間戳。

`gimbal_driver` 不會週期性重發已收的 path。它只在收到 topic 消息時嘗試下發；兩個 MapPath
入口共用裁判 `0x0307` 的 **1 Hz** 限頻，時間窗口內的新路徑會直接丟棄、不排隊。每次接受的
path 使用一個新的 8-bit sequence，連續寫出 index `0`、`1` 兩段，不插入 sleep 或重試。對正式
`/ly/game/path`，`header.stamp` 為 0 或距上位機 ROS 時間超過
`io_config.game_path_fresh_timeout_ms`（預設 5000ms）會拒絕下發。收到帶新 timestamp 的 path
才恢復發送。這避免導航停止更新或上游重播舊 path 時持續塗亂小地圖。

`SenderId` 使用裁判附錄二的自身**機器人 ID**：紅方哨兵為 `7`，藍方哨兵為 `107`。
`gimbal_driver` 將此身份封裝為 `/ly/game/sentry/info.self_robot_id`（隊色未知時為 `0`）；
單獨 `ros2 launch gimbal_driver gimbal_driver.launch.py` 預設也會啟動該 bridge（須先 source
同一工作區的 `navi_tf_bridge`）；正式 `sentry_all` 僅保留這一個 owner，避免同一路徑重複下發。
`map_path_to_game_path_node` 只訂閱該 SentryInfo 欄位。`self_robot_id=0` 時不發布
`/ly/game/path`，因此不會用固定 `0` 或選手端 ID 下發。

`/ly/control/map_path` 保留為既有手動/測試相容入口；兩個 topic 都會下發同一組 `0x02` fragments，
現場不可同時發布兩者，避免重複送路徑。

| byte offset | 字段 | 類型 | 說明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x02` |
| 2 | `Sequence` | `uint8` | 每份邏輯路徑遞增；兩段必須相同 |
| 3 | `FragmentIndex` | `uint8` | 第一段 `0`，第二段 `1` |
| 4 | `FragmentCount` | `uint8` | 固定 `2` |
| 5 | `PayloadLength` | `uint8` | 第一段 `56`，第二段 `49` |
| 6-61 | `Payload[56]` | `uint8[56]` | 105B `map_data_t` 按序切片；第二段未使用位置填 0 |
| 62-63 | `CRC16` | `uint16` | little-endian；計算 byte 0-61 |

CRC16 使用 reflected `0x1021`（右移多項式 `0x8408`）、init `0xFFFF`、無 xorout；算法與
`2026HeroAim` 的 64B chunk 一致。下位機驗證 `HeadFlag`、ID、CRC、`FragmentCount=2`、index、
payload 長度與 sequence；只有 index 0/1 同 sequence 都有效時才依序拼回 105B payload，再補回
`!`/`0x02` 形成原始 107B `MapPathFrame`。收到新的 index 0、錯誤 CRC、越界欄位或重複/不匹配 sequence
時，丟棄未完成暫存，不得使用半份路徑。

下位機可用 ASCII `123456789` 驗證此 CRC16 算法，結果必須為 `0x6F91`。

上位機只接受 `intention=1/2/3`。邏輯 `MapPath.msg` 的 `delta_x_dm`、`delta_y_dm` 仍各恰好 49 個元素：
50 點規格沒有縮減。`/ly/download/typeid0x02` 與 raw log 會各記錄兩個實際 64B fragment。
下位機的完整重組狀態機、錯誤處理、裁判 `0x0307` 一次性封裝與聯調步驟見
[map_path_fragment_reassembly.md](map_path_fragment_reassembly.md)。

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

## 8. `GimbalTrajectoryFrame`（`0x05`，26B）

此 frame 与旧 `0x00` 控制帧并行发送，不携带底盘速度和 FireCode。所有浮点数为
IEEE-754 little-endian，单位为角度 `deg`、角速度 `deg/s`、角加速度 `deg/s^2`。

| byte offset | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x05` |
| 2-5 | `Yaw` | `float32` | 目标 yaw 角 |
| 6-9 | `Pitch` | `float32` | 目标 pitch 角 |
| 10-13 | `YawOmega` | `float32` | 目标 yaw 角速度 |
| 14-17 | `PitchOmega` | `float32` | 目标 pitch 角速度 |
| 18-21 | `YawAlpha` | `float32` | 目标 yaw 角加速度 |
| 22-25 | `PitchAlpha` | `float32` | 目标 pitch 角加速度 |

驱动从 `/ly/control/trajectory`（`gimbal_driver/msg/GimbalTrajectory`）缓存六个字段，每次消息更新
额外写入一帧；旧 `0x00` 控制帧仍由 `/ly/control/angles`、`/ly/control/vel` 和
`/ly/control/firecode` 原路径发送。驱动使用 SensorData QoS，并拒绝六个字段中包含
`NaN/Inf` 的消息；轨迹停止后的 200ms 回退由下位机负责，驱动不会伪造禁用帧。

## 9. 舊協議遷移

| 舊布局 | 新布局 |
|---|---|
| `0x00` 固定 17B，最後 4B 為 `SentryCmd` | `0x00` 改 13B，只保留速度/角度/FireCode |
| `SentryCmd` 夾在控制包 byte 13-16 | 獨立為 `0x01` 6B frame |
| `0x01` 是自身座標 | 自身座標改為 `0x04`，內容與 CRC8 規則不變 |
| 無 `0x0307` / `0x0308` compact frame | 新增 `0x02` 兩段 64B 路徑（重組 107B）、`0x03` 36B 自訂訊息 |

此變更不保留舊下行解析兼容。下位機與上位機必須同時切換，否則舊 `0x01` 座標包會被新固件當作
`SentryCommandFrame`，造成錯誤解析。
