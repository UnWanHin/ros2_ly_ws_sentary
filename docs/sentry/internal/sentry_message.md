# 哨兵選手端通知

Updated: 2026-07-28

`sentry_message_node` 是 `behavior_tree` 套件內獨立、只讀的通知節點。它不參與 BT 仲裁，
不修改導航、雲台或火控；唯一輸出是 `/ly/control/custom_info`，由 `gimbal_driver` 轉為下行
`DownlinkTypeID=0x03`，再由下位機封裝裁判 `0x0308 custom_info_t`。

```text
既有裁判 / 位置 / aim topic
-> sentry_message_node
-> /ly/control/custom_info
-> gimbal_driver 0x03
-> 下位機 -> 裁判 0x0308 -> 己方選手端
```

## 安全開關

設定檔是 `src/behavior_tree/config/Message.yaml`：

```yaml
SentryMessage:
  Enable: false
```

預設為 `false`。節點在這個值為 false 時不建立 publisher、subscriber 或 timer，因此不能產生
任何自訂訊息下發。啟用前還必須設定接收者表；`Recipients.Red/Blue.*` 的預設值都是 `0`，
`0` 不會作為裁判接收者使用。這兩個條件避免測試中的刷屏影響隊友。

`Recipients` 填的是**選手端 receiver ID**，不是機器人 ID。發送者由
`/ly/game/sentry/info.self_robot_id` 決定，紅方哨兵為 7、藍方哨兵為 107；節點未取得有效自身
ID 時同樣不會發送。

## 目前事件

所有血量與場地事件只接受有非零、未過期 header stamp 的資料；第一次收到資料只建立基線，
不通知。每個事件有自己的冷卻，並共同受 `GlobalMinIntervalMs` 限制。

| 事件 | 資料來源 | 觸發條件 | 接收者 |
|---|---|---|---|
| `ATTACK <id>` | `/ly/aim/select_target`、`/ly/aim/result` | fire 從 false 變 true，且有已選 target | 全部已配置己方選手端 |
| `BASE HIT -N` | `/ly/friend/base_hp` | 新鮮 HP 相對上次基線下降 | 全部已配置己方選手端 |
| `OUTPOST HIT -N` | `/ly/friend/op_hp` | 新鮮 HP 相對上次基線下降 | 全部已配置己方選手端 |
| `CASTLE ALERT` | `/ly/game/event_data` | 己方堡壘狀態由非 2/3 轉為 2 或 3 | 全部已配置己方選手端 |
| `SENTRY NEAR` | `/ly/position/data` | 哨兵首次進入某友軍的距離閾值 | 該友軍選手端 |
| `ENEMY <id> <m>M` | `/ly/position/data` | 某友軍最近敵人進入威脅距離，或最近敵人 ID 改變 | 該友軍選手端 |

Base/Outpost 以「扣血」而非「未滿血」為事件；因此不會因為長期殘血持續發送。

## 文字與節流

裁判欄位固定為 30 bytes UTF-16LE，最多 15 個 BMP 中文字元或 15 個 ASCII 英文/數字/空格。
節點目前使用短英文模板，並在進入 driver 前轉為固定 30-byte UTF-16LE。超長訊息被安全截斷，
不會切斷 surrogate pair。

相關串口封包的欄位與下位機責任見
[downlink_control_frame.md](../embedded/downlink_control_frame.md) 與
[serial_data_mapping.md](../embedded/serial_data_mapping.md)。
