# External Aim Decision-Only Integration Plan

Updated: 2026-05-10

本文記錄本倉從「內部輔瞄鏈 + 決策」切到「只做決策，外部 aim 提供目標/角度/開火門控」之前的重大變化分析。後續大改動應先按本文確認接口邊界，再在新分支 `Behavion` 上完全對接。

## 目標

新分支的目標不是再維護一條內部輔瞄鏈，而是讓 `ros2_ly_ws_sentry` 成為決策與下位機控制收口：

- 本倉保留 `behavior_tree`、`gimbal_driver`、導航/TF/FaceMode、regional/league 決策、posture、sentry_cmd、下位機通訊。
- 外部工程負責相機、檢測、追蹤、彈道、aim target list、yaw/pitch、fire 判斷。
- 本倉不啟動會占用相機的內部節點：`detector`、`tracker_solver`、`predictor`、`outpost_hitter`、`buff_hitter`。
- BT 仍是 `/ly/control/angles`、`/ly/control/firecode`、`/ly/control/vel`、`/ly/control/posture`、`/ly/control/sentry_cmd` 的唯一正式 owner。

## 外部接口證據

外部消息包是 `sentry_msgs`，不是 `sentry_msg`。接口位於：

- `/home/unwanhin/sentry.common/src/sentry_msgs/msg/AimResult.msg`
- `/home/unwanhin/sentry.common/src/sentry_msgs/msg/AimTarget.msg`
- `/home/unwanhin/sentry.common/src/sentry_msgs/msg/AimTargetArray.msg`

消息字段：

```text
AimResult:
  std_msgs/Header header
  bool fire
  float32 pitch
  float32 yaw

AimTarget:
  std_msgs/Header header
  geometry_msgs/Point position
  uint8 id

AimTargetArray:
  std_msgs/Header header
  sentry_msgs/AimTarget[] aim_targets
```

外部 `sentry.aim` 目前使用的 topic：

| Topic | Type | Direction | 語義 |
|---|---|---|---|
| `/ly/aim/armor_targets` | `sentry_msgs/msg/AimTargetArray` | external aim -> BT | 可擊打目標列表。 |
| `/ly/aim/select_target` | `sentry_msgs/msg/AimTarget` | BT -> external aim | BT 選中的目標 id；外部 decider 目前主要使用 `id`。 |
| `/ly/aim/result` | `sentry_msgs/msg/AimResult` | external aim -> BT | yaw/pitch 與 fire 門控。 |

`AimResult` 是在 BT 發出 `/ly/aim/select_target` 後，外部 aim 根據選中目標輸出的結果。它不是 detector/predictor 的直接替代消息，不能無條件映射成 `/ly/predictor/target`。

## 不能走 AimResult -> /ly/predictor/target

舊 `/ly/predictor/target` 的語義只有：

- `status=true/false`：有沒有有效 target
- `yaw/pitch`：目標角度

而 BT 目前普通 autoaim 開火邏輯是：只要 target valid，且 `StopFire=false`，就按 `fireRateClock` 自己翻轉 `FireCode.FireStatus`。這會忽略外部 `AimResult.fire=false`。

所以正確鏈路是：

```text
BT target decision
  -> /ly/aim/select_target
external aim
  -> /ly/aim/result(yaw, pitch, fire)
BT final control
  -> /ly/control/angles
  -> /ly/control/firecode
```

`AimResult.fire` 必須直接進 BT 的最終火控門控，而不是被壓成 `Target.status`。

## 相機和輔瞄節點所有權

外部模式下，相機只能由外部工程占用。本倉正式 launch 必須禁止內部相機/輔瞄鏈：

- 不啟動 `detector`
- 不啟動 `tracker_solver`
- 不啟動 `predictor`
- 不啟動 `outpost_hitter`
- 不啟動 `buff_hitter`

目前臨時接口已加入 `use_external_aim:=true`，啟動時會關閉上述內部節點，並把 BT 的 `ExternalAim.Enable` 打開。完整對接分支應保留這個 launch 邊界，並把它做成主入口語義。

## TF 所有權

外部 `sentry.aim` 的完整 launch 可能會啟動 `sentry_tf`；本倉已有 `tf_tree`。兩邊都會發布類似：

```text
base_link -> gimbal_big_yaw -> gimbal_small_yaw -> gimbal_world
gimbal_small_yaw -> gimbal_barrel_joint -> gimbal_barrel
gimbal_barrel -> gx_camera
```

完整對接時必須保證 TF 只有一個 owner：

- 若本倉保留導航、FaceMode、regional 決策，建議繼續由本倉 `tf_tree` 做 TF owner。
- 外部 aim 不應用會重複發布 TF 的全量 launch；應只啟動相機/aim/decider/controller 相關節點，或把外部 TF 發布關掉。
- 如果決定由外部 TF 做 owner，就要同步檢查本倉 `navi_tf_bridge`、FaceMode、定位反算依賴的 frame 和外參，避免與現有地圖/導航鏈路不一致。

## Target List 對 BT 的影響

BT 現在多處依賴內部 `/ly/detector/armors` 生成的 `armorList/hitableTargets/targetArmor`：

- `ProcessData()` 用 armor list 建 `hitableTargets`。
- `SetAimTarget()` / `TrySetAimTargetByAutonomy()` 用 `hitableTargets`、敵方血量、距離和優先級選目標。
- `Chase` 用 `targetArmor.Distance`、`nextAngles - gimbalAngles`、官方敵方位置去生成 `/ly/navi/target_rel` 或追擊速度。

外部模式下沒有內部 `/ly/detector/armors`。因此 `/ly/aim/armor_targets` 必須成為 BT 的可擊打目標來源：

- `AimTargetArray.aim_targets[].id` 映射到 `ArmorType`。
- `position` 用來計算距離，並作為 Chase 的相對目標/官方目標補充來源。
- 若長時間沒有當前 target id 的 candidate，BT 應視為未鎖定目標，不接受新的 `AimResult` 開火。

臨時版已做最小替代：`ExternalAim.Enable=true && UseTargetArrayAsArmorList=true` 時，BT 用 `/ly/aim/armor_targets` 填 `armorList`，讓現有 target selection 能繼續工作。完整對接應把這部分從「兼容 armorList」升級成清晰的 external target model。

## Chase 需要重做的部分

目前 Chase 是圍繞內部 autoaim 的 `activeAimData` 和 `targetArmor.Distance` 設計的。外部模式下要重新定義 Chase 的 target source：

- 角度源：`AimResult.yaw/pitch`。
- 距離源：優先使用 `/ly/aim/armor_targets` 裡當前 id 的 `position` 距離。
- 相對目標源：若 `position` 是機體/相機/世界某 frame 下的點，必須明確 frame；否則只能用 yaw/pitch + distance 近似。
- 官方坐標源：如果外部 aim 不提供官方地圖坐標，仍可沿用 `/ly/position/data` 的敵方位置；但要保持 freshness gate。

完整對接前要確認 `AimTarget.position` 的 frame_id 和單位。如果外部 position 是 camera/barrel/base_link 坐標，BT 需要明確轉成 Chase 使用的相對目標；如果只是外部內部用，不可靠，就不能拿來直接導航。

## Fire 和角度語義

外部 aim 模式下：

- `AimResult.yaw/pitch` 只表示外部解出的目標角。
- `AimResult.fire=true` 表示本幀允許擊發，BT 可以翻轉一次 `FireCode.FireStatus`。
- `AimResult.fire=false` 表示可以轉角但不擊發。
- stale result 或沒有當前 target candidate 時，BT 不應開火。
- `AimMode` bit 仍由 BT 決定；正常鎖目標時置 `AimMode=1`，FaceMode/FollowMode/WaitBeforeGame 等安全鏈路仍保持優先。

外部 `armor_controller_node` 有 `publish_legacy_control_topics`，它可能直接發布 `/ly/control/angles` 和舊型別 `/ly/control/firecode`。完整對接必須關掉這個 legacy 發布，否則會和 BT 搶控制 owner，且 firecode 型別也與本倉 `gimbal_driver/msg/FireCode` 不一致。

## 建議大改動步驟

1. 建新分支 `Behavion`。
2. 先只做接口整理，不刪決策：
   - `behavior_tree` 明確依賴 `sentry_msgs`，或保留可選依賴但外部模式必須檢查已編進接口。
   - 保留 `use_external_aim`，並讓 external 模式成為決策-only 主入口。
3. 拆掉內部輔瞄 launch 所有權：
   - external 模式不啟動 detector/tracker/predictor/outpost/buff。
   - 後續如果 branch 完全不需要內部輔瞄，可以再移除相關 launch 預設、配置和文檔入口。
4. 在 BT 內建立正式 external target model：
   - `ExternalAimTarget{id, position, distance, frame, last_seen}`
   - target selection 直接吃 external target list，不再偽裝成 `/ly/detector/armors`。
5. 改 GameLoop target source：
   - internal aim mode 使用 `autoAimData`
   - external aim mode 使用 `externalAimData`
   - buff/outpost 是否仍保留需重新決策；如果外部 aim 也提供這些，BT 不應再啟動本倉 hitter。
6. 重做 Chase 的 target source：
   - 明確 position frame 和 freshness
   - 不能從外部 aim 可靠拿到距離時，Chase 降級只用官方敵方位置或關閉追擊
7. 加 runbook：
   - 如何 source/build `sentry.common`
   - 如何啟動外部 aim 且不啟動外部 TF legacy control
   - 如何啟動本倉 decision-only stack
8. 驗證：
   - `colcon build --packages-select behavior_tree`
   - `./scripts/selfcheck.sh sentry --static-only`
   - launch show-args 檢查 `use_external_aim`
   - runtime topic contract：
     - `/ly/aim/armor_targets` 有 publisher，BT 有 subscriber
     - `/ly/aim/select_target` BT 有 publisher，external aim 有 subscriber
     - `/ly/aim/result` external aim 有 publisher，BT 有 subscriber
     - `/ly/control/angles`、`/ly/control/firecode` 只有 BT 發布
     - detector/tracker/predictor/outpost_hitter/buff_hitter 沒有啟動

## 當前臨時改動狀態

目前工作區已做的臨時對接屬於「接口先接通」：

- `behavior_tree` 增加 `ExternalAim` config。
- 找到 `sentry_msgs` 時編譯 `/ly/aim/*` topic；找不到時保持原 workspace 可 build。
- `sentry_all.launch.py` 增加 `use_external_aim`，打開時不啟動內部相機/輔瞄鏈。
- BT 可發布 `/ly/aim/select_target`，接收 `/ly/aim/armor_targets` 和 `/ly/aim/result`。
- 外部 `AimResult.fire` 已直接控制開火翻轉，不走 `/ly/predictor/target`。
- 文檔已在 message flow 和 topic structure 裡補了外部 aim topic。

這還不是「完全對接」：

- `sentry_msgs` 在當前 shell 未 source/build，因此本倉 build 目前是 external aim 可選接口關閉狀態。
- external target model 還是先兼容填 `armorList`，未徹底從內部 detector data model 中抽離。
- Chase 還沒有完整改成外部 target list/position 的一等數據源。
- TF owner 和外部 launch 方式仍需在真機/完整工作區上確認。

## 分支注意

本次沙盒環境 `.git` 是 read-only，無法直接建立 `Behavion`。在真機工作區應先執行：

```bash
git switch -c Behavion
```

再把本計劃中的完全對接改動落到該分支。
