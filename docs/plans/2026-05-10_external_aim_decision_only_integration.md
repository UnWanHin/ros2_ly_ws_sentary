# External Aim Decision-Only Integration Plan

Updated: 2026-05-10

本文記錄本倉從「內部輔瞄鏈 + 決策」切到「只做決策，外部 aim 提供目標/角度/開火門控」的重大變化分析與 `Behavion` 分支落地狀態。

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
  bool follow
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
| `/ly/aim/armor_targets` | `sentry_msgs/msg/AimTargetArray` | external aim -> BT | 可擊打目標列表，元素是 `AimTarget.msg`。 |
| `/ly/aim/select_target` | `sentry_msgs/msg/AimTarget` | BT -> external aim | BT 選中的目標 id，帶 `std_msgs/Header.stamp`。 |
| `/ly/aim/result` | `sentry_msgs/msg/AimResult` | external aim -> BT | follow、yaw/pitch 與 fire 門控。 |

`sentry.aim` 的 `aim_armor_decider_node` 需要用 `SensorDataQoS` 發 `/ly/aim/armor_targets`、訂閱
`/ly/aim/select_target`。BT 因此對 `/ly/aim/armor_targets` 使用 `SensorDataQoS` 訂閱，避免 reliable
subscriber 對 best-effort publisher 不匹配而收不到候選目標。

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
  -> /ly/aim/result(follow, fire, yaw, pitch)
BT final control
  -> /ly/control/angles
  -> /ly/control/firecode
```

`AimResult.follow` 必須直接進 BT 的角度接管門控，`AimResult.fire` 必須直接進 BT 的最終火控門控，而不是被壓成 `Target.status`。

## 相機和輔瞄節點所有權

外部模式下，相機只能由外部工程占用。本倉正式 launch 必須禁止內部相機/輔瞄鏈：

- 不啟動 `detector`
- 不啟動 `tracker_solver`
- 不啟動 `predictor`
- 不啟動 `outpost_hitter`
- 不啟動 `buff_hitter`

`Behavion` 中已不再保留官方 `use_external_aim` 開關：外部 aim 是唯一正式鏈路，`sentry_all` 不啟動上述內部節點，BT 內部強制 `ExternalAim.Enable=true`。

## TF 所有權

外部 `sentry.aim` 的完整 launch 可能會啟動 `sentry_tf`；本倉已有 `tf_tree`。兩邊都會發布類似：

```text
base_link -> gimbal_big_yaw -> gimbal_small_yaw -> gimbal_world
gimbal_small_yaw -> gimbal_barrel_joint -> gimbal_barrel
gimbal_barrel -> gx_camera
```

完整對接時必須保證 TF 只有一個 owner：

- `Behavion` 正式對接改為由外部 `sentry_tf` 做 TF owner，因為外部工程還有其他功能依賴它。
- 本倉 `sentry_all` 默認 `use_tf_tree=false`，不再自動拉起本地 `tf_tree`，避免和外部 `sentry_tf` 重複發布同一組 child frame。
- 本倉 `tf_tree` 保留作為 fallback；只有在不拉外部 `sentry_tf` 的獨立調試場景才設 `use_tf_tree:=true`。
- 外部 TF 必須提供本倉 `navi_tf_bridge`、FaceMode、定位反算依賴的 frame，尤其是 `base_link`、`gimbal_big_yaw`、`gimbal_small_yaw`、`gimbal_world`、`gimbal_barrel_joint`、`gimbal_barrel`、`gx_camera`。

## Target List 對 BT 的影響

BT 現在多處依賴內部 `/ly/detector/armors` 生成的 `armorList/hitableTargets/targetArmor`：

- `ProcessData()` 用 armor list 建 `hitableTargets`。
- `SetAimTarget()` / `TrySetAimTargetByAutonomy()` 用 `hitableTargets`、敵方血量、距離和優先級選目標。
- `Chase` 用 `targetArmor.Distance`、`nextAngles - gimbalAngles`、官方敵方位置去生成 `/ly/navi/target_rel` 或追擊速度。

外部模式下沒有內部 `/ly/detector/armors`。因此 `/ly/aim/armor_targets` 必須成為 BT 的可擊打目標來源：

- `AimTargetArray.aim_targets[].id` 映射到 `ArmorType`。
- `position` 會被缓存为当前目标的相对/局部点；Chase 只有在该 point 新鲜且 `header.frame_id` 非空时才直接发布到 `/ly/navi/target_rel`，否则退回 yaw/pitch 誤差和距離近似。官方坐标追击仍可走 `/ly/position/data`。
- `/ly/aim/armor_targets` 負責 BT 選目標和 Chase point；`/ly/aim/result.follow` 負責最終角度/開火是否接管。

落地版保留現有 `armorList/hitableTargets/targetArmor` 決策資料結構，但資料源改成 `/ly/aim/armor_targets`。這樣可以不重寫 regional/Task/Posture 的選目標邏輯，同時確保正式鏈路不再依賴內部 detector。

## Chase 對接狀態

Chase 保留原本 BT 內的追擊輸出策略，但資料源已改成外部 aim：

- 角度源：`AimResult.follow=true` 時的 `AimResult.yaw/pitch`。
- 距離源：`/ly/aim/armor_targets` 裡當前 id 的 `position` 長度，沿用 `targetArmor.Distance`，單位按外部 aim 的世界/相機幾何輸出視為 m。
- `/ly/navi/target_rel`：有新鮮且帶 `frame_id` 的 `AimTarget.position` 時直接用該 xyz，並把 `AimTarget.header.frame_id` 帶給 `navi_tf_bridge`；沒有新鮮 point 或 frame_id 缺失時退回 BT 用 yaw/pitch 誤差與距離近似的相對目標。
- 官方坐標追擊：仍沿用 `/ly/position/data` 的敵方/自身官方坐標，帶 freshness gate；外部 aim 不需要提供官方地圖坐標。

所以外部 aim 必須保證 `AimTarget.id` 和本倉 `ArmorType` 數值一致，`position` 是米制點，且 `header.frame_id`
能被 `navi_tf_bridge` 走 TF 轉到 `map`。如果 frame_id 为空，BT 不把该 point 当成 Chase 真值使用。

## Fire 和角度語義

外部 aim 模式下：

- `AimResult.follow=true` 表示外部 aim 已識別到可跟隨車體，BT 接收本幀 yaw/pitch 並轉發 `/ly/control/angles`。
- `AimResult.follow=false` 表示外部 aim 未識別到車體，BT 不接管本幀 yaw/pitch，繼續自身巡邏/FaceMode/決策輸出。
- `AimResult.yaw/pitch` 只在 `follow=true` 時表示外部解出的目標角。
- `AimResult.fire=true` 表示本幀允許擊發，BT 可以翻轉一次 `FireCode.FireStatus`。
- `AimResult.fire=false` 表示 `follow=true` 時可以轉角但不擊發。
- stale result 或 `follow=false` 時，BT 不應開火。
- `AimMode` bit 仍由 BT 決定；正常鎖目標時置 `AimMode=1`，FaceMode/FollowMode/WaitBeforeGame 等安全鏈路仍保持優先。

外部 `armor_controller_node` 有 `publish_legacy_control_topics`，它可能直接發布 `/ly/control/angles` 和舊型別 `/ly/control/firecode`。完整對接必須關掉這個 legacy 發布，否則會和 BT 搶控制 owner，且 firecode 型別也與本倉 `gimbal_driver/msg/FireCode` 不一致。

## 已落地的大改動

1. `behavior_tree` 改為正式依賴 `sentry_msgs`，找不到外部消息包時不再靜默降級。
2. `ExternalAim.Enable` 在 BT 內強制為 `true`，舊 JSON 裡的 `Enable` 只保留為文檔字段。
3. `sentry_all.launch.py` 移除 detector/tracker/predictor/outpost/buff 節點和相關開關；官方 wrapper launch 也不再查找這些包。
4. `scripts/launch/start_sentry_all.sh` 不再注入內部輔瞄配置或 `use_*` 開關。
5. `AimResult.follow` 直接驅動 BT 角度接管；`AimResult.fire` 直接驅動 BT 最終火控；`follow=true, fire=false` 時只跟角不打彈。
6. 普通、前哨、打符 aim mode 的角度源都統一用外部 `AimResult`；內部 `/ly/predictor/target`、`/ly/buff/target`、`/ly/outpost/target` 在正式外部模式下被忽略。
7. 前哨任務在距 `BuffOutpost` `VisualScoutFaceDistanceCm` 半徑內就會選擇 `ArmorType::Outpost` 並發 `/ly/aim/select_target`，不用等到原本 100cm 近點。
8. BT 對 `/ly/aim/armor_targets` 使用 `SensorDataQoS`，兼容外部 decider 的 best-effort 發布。
9. `scripts/launch/start_sentry_all.sh`、showcase、navi_debug、chase wrapper 不再自動注入內部輔瞄配置。
10. `selfcheck.sh sentry` 的 runtime graph 改成檢查 BT `/ly/aim/*` 契約，並確認內部輔瞄節點沒有出現在正式鏈路。

## Build/Run 前置

`sentry_msgs` 來自外部 `sentry.common`。正式構建前需要先 build/source 外部消息包，例如：

```bash
cd ~/sentry.common
colcon build --packages-select sentry_msgs
source install/setup.bash

cd ~/ros2_ly_ws_sentry
colcon build --packages-select behavior_tree
```

外部 `sentry.aim` 啟動時要關掉 `armor_controller_node.publish_legacy_control_topics`，避免它直接發布 `/ly/control/angles` 或舊型別 `/ly/control/firecode` 與 BT 搶 owner。
