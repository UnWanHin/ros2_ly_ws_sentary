# `sentry.aim` / `sentry.common` 依賴審計

Updated: 2026-07-19

## 範圍與結論

本記錄只盤點 source 與實機安裝環境，不修改 ROS 程式、launch、設定或遠端環境。檢查對象是
`192.168.3.50` 的 `~/sentry.common`、`~/sentry.aim` 與
`~/ros2_ly_ws_sentry`。

結論不是「`sentry.aim` 可以無條件完全取代所有 common 內容」：

1. 正式外部 Aim 介面可由 `sentry.aim/src/sentry_msgs` 提供，且應只選擇此版本。
2. gimbal TF 應由 `sentry.aim/src/sentry_tf` 作唯一 owner；它覆蓋目前正式需要的核心
   `base_link -> gimbal_small_yaw -> gimbal_world`、barrel 與相機靜態外參鏈。
3. 本倉的 `auto_aim_common/msg/GoalReach`、`RelativeTarget` 仍是本倉內部導航契約，
   `aim_msgs` 沒有語義或欄位等價物，不能移除或以 `aim_msgs` 取代。
4. 遠端現在仍被 `sentry.common` overlay 汙染，尚未真的做到只使用 `sentry.aim`。在修正
   source chain、frame 契約並完成 runtime 驗收前，不可宣稱完成切換。

## Source 盤點

| `sentry.common` package | `sentry.aim` 對應 | 結論 |
| --- | --- | --- |
| `sentry_msgs` | 同名 `sentry_msgs` | 功能上是正式 Aim boundary 的替代來源，但同名介面有 ABI 差異，禁止兩者共存。 |
| `sentry_tf` | 同名 `sentry_tf` | aim 版本是預期唯一 TF owner；frame、外參和 launch 已不同，需以 aim 版本為準。 |
| `sentry_gx_camera` | `aim_camera_driver` 的 GX component | 外部相機責任已移入 aim stack；本倉不依賴舊 camera package。 |
| `sentry_usb_camera` | `aim_camera_driver` 的 USB component | 外部相機責任已移入 aim stack；本倉不依賴舊 camera package。 |
| `sentry_gimbal` | 無直接同名替代 | 本倉和 aim TF 均依賴 `gimbal_driver` 的 `/ly/gimbal/angles`，不是舊 `sentry_gimbal`。 |

`sentry.common` 的 `sentry_msgs/AimResult` 是 `header, follow, fire, pitch, yaw`；
aim 版本在相同型別名稱下多了 `yaw_omega`、`pitch_omega`、`yaw_alpha`、`pitch_alpha`。
這是 ROS message definition mismatch，不可讓 publisher 和 subscriber 分別從不同 workspace
載入同名 package。

## 本倉仍保留的公共訊息

source 搜索結果只有兩個 `auto_aim_common` ROS message 為正式鏈路所需：

| 訊息 | owner / 流向 | 為何不能由 `aim_msgs` 取代 |
| --- | --- | --- |
| `GoalReach` | `behavior_tree` -> `/ly/navi/reach_state` | 是 composite reached/unreachable/timeout 診斷狀態，含目標 ID、導航回授新鮮度、距離與 timeout 證據。 |
| `RelativeTarget` | `behavior_tree` -> `navi_tf_bridge` `/ly/navi/target_rel` | 是追擊導航用相對點，含 frame、valid、距離、yaw/pitch error、armor type 與 aim mode。 |

`sentry.aim/aim_msgs` 提供的是相機檢測、追蹤、瞄準與 MPC 的資料型別，例如
`TargetState`、`ArmorSetArray`、`ControlAngles`、`GimbalState`。它們沒有 `GoalReach` 或
`RelativeTarget` 的導航語義。因此 `auto_aim_common` 應保留為本倉自有最小公共介面包；這不表示
恢復舊 detector/tracker/predictor。

## TF 契約檢查

aim 的 `sentry_tf` 從 `/ly/gimbal/angles` 訂閱 `gimbal_driver/msg/GimbalAngles`，配置的核心鏈為：

```text
base_link -> gimbal_small_yaw -> gimbal_world
gimbal_small_yaw -> gimbal_barrel_joint -> gimbal_barrel
gimbal_barrel -> gx_camera_0
gimbal_barrel -> gx_camera_1
gimbal_small_yaw -> usb_camera
```

並額外發布三個相機 optical frame。這可滿足本倉正常 AimTarget/追擊使用的
`gimbal_world`，因為 `navi_tf_bridge/config/tf_config.yaml` 的
`target_rel_default_frame` 已是 `gimbal_world`。

仍有一個明確的 frame mismatch：`map_aim_point_node` 的預設 `camera_frame` 是
`gx_camera`，但 aim TF 配置只發布 `gx_camera_0` 和 `gx_camera_1`。使用 camera / camera_projection
FaceMode 時會找不到預設 `gx_camera`；只有 map / `gimbal_world` 流程不受此差異影響。這必須在
後續整合工作中由明確選定的 camera index 與 launch/config 契約解決，不能靠舊 TF overlay 掩蓋。

遠端檢查當刻沒有 ROS node、`/tf`、`/tf_static` 或 `/ly/gimbal/angles` publisher，所以無法驗收
動態 yaw/pitch 更新、實際 TF timestamp 與 `tf2_echo` 連通性。本記錄只能確認 source 與靜態
launch 設計，不把它寫成實機 runtime 通過。

## 現場 overlay 風險

即使在乾淨 shell 內依序 source ROS、`sentry.aim/install/setup.bash`、本倉 setup，遠端的
`AMENT_PREFIX_PATH` 仍包含 `~/sentry.common/install/*`。`ros2 pkg prefix` 實測得到：

```text
sentry_msgs -> ~/sentry.common/install/sentry_msgs
sentry_tf   -> ~/sentry.common/install/sentry_tf
aim_msgs    -> ~/sentry.aim/install/aim_msgs
```

原因有兩層：

- `~/.bashrc` 直接 source 兩次 common 的 `sentry_msgs/local_setup.bash`。
- `sentry.aim/install/setup.bash` 的生成 prefix chain 也帶入了 common underlay，表示 aim 很可能在
  common 已 source 的環境中建立。

因此目前是 split-brain：可能取得 aim 的部分訊息包，同時卻啟動 common 的 `sentry_tf`。本倉
`scripts/lib/ros_launch_common.sh` 和 `scripts/selfcheck/sentry.sh` 也仍把 common 當作預設
`sentry_msgs` fallback。這些是下一次正式遷移的工作項，這次沒有修改。

## 後續驗收門檻

正式移除 common 前，應在沒有 common underlay 的 shell 重新 build `sentry.aim`，並確認：

```bash
ros2 pkg prefix sentry_msgs
ros2 pkg prefix sentry_tf
ros2 interface show sentry_msgs/msg/AimResult
```

前兩者都必須指向 `~/sentry.aim/install/...`，且 `AimResult` 必須顯示四個 dynamics 欄位。再啟動
實際的 gimbal driver 與 aim TF 後，驗收：

```bash
ros2 topic info /tf -v
ros2 topic info /tf_static -v
ros2 run tf2_ros tf2_echo base_link gimbal_barrel
ros2 run tf2_ros tf2_echo gimbal_barrel gx_camera_0
ros2 run tf2_ros tf2_echo gimbal_barrel gx_camera_1
ros2 run tf2_ros tf2_echo gimbal_small_yaw usb_camera
```

若正式使用 camera-based FaceMode，另需明確驗證所選 `gx_camera_0` 或 `gx_camera_1` 與
`map_aim_point_node.camera_frame` 一致。完成這些檢查後，才可更新本倉腳本的外部 message setup
預設並移除 common fallback。
