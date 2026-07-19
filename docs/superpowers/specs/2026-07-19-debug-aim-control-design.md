# Debug Aim Control Design

Date: 2026-07-19

## Purpose

`gimbal_driver` 的 `debug_node.launch.py` 在不啟動 `behavior_tree` 的前提下，提供一個
可選的正式控制輸出替代入口。它可分別接入導航控制與外部 aim 結果，用於單節點實機調試；
它不是第二條正式決策鏈。

## Configuration Contract

`src/gimbal_driver/config/debug_mode.yaml` 保持為 debug bridge 的唯一 profile，新增：

```yaml
navi_mode: true
aim_mode: false
patrol: false
```

- `navi_mode`：啟用 `/ly/navi/vel` 和 `/ly/navi/should_rotate` 的調試輸入。
- `aim_mode`：啟用外部 `/ly/aim/result` 的正式 aim 輸出映射。
- `patrol`：只有沒有新鮮、有效 aim 結果時，才按正式 `behavior_tree/config/Patrol.yaml`
  的 `PatrolScan.Mode` 輸出 patrol 角度。
- 既有 `rotate_level`、`follow_mode_when_false`、`stale_timeout_ms`、`publish_hz`
  保持相容；`publish_hz` 預設 100 Hz。

三個開關相互獨立。`patrol=true` 不要求 `navi_mode=true`，但只在 `aim_mode=true`
沒有有效 aim 時才作為 fallback；`aim_mode=false` 時不訂閱 `/ly/aim/result`，且 patrol
可直接作為手動雲台掃描模式。

## Ownership and Data Flow

`debug.py` 是單一 debug-control bridge，並繼續是
`debug_node.launch.py` 唯一的 debug 控制 publisher：

```text
/ly/navi/vel, /ly/navi/should_rotate  -- navi_mode --> debug-control bridge
/ly/aim/result                        -- aim_mode  --> debug-control bridge
Patrol.yaml                            -- patrol    --> debug-control bridge
debug-control bridge --> /ly/control/vel
debug-control bridge --> /ly/control/angles
debug-control bridge --> /ly/control/firecode
gimbal_driver --> lower machine
```

因此不新增第二個 aim bridge，也不允許它和 behavior_tree 並行運行。這避免多個 ROS
publisher 對 `/ly/control/firecode` 的不同欄位競態覆蓋。

## Formal Aim Semantics

`/ly/aim/result` 使用 `sentry_msgs/msg/AimResult`：`follow`、`fire`、`yaw`、`pitch`。

- `follow=true` 且 yaw/pitch 均為有限值，才代表有效 aim 結果；它不是
  `FireCode.follow_mode`。
- 有效 aim 時，bridge 發布 yaw/pitch 到 `/ly/control/angles`，並在 FireCode 中發布
  `AimMode=true`。
- 有效且 `fire=true` 時，沿用正式 BT 的 FireStatus toggle 語義，對每一筆新鮮 aim
  結果只觸發一次。
- 無效、過期或 `follow=false` 時，bridge 發布 `AimMode=false` 並不觸發開火；若
  `patrol=true` 則切至 patrol angle fallback，否則保持目前回授雲台角度。
- `ResultFreshTimeoutMs` 以 debug profile 的 `stale_timeout_ms` 作為單節點 stale
  合約，避免單獨引入與現有 debug profile 重複的 timeout key。

## Navigation Semantics

當 `navi_mode=true`：

- bridge 以 `publish_hz` 發布 `/ly/control/vel`；`/ly/navi/vel` 超過
  `stale_timeout_ms` 未更新，持續發布零速度。
- `/ly/navi/should_rotate=false` 且 `follow_mode_when_false=true` 時，發布
  `FollowMode=true`；同時將 `Rotate=0`。
- `should_rotate=true` 時，發布 `Rotate=rotate_level` 並清除由導航寫入的
  FollowMode，與正式 `NaviRotateControl` 的最終輸出語義一致。

`navi_mode=false` 時，不建立導航輸入訂閱，也不發布 velocity 或導航持有的 FireCode
欄位。aim 的 FireStatus/AimMode 與導航的 FollowMode/Rotate 由同一份 FireCode 快照合併後
發出。

## Explicit Non-Goals

此入口不實作或替代：

- behavior_tree 的目標選擇、Tactical/Regional 決策、導航目標與姿態策略；
- FaceMode 仲裁與 `/ly/face_mode/angles`；
- `/ly/control/posture`、`/ly/control/sentry_cmd`；
- MPC `/ly/control/trajectory`。該 topic 維持 gimbal_driver 現有獨立 `0x05` 下發行為。

## Verification

實作須提供可離線執行的 Python 單元測試，至少驗證：mode gate、AimResult 有效性與 stale、
FireCode 欄位合併、每筆 fire 只 toggle 一次、導航速度 stale 歸零、patrol fallback。

驗收包含：

```bash
source /opt/ros/humble/setup.bash
source ../sentry.common/install/setup.bash
colcon build --packages-select gimbal_driver --symlink-install
source install/setup.bash
./scripts/selfcheck.sh sentry --static-only
```

完成後同步更新 gimbal_driver module 文件、ROS topic 文件和 Understand Anything 圖譜，並以
launch 檢查確認 debug mode 不啟動 behavior_tree。
