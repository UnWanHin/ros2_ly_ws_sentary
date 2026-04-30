# Auto-Aim 時間戳 / PnP / Predictor 響應整理

日期：2026-05-01

## 背景

本次是針對哨兵自瞄鏈路的 P0 清理與低風險優化，目標是先把輸入資料變乾淨，再調 predictor：

- PnP 選解偶爾跳，`tracker_solver` 與下游 predictor 吃到不穩定 yaw/位置。
- detector 取圖後用當前 `now()` 和 atomic 雲台角，圖像與雲台角不一定是同一時刻。
- predictor 有裝甲板結果但沒有匹配到整車 bbox 時，可能刷新狀態時間或保留不可有效更新的模型。
- behavior_tree 以前把 `/ly/predictor/target` 回調直接當有效目標，沒有尊重 `msg->status`。

參考方向：

- `sentry.aim`：短 target timeout、用消息時間戳做 TF/控制、哨兵外參。
- `TDrone`：只參考時間對齊與響應節奏思路，不搬它的無人機外參，也不搬它的預測模型。

按 2026-05-01 的判斷，預測主體仍保留本倉庫原本 predictor；本次沒有改成 TDrone predictor。

## 根因 / 判斷

`config/base_config.yaml` 的 `solver_config.camera_intrinsic_matrix` 已複核是標準正號矩陣。YAML 裡每行前面的 `-` 是 list item，不是負號，所以這次沒有改 K 矩陣。

真正高風險點在以下幾處：

- `tracker_solver` 舊 PnP 選解用 static `prev_armor_yaw`，不同車和不同裝甲板會互相污染。
- whole-car bbox 推 yaw 只能作弱提示，不能直接靠它翻轉 yaw 正負。
- detector 發 `/ly/detector/armors` 時，圖像時間與雲台角沒有做配對。
- predictor 只有匹配到 whole-car bbox 的 measurement 才能真正 `MotionModel::Update()`，但舊鏈路會讓 armor-only 幀影響有效觀測時間。
- BT 忽略 predictor `status` 會讓後面排查失效目標時很混亂。

## 改動內容

### tracker_solver

涉及文件：

- `src/tracker_solver/include/solver/solver.hpp`
- `src/tracker_solver/src/solver.cpp`

行為：

- PnP 選解改為 per `(car_id, armor_id)` yaw history，去掉跨目標 static 記憶污染。
- 每個 IPPE 解都計算 reprojection error。
- 有 history 時加 yaw continuity penalty；有 whole-car bbox 時只加弱 yaw hint penalty。
- 首次無 history 時主要看 reprojection error。
- whole-car bbox 無效或 PnP 解不可用時回退 armor-only PnP。

### detector

涉及文件：

- `src/detector/detector_node.cpp`
- `src/detector/module/Camera.hpp`

行為：

- `/ly/gimbal/angles` 回調保存帶 `header.stamp` 的 ring buffer。
- 取圖成功後立刻產生 `frame_stamp`，用它查最近的雲台角。
- `Armors.header.stamp`、`Armors.yaw`、`Armors.pitch` 與內部 `TimedArmors` 都使用同一組 frame/gimbal 配對結果。
- `Camera::GetImage()` 保留 Daheng `nFrameID` / `nTimestamp` 到 `CameraFrameMeta`，目前只做 debug 記錄，不把設備時間硬轉 ROS time。

### predictor

涉及文件：

- `src/predictor/include/predictor/predictor.hpp`
- `src/predictor/src/predictor.cpp`
- `src/predictor/predictor_node.cpp`
- `src/predictor/config/predictor_config.yaml`

行為：

- `Predictor::update()` 回傳 `PredictorUpdateStats`。
- 只有匹配到整車 bbox 並真正 `MotionModel::Update()` 的 measurement 才增加 `model_update_count`。
- `predictor_node` 只有 `model_update_count > 0` 才刷新 `last_observation_time_`。
- 不再因為 armor-only 幀就新建/保留可預測模型。
- 新增 `predictor_config.coast_timeout_sec`，默認 `0.10`，對齊 `sentry.aim` 的短目標超時思路。
- `publish_only_on_new_tracker_frame` 默認 `false`，保留 100Hz timer 輸出節奏；這是響應節奏參考，不是 TDrone 預測模型移植。

### behavior_tree

涉及文件：

- `src/behavior_tree/src/SubscribeMessage.cpp`

行為：

- `/ly/predictor/target` 回調尊重 `msg->status`。
- `FireStatus`、`Valid`、`Fresh` 都由 `msg->status` 決定。
- 只有有效 target 才 latch yaw/pitch、更新 `LastValidTime`、置位 `isFindTargetAtomic`。

### sentry.aim 外參同步

涉及文件：

- `config/base_config.yaml`

行為：

- `barrel_to_camera.yaw` 與 slash alias `"barrel_to_camera/yaw"` 從 `-1.3746` 改為 `-1.5746`。
- 來源是 `../sentry.aim/src/sentry_tf/config/sentry_tf.yaml` 的實際 YAML 值。
- 只同步哨兵外參，不使用 TDrone 外參。

## 關鍵參數 / 閾值

`src/predictor/config/predictor_config.yaml`：

- `predictor_config.publish_only_on_new_tracker_frame: false`
  - `false`：predictor 保持 100Hz 控制輸出節奏，響應更連續。
  - `true`：只在新 tracker 幀後輸出，能減少 stale 控制，但會受 detector fps 影響。
- `predictor_config.require_observation_fresh_for_target: true`
  - `true`：觀測超時後不發布有效 target。
- `predictor_config.coast_timeout_sec: 0.10`
  - 調大：短暫丟檢更平滑，但更容易吃舊目標。
  - 調小：失效更快，但抖動或偶發漏幀時更容易斷跟。

`src/predictor/config/predictor_config.yaml`：

- `controller_config.shoot_delay: 0.02`
  - 目前未重調；這個要靠實機測總延遲後再改。

`src/tracker_solver/src/solver.cpp` 目前仍在代碼內：

- `kYawHistoryPenaltyPxPerRad = 2.0`
- `kWholeCarHintPenaltyPxPerRad = 4.0`

暫時不做成 YAML，是因為 PnP 還需要先用實機 log 看 reprojection error / yaw jump，再決定是否暴露成調參項。

`src/detector/detector_node.cpp` 目前仍在代碼內：

- `kMaxGimbalAngleSamples = 200`

這個只控制雲台角 ring buffer 長度，不是控制響應快慢的調參入口。

## 微調建議

現場不要先大調 EKF 噪聲。先確認輸入資料：

- `/ly/detector/armors` 的 `header.stamp`、`yaw`、`pitch` 是否穩定跟隨圖像。
- `/ly/tracker/results` 的裝甲板 yaw 是否還會跨目標跳解。
- `/ly/predictor/target` 是否在丟觀測後約 100ms 內停止發布有效 target；若後續發布 invalid target，BT 也應按 `status=false` 處理。
- `/ly/control/angles` 是否還在 target invalid 後繼續追舊角。

若要快調：

- 先調 `predictor_config.coast_timeout_sec`，建議在 `0.08` 到 `0.15` 秒之間試。
- 再根據實測延遲調 `controller_config.shoot_delay`。
- 最後才碰 EKF 噪聲、anti-rotate、PnP penalty。

不建議：

- 不要把 TDrone 的 predictor 直接搬過來；它的目標模型與無人機場景不匹配。
- 不要把 TDrone 外參套到哨兵。
- 不要在時間戳和 PnP 還沒驗乾淨前大改 predictor 結構。

## 驗證

已執行：

```bash
git diff --check
source /opt/ros/humble/setup.bash && colcon build --packages-select detector tracker_solver predictor behavior_tree --allow-overriding detector tracker_solver predictor behavior_tree
```

結果：

- `git diff --check` 通過。
- `detector`、`tracker_solver`、`predictor`、`behavior_tree` 均 build 通過。

已執行：

```bash
./scripts/selfcheck.sh sentry --skip-hz
```

結果：

- 失敗原因是既有環境/腳本問題：
  - 缺 `scripts/launch/start_autoaim_debug.sh`
  - 當前沒有 ROS2 nodes active，所以 runtime graph check 失敗
- 這次自瞄代碼改動本身已通過 targeted build，但尚未完成帶實機/相機的 launched runtime check。

## 後續風險

- Daheng `nTimestamp` 還沒有映射到 ROS clock；目前只是保留 metadata 供排查。
- predictor 還沒有做真正 armor-only EKF update；現在只是避免 armor-only 幀刷新有效模型。
- `tracker_solver` 和 `predictor/src/solver.cpp` 仍有 solver copy 分歧；本次只改真正發布 PnP 結果的 `tracker_solver`。
- `shoot_delay` 沒有實測總延遲前不要亂改。
- 沒有移植 `sentry.aim` 全套 TF/controller，只同步哨兵外參與短超時響應思想。
