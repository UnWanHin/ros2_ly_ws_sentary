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
- `TDrone`：solver/PnP 思路可作準確性參考；響應上參考它的時間對齊與週期輸出節奏。

按 2026-05-01 的判斷，預測主體仍保留本倉庫原本 predictor；本次沒有改成 TDrone predictor，也不搬它的無人機外參。

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
- `src/predictor/include/predictor/motion_model.hpp`
- `src/predictor/src/predictor.cpp`
- `src/predictor/src/controller.cpp`
- `src/predictor/predictor_node.cpp`
- `src/predictor/config/predictor_config.yaml`

行為：

- `Predictor::update()` 回傳 `PredictorUpdateStats`。
- 只有匹配到整車 bbox 並真正 `MotionModel::Update()` 的 measurement 才增加 `model_update_count`。
- `predictor_node` 只有 `model_update_count > 0` 才刷新 `last_observation_time_`。
- 不再因為 armor-only 幀就新建/保留可預測模型。
- 新增 `predictor_config.coast_timeout_sec`，默認 `0.10`，對齊 `sentry.aim` 的短目標超時思路。
- `publish_only_on_new_tracker_frame` 默認 `false`，保留 100Hz timer 輸出節奏；這是響應節奏參考，不是 TDrone 預測模型移植。
- 對照 `change_buff_infantry/upload-infantry-buff-20260428` 後，採用步兵 predictor 的 pitch 觀測處理：`pitch_top/pitch_bottom/pitch_center` 不再由整車 bbox 上下邊界推導，而是保持為裝甲板中心 pitch，避免 bbox 像素量化造成 pitch 階梯。
- 對照步兵 controller 後，裝甲板彈道計算成功時把實際 `time` 回寫到 `flyTime`，讓下一輪控制用最新飛行時間做提前量；舊邏輯只在車中心估算/無可用裝甲板 fallback 時更新，可能讓提前時間滯後。

### 步兵 predictor 對照

參考分支：

- `https://github.com/xty2025/change_buff_infantry/tree/upload-infantry-buff-20260428`
- 本地分析路徑：`/tmp/change_buff_infantry_upload_20260428`

判斷：

- 該分支的步兵 predictor 與本倉庫 predictor 同源，都是 12 維狀態、10 維觀測的整車 EKF。
- 可直接借鑑的是 pitch 觀測策略：它在 `measureFunc` 中讓 `m[7] = m[8] = m[9] = m[0]`，避免 bbox top/bottom pitch 帶來階梯。
- 不直接搬它的全套 controller / config / 外參；步兵相機、彈道表、串口控制和哨兵 ROS2 鏈路不同。
- 本倉庫仍保留 `predictor_config.coast_timeout_sec` 與 BT status gate 來保證響應和失效行為。
- 該分支 controller 每次 ballistic 成功會刷新 `cached_fly_time_`；本倉庫已對齊這點，避免 flight-time 提前量用舊值。

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
- predictor pitch 觀測策略：
  - 目前固定使用裝甲板中心 pitch 作 `pitch_top/pitch_bottom/pitch_center`。
  - 這不是 YAML 參數；它是為了跟步兵 predictor 一樣避免 bbox 上下邊界量化階梯，先作為穩定性修正保留在代碼中。

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
- predictor log 裡 `armor_count/car_count/model_update_count` 的關係；如果 `armor_count > 0` 但 `model_update_count` 長時間為 0，說明響應慢主要卡在 car bbox 匹配/更新門檻，而不是 timer。

若要快調：

- 先調 `predictor_config.coast_timeout_sec`，建議在 `0.08` 到 `0.15` 秒之間試。
- 再根據實測延遲調 `controller_config.shoot_delay`。
- 最後才碰 EKF 噪聲、anti-rotate、PnP penalty。

不建議：

- 不要把 TDrone 的 predictor 直接搬過來；它的目標模型與無人機場景不匹配。
- 不要把 TDrone 外參套到哨兵。
- TDrone 的 solver/PnP 可以繼續作對照，但要逐項驗證座標系與相機模型後再移植。
- 不要在時間戳和 PnP 還沒驗乾淨前大改 predictor 結構。

## 驗證

已執行：

```bash
git diff --check
source /opt/ros/humble/setup.bash && colcon build --packages-select detector tracker_solver predictor behavior_tree --allow-overriding detector tracker_solver predictor behavior_tree
source /opt/ros/humble/setup.bash && colcon build --packages-select predictor --allow-overriding predictor
```

結果：

- `git diff --check` 通過。
- `detector`、`tracker_solver`、`predictor`、`behavior_tree` 均 build 通過。
- 步兵 predictor pitch 觀測移植與 controller flight-time 回寫後，`predictor` 單包 build 通過。

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
- 如果實機仍然覺得 predictor 響應慢，下一個要驗證的是 car bbox 是否每幀穩定匹配。若 car bbox 掉幀，現在 EKF 不會用 armor-only 幀更新模型；正確改法是給 armor-only update 單獨觀測模型或高 R masking，而不是把缺失的 bbox 邊界硬塞進現有 10 維觀測。
- `tracker_solver` 和 `predictor/src/solver.cpp` 仍有 solver copy 分歧；本次只改真正發布 PnP 結果的 `tracker_solver`。
- `shoot_delay` 沒有實測總延遲前不要亂改。
- 沒有移植 `sentry.aim` 全套 TF/controller，只同步哨兵外參與短超時響應思想。
