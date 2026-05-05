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
- `src/tracker_solver/car_tracker_solver_node.cpp`

行為：

- PnP 選解改為 per `(car_id, armor_id)` yaw history，去掉跨目標 static 記憶污染。
- 每個 IPPE 解都計算 reprojection error。
- 有 history 時加 yaw continuity penalty；有 whole-car bbox 時只加弱 yaw hint penalty。
- 首次無 history 時主要看 reprojection error。
- whole-car bbox 無效或 PnP 解不可用時回退 armor-only PnP。
- `solver` 保存本幀 PnP debug record，`tracker_solver_node` 在 AimTimer 開啟時輸出 `event=pnp`，包含 IPPE 解數、選中解、reprojection error、score、armor/world yaw 和解算後 world xyz。
- solver 啟動時檢查相機矩陣是否是標準正焦距 / `K[2,2] = 1`；單節點缺參時先初始化 identity/zero，避免未初始化矩陣讓 PnP 診斷輸出亂值。

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
- `predictor_node` 同時保存有效 update 對應的 tracker `header.stamp`，並用 `predictor_config.max_tracker_age_sec` 限制太舊的 tracker 幀，避免 callback 時間是新的、但觀測本身已經落後。
- `predictor_node` 1Hz 輸出 `armors/cars/model_updates/tracker_age_ms/max_xyz_jump/max_yaw_jump_deg`，用於區分「時間舊」和「world 坐標抖」。
- AimTimer 開啟時，`predictor_node` 每個 tracker update 輸出 `event=tracker_update`，每個 publish timer 輸出 `event=target_timer`，用於對齊 tracker 幀時間、模型 update、控制輸出和 target suppression 原因。
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
- `src/behavior_tree/src/Logger.cpp`
- `config/common.yaml`
- `scripts/launch/start_sentry_all.sh`

行為：

- `/ly/predictor/target` 回調尊重 `msg->status`。
- `FireStatus`、`Valid`、`Fresh` 都由 `msg->status` 決定。
- 只有有效 target 才 latch yaw/pitch、更新 `LastValidTime`、置位 `isFindTargetAtomic`。
- BT 自己的文件 log 從原來默認 `~/Log/BT_*.log` 改為 `~/Log/BT/BT_*.log`。
- BT 普通文件 log 增加 `bt_file_log_enable` 開關；關閉後仍保留 console log，不再生成 `BT_*.log`。
- `BT_LOG_DIR` 環境變數仍可覆蓋；未設時 `start_sentry_all.sh` 從 `config/common.yaml` 讀 `bt_log_dir: ~/Log/BT`。
- `BT_APP_FILE_LOG_ENABLE` 環境變數仍可臨時覆蓋；未設時 `start_sentry_all.sh` 從 `config/common.yaml` 讀 `bt_file_log_enable`。

### AimTimer 診斷日誌

涉及文件：

- `config/base_config.yaml`
- `config/common.yaml`
- `scripts/launch/start_sentry_all.sh`
- `src/behavior_tree/launch/sentry_all.launch.py`
- `src/tracker_solver/car_tracker_solver_node.cpp`
- `src/tracker_solver/include/solver/solver.hpp`
- `src/tracker_solver/src/solver.cpp`
- `src/predictor/predictor_node.cpp`

開關：

```yaml
aim_timer_log:
  enable: true
  dir: "~/Log/AimTimer"

"aim_timer_log/enable": true
"aim_timer_log/dir": "~/Log/AimTimer"
```

默認仍是 `false`，避免正常跑車時一直寫高頻文件。打開後會生成：

- `~/Log/AimTimer/AT_YYYYMMDD_HHMMSS_tracker_solver.log`
- `~/Log/AimTimer/AT_YYYYMMDD_HHMMSS_predictor.log`

`common.yaml` 也提供啟動層開關，`start_sentry_all.sh` 會把它轉成 launch 參數覆蓋到 tracker_solver / predictor：

```yaml
aim_timer_log_enable: false
aim_timer_log_dir: ~/Log/AimTimer
```

兩層配置的作用不同：

- `base_config.yaml`：直接 `ros2 launch` 或單節點啟動時的 ROS 參數默認。
- `common.yaml`：`scripts/launch/start_sentry_all.sh` 的現場入口；值會覆蓋 `base_config.yaml`。

關鍵事件：

- `node=tracker_solver event=tracker_frame`
  - `input_age_ms`：detector armors stamp 到 tracker_solver callback 的時間差。
  - `max_xyz_jump` / `max_yaw_jump_deg`：tracker_solver 解算後 world 坐標和 yaw 的幀間最大跳變。
  - `det_armors/track_armors/det_cars/track_cars`：檢測和 tracker 輸出的數量關係。
- `node=tracker_solver event=pnp`
  - `mode=whole_car|armor_only`：當幀用了整車 bbox 提示還是純裝甲板 PnP。
  - `solutions/selected`：IPPE 解數和選中解 index。
  - `reproj_px/score`：重投影誤差和加 penalty 後的選解分數。
  - `has_history/has_whole_car_hint`：是否用了 yaw continuity / 整車弱提示。
  - `armor_yaw_deg/world_yaw_deg/world_x/world_y/world_z`：PnP 直接輸出與轉到 world 後的位置。
- `node=predictor event=tracker_update`
  - `tracker_age_ms`：tracker results stamp 到 predictor callback 的時間差。
  - `armors/cars/model_updates`：是否有裝甲、有整車 bbox、EKF 是否真正 update。
  - `max_xyz_jump/max_yaw_jump_deg`：predictor 看到的 tracker 輸入是否已經抖。
- `node=predictor event=target_timer`
  - `receive_age_ms`：最近一次有效 model update 到控制 timer 的時間。
  - `tracker_stamp_age_ms`：最近一次有效觀測 stamp 到控制 timer 的時間。
  - `observation_fresh/status/publish/reason`：target 是正常發布，還是因 stale / no_prediction / controller invalid 被壓掉。
  - `yaw_cmd_deg/pitch_cmd_deg/gimbal_yaw_deg/gimbal_pitch_deg`：控制輸出和當前雲台角。

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
- `predictor_config.max_tracker_age_sec: 0.15`
  - 用 tracker message `header.stamp` 判斷觀測是否足夠新。
  - 如果 `tracker_stamp_age_ms` 長期接近或超過這個值，說明慢主要是時間鏈路/檢測延遲，不應靠加大 EKF 追蹤參數掩蓋。
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

`config/base_config.yaml`：

- `aim_timer_log.enable: false`
  - 改成 `true` 後輸出 AimTimer 文件日誌。
- `aim_timer_log.dir: "~/Log/AimTimer"`
  - 可以臨時改到 `/tmp/AimTimerTest` 做本機測試。

`config/common.yaml`：

- `bt_file_log_enable: true`
  - `true`：生成 `~/Log/BT/BT_YYYYMMDD_HHMMSS.log`。
  - `false`：只保留 console log，不寫 BT 普通文件 log。
- `bt_log_dir: ~/Log/BT`
  - 只在 `bt_file_log_enable: true` 時由啟動腳本創建。
- `aim_timer_log_enable: false`
  - `start_sentry_all.sh` 讀取後傳給 `sentry_all.launch.py`，再覆蓋 tracker_solver / predictor 的 `aim_timer_log.enable`。
- `aim_timer_log_dir: ~/Log/AimTimer`
  - `start_sentry_all.sh` 讀取後傳給 `sentry_all.launch.py`，再覆蓋 tracker_solver / predictor 的 `aim_timer_log.dir`。

`src/detector/detector_node.cpp` 目前仍在代碼內：

- `kMaxGimbalAngleSamples = 200`

這個只控制雲台角 ring buffer 長度，不是控制響應快慢的調參入口。

## 微調建議

現場不要先大調 EKF 噪聲。先確認輸入資料：

- `/ly/detector/armors` 的 `header.stamp`、`yaw`、`pitch` 是否穩定跟隨圖像。
- `/ly/tracker/results` 的裝甲板 yaw 是否還會跨目標跳解。
- `/ly/predictor/target` 是否在丟觀測後約 100ms 內停止發布有效 target；若後續發布 invalid target，BT 也應按 `status=false` 處理。
- `/ly/control/angles` 是否還在 target invalid 後繼續追舊角。
- predictor log 裡 `armors/cars/model_updates` 的關係；如果 `armors > 0` 但 `model_updates = 0` 長時間出現，說明響應慢主要卡在 car bbox 匹配/更新門檻，而不是 timer。
- predictor log 裡 `tracker_age_ms`；如果它長期偏大，說明控制端拿到的不是最新觀測。
- predictor log 裡 `max_xyz_jump/max_yaw_jump_deg`；如果這兩個明顯跳，說明 tracker_solver 輸出的世界坐標/裝甲 yaw 在抖。
- AimTimer `tracker_solver event=pnp` 裡 `reproj_px` 若長期很大或 `selected` 在 0/1 之間頻繁切，優先查 PnP 角點順序、相機 K/D、裝甲尺寸、whole-car hint 是否污染選解。
- AimTimer `tracker_solver event=tracker_frame input_age_ms` 已經很大時，慢在 detector -> tracker_solver 之前；`predictor event=tracker_update tracker_age_ms` 才變大時，慢在 tracker_solver -> predictor 或 ROS 調度。
- AimTimer `predictor event=target_timer observation_fresh=0` 且 `reason=observation_stale/no_predictions_and_stale`，表示控制端正在拒絕舊觀測，不應再靠延長超時掩蓋。

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
source /opt/ros/humble/setup.bash && colcon build --packages-select tracker_solver predictor behavior_tree --allow-overriding tracker_solver predictor behavior_tree
```

結果：

- `git diff --check` 通過。
- `detector`、`tracker_solver`、`predictor`、`behavior_tree` 均 build 通過。
- 步兵 predictor pitch 觀測移植與 controller flight-time 回寫後，`predictor` 單包 build 通過。
- AimTimer / BT log 分組改動後，`tracker_solver`、`predictor`、`behavior_tree` targeted build 通過。

已執行 AimTimer 開關短測：

```bash
mkdir -p /tmp/ros2_logs /tmp/AimTimerTest2
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 2s ros2 run predictor predictor_node --ros-args -p aim_timer_log.enable:=true -p aim_timer_log.dir:=/tmp/AimTimerTest2
timeout 2s ros2 run tracker_solver tracker_solver_node --ros-args -p aim_timer_log.enable:=true -p aim_timer_log.dir:=/tmp/AimTimerTest2
```

結果：

- 兩個命令因 `timeout` 退出，這是預期。
- 沙盒環境有 FastDDS UDP socket 權限警告，不影響本次文件建立驗證。
- 成功生成：
  - `/tmp/AimTimerTest2/AT_20260501_023240_predictor.log`
  - `/tmp/AimTimerTest2/AT_20260501_023240_tracker_solver.log`

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
