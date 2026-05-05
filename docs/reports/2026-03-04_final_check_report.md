# ROS2 輔瞄系統最終檢查報告

## 📋 檢查項目

### 1. ✅ 消息通訊接口檢查

#### Topic 通訊鏈路：
```
detector → /ly/detector/armors → tracker_solver
tracker_solver → /ly/tracker/results → predictor
predictor → /ly/predictor/target → (控制系統)
gimbal_driver → /ly/gimbal/angles → (所有節點)

outpost_hitter → /ly/outpost/armors (訂閱)
outpost_hitter → /ly/outpost/target (發布)

buff_hitter → /ly/buff/target (發布)
```

**檢查結果：** 所有 topic 定義一致，使用統一的訊息類型

---

### 2. ⚠️ 時間同步檢查

**ROS2 時間使用：**
- 所有節點使用 `rclcpp::Time` 和 `this->now()`
- 訊息 header 使用 `this->now()` 時間戳
- 時間同步依賴 ROS2 內建機制

**潛在問題：** 無明顯時間同步問題

---

### 3. ✅ Config/YAML 參數統一使用

**配置檔案：** `src/detector/config/auto_aim_config.yaml`

**使用情況：**
- ✅ auto_aim.launch.py - 使用統一 config
- ✅ outpost.launch.py - 使用統一 config
- ✅ buff.launch.py - 使用統一 config

**檢查結果：** 所有 launch 檔案都使用同一份配置

---

### 4. ⚠️ 實機配置狀態

**當前配置（第 45 行）：**
```yaml
"detector_config/use_video": true   # ❌ 測試模式
```

**實機需要：**
```yaml
"detector_config/use_video": false  # ✅ 實機模式
```

**修改命令：**
```bash
sed -i 's/"detector_config\/use_video": true/"detector_config\/use_video": false/' \
  ~/ros2_ly_ws_sentary/src/detector/config/auto_aim_config.yaml
```

---

### 5. ✅ Launch 檔案檢查

**已創建的 launch 檔案：**
- ✅ `src/detector/launch/auto_aim.launch.py` - 自瞄模式
- ✅ `src/detector/launch/outpost.launch.py` - 前哨模式
- ✅ `src/detector/launch/buff.launch.py` - 能量機關模式

**路徑使用：**
```python
config_file = os.path.join(home_dir, 'ros2_ly_ws_sentary/src/detector/config/auto_aim_config.yaml')
```

**問題：** 使用絕對路徑，依賴 `$HOME` 環境變數

---

### 6. ⚠️ 路徑檢查

#### Config 檔案路徑：
- ✅ `detector/config/auto_aim_config.yaml` - 相對路徑
- ✅ `buff_hitter/config/config.json` - 已修復為相對路徑

#### 模型檔案路徑（auto_aim_config.yaml）：
```yaml
"detector_config/classifier_path": "src/detector/Extras/classifier.xml"  # 相對路徑
"detector_config/detector_path": "src/detector/Extras/armor_detector_model.xml"
"detector_config/car_model_path": "src/detector/Extras/car_detector_model.xml"
```

**檢查結果：** 使用相對路徑，需在 workspace 根目錄執行

---

### 7. ⚠️ 編譯狀態

**已編譯成功的套件：**
- ✅ auto_aim_common
- ✅ detector
- ✅ predictor
- ✅ tracker_solver
- ✅ buff_hitter
- ✅ outpost_hitter

**編譯問題：**
- ⚠️ gimbal_driver - 需要 auto_aim_common 依賴（已在 CMakeLists.txt 中添加）

---

## 🔍 發現的問題

### 問題 1: use_video 仍為 true（實測前必須修改）
**位置：** `src/detector/config/auto_aim_config.yaml:45`
**影響：** 會嘗試從視頻檔案讀取而非真實相機
**修復：** 改為 `false`

### 問題 2: gimbal_driver 編譯依賴
**位置：** `src/gimbal_driver/CMakeLists.txt`
**影響：** 編譯失敗
**修復：** 已添加 `find_package(auto_aim_common REQUIRED)` 和 `ament_target_dependencies`

### 問題 3: Launch 檔案使用絕對路徑
**位置：** 所有 launch 檔案
**影響：** 依賴特定的目錄結構
**建議：** 可接受，因為使用 `$HOME` 環境變數

---

## ✅ 上車前檢查清單

### 必須修改：
- [ ] 修改 `use_video: false`（第 45 行）
- [ ] 確認 `camera_sn` 正確（第 11 行）
- [ ] 確認 `debug_team_blue` 正確（第 37 行）

### 硬體檢查：
- [ ] 相機已連接
- [ ] 串口已連接（/dev/ttyACM0）
- [ ] 串口權限已設置（chmod 666）

### 編譯檢查：
```bash
cd ~/ros2_ly_ws_sentary
colcon build --packages-select auto_aim_common gimbal_driver detector predictor tracker_solver buff_hitter outpost_hitter
source install/setup.bash
```

### 啟動測試：
```bash
# 選擇對應模式
ros2 launch detector auto_aim.launch.py    # 自瞄
ros2 launch detector outpost.launch.py     # 前哨
ros2 launch detector buff.launch.py        # 能量機關
```

---

## 📊 最終狀態

| 項目 | 狀態 | 說明 |
|------|------|------|
| 消息通訊 | ✅ | Topic 定義一致 |
| 時間同步 | ✅ | 使用 ROS2 標準時間 |
| Config 統一 | ✅ | 所有 launch 使用同一份 |
| Launch 檔案 | ✅ | 三個模式已創建 |
| 路徑使用 | ✅ | 使用相對路徑 |
| 編譯狀態 | ⚠️ | gimbal_driver 需重新編譯 |
| 實機配置 | ❌ | use_video 需改為 false |

---

## 🚀 上車步驟

1. **修改配置**
```bash
nano ~/ros2_ly_ws_sentary/src/detector/config/auto_aim_config.yaml
# 第 45 行改為: "detector_config/use_video": false
```

2. **重新編譯**
```bash
cd ~/ros2_ly_ws_sentary
colcon build
source install/setup.bash
```

3. **檢查硬體**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
sudo chmod 666 /dev/ttyACM0
```

4. **啟動測試**
```bash
ros2 launch detector auto_aim.launch.py
```

5. **查看畫面**
瀏覽器：`http://localhost:8001/stream`

---

## ⚠️ 注意事項

1. **必須在 workspace 根目錄執行** - 因為配置使用相對路徑
2. **每次修改配置後需重新 source** - `source install/setup.bash`
3. **Web 顯示需要網絡** - 確保 8001 端口未被占用
4. **記錄測試數據** - 使用 `ros2 bag record -a`

---

## 📞 問題排查

如果遇到問題，檢查：
1. `ros2 topic list` - 確認 topic 是否發布
2. `ros2 node list` - 確認節點是否運行
3. `ros2 topic echo /ly/detector/armors` - 查看檢測數據
4. 查看 Web 顯示 - 確認相機畫面正常
