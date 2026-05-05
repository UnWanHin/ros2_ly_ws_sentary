# ROS2 輔瞄系統測試指南

## 📋 測試前準備

### 1. 硬體連接檢查
```bash
# 檢查相機是否連接
ls /dev/video* 

# 檢查串口設備（雲台驅動）
ls /dev/ttyACM* /dev/ttyUSB*

# 給予串口權限
sudo chmod 666 /dev/ttyACM0  # 根據實際設備調整
```

### 2. 編譯工作空間
```bash
cd ~/ros2_ly_ws_sentary
colcon build
source install/setup.bash
```

### 3. 檢查配置檔案
```bash
# 確認配置檔案存在
cat src/detector/config/auto_aim_config.yaml | head -20

# 確認模型檔案存在
ls src/detector/Extras/*.xml
ls src/buff_hitter/utils/buff_models/*.xml
```

---

## 🎯 測試流程

### 測試 1: 自瞄模式（打裝甲板）

#### 啟動命令
```bash
ros2 launch detector auto_aim.launch.py
```

#### 啟動的節點
- `gimbal_driver` - 雲台驅動
- `detector` - 裝甲板檢測
- `tracker_solver` - 追蹤與解算
- `predictor` - 預測

#### 預期效果
✅ **正常啟動標誌：**
```
[INFO] [gimbal_driver]: Serial port opened: /dev/ttyACM0
[INFO] [detector]: Camera initialized successfully
[INFO] [detector]: Armor detector model loaded
[INFO] [tracker_solver]: Tracker initialized
[INFO] [predictor]: Predictor initialized
```

✅ **運行時輸出：**
```
[INFO] [detector]: Detected 3 armors
[INFO] [tracker_solver]: Tracking target ID: 1
[INFO] [predictor]: Target position: (x, y, z)
```

✅ **Web 顯示：**
- 瀏覽器打開：`http://localhost:8001/stream`
- 應該看到相機畫面 + 裝甲板框框

#### 測試步驟
1. 啟動後觀察終端輸出，確認無錯誤
2. 檢查 Web 顯示是否有畫面
3. 將裝甲板放入視野，觀察是否有檢測框
4. 移動裝甲板，觀察追蹤是否穩定
5. 檢查雲台是否跟隨目標移動

#### 常見問題排查
```bash
# 檢查 topic 是否正常發布
ros2 topic list
ros2 topic echo /ly/detector/armors
ros2 topic echo /ly/predictor/target

# 檢查節點狀態
ros2 node list
ros2 node info /detector
```

---

### 測試 2: 前哨站模式

#### 啟動命令
```bash
ros2 launch detector outpost.launch.py
```

#### 啟動的節點
- `gimbal_driver` - 雲台驅動
- `outpost_hitter` - 前哨站檢測與打擊

#### 預期效果
✅ **正常啟動標誌：**
```
[INFO] [gimbal_driver]: Serial port opened
[INFO] [outpost_hitter]: Outpost detector initialized
[INFO] [outpost_hitter]: Waiting for armors...
```

✅ **運行時輸出：**
```
[INFO] [outpost_hitter]: Detected outpost boards
[INFO] [outpost_hitter]: Predicting rotation
[INFO] [outpost_hitter]: Target selected: Board X
```

#### 測試步驟
1. 確認前哨站模型在視野內
2. 觀察是否檢測到三塊裝甲板
3. 檢查旋轉方向判斷是否正確
4. 驗證預測提前量是否合理

---

### 測試 3: 能量機關模式

#### 啟動命令
```bash
ros2 launch detector buff.launch.py
```

#### 啟動的節點
- `gimbal_driver` - 雲台驅動
- `buff_hitter` - 能量機關檢測與打擊

#### 預期效果
✅ **正常啟動標誌：**
```
[INFO] [gimbal_driver]: Serial port opened
[INFO] [buff_hitter]: Buff detector initialized
[INFO] [buff_hitter]: Model loaded: red_best.xml
[INFO] [buff_hitter]: Waiting for rune...
```

✅ **運行時輸出：**
```
[INFO] [buff_hitter]: Detected rune center
[INFO] [buff_hitter]: Detected active blade
[INFO] [buff_hitter]: Rotation speed: 1.047 rad/s
[INFO] [buff_hitter]: Predicted hit point calculated
```

#### 測試步驟
1. 確認能量機關在視野內
2. 觀察是否檢測到中心 R 標
3. 檢查是否識別到待擊打扇葉
4. 驗證旋轉速度計算是否正確
5. 檢查預測擊打點是否合理

---

## 🔍 診斷工具

### 查看所有 Topic
```bash
ros2 topic list
```

### 監聽關鍵 Topic
```bash
# 雲台角度
ros2 topic echo /ly/gimbal/angles

# 檢測結果
ros2 topic echo /ly/detector/armors

# 預測目標
ros2 topic echo /ly/predictor/target

# 控制指令
ros2 topic echo /ly/control/angles
```

### 查看節點狀態
```bash
ros2 node list
ros2 node info /detector
```

### 查看參數
```bash
ros2 param list /detector
ros2 param get /detector camera_param.ExposureTime
```

### 錄製測試數據
```bash
# 錄製所有 topic
ros2 bag record -a

# 錄製特定 topic
ros2 bag record /ly/gimbal/angles /ly/detector/armors /ly/predictor/target
```

---

## ⚠️ 常見問題與解決方案

### 問題 1: 相機無法打開
```
[ERROR] [detector]: Failed to initialize camera
```
**解決方案：**
```bash
# 檢查相機連接
ls /dev/video*

# 檢查相機 SN 是否正確
# 編輯 src/detector/config/auto_aim_config.yaml
# 修改 camera_param/camera_sn
```

### 問題 2: 串口無法打開
```
[ERROR] [gimbal_driver]: Failed to open serial port
```
**解決方案：**
```bash
# 給予權限
sudo chmod 666 /dev/ttyACM0

# 或加入 dialout 組
sudo usermod -aG dialout $USER
# 需要重新登入
```

### 問題 3: 模型載入失敗
```
[ERROR] [detector]: Failed to load model
```
**解決方案：**
```bash
# 檢查模型檔案是否存在
ls src/detector/Extras/*.xml

# 檢查路徑配置
grep "detector_path" src/detector/config/auto_aim_config.yaml
```

### 問題 4: 沒有檢測到目標
```
[INFO] [detector]: No armors detected
```
**檢查項目：**
1. 光照條件是否合適
2. 曝光時間是否正確（`camera_param.ExposureTime`）
3. 增益是否合適（`camera_param.Gain`）
4. 隊伍顏色設置是否正確（`debug_team_blue`）

### 問題 5: Web 顯示無畫面
**解決方案：**
```bash
# 檢查配置
grep "web_show" src/detector/config/auto_aim_config.yaml

# 確認設為 true
# 瀏覽器訪問 http://localhost:8001/stream
```

---

## 📊 性能指標

### 正常運行指標
- **檢測幀率**: 60-120 FPS
- **追蹤延遲**: < 20ms
- **預測誤差**: < 5cm (5m 距離)
- **雲台響應**: < 50ms

### 監控命令
```bash
# 查看 CPU 使用率
top -p $(pgrep -f detector_node)

# 查看 topic 頻率
ros2 topic hz /ly/detector/armors
ros2 topic hz /ly/predictor/target
```

---

## 🎯 測試檢查清單

### 自瞄模式
- [ ] 節點正常啟動
- [ ] 相機畫面正常
- [ ] 能檢測到裝甲板
- [ ] 追蹤穩定不丟失
- [ ] 預測位置合理
- [ ] 雲台跟隨流暢
- [ ] Web 顯示正常

### 前哨站模式
- [ ] 節點正常啟動
- [ ] 能檢測到三塊板
- [ ] 旋轉方向判斷正確
- [ ] 預測提前量合理
- [ ] 擊打時機準確

### 能量機關模式
- [ ] 節點正常啟動
- [ ] 能檢測到中心 R
- [ ] 能識別待擊打扇葉
- [ ] 旋轉速度計算正確
- [ ] 預測擊打點準確

---

## 🚀 上車實測建議

1. **先在實驗室測試**
   - 使用錄製的視頻測試
   - 確認所有功能正常

2. **分步驟上車**
   - 第一步：只測試檢測（不開火）
   - 第二步：測試追蹤穩定性
   - 第三步：測試完整打擊流程

3. **記錄測試數據**
   ```bash
   ros2 bag record -a -o test_$(date +%Y%m%d_%H%M%S)
   ```

4. **準備應急方案**
   - 保留手動控制模式
   - 準備快速停止命令（Ctrl+C）

---

## 📞 需要幫助？

如果遇到問題，提供以下信息：
1. 完整的錯誤訊息
2. `ros2 topic list` 輸出
3. `ros2 node list` 輸出
4. 配置檔案內容
5. 測試環境描述（光照、距離等）
