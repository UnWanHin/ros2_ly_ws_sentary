# 實測配置修改指南

## 📝 配置檔案位置
[`src/detector/config/auto_aim_config.yaml`](../../src/detector/config/auto_aim_config.yaml:1)

---

## ⚠️ 實測前必須修改的參數

### 1. **關閉虛擬模式（最重要！）**

```yaml
# 第 45 行 - 必須改為 false
"detector_config/use_video": false   # ❌ 測試用視頻模式
"detector_config/use_video": false   # ✅ 實測用真實相機
```

**說明：** `use_video: true` 會從視頻檔案讀取畫面，實測時必須改為 `false` 使用真實相機。

---

### 2. **確認相機序列號**

```yaml
# 第 11 行 - 確認是否為您的相機 SN
"camera_param/camera_sn": "KE0200060396"
```

**如何查看相機 SN：**
```bash
# 如果有大恆相機 SDK 工具
GxViewer  # 打開後查看設備列表

# 或者查看設備信息
ls /dev/video*
v4l2-ctl --list-devices
```

---

### 3. **調整隊伍顏色**

```yaml
# 第 37 行 - 根據實際對手顏色設置
"detector_config/debug_team_blue": true   # 我方藍色，打紅色
"detector_config/debug_team_blue": false  # 我方紅色，打藍色
```

---

### 4. **相機參數調整（根據現場光照）**

```yaml
# 第 12-16 行
"camera_param/ExposureTime": 4000.0  # 曝光時間（微秒）
"camera_param/Gain": 16.0            # 增益
```

**調整建議：**
- **室內/暗環境：** ExposureTime: 5000-8000, Gain: 16-20
- **室外/亮環境：** ExposureTime: 2000-4000, Gain: 8-12
- **標準比賽場地：** ExposureTime: 4000, Gain: 16（當前值）

---

## 📋 完整實測配置對照表

| 參數 | 測試模式 | 實測模式 | 說明 |
|------|---------|---------|------|
| `use_video` | `true` | `false` | ⚠️ 必改 |
| `use_ros_bag` | `false` | `false` | 保持 |
| `camera_sn` | 任意 | 實際SN | 確認 |
| `debug_team_blue` | `true` | 根據對手 | 確認 |
| `web_show` | `true` | `true` | 可選 |
| `show` | `false` | `false` | 可選 |
| `draw` | `true` | `true` | 建議開啟 |
| `save_video` | `false` | `false` | 可選 |
| `ExposureTime` | 4000 | 根據光照 | 調整 |
| `Gain` | 16 | 根據光照 | 調整 |

---

## 🔧 快速修改命令

### 方法 1: 直接編輯（推薦）
```bash
nano ~/ros2_ly_ws_sentary/src/detector/config/auto_aim_config.yaml
```

修改以下行：
```yaml
第 45 行: "detector_config/use_video": false
第 37 行: "detector_config/debug_team_blue": true/false  # 根據對手
第 11 行: "camera_param/camera_sn": "你的相機SN"
```

### 方法 2: 使用 sed 批量修改
```bash
cd ~/ros2_ly_ws_sentary

# 關閉視頻模式
sed -i 's/"detector_config\/use_video": true/"detector_config\/use_video": false/' \
  src/detector/config/auto_aim_config.yaml

# 設置隊伍顏色為紅色（打藍色）
sed -i 's/"detector_config\/debug_team_blue": true/"detector_config\/debug_team_blue": false/' \
  src/detector/config/auto_aim_config.yaml
```

---

## 🎯 實測前檢查清單

### 硬體檢查
- [ ] 相機已連接並供電
- [ ] 串口已連接（雲台驅動）
- [ ] 網線已連接（如果需要）
- [ ] 電源供應穩定

### 配置檢查
```bash
# 檢查關鍵配置
grep "use_video" src/detector/config/auto_aim_config.yaml
# 應該顯示: "detector_config/use_video": false

grep "debug_team_blue" src/detector/config/auto_aim_config.yaml
# 確認顏色設置正確

grep "camera_sn" src/detector/config/auto_aim_config.yaml
# 確認相機序列號正確
```

### 權限檢查
```bash
# 串口權限
sudo chmod 666 /dev/ttyACM0

# 或永久加入組
sudo usermod -aG dialout $USER
```

---

## 🔍 現場調試參數

### 如果檢測不到目標

**1. 調整曝光時間**
```yaml
# 太暗 → 增加曝光
"camera_param/ExposureTime": 6000.0

# 太亮/過曝 → 減少曝光
"camera_param/ExposureTime": 3000.0
```

**2. 調整增益**
```yaml
# 畫面太暗 → 增加增益
"camera_param/Gain": 20.0

# 畫面噪點多 → 減少增益
"camera_param/Gain": 12.0
```

**3. 調整白平衡**
```yaml
# 紅色偏暗 → 增加紅色增益
"camera_param/RedBalanceRatio": 1.5

# 藍色偏暗 → 增加藍色增益
"camera_param/BlueBalanceRatio": 1.8
```

### 實時調整參數（不重啟節點）

ROS2 支持動態參數修改：
```bash
# 查看當前參數
ros2 param list /detector

# 修改曝光時間
ros2 param set /detector camera_param.ExposureTime 5000.0

# 修改增益
ros2 param set /detector camera_param.Gain 18.0
```

---

## 📊 推薦配置組合

### 標準室內場地
```yaml
"camera_param/ExposureTime": 4000.0
"camera_param/Gain": 16.0
"detector_config/use_video": false
"detector_config/web_show": true
"detector_config/draw": true
```

### 強光環境
```yaml
"camera_param/ExposureTime": 2500.0
"camera_param/Gain": 10.0
"detector_config/use_video": false
```

### 弱光環境
```yaml
"camera_param/ExposureTime": 6000.0
"camera_param/Gain": 20.0
"detector_config/use_video": false
```

---

## ⚡ 快速切換配置

### 創建配置備份
```bash
cd ~/ros2_ly_ws_sentary/src/detector/config

# 備份測試配置
cp auto_aim_config.yaml auto_aim_config_test.yaml

# 創建實測配置
cp auto_aim_config.yaml auto_aim_config_real.yaml
# 然後編輯 auto_aim_config_real.yaml 改為實測參數
```

### 快速切換
```bash
# 切換到實測配置
cp auto_aim_config_real.yaml auto_aim_config.yaml

# 切換回測試配置
cp auto_aim_config_test.yaml auto_aim_config.yaml
```

---

## 🚨 常見錯誤

### 錯誤 1: 忘記關閉 use_video
```
[ERROR] [detector]: Failed to open camera
```
**原因：** `use_video: true` 但視頻檔案不存在或相機未連接  
**解決：** 改為 `use_video: false`

### 錯誤 2: 相機 SN 不匹配
```
[ERROR] [detector]: Camera not found: KE0200060396
```
**解決：** 檢查並修改 `camera_sn` 為實際相機序列號

### 錯誤 3: 顏色設置錯誤
```
[INFO] [detector]: No armors detected
```
**可能原因：** `debug_team_blue` 設置與實際對手顏色相反  
**解決：** 切換 `true/false`

---

## 📝 實測配置模板

複製以下內容到 `auto_aim_config.yaml` 的對應位置：

```yaml
# ========== 實測配置 ==========
"camera_param/camera_sn": "你的相機SN"
"camera_param/ExposureTime": 4000.0
"camera_param/Gain": 16.0

"detector_config/use_video": false        # ⚠️ 必須 false
"detector_config/use_ros_bag": false
"detector_config/debug_team_blue": true   # 根據對手調整
"detector_config/web_show": true
"detector_config/draw": true
"detector_config/save_video": false
```

---

## ✅ 修改完成後

```bash
# 1. 重新編譯（如果改了 launch）
cd ~/ros2_ly_ws_sentary
colcon build --packages-select detector

# 2. 重新 source
source install/setup.bash

# 3. 啟動測試
ros2 launch detector auto_aim.launch.py

ros2 launch detector auto_aim.launch.py
ros2 launch detector outpost.launch.py
ros2 launch detector buff.launch.py



# 4. 檢查 Web 顯示
# 瀏覽器打開: http://localhost:8001/stream
```
