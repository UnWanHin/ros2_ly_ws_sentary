# tracker_solver — 裝甲板跟蹤與座標解算節點

## 概述

`tracker_solver` 是視覺流水線中的中間環節，接收 `detector` 發布的**圖像座標系下**的裝甲板和車輛檢測結果，輸出**IMU（世界）座標系下**的跟蹤結果。

**功能**：
1. 跟蹤（Tracker）：用匈牙利算法做裝甲板的多目標跟蹤，解決跨幀匹配
2. 解算（Solver）：通過雲台角和相機矩陣把圖像坐標轉換為IMU座標
3. 將 `Trackers.msg` 發布給 `predictor`

---

## 目錄結構

```
tracker_solver/
├── CMakeLists.txt
├── package.xml
├── car_tracker_solver_node.cpp   # 主節點（CarTrackerSolverNode 類）
├── include/
│   ├── car_tracker/              # 跟蹤器頭文件
│   │   ├── tracker.hpp           # 跟蹤器接口
│   │   ├── tracker_matcher.hpp   # 匹配器（匈牙利算法）
│   │   └── （第三個文件）
│   └── solver/
│       └── solver.hpp            # 座標解算接口
└── src/
    ├── solver.cpp                # solver 實現
    └── tracker.cpp               # tracker 實現
```

---

## 核心文件詳解

### `car_tracker_solver_node.cpp` — 主節點

#### 初始化順序（非常重要）

這個節點有**特殊的兩段初始化設計**，原因是 `solver` 需要從 ROS 參數服務器讀取相機內參，而讀取需要節點指針已經就緒：

```cpp
int main() {
    rclcpp::init();
    auto app = make_shared<CarTrackerSolverNode>();  // 1. 構造，只初始化node對象
    
    // 2. 先把全局指針設好
    ly_auto_aim::solver::global_tracker_solver_node = node_ptr;
    global_node_ptr = node_ptr;
    
    // 3. 然後才調用含參數讀取的Init()
    app->Init();  // 這裡創建 tracker 和 solver，solver可以讀到參數了
    
    rclcpp::spin(node_ptr);
}
```

> **踩坑記錄**：如果在構造函數裡直接 `createSolver()`，此時全局指針還是 nullptr，solver 拿不到相機內參導致崩潰或參數讀取錯誤。

#### 訂閱的 Topics

| Topic | 說明 |
|-------|------|
| `/ly/detector/armors` | 接收裝甲板和車輛的圖像座標（帶雲台角） |

#### 發布的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/tracker/results` | `Trackers` | 裝甲板和車輛的 IMU 座標跟蹤結果 |

#### `detection_callback()` — 核心流程

```
detection_callback(Armors.msg)
│
├── convertToDetections()     // Armors.msg → vector<Detection>（保留角點像素座標）
├── convertToCarDetections()  // Armors.msg.cars → vector<CarDetection>（保留BBox）
│
├── GimbalAngleType gimbal_angle{msg->pitch, msg->yaw}  // 取該幀綁定的雲台角
│
├── tracker->merge(detections)       // 裝甲板跟蹤更新（匈牙利算法匹配）
├── tracker->merge(car_detections)   // 車輛跟蹤更新
│
├── timestamp = rclcpp::Time(msg->header.stamp).seconds()  // 時間戳（注意清洗不同時鐘源）
│
├── track_results = tracker->getTrackResult(timestamp, gimbal_angle)  // 獲取當前跟蹤結果
├── solver->solve_all(track_results, gimbal_angle)   // 座標解算（圖像→IMU）
│
└── publish_all() → /ly/tracker/results
```

> **關鍵設計**：時間戳用 `rclcpp::Time(msg->header.stamp).seconds()` 先轉換為 double 再包成 `TimeStamp`，解決 ROS2 中 "Different Time Sources" 報錯（當消息時鐘與系統時鐘來源不同時會崩潰）。

---

### `src/tracker.cpp` + `include/car_tracker/tracker.hpp`

**`Tracker`** — 多目標跟蹤器

使用匹配算法（通常是 IoU 匈牙利算法）進行跨幀裝甲板關聯：

| 函數 | 說明 |
|------|------|
| `merge(detections)` | 用本幀檢測更新跟蹤狀態（新目標初始化、已有目標更新、丟失目標標記） |
| `merge(car_detections)` | 同上，用於車輛 |
| `getTrackResult(timestamp, gimbal)` | 返回 `TrackResultPairs`（裝甲板跟蹤結果 + 車輛跟蹤結果） |

**跟蹤器會為每個跟蹤目標維護**：
- `car_id`：車輛ID
- `armor_id`：裝甲板ID（區分同一輛車的多塊裝甲板）
- 歷史位置（用於卡爾曼濾波或EKF預測在丟幀時補全位置）

---

### `include/car_tracker/tracker_matcher.hpp`

**`TrackerMatcher`** — 裝甲板匹配器

實現跨幀裝甲板到跟蹤軌跡的匹配邏輯，通常使用：
- **匈牙利算法**（最優分配）
- **代價函數**：角點距離 + type是否一致

---

### `src/solver.cpp` + `include/solver/solver.hpp`

**`Solver`** — 座標解算器

把裝甲板的圖像坐標（像素）+ 雲台角 → IMU 座標系的3D位置：

```
[像素角點] → PnP → [相機座標 xyz_cam]
[xyz_cam] + [gimbal_angle(pitch, yaw)] → 旋轉矩陣 → [IMU座標 xyz_imu]
```

**全局指針**：
```cpp
extern std::shared_ptr<rclcpp::Node> global_tracker_solver_node;
```
`createSolver()` 內部通過這個全局節點指針讀取相機內參 YAML 參數。

**`location::Location::registerSolver(solver)`**：將 solver 注冊到全局 Location 管理器，供其他模塊（如 `predictor`）使用同一個解算器實例。

---

## 全局指針設計說明

```cpp
// tracker_solver 的全局指針
extern shared_ptr<rclcpp::Node> global_tracker_solver_node;  // solver用

// predictor 的全局指針（另一套）
extern shared_ptr<rclcpp::Node> global_predictor_solver_node;
extern shared_ptr<rclcpp::Node> global_predictor_node;
extern shared_ptr<rclcpp::Node> global_controller_node;
```

這是 **ROS1 舊代碼遺留的設計模式**（原來是 `ros::NodeHandle` 全局變量），遷移到 ROS2 時保留了這個模式以最小化改動量。

未來重構為 BT v4 雙黑板時，這些全局指針可以考慮統一由 `behavior_tree` 節點管理。

---

## 數據流

```
/ly/detector/armors (Armors.msg)
         │
         ▼
CarTrackerSolverNode::detection_callback()
         │
         ├── tracker->merge()    ← 跟蹤匹配
         ├── solver->solve_all() ← 座標轉換（圖像→IMU）
         │
         ▼
/ly/tracker/results (Trackers.msg)
    ├── armor_trackers[]: x,y,z(IMU), yaw, armor_id, car_id
    └── car_trackers[]: car_id, bounding_rect
```

---

## 修改注意事項

- **相機內參改變**：修改相機後需要更新 YAML 文件裡的相機矩陣，`solver` 在 `Init()` 時讀取
- **時間戳問題**：`rclcpp::Time` 會區分時鐘源（系統時鐘 vs ROS時鐘），用 `.seconds()` 抽取 double 是最安全的跨時鐘操作方式
- **跟蹤器丟幀**：`tracker` 內部有最大丟幀次數（MAX_LOSS_COUNT 類似閾值），超過後自動刪除跟蹤目標
- **雙黑板重構方向**：`solver` 的全局配置（內參等）未來可以放入「靜態配置黑板」，`tracker` 的跟蹤狀態可以放入「動態數據黑板」
