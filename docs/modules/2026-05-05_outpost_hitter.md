# outpost_hitter — 前哨站打擊節點

## 概述

`outpost_hitter` 是專門針對**前哨站**（旋轉裝甲板）的特化打擊節點。前哨站是一個以固定角速度旋轉的圓盤（三塊裝甲板均勻分布120°），需要預測旋轉角速度來命中。

**功能**：接收 `detector` 的前哨裝甲板 → PnP解算 → 判斷旋轉方向 → EKF預測 → 計算最佳打擊角 → 發布到 `behavior_tree`

---

## 目錄結構

```
outpost_hitter/
├── CMakeLists.txt / package.xml
├── outpost_hitter_node.cpp     # 主節點（OutpostHitterNode 類）
├── include/
│   ├── controller/
│   │   ├── BoardSelector.hpp   # 最佳打擊方案選擇器
│   │   └── MuzzleSolver.hpp    # 炮口角度解算（含提前量）
│   ├── detector/               # 前哨站相關檢測類型
│   ├── predictor/
│   │   ├── OutpostPredictor.hpp  # EKF旋轉預測器
│   │   ├── DerectionJudger.hpp   # 旋轉方向判斷器
│   │   └── TopFilter.hpp
│   ├── solver/
│   │   └── PoseSolver.hpp      # PnP解算器（圖像→世界座標）
│   └── utils/utils.h
└── src/
    ├── BoardSelector.cpp / MuzzleSolver.cpp
    ├── OutpostPredictor.cpp / DirectionJudger.cpp
    ├── PoseSolver.cpp / TopFilter.cpp
```

---

## 核心流程

### `outpost_detection_callback()` 五步流程

```
1. convertToDetections()
   ├── 只保留 type==7 的裝甲板（前哨站裝甲板）
   └── 過濾傾斜角超過30°的裝甲板（几乎水平的不打）

2. SOLVER::PoseSolver::solveArmorPoses()
   └── 圖像角點 + 雲台角 → ArmorPose（pitch/yaw/distance，世界座標系）

3. 選被最近的裝甲板（min_element by distance）

4a. PREDICTOR::DirectionJudger::updateWorldPYD()
    └── 累積多幀yaw觀測來判斷旋轉方向（順/逆時針）

4b. PREDICTOR::OutpostPredictor::runPredictor()
    └── EKF預測 theta（旋轉角）和 omega（角速度）
    └── 返回 OutpostInformation{is_valid, outpost_theta, outpost_omega}

5. CONTROLLER::MuzzleSolver::solveMuzzle()
   ├── 計算飛行時間 → 提前量 = omega × 飛行時間
   └── 返回 BoardInformations（多個候選打擊角）
   → CONTROLLER::BoardSelector::selectBestBoard()
   → 發布 /ly/outpost/target
```

---

## 各文件說明

| 文件 | 類名 | 功能 |
|------|------|------|
| `outpost_hitter_node.cpp` | `OutpostHitterNode` | 主節點，協調各模塊，訂閱/發布ROS消息 |
| `PoseSolver.cpp/.hpp` | `SOLVER::PoseSolver` | PnP解算，用前哨裝甲板真實尺寸，通過 `SOLVER::global_solver_node` 讀相機內參 |
| `DirectionJudger.cpp/.hpp` | `PREDICTOR::DirectionJudger` | 累積觀測判斷旋轉方向（順/逆時針），方向判斷後才能初始化預測器 |
| `OutpostPredictor.cpp/.hpp` | `PREDICTOR::OutpostPredictor` | EKF預測前哨站旋轉角（theta）和角速度（omega），支持量測更新（USE_MEASURE_UPDATE） |
| `MuzzleSolver.cpp/.hpp` | `CONTROLLER::MuzzleSolver` | 計算提前量（飛行時間×角速度），輸出各時機的aim_yaw/aim_pitch |
| `BoardSelector.cpp/.hpp` | `CONTROLLER::BoardSelector` | 從候選打擊方案中選yaw偏差最小的，超過80°的被過濾 |
| `TopFilter.cpp/.hpp` | `PREDICTOR::TopFilter` | 頂部裝甲板過濾（功能未完整）|

---

## Topic 連接

| 方向 | Topic | 說明 |
|------|-------|------|
| 訂閱 | `/ly/outpost/armors` | 來自 `detector`（`outpost_enable=true` 時） |
| 發布 | `/ly/outpost/target` | 前哨站瞄準角度，發給 `behavior_tree` |

---

## ⚠️ 修改注意事項

- **調試殘留**：`outpost_hitter_node.cpp` 第92-93行有 `yaw_now = 1000.0f; pitch_now = 10.0f;`，正式使用必須刪除，改用 `msg->yaw/pitch`
- **彈速硬編碼**：`setBulletSpeed(23.0)` 應改為訂閱 `/ly/bullet/speed`
- **全局指針**：`SOLVER::global_solver_node` 與 `predictor` 的 `global_predictor_solver_node` **不同**，各自獨立讀相機內參
- **上游依賴**：`behavior_tree` 通過 `/ly/vision/mode=3` 控制是否激活前哨模式；`detector` 據此決定是否通過 `/ly/outpost/armors` 輸出前哨裝甲板
