# ROS2 決策與輔瞄系統文檔索引

> 本目錄包含 `ros2_ly_ws_sentary` 工作空間的所有子項目詳細文檔。

---

## 系統整體架構

```
                        ┌─────────────────────────────────────┐
                        │          gimbal_driver               │
                        │  （串口←→電控板，唯一硬件接入點）     │
                        └───┬──────────────────────────────────┘
           /ly/gimbal/angles│      ↑ /ly/control/{angles,firecode,vel}
                            ▼      │
                        ┌──────────┴──────────┐
                        │       detector       │
                        │  （相機→YOLO→PnP）   │
                        └──┬────┬─────────────┘
         /ly/detector/armors│    │/ly/outpost/armors  /ly/ra/angle_image
                            │    │                         │
                            ▼    ▼                         ▼
                    ┌─────────┐ ┌───────────────┐  ┌──────────────┐
                    │tracker  │ │outpost_hitter  │  │  buff_hitter │
                    │_solver  │ │（前哨站 EKF）  │  │  （打符）    │
                    └────┬────┘ └───────┬───────┘  └──────┬───────┘
      /ly/tracker/results│              │                   │
                         ▼             │/ly/outpost/target  │ly/buff/target
                    ┌─────────┐        │                   │
                    │predictor│        │                   │
                    │（EKF+  │        │                   │
                    │ 彈道）  │        │                   │
                    └────┬────┘        │                   │
     /ly/predictor/target│             │                   │
                         └────────────►├◄──────────────────┘
                                       │
                               ┌───────┴──────────────┐
                               │    behavior_tree       │
                               │  （決策 + 模式切換）  │
                               └──────────────────────-┘
```

---

## 子項目文檔清單

| 子項目 | 文檔 | 功能簡述 |
|--------|------|----------|
| `auto_aim_common` | [auto_aim_common.md](auto_aim_common.md) | 公共消息和頭文件，整個系統的基礎依賴 |
| `gimbal_driver` | [gimbal_driver.md](gimbal_driver.md) | 串口硬件驅動，讀電控數據/發控制指令的唯一入口 |
| `detector` | [detector.md](detector.md) | 相機圖像→YOLO裝甲板+車輛檢測→PnP解算3D位置 |
| `tracker_solver` | [tracker_solver.md](tracker_solver.md) | 跨幀多目標跟蹤 + 圖像座標→IMU座標解算 |
| `predictor` | [predictor.md](predictor.md) | EKF預測目標位置 + 彈道補償 → 最終瞄準角 |
| `behavior_tree` | [behavior_tree.md](behavior_tree.md) | 決策中心：匯總所有數據，控制模式切換和雲台指令 |
| `sentry_manual` | [sentry_decision_autoaim_manual.md](sentry_decision_autoaim_manual.md) | 交接手冊：當前決策流程、SetPosition說明、輔瞄調參與鏈路自檢 |
| `sentry_posture_change` | [sentry_posture_interface_change_2026-03-03.md](sentry_posture_interface_change_2026-03-03.md) | 哨兵姿態接口改造記錄：Topic 契約、代碼改動、下位機對接事項 |
| `outpost_hitter` | [outpost_hitter.md](outpost_hitter.md) | 前哨站專用打擊節點（旋轉預測+打擊時機選擇） |
| `buff_hitter` | [buff_hitter.md](buff_hitter.md) | 能量機關（打符）識別與瞄準節點 |
| `record` | [record.md](record.md) | 錄像和射擊表CSV數據存儲目錄（非ROS節點） |
| `shooting_table_calib` | [shooting_table_calib.md](shooting_table_calib.md) | 彈道標定工具（集成detector+tracker+predictor，含鍵盤交互） |

---

## 關鍵 ROS2 Topic 速查表

| Topic | 消息類型 | 發布者 | 主要訂閱者 |
|-------|----------|--------|------------|
| `/ly/gimbal/angles` | `GimbalAngles` | gimbal_driver | detector, tracker_solver, predictor, behavior_tree, shooting_table_calib |
| `/ly/control/angles` | `GimbalAngles` | behavior_tree / shooting_table_calib | gimbal_driver |
| `/ly/control/firecode` | `UInt8` | behavior_tree / shooting_table_calib | gimbal_driver |
| `/ly/control/posture` | `UInt8` | behavior_tree（姿態決策） | gimbal_driver |
| `/ly/me/is_team_red` | `Bool` | gimbal_driver | behavior_tree, detector |
| `/ly/detector/armors` | `Armors` | detector | tracker_solver |
| `/ly/outpost/armors` | `Armors` | detector（outpost_enable時） | outpost_hitter |
| `/ly/ra/angle_image` | `AngleImage` | detector（ra_enable時） | buff_hitter |
| `/ly/tracker/results` | `Trackers` | tracker_solver | predictor |
| `/ly/predictor/target` | `Target` | predictor | behavior_tree |
| `/ly/outpost/target` | `Target` | outpost_hitter | behavior_tree |
| `ly/buff/target` | `Target` | buff_hitter | behavior_tree |
| `/ly/bt/target` | `Int32（ArmorType）` | behavior_tree | detector, predictor |
| `/ly/aa/enable` | `Bool` | behavior_tree | detector, buff_hitter |
| `/ly/outpost/enable` | `Bool` | behavior_tree | detector |
| `/ly/gimbal/posture` | `UInt8` | gimbal_driver | behavior_tree（姿態回讀） |
| `/ly/bullet/speed` | `Float32` | gimbal_driver | predictor（應改）、outpost_hitter（應改）、buff_hitter（應改） |

---

## BehaviorTree v4 雙黑板重構計劃

> 目前 `behavior_tree` 節點：BT樹已加載但 `tickRoot()` 被注釋，實際運行的是硬編碼策略函數。

### 雙黑板設計目標

| 黑板 | 職責 | 數據來源 |
|------|------|----------|
| **感知黑板（Perception BB）** | 存儲所有從 ROS 訂閱到的感知數據 | `SubscribeMessage.cpp` 的各回調函數 |
| **決策黑板（Decision BB）** | 存儲 BT 節點決策結果（模式、目標、速度等） | BT 節點動作執行後寫入 |

### 需要從 ROS1 改成 ROS2 的地方（已知問題）

| 文件 | 問題 | 修復方法 |
|------|------|----------|
| `GameLoop.cpp` | `while (ros::ok())` | 改為 `while (rclcpp::ok())` |
| `behavior_tree/src/Application.cpp` | `ros::NodeHandle` 殘留 | 已遷移，再確認 |
| `buff_hitter/main.cpp` | 硬編碼 config.json 路徑 | 改用 `ament_index_cpp::get_package_share_directory()` |
| `outpost_hitter_node.cpp` L92-93 | `yaw_now = 1000.0f` 調試殘留 | 刪除，改用 `msg->yaw/pitch` |
| 所有節點 | 彈速硬編碼 (`23.0f`) | 統一訂閱 `/ly/bullet/speed` |
