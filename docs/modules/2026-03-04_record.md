# record — 錄像文件目錄

## 概述

`record` **不是一個 ROS2 節點**，而是一個**數據存儲目錄**，用於存放各種測試和標定時的錄像以及射擊表 CSV 數據。

---

## 目錄結構

```
record/
├── record.mkv                          # 錄像文件（比賽或測試場景的相機錄像）
└── shooting_table_*.csv                # 射擊表數據（多份時間戳命名）
    ├── shooting_table_1753380882.csv
    ├── shooting_table_1753381417.csv
    └── ...（共21份 CSV 文件）
```

---

## 文件說明

### `record.mkv`

一個 MKV 格式的視頻文件（約24MB），是相機采集的測試錄像。  
`detector` 節點在 `use_video=true` 模式下可以直接讀取此類視頻文件（通過 `camera_param/video_path` 配置）作為輸入源，用於離線調試。

### `shooting_table_*.csv`

由 `shooting_table_calib` 節點生成的射擊彈道標定數據。文件名中的數字是 Unix 時間戳（如 `1753380882`）。

每份 CSV 文件約 157-291 字節，格式包含標定拍攝距離、測量到的實際落點俯仰角偏差等，供 `shooting_table_calib` 節點擬合彈道補償曲線。

---

## 與其他節點的關係

| 關係 | 說明 |
|------|------|
| `detector` 讀取 `record.mkv` | 在 `use_video=true` 配置下，`Camera::Initialize(video_path)` 使用 OpenCV 讀取此視頻 |
| `shooting_table_calib` 生成 CSV | 每次標定結束後以當前時間戳命名保存，可選擇最新的作為正式標定結果 |

---

## 使用場景

1. **離線調試**：把 `record.mkv` 的路徑設為 `detector_config/video_path`，在沒有相機的環境下回放測試
2. **彈道標定**：用 `shooting_table_calib` 對著靶板標定不同距離的落點，生成 CSV 後在 `predictor/solver` 中加載
