# Gimbal 單節點 Debug Profile 設計

## 狀態

已確認，2026-07-16。

## 問題

`gimbal_driver_config.yaml` 同時承載下位機正式基線和導航直連調試參數；
`sentry_all.launch.py` 又會把全域 `config_file` 傳入 driver。兩者令調試 profile
可能進入正式 `/ly/navi/vel -> BT -> /ly/control/vel` 鏈路，造成兩個控制來源寫入同一個
下位機控制 frame。

## 決定

1. `src/gimbal_driver/config/gimbal_driver_config.yaml` 只保留正式硬件與協議設定。
   不再宣告 `navigation_test`、`navigation_test_stale_timeout_ms`、`navigation_mode` 或
   `io_config/navigation_mode/should_rotate/follow_mode_when_false`。
2. `src/gimbal_driver/config/navigation_test.yaml` 是唯一導航直連調試 profile，完整擁有新版
   `navigation_mode` 所需設定。`main.cpp` 的舊 `navigation_test` 讀取相容性暫不移除，但
   profile 不使用它。
3. 正式 `sentry_all.launch.py` 的 gimbal node 只接收 gimbal 正式基線和正式 launch 覆蓋；
   不再接收跨模組 `base_config_file` 或全域 `config_file`。
4. 新增 `gimbal_driver/debug_node.launch.py` 作為只啟動 driver 的調試入口。它按順序載入
   正式基線與 `navigation_test.yaml`，可透過 `debug_config_file:=...` 指向其他未來的單節點
   profile（如 aim debug）。一般 `gimbal_driver.launch.py` 保持無 debug profile 的安全預設。

## 使用契約

```bash
# 正式單節點 driver：沒有導航直連。
ros2 launch gimbal_driver gimbal_driver.launch.py

# 導航直連單節點調試：只編輯 navigation_test.yaml。
ros2 launch gimbal_driver debug_node.launch.py

# 離車／虛擬串口。
ros2 launch gimbal_driver debug_node.launch.py use_virtual_device:=true
```

`debug_node.launch.py` 不啟動 BT；不得與任何發布正式 `/ly/control/*` 的 BT 實例並行。

## 驗收

- 正式基線沒有任何 navigation debug key。
- navigation profile 包含完整 `navigation_mode` 直連設定。
- `debug_node.launch.py` 載入順序為 baseline、debug profile、明確 CLI 覆蓋。
- `sentry_all.launch.py` 不再把全域 overlay 傳給 gimbal driver。
- gimbal package 可建置，靜態 selfcheck 通過，相關說明與圖譜一致。
