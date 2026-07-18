# 裁判小地圖坐標導航

Updated: 2026-07-19

## 結論

裁判 `0x0303 map_command_t` 已接入正式 `behavior_tree` Task 層。有效的坐標模式小地圖點會在
預設 45 秒內擁有導航輸出，仍經既有官方座標標定鏈路，不新增 `/goal_pose` publisher。

```text
lower TypeID=9
  -> /ly/game/map_command
  -> MapCommandTask (dedup + hold)
  -> /ly/navi/goal_pos_raw [official cm]
  -> navi_tf_bridge
  -> /goal_pose
```

## 接受與優先級

- 只接受 `has_target_position=true`、有限、非 `(0,0)`、且落在官方 `0..2800cm x 0..1500cm` 的坐標；厘米取整後為 `(0,0)` 的微小值也拒絕。
- `target_robot_id != 0` 沒有坐標，保留 topic 資料但不導航。
- 同點 20cm 內的 5 次 100ms 重送與持續 1Hz 最新包不延長任務；相同點到期後也不會自動重啟。新座標才會開新 45 秒窗口。
- Task 層高於 Default、Buff/Outpost、RegionalDefense、Special 和 Chase；整個 Hard 層（Recovery 及不可中斷的 ReadyRoadland 穿越）仍然最高。
- Recovery 期間每拍都會記錄並取消最新小地圖命令，避免恢復完成後被原有 1Hz 重送重新接管。

## 邊界

MapCommand 是任意官方地圖坐標，不屬任何 `BaseGoal` 或 MainArea。因此它不修改
`GoalReachState`、區域 arrived、區域 watchdog 或 AreaManager task。任務期間會清除舊目標的
external-status binding 並暫停 `/ly/navi/reach_state`，任務只以持有時間控制輸出。
正常區域導航在它逾時或被 Recovery 取消後，保持原有策略選點邏輯。

## 驗收

- `MapCommandTask` 單元測試覆蓋零點/目標機器人拒絕、厘米轉換、重送去重、逾時、取消後同點不重啟及新點接替。
- `behavior_tree` 目標建置與全部 CTest、靜態 self-check、圖譜 JSON 驗證列入同次改動驗收。
