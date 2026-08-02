# 2026-08-02 姿態與 Outpost Bag 分析

Updated: 2026-08-02

## 狀態

本文件起初是遠端實機 bag/BT 日誌的分析紀錄；2026-08-02 已依據本文根因完成本機
`behavior_tree` 姿態仲裁修正，ROS topic、導航輸出與下位機協議均未改動。

本次 SSH 操作僅做了檔案列表、`ros2 bag info`、短時間 `ros2 bag play`、topic echo 與日誌篩選；沒有在遠端執行 `git pull`、`git push`、編譯或寫入專案檔案。

## 資料來源

| Bag | BT 日誌 | 時間範圍 |
| --- | --- | --- |
| `~/record/20260802_090230/main` | `~/Log/BT/BT_20260802_090226.log` | 09:02:32 - 09:11:32 |
| `~/record/20260802_091450/main` | `~/Log/BT/BT_20260802_091443.log` | 約 09:14 - 09:27 |

兩份 bag 都包含 `/ly/aim/result`、`/ly/aim/select_target`、`/ly/enemy/op_hp`、`/ly/control/posture`、`/ly/upload/typeid10` 等必要 topic。

## 主要發現

### 1. Outpost transit 會壓過 follow Attack

第一份 BT 日誌在 `09:04:57` 出現：

```text
has_target_recent=1
transit_attack_allowed=1
task_source=outpost
desired=Move
cmd=3
```

這不是 `fire=false` 導致的。`follow=true` 已經被算入 `has_target_recent`，而且 transit policy 也顯示允許 Attack；問題在 `SelectDesiredPosture()` 更早把整個 Outpost transit 強制成 Move。

目前程式位置：

- `src/behavior_tree/src/PostureLogic.cpp:429-436`
- `src/behavior_tree/include/PostureTypes.hpp:375-382`

`ResolveTaskPostureRequest()` 本身已經支援 transit 期間的 Attack，但上游傳入的 `scored_desired` 已被固定為 Move，因此該分支無法生效。

### 2. HardMove retry 耗盡後可能選出 Attack

`PostureManager` 在 pending retry 耗盡時，如果 policy 沒有要求保留當前姿態，會呼叫通用的 `choose_alternative_posture()`：

- `src/behavior_tree/src/PostureManager.cpp:269-281`
- `src/behavior_tree/include/PostureTypes.hpp:125-127`

Recovery/HardMove 的日誌因此可能同時出現：

```text
task_source=recovery
task_intent=hard_move
desired=Move
cmd=1
```

這是姿態 fallback 的優先級錯誤，不代表下位機拒收 Move。Recovery/HardMove 應保持最高優先級，不能在 retry 失敗後退化成 Attack。

### 3. 強化 Move 被過早判定 unavailable

第一份 BT 日誌的強化 Move 請求：

```text
09:07:37.197 cmd=6 ... pending_source=recovery_move
09:07:37.799 cmd=6 ... reason=retry_pending
09:07:38.102 cmd=6 ... reason=retry_pending
09:07:38.103 recovery_enhanced_unavailable=1
```

但後續回讀才出現 `current=Move current_enhanced=1`。也就是上位機約 1.2 秒後放棄，而下位機的強化狀態回讀較晚到達。這是 ACK/retry 窗口太短的時序問題。

兩份日志的姿態命令統計：

| 日誌 | 普通 Attack | 普通 Defense | 普通 Move | 強化 Attack(4) | 強化 Defense(5) | 強化 Move(6) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 09:02:30 | 535 | 150 | 1353 | 0 | 0 | 6 |
| 09:14:50 | 374 | 287 | 681 | 0 | 0 | 0 |

### 4. 強化 Attack 沒有觸發的必要條件

`OutpostEngagementLock` 不是只看前哨掉血。它要求同一時序窗口內同時滿足：

- Target 7 fresh；
- `/ly/aim/select_target.id == 7`；
- 前哨 HP fresh 且大於 0；
- 觀測到前哨 HP 下降；
- 強化 Attack 額度 fresh 且大於 0；
- 當前普通姿態為 Attack；
- 沒有 pending，並且姿態切換冷卻完成。

目前程式位置：`src/behavior_tree/src/OutpostEngagementLock.cpp:53-123`。

Bag 中確實能看到選中 ID 7，但目前不能只從摘要統計證明「前哨 HP 下降的同一幀」所有 fresh/selected 條件都成立。因此目前證據支持「強化 Attack 被條件或 pending 擋住」，不支持把問題歸因於下位機鏈路。

### 5. 強化 Defense 沒有形成有效 ProtectHero hold

兩份日志沒有看到完整、穩定的 `protect_hero` 到點駐留與受擊窗口重合。現有強化 Defense 需要 ProtectHero hold 已成立，並在設定的 damage window 內達到傷害閾值；Recovery 或 transit 期間受擊不會直接觸發 ProtectHero 強化 Defense。

## 已落地修正

### SoftTransit 不再強制 Move

`SoftTransit` 現在只表示目前 Task/Tactical 擁有導航 goal，不再直接指定底盤姿態。評分候選
為 Attack 或 Defense 時直接使用；只有候選為 Move 時才沿用既有動態 ETA/Move 額度保留。
Recovery 和新鮮 `/ly/navi/should_rotate=false` 仍是更高優先級 `HardMove`。

### Target 7 短暫失鮮保持

`OutpostEngagementLock` 直接復用唯一的 `Posture.TargetKeepMs=800` 作為 Target 7 grace。
連續 800 ms 內失去 Target 7 或選擇暫時切到其他目標時，lock 仍保持，且不清除普通/強化
姿態 pending；grace 到期才釋放前哨 lock。`TargetLost` 釋放 lock 時不會取消通用 pending；
敵方前哨 HP 失鮮/為 0、導航明確不可達、自身 HP 閾值等明確退出原因仍保持原本的 pending 取消。

### 強化姿態 ACK/retry

普通姿態 `1/2/3` 保持 `600 ms / 300 ms / 3 次`。只有強化姿態 `4/5/6` 使用
`800 ms / 300 ms / 5 次`，讓 TypeID 7/10 回讀約兩秒內收斂，覆蓋本次 bag 約 1.4 秒才
回讀強化 Move 的情況。強化請求最終耗盡時，各自既有的普通姿態 fallback 仍生效。

`HardMove` 的 retry 耗盡不再經由泛用替代姿態選擇器改送 Attack；下一個 BT tick 仍會以最高
優先級要求 Move。

## 分析與完整鏈路

### 姿態完整鏈路

```text
/ly/aim/armor_targets + /ly/aim/result(follow/fire)
  -> SubscribeMessage: externalAimData / lastTargetSeenTime
  -> UpdateBlackBoard: IsFindTarget (只代表本拍 fresh follow)
  -> main.xml: SetAimTarget -> RefreshOutpostEngagementLock
  -> main.xml: SelectPosture
     -> ResolveTaskPostureIntent (SoftTransit / HardMove / Recovery ...)
     -> SelectDesiredPosture (基礎分數、目標、受擊、姿態額度)
     -> ResolveTaskPostureRequest
     -> PostureManager (5 秒切換冷卻、pending、ACK、retry)
  -> /ly/control/posture
  -> gimbal_driver 將 1..6 下發；BT 不發命令時保留下位機現有姿態
```

`/ly/aim/result.fire` 只控制開火，不是 Attack 姿態的必要條件。`follow=true` 會使本拍目標有效，並更新 `lastTargetSeenTime`；姿態層另有 `TargetKeepMs` 短暫保持窗口。

### 原先造成 Attack -> Move -> Attack 的兩個來源

1. `SelectDesiredPosture()` 將 Outpost transit 直接 return Move。這使 transit 被誤當成姿態硬命令，而非導航中的姿態選擇場景。
2. `RefreshOutpostEngagementLock()` 的 Target 7 任一拍失鮮會得到 `TargetLost`，而 `Application` 對所有 lock exit 一律呼叫 `postureManager_.CancelPending()`。下一拍 lock 再次成立時，PostureManager 沒有 pending 可保留，於是又重新發 Attack。這正是實機日誌中短間隔重複 `cmd=1` 的直接原因。

第二項與下位機無關：BT 每拍都可能清除自己尚未 ACK 的請求。下位機只按收到的 `/ly/control/posture` 轉發，不會自行在 Attack 和 Move 之間選擇。

### 目前內部契約

SoftTransit 的語義應改為：**導航目標屬於當前 Task/Tactical，但 Attack、Defense、Move 都是合法姿態候選；Transit 不得自行強制 Move。**

姿態優先級應保持：

```text
Recovery / navi_should_rotate=false (HardMove)
  > 受擊 burst 的 HardDefense
  > 已發出的、尚待 ACK 的高優先級強化請求
  > SoftTransit 姿態評分與資源管理
```

對 SoftTransit：

- `follow=true` 或 `TargetKeepMs` 仍在保持窗口：Attack 可作為普通候選，不依賴 `fire=true`。
- 受擊、低血或 ProtectHero hold：Defense 可作為普通候選。
- 無目標且 Move 額度需要保留，或評分確實選 Move：才由動態 ETA/額度策略選 Move 或替代姿態。
- 任何短暫 Target 7 失鮮都不應把已選的 Attack/Defense 硬切成 Move。

實際修正範圍：

1. 移除 `SelectDesiredPosture()` 中 `outpost_task_active -> Move` 的硬 return，讓既有分數、遲滯和受擊邏輯產生候選。
2. `ResolveTaskPostureRequest(SoftTransit)` 接受評分選出的 Attack 或 Defense；只有評分為 Move 時才交給 `SelectTransitPosture()` 做 Move 額度保留。
3. `OutpostEngagementLock` 對 `TargetLost` 使用短暫 grace（可沿用 `Posture.TargetKeepMs`），並且不因 TargetLost 清除普通或強化姿態 pending。前哨 HP 為 0、導航不可達等明確終止條件才停止 lock 專屬重試。
4. 保留 `HardMove` 的高優先級，但在其 retry 耗盡後維持 Move 或當前安全態，禁止通用 fallback 選 Attack。

這些都是 BT 內部姿態仲裁修正；不改 `/ly/aim/*`、`/ly/control/posture` 的 topic/message，不改 gimbal_driver 的下發格式或導航控制話題。

### 驗收結果

- Outpost SoftTransit 中，Attack/Defense 不會因單拍 `follow` 或 Target 7 抖動被改成 Move。
- 目標短失不會先清 pending 再高頻重發；超過 800 ms 才依正常分數、資源與安全規則轉換。
- Recovery 和 `navi_should_rotate=false` 仍可立即以高優先級要求 Move。
- `/ly/control/posture` 不再出現由 `TargetLost -> CancelPending` 導致的高頻 1/3 往返。
- 強化 Attack/Defense/Move 使用強化專用 ACK/retry 視窗，普通姿態時序不變。

## 驗證紀錄

直接執行測試二進制：

- `build/behavior_tree/test_posture_manager`: 31/31 通過
- `build/behavior_tree/test_outpost_engagement_lock`: 10/10 通過

`colcon test --packages-select behavior_tree` 在本機測試啟動器階段失敗，原因是環境缺少 `ament_cmake_test` Python 模組；不是測試案例 assertion 失敗。修正完成後仍需先修復/準備 ROS 測試環境，再執行完整 `colcon test`。
