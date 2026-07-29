# Patrol Scan Modes

Updated: 2026-07-29

本文只說 `behavior_tree` 內部雲台巡邏掃描。它不是 `/ly/vision/mode`，也不是區域導航巡邏；它是在 BT 沒有可用目標角度時，自己計算 `/ly/control/angles` 的 fallback 掃描。

## Canonical Config

巡邏掃描的當前配置入口是：

```text
src/behavior_tree/config/Patrol.yaml
```

主入口會把它作為 `patrol_config_file` 注入：

- `src/behavior_tree/launch/sentry_all.launch.py`
- `src/behavior_tree/launch/behavior_tree.launch.py`
- `src/behavior_tree/launch/outpost_regional_test.launch.py`

運行時讀取到：

```text
config.PatrolScanSettings
```

`bt_config_file` JSON 先被 `Configuration.cpp` 讀入，隨後 launch 注入的 `Patrol.yaml` 會通過 ROS params 覆蓋同名 `PatrolScan` 欄位；因此靜態 `Scripts/ConfigJson/*.json` 不再承載 `PatrolScan.Mode`。如果 debug profile 需要局部差異，只覆蓋具體的 `PatrolScan.TaskOverrides` key。

舊的 `FaceMode.FallbackToPatrolScanMode2`、`FaceMode.FallbackPatrolScanMode`、`FaceMode.OutpostFallbackPatrolScanMode` 仍可讀取作兼容，但初始化後會同步成 `PatrolScan.TaskOverrides` 的 mirror；新配置不要再寫到 `Task.yaml` 的 `FaceMode` 下。

## Modes

`PatrolScan.Mode` 是沒有任務覆蓋時的默認掃描模式。

| Mode | 當前用途 | Yaw | Pitch |
|---|---|---|---|
| `1` | 單向掃描 | 每 tick 按 `Mode1.YawStepDegPerTick` 增加；普通裝甲受擊短窗口可用 `YawBoostStepDegPerTick` | `PitchCenterDeg + PitchHalfRangeDeg * sin(t / PitchPeriodMs)` |
| `2` | 默認左右擺頭掃描，也是 FaceMode fallback 主用模式 | 以當前 yaw 初始化中心，按 `YawHalfRangeDeg` 正弦擺動；每周期按 `CenterDriftPerCycleDeg` 漂移中心；受擊短窗口可用 boost step | `Mode2` 的 center/half-range/period 正弦 |
| `3` | 高 pitch 掃描 profile | 每 tick 按 `Mode3.YawStepDegPerTick` 單向掃描 | `PitchOffsetDeg + PitchHalfRangeDeg * sin(t / PitchPeriodMs)` |

當前 `Patrol.yaml` 裡的 mode2 已切回 `51d6508 clean` / `a4787dc patrol test` 那套較早參數：

```yaml
Mode2:
  YawStepDegPerTick: 1.0
  YawBoostStepDegPerTick: 1.1
  YawHalfRangeDeg: 30.0
  CenterDriftPerCycleDeg: -70.0
  PitchCenterDeg: 0.0
  PitchHalfRangeDeg: 13.0
  PitchPeriodMs: 500.0
```

`80a1e5f merge` 曾把 mode2 改成 `0.3 / 1.5 / 30.0 / -70.0 / 5.0 / 15.0 / 2000.0`；當前已回到 500ms 週期這套。

## Runtime Selection

`GameLoop.cpp::PublishTogether()` 的選擇順序是：

1. 有 FaceMode 角度：用 `/ly/face_mode/angles`，停止 patrol scan。
2. 有有效視覺/外部 aim 目標角度：用目標角度；若此前是 mode2，暫停並保留其中心、phase 與已累積的 center drift。
3. 無目標、不是被 `AimDebug.StopScan` 禁止、且距離上次看到目標超過 2s：進 patrol scan。
4. 進 scan 後先取 `PatrolScan.Mode`，再按任務狀態覆蓋：
   - `OutpostDamageAbortMode`：前哨任務因受擊退出後的短時搜索。
   - `OutpostFaceModeFallbackMode`：前哨已請求 FaceMode 但沒有 `/ly/face_mode/angles`。
   - `FaceModeFallbackMode`：其他 FaceMode fallback。

如果 `FaceModeFallbackEnable=false`，FaceMode 已被請求但沒有角度時不會進 patrol fallback，而是保持當前雲台角度。

mode2 從 FaceMode、視覺 aim 或目標消失後的 2 秒 hold 恢復時，不會把當前雲台角當成新中心，也不會把 phase 歸零。它會先凍結暫停前、已套用 `CenterDriftPerCycleDeg` 的中心與相對擺動位置，並由 `PassiveMotion` 從當前回授角平滑移到該保存點；yaw 誤差小於 1 度後才繼續推進 phase 與 center drift。正常連續 mode2 不通過 `PassiveMotion`，避免掃描軌跡比限速器快時累積落後。切到 mode1、mode3 或 Buff 則仍會完整重置巡邏狀態。

當前 `Patrol.yaml` 的任務覆蓋為：

```yaml
TaskOverrides:
  FaceModeFallbackEnable: true
  FaceModeFallbackMode: 2
  OutpostFaceModeFallbackMode: 2
  OutpostDamageAbortMode: 2
  StartGatePitchOffsetDeg: 10.0
  StartGatePitchOffsetApplyToMode3: false
  OutpostPitchOffsetDeg: 15.0
  OutpostPitchOffsetApplyToMode3: true
```

所以正式默認與 FaceMode 失角、前哨 FaceMode 失角、前哨受擊退出搜索都走 mode2；不是再由 `FaceMode` 分散指定。

## Start Gate FaceMode

`Task.yaml` 的 `StartGate.GimbalStrategy` 控制 gated 啟動、收到 `/ly/game/is_start=true` 前的雲台行為：

```yaml
StartGate:
  GimbalStrategy: patrol  # patrol | face_mode_outpost

FaceMode:
  Enable: false
```

- `patrol`（正式目前設定）：沿用 `PatrolScan.Mode` 與 `StartGatePitchOffsetDeg`，不請求開局 FaceMode。
- `face_mode_outpost`：持續對敵方前哨發布 `/ly/face_mode/target_raw`，要求
  `/ly/gimbal/facemode` 在 `FaceModeStatusFreshMs`（默認 500 ms）內確認 solver 非 manual、正在輸出，並已處理
  本次 StartGate 請求後的新 target generation。角度 callback 之間會使用最後一筆有效
  `/ly/face_mode/angles`，最長保持 `FaceMode.LostTargetHoldMs`（默認 300 ms），避免 BT tick 比 solver callback
  快時在 FaceMode 與 Patrol 間閃爍。`function=false`、status 過期、manual target、target generation 未推進，或持有
  角度超時時，強制改用 `OutpostFaceModeFallbackMode` 掃描；任一後續有效 solver 回讀都會自動重新接管 FaceMode。
  這條開局安全回退不受一般 `FaceModeFallbackEnable` 或 `AllowGimbalPatrolBeforeStart` 關閉影響。

`FaceMode.Enable=false`（正式目前設定）會停用開賽後所有固定點 FaceMode；Outpost/Buff 不再發布
`/ly/face_mode/target_raw`，`/ly/face_mode/angles` 也不會接管雲台。外部 `/ly/aim/result` 的視覺瞄準不受影響。
要重新啟用開局前哨 FaceMode，需同時將 `FaceMode.Enable` 設為 `true`，並把
`StartGate.GimbalStrategy` 設為 `face_mode_outpost`。

這個策略只改雲台角度來源；開局底盤速度、fire、rotate、follow 仍持續壓為 0。

`behavior_tree` 只會在 FaceMode 結論變化時輸出一次診斷：`StartGate FaceMode` 會列出 target
是否成功發布、solver status 新鮮度、`function/manual_target`、target generation 與角度可用性（含持有窗口），並以
`outcome` 指明採用或回退原因；正常運行期則以 `FaceMode runtime` 記錄來源、視覺/導航仲裁、失角
fallback 與最終接管結果。這些日誌只做可觀測性，不改變仲裁與巡邏策略。

前哨專項 debug profile 仍可局部覆蓋，例如 `src/behavior_tree/config/OutpostRegionalTest.yaml` 和 `src/behavior_tree/Scripts/ConfigJson/regional/debug/outpost_regional_test.json` 會把 `PatrolScan.TaskOverrides.OutpostFaceModeFallbackMode` 設成 `3`，用來測高位 fallback；這是 profile override，不是主鏈默認。

## Pitch Offsets

兩個任務級 pitch offset 也收在 `Patrol.yaml`：

| Key | 使用位置 | 語義 |
|---|---|---|
| `StartGatePitchOffsetDeg` | `WaitBeforeGame.cpp` | gated 啟動等待 `/ly/game/is_start=true` 期間，允許雲台掃描時額外抬 pitch；默認 `+10 deg`，默認不套到 mode3 |
| `OutpostPitchOffsetDeg` | `GameLoop.cpp` | `AimMode::Outpost` 無目標 scan 時，在所選 mode 的 pitch 曲線上額外抬 pitch；默認 `+15 deg`，默認可套到 mode3 |

這些 offset 只在無目標 scan 分支生效。一旦 FaceMode 或視覺/外部 aim 給出有效角度，BT 直接使用該角度，不再疊加 patrol pitch offset。

## Passive Motion

`PatrolScan.PassiveMotion` 對固定點 FaceMode，以及被 FaceMode/視覺 aim 中斷後的 mode2 恢復段的最終 `/ly/control/angles` 做速率限制：

```yaml
PassiveMotion:
  YawRateDegPerSec: 120.0
  PitchRateDegPerSec: 60.0
  MaxIntervalMs: 25
```

`MaxIntervalMs` 限制單次 BT 阻塞後可補發的角度量。正常 mode1/mode2/mode3 Patrol、視覺/外部 Aim、fire tracking 與 `naviLowerHead` 直控不通過此限制，保持原本掃描或追蹤響應。

## Link Chain

```text
Patrol.yaml
  -> ROS params patrol_config_file
  -> Configuration.cpp parses PatrolScan / PatrolScan.TaskOverrides
  -> config.PatrolScanSettings
  -> WaitBeforeGame.cpp start-gate scan
  -> GameLoop.cpp no-target / FaceMode fallback / Outpost fallback scan
  -> /ly/control/angles
  -> gimbal_driver
  -> lower machine
```

`PatrolScan` 只影響 BT 自己算出的 `/ly/control/angles` fallback。它不改 `/ly/vision/mode`，不改導航點位，不改 `/ly/control/firecode` 的開火語義。

手動下發測試腳本 `scripts/gimbal/patrolmode*.sh` 也讀同一份 `Patrol.yaml`；使用 `--outpost` 時，pitch bias 取自 `PatrolScan.TaskOverrides.OutpostPitchOffsetDeg`。

## Source Map

- `src/behavior_tree/config/Patrol.yaml`：canonical `PatrolScan` 配置。
- `src/behavior_tree/module/BasicTypes.hpp`：`PatrolScanSetting` 和 legacy `FaceModeSetting` mirror 字段。
- `src/behavior_tree/src/Configuration.cpp`：JSON/YAML/ROS param 解析、legacy FaceMode fallback 兼容、sanitize、mirror。
- `src/behavior_tree/src/GameLoop.cpp`：正式 runtime 無目標 scan、FaceMode fallback、Outpost fallback、Outpost pitch offset。
- `src/behavior_tree/src/WaitBeforeGame.cpp`：開局 gate 前的 gimbal patrol 和 start-gate pitch offset。
- `src/behavior_tree/config/OutpostRegionalTest.yaml`：前哨 debug profile 的 mode3 override 範例。
