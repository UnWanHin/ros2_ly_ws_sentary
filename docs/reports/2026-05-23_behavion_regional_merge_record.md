# Behavion / Regional Merge Record

Date: 2026-05-23

Current status note (2026-07-08): patrol scan task overrides and pitch offsets have
since been centralized under `src/behavior_tree/config/Patrol.yaml`
`PatrolScan.TaskOverrides`. Historical mentions below of code constants or
`FaceMode.*PatrolScanMode*` keys describe the 2026-05-23 merge state, not the
current preferred configuration surface. See
`docs/sentry/regional/patrol_scan_modes.md`.

Base branch: `Behavion`
Base commit before merge work: `c98c85e` (`優先級有問題`)
Merged source: `origin/Regional`
Regional commit used: `05081e7` (`merge in car last day`)
Common ancestor: `3336aaa` (`day2.5`)

Note: the sandbox could not create a real Git merge state because `.git/ORIG_HEAD.lock`
could not be written. The merge was applied manually to the working tree.

## Merge Policy

- Keep `Behavion` as the structural base.
- Keep `Behavion` patrol/facemode configuration architecture instead of reverting to
  hardcoded scan constants.
- Import `Regional` car-side tuning values into YAML and C++ fallback defaults.
- Import `Regional` runtime checks for `sentry_msgs/msg/AimResult.bool follow`.
- Import `Regional` final map/aim point coordinate updates.
- Record every changed file below.

## Changed Files

### `docs/sentry/regional/decision_framework.md`

- Updated the Outpost behavior description.
- Replaced the old "wait until BuffOutpost reached" wording with
  `VisualScoutFaceDistanceCm` readiness.
- Documented that Outpost visual mode and FaceMode can start after entering the face
  distance, without strictly requiring `/ly/navi/reached=true`.
- Documented that post-window scout uses the face-distance condition before short
  FaceMode search.

### `docs/sentry/regional/vision_task_patrol_modes.md`

- Updated `Updated:` from `2026-05-11` to `2026-05-15`.
- Updated Outpost visual-scout flow to say the robot keeps armor vision before
  `VisualScoutFaceDistanceCm`.
- Updated Outpost visual-scout flow to say `AimMode::Outpost`,
  `/ly/vision/mode=3`, and enemy Outpost FaceMode start after entering
  `VisualScoutFaceDistanceCm`.
- Updated the post-window scout description to use `VisualScoutFaceDistanceCm`
  instead of requiring arrival at `BuffOutpost`.
- Updated navigation/facemode wording so FaceMode can point at enemy `OutpostPose`
  after face-distance readiness even if `/ly/navi/reached` is still false.
- Added patrol scan notes for the merged behavior:
  start-gate wait adds `+10 deg` pitch center offset for non-mode-3 scan;
  game loop does not keep that start-gate offset after entering normal operation.
- Added note that Outpost FaceMode fallback defaults to mode 2, with BT adding the
  existing `+15 deg` Outpost no-target pitch lift in scan behavior.

### `scripts/lib/ros_launch_common.sh`

- Tightened `source_optional_sentry_msgs()` so existing `sentry_msgs` is accepted only
  when `sentry_msgs/msg/AimResult` includes the exact field `bool follow`.
- Tightened sourced setup candidates with the same `AimResult.follow` check.
- Rewrote `sentry_msgs_aim_result_has_follow()` to store interface output first,
  then grep the cached text. This avoids pipeline failure ambiguity.

### `scripts/selfcheck/sentry.sh`

- Added `sentry_msgs_aim_result_has_follow()`.
- Tightened `source_optional_sentry_msgs()` so it only passes when `sentry_msgs` exists
  and `AimResult.follow` exists.
- Tightened sourced setup candidates with the same field check.
- Updated the warning to mention both missing `sentry_msgs` and missing
  `AimResult.follow`.
- Reworked `check_ros_interface_field()` to store `ros2 interface show` output before
  checking the expected field.

### `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`

- Changed `Task.Outpost` from `false` to `true`, matching `Regional`.
- Added target priority `7` to `AimTargetPriority`, matching `Regional`.
- Kept the existing `Behavion` `DecisionAutonomy.NaviGoal.BuffOutpostCompat` block.

### `src/behavior_tree/config/Base.yaml`

- Kept `Behavion`'s shared `RegionalAreaTask.PatrolSelection` structure.
- Changed `MyBase.GoalHoldSec` from `15` to `420`, matching `Regional`.
- Changed `MyBase.Patrol.GoalWeights.CastleLeft1` from `9.0` to `6.0`.
- Changed `MyBase.Patrol.GoalWeights.CastleLeft2` from `7.0` to `9.0`.
- Kept `CastleRight2` at `8.0`.
- Kept `CastleRight1` at `7.0`.
- Changed `HoleRoad` from `12.0` to `11.0`.
- Changed `OutpostGuard` from `16.0` to `12.0`.
- Changed `BuffOutpost` from `12.0` to `100.0`, matching the Regional car-side
  preference for BuffOutpost-heavy Base patrol.

### `src/behavior_tree/config/Patrol.yaml`

- Imported `Regional` patrol scan tuning into the `Behavion` YAML configuration.
- Mode 1:
  - Kept `YawStepDegPerTick=9.0`.
  - Kept `YawBoostStepDegPerTick=10.0`.
  - Changed `PitchCenterDeg` from `0.0` to `5.0`.
  - Changed `PitchHalfRangeDeg` from `13.0` to `15.0`.
  - Changed `PitchPeriodMs` from `500.0` to `2000.0`.
- Mode 2:
  - Changed `YawStepDegPerTick` from `1.0` to `0.3`.
  - Changed `YawBoostStepDegPerTick` from `1.1` to `1.5`.
  - Kept `YawHalfRangeDeg=30.0`.
  - Kept `CenterDriftPerCycleDeg=-70.0`.
  - Changed `PitchCenterDeg` from `0.0` to `5.0`.
  - Changed `PitchHalfRangeDeg` from `13.0` to `15.0`.
  - Changed `PitchPeriodMs` from `500.0` to `2000.0`.
- Mode 3:
  - Changed `YawStepDegPerTick` from `1.0` to `6.0`.
  - Changed `PitchOffsetDeg` from `10.0` to `0.0`.
  - Changed `PitchHalfRangeDeg` from `3.0` to `12.0`.
  - Changed `PitchPeriodMs` from `1300.0` to `2000.0`.

### `src/behavior_tree/config/Task.yaml`

- Kept `Task.Outpost=true`.
- Changed `OutpostConfirm.VisualScoutHoldMs` from `50000` to `40000`, matching
  `Regional`.
- Changed `OutpostConfirm.VisualScoutFaceDistanceCm` from `200` to `300`, matching
  `Regional`.
- Kept `PostWindowScoutEnable=true`.
- Kept `PostWindowScoutIntervalSec=60`.
- Changed `PostWindowScoutHoldMs` from `5000` to `3000`, matching `Regional`.
- Kept `ArmorWarningDistanceCm=5000` from `Behavion`.
- Kept `PostArmorFaceSearchMs=5000` from `Behavion`, because `Regional` did not
  explicitly change that value from the common ancestor.
- Kept `OpeningHoldSec=120` from `Behavion`.
- Kept `OpeningHoldUntilWindowEnd=true` from `Behavion`.
- Changed `FaceMode.OutpostFallbackPatrolScanMode` from `3` to `2`, matching
  `Regional`.

### `src/behavior_tree/module/Area.hpp`

- Changed `BuffOutpost` coordinates from `{1196, 1256}/{1604, 244}` to
  `{1220, 1350}/{1580, 150}`, matching `Regional`.
- Changed `OutpostPose` z height from `100.0` to `150.0`, matching `Regional`.
- Changed `BuffPose` z height from `100.0` to `150.0`, matching `Regional`.
- Kept `OutpostAimTarget` and `BuffAimTarget` at z `100.0`.

### `src/behavior_tree/module/BasicTypes.hpp`

- Updated `PatrolScanSetting` fallback defaults to match the merged `Patrol.yaml`.
- Mode 1 fallback defaults now use pitch center `5.0`, half range `15.0`, period
  `2000.0`.
- Mode 2 fallback defaults now use yaw step `0.3`, yaw boost `1.5`, pitch center
  `5.0`, pitch half range `15.0`, period `2000.0`.
- Mode 3 fallback defaults now use yaw step `6.0`, pitch offset `0.0`, pitch half
  range `12.0`, period `2000.0`.

### `src/behavior_tree/src/Configuration.cpp`

- Updated patrol scan validation fallback values to match `Patrol.yaml` and
  `PatrolScanSetting`.
- Mode 1 validation fallback now uses pitch center `5.0`, half range `15.0`, period
  `2000.0`.
- Mode 2 validation fallback now uses yaw step `0.3`, yaw boost `1.5`, pitch center
  `5.0`, half range `15.0`, period `2000.0`.
- Mode 3 validation fallback now uses yaw step `6.0`, pitch offset `0.0`, pitch half
  range `12.0`, period `2000.0`.

### `src/behavior_tree/src/WaitBeforeGame.cpp`

- Kept `Behavion`'s config-driven patrol scan implementation.
- Added `kGatePatrolOpeningPitchOffsetDeg=10.0f`, matching `Regional` start-gate
  behavior.
- Applied the `+10 deg` start-gate pitch offset only when `patrol_mode != 3`.
- Mode 3 start-gate scan still uses its configured mode-3 pitch behavior without the
  opening offset.

## Reviewed But Not Directly Modified

### `src/behavior_tree/src/GameLoop.cpp`

- `Regional` changed hardcoded patrol constants in `GameLoop.cpp`.
- `Behavion` already moved this behavior behind `config.PatrolScanSettings`.
- The merged result keeps `Behavion`'s config-driven `GameLoop.cpp` and moves the
  `Regional` patrol values into `Patrol.yaml`, `PatrolScanSetting`, and validation
  fallback defaults instead.
- `Behavion` already contains the face-distance Outpost flow that `Regional` added:
  it can use `outpost_visual_scout_face_ready` instead of strictly requiring
  `outpost_visual_scout_point_reached`.
- Source discrepancy noted: `origin/Regional`'s `Task.yaml` comment says Outpost
  FaceMode fallback mode 2 raises pitch center by `+15 deg`, but
  `origin/Regional`'s `GameLoop.cpp` hardcoded `kPatrolOutpostPitchOffsetDeg=0.0`
  and removed the final `nextAngles.Pitch += 15.0f` adjustment. The merged result
  keeps the Behavion `+15 deg` adjustment because it matches the selected
  `Task.yaml` semantics and the existing Behavion behavior. This should be checked
  on robot if Outpost fallback pitch looks too high.

## Non-Parameter Code Difference Audit

This section records branch differences that were not only config values.

### `Regional` non-parameter code changes from `3336aaa`

- `scripts/lib/ros_launch_common.sh`
  - Added runtime validation that `sentry_msgs/msg/AimResult` includes `bool follow`.
  - Used the same validation when sourcing fallback sentry common setup files.
- `scripts/selfcheck/sentry.sh`
  - Added the same `AimResult.follow` validation to selfcheck.
  - Changed interface-field checking to read interface text first, then test the
    expected field.
- `src/behavior_tree/module/Area.hpp`
  - Changed compiled-in `BuffOutpost` navigation coordinates.
  - Changed compiled-in `OutpostPose` and `BuffPose` z height from `100.0` to `150.0`.
- `src/behavior_tree/src/GameLoop.cpp`
  - Changed patrol scan behavior in code on `Regional`: pitch center/range/period,
    mode-2 yaw step/boost, and mode-3 fallback yaw/pitch behavior.
  - Changed Outpost visual scout behavior from strict `BuffOutpost reached` gating to
    `outpost_visual_scout_face_ready` / `VisualScoutFaceDistanceCm` gating.
  - Improved Outpost logs to include `face_distance_ready`.
- `src/behavior_tree/src/WaitBeforeGame.cpp`
  - Changed start-gate patrol pitch behavior by adding `+10 deg` opening pitch offset.

### `Behavion` non-parameter code retained from `3336aaa`

- `docs/architecture/2026-05-04_control_angles_data_flow.md`
  - Retained Behavion-side control-angle data-flow documentation.
- `docs/sentry/internal/ros2_topic_tree.md`
  - Retained Behavion-side ROS2 topic tree documentation.
- `scripts/aim/Outpost_Simlator.sh`
  - Added Outpost simulation helper script.
- `scripts/aim/outpost_regional.sh`
  - Modified Outpost regional helper behavior.
- `scripts/debug/patrolmode3_test.sh`
  - Added patrol mode 3 debug helper.
- `scripts/feature_test/scan_gimbal_test.py`
  - Modified the standalone scan-gimbal feature test used for patrol/facemode
    verification.
- `scripts/gimbal/patrolmode1.sh`
  - Added patrol mode 1 gimbal helper.
- `scripts/gimbal/patrolmode2.sh`
  - Added patrol mode 2 gimbal helper.
- `scripts/gimbal/patrolmode3.sh`
  - Added patrol mode 3 gimbal helper.
- `scripts/gimbal/patrolmode_common.sh`
  - Added shared patrol mode shell helper.
- `scripts/gimbal/patrolmode_pub.py`
  - Added patrol mode publisher helper.
- `scripts/launch/start_sentry_all.sh`
  - Modified sentry-all launcher wiring used by the merged behavior-tree stack.
- `src/behavior_tree/launch/behavior_tree.launch.py`
  - Added the `Patrol.yaml` launch parameter file to the standalone behavior-tree
    launch path.
- `src/behavior_tree/launch/outpost_regional_test.launch.py`
  - Updated the Outpost regional test launch path for the patrol/facemode config.
- `src/behavior_tree/launch/sentry_all.launch.py`
  - Added the `Patrol.yaml` launch parameter file to the full sentry stack.
- `src/behavior_tree/config/AreaManager.yaml`
  - Retained Behavion-side AreaManager settings, including BuffOutpost compatibility
    and shared patrol selection support.
- `src/behavior_tree/config/OutpostRegionalTest.yaml`
  - Retained Behavion-side Outpost regional test configuration.
- `src/behavior_tree/include/Application.hpp`
  - Added `ApplyPatrolScanParameterOverrides()`.
- `src/behavior_tree/include/AreaManager.hpp`
  - Added BuffOutpost transition kinds via HoleRoad.
  - Added `HoldCurrentBaseGoal` input for Base patrol.
  - Added APIs for BuffOutpost/Navi transition compatibility.
  - Added patrol-goal last-arrival tracking.
- `src/behavior_tree/module/BasicTypes.hpp`
  - Added detailed `PatrolScanSetting` fields.
  - Added Outpost opening hold fields.
  - Added BuffOutpost compatibility fields.
  - Added shared patrol goal selection settings.
- `src/behavior_tree/src/AreaManager.cpp`
  - Added weighted patrol selection freshness/unvisited/recent-visit scoring.
  - Added BuffOutpost via HoleRoad compatibility planning.
  - Added hold-current-base-goal behavior while armor target handling is active.
- `src/behavior_tree/src/Configuration.cpp`
  - Added parsing and ROS parameter overrides for detailed patrol scan settings.
  - Added parsing/validation for Outpost opening hold fields.
  - Added parsing/validation for BuffOutpost compatibility settings.
  - Added parsing for shared patrol goal selection.
- `src/behavior_tree/src/GameLoop.cpp`
  - Added config-driven patrol scan behavior.
  - Added Outpost opening hard hold behavior.
  - Added chase target area-scope blocking.
  - Added Base patrol hold while an armor target is active.
  - Added Recovery routing through navigation area transition compatibility.
- `src/behavior_tree/src/StrategyManager.cpp`
  - Adjusted strategy-manager behavior for the merged priority/regional task flow.
- `src/behavior_tree/src/WaitBeforeGame.cpp`
  - Converted start-gate gimbal patrol from hardcoded constants to
    `config.PatrolScanSettings`.
- `src/gimbal_driver/CMakeLists.txt`
  - Added `FaceModeStatus.msg` to generated gimbal-driver interfaces.
- `src/gimbal_driver/msg/FaceModeStatus.msg`
  - Added a message for exposing FaceMode status.
- `src/navi_tf_bridge/launch/map_aim_point.launch.py`
  - Updated map aim point launch configuration used by FaceMode.
- `src/navi_tf_bridge/src/pointer_solver_node.cpp`
  - Modified FaceMode/map aim point behavior used by the merged FaceMode chain.

## Complete Branch File Inventory

The lists below are generated from the common ancestor `3336aaa`. They record every
file that differed on either side and how it was handled.

### Behavion-only files retained

- `docs/architecture/2026-05-04_control_angles_data_flow.md`
- `docs/sentry/internal/ros2_topic_tree.md`
- `scripts/aim/Outpost_Simlator.sh`
- `scripts/aim/outpost_regional.sh`
- `scripts/debug/patrolmode3_test.sh`
- `scripts/feature_test/scan_gimbal_test.py`
- `scripts/gimbal/patrolmode1.sh`
- `scripts/gimbal/patrolmode2.sh`
- `scripts/gimbal/patrolmode3.sh`
- `scripts/gimbal/patrolmode_common.sh`
- `scripts/gimbal/patrolmode_pub.py`
- `scripts/launch/start_sentry_all.sh`
- `src/behavior_tree/config/AreaManager.yaml`
- `src/behavior_tree/config/OutpostRegionalTest.yaml`
- `src/behavior_tree/config/Patrol.yaml`
- `src/behavior_tree/include/Application.hpp`
- `src/behavior_tree/include/AreaManager.hpp`
- `src/behavior_tree/launch/behavior_tree.launch.py`
- `src/behavior_tree/launch/outpost_regional_test.launch.py`
- `src/behavior_tree/launch/sentry_all.launch.py`
- `src/behavior_tree/module/BasicTypes.hpp`
- `src/behavior_tree/src/AreaManager.cpp`
- `src/behavior_tree/src/Configuration.cpp`
- `src/behavior_tree/src/StrategyManager.cpp`
- `src/gimbal_driver/CMakeLists.txt`
- `src/gimbal_driver/msg/FaceModeStatus.msg`
- `src/navi_tf_bridge/launch/map_aim_point.launch.py`
- `src/navi_tf_bridge/src/pointer_solver_node.cpp`

### Regional-only files imported

- `docs/sentry/regional/decision_framework.md`
- `docs/sentry/regional/vision_task_patrol_modes.md`
- `scripts/lib/ros_launch_common.sh`
- `scripts/selfcheck/sentry.sh`
- `src/behavior_tree/module/Area.hpp`

### Files changed on both sides and manually reconciled

- `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- `src/behavior_tree/config/Base.yaml`
- `src/behavior_tree/config/Task.yaml`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/src/WaitBeforeGame.cpp`

## Remaining Differences Versus `origin/Regional`

After the merge worktree is compared directly to `origin/Regional`, these files are
still different. This is expected because the merge keeps Behavion as the structural
base rather than making the tree identical to Regional.

### Added on merged Behavion versus Regional

- `scripts/aim/Outpost_Simlator.sh`
- `scripts/debug/patrolmode3_test.sh`
- `scripts/gimbal/patrolmode1.sh`
- `scripts/gimbal/patrolmode2.sh`
- `scripts/gimbal/patrolmode3.sh`
- `scripts/gimbal/patrolmode_common.sh`
- `scripts/gimbal/patrolmode_pub.py`
- `src/behavior_tree/config/Patrol.yaml`
- `src/gimbal_driver/msg/FaceModeStatus.msg`

### Modified on merged Behavion versus Regional

- `docs/architecture/2026-05-04_control_angles_data_flow.md`
- `docs/sentry/internal/ros2_topic_tree.md`
- `docs/sentry/regional/vision_task_patrol_modes.md`
- `scripts/aim/outpost_regional.sh`
- `scripts/feature_test/scan_gimbal_test.py`
- `scripts/launch/start_sentry_all.sh`
- `scripts/selfcheck/sentry.sh`
- `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- `src/behavior_tree/config/AreaManager.yaml`
- `src/behavior_tree/config/Base.yaml`
- `src/behavior_tree/config/OutpostRegionalTest.yaml`
- `src/behavior_tree/config/Task.yaml`
- `src/behavior_tree/include/Application.hpp`
- `src/behavior_tree/include/AreaManager.hpp`
- `src/behavior_tree/launch/behavior_tree.launch.py`
- `src/behavior_tree/launch/outpost_regional_test.launch.py`
- `src/behavior_tree/launch/sentry_all.launch.py`
- `src/behavior_tree/module/Area.hpp`
- `src/behavior_tree/module/BasicTypes.hpp`
- `src/behavior_tree/src/AreaManager.cpp`
- `src/behavior_tree/src/Configuration.cpp`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/src/StrategyManager.cpp`
- `src/behavior_tree/src/WaitBeforeGame.cpp`
- `src/gimbal_driver/CMakeLists.txt`
- `src/navi_tf_bridge/launch/map_aim_point.launch.py`
- `src/navi_tf_bridge/src/pointer_solver_node.cpp`

### How These Were Merged

- Directly imported the `Regional` script checks and compiled coordinate updates.
- Did not directly import `Regional`'s hardcoded `GameLoop.cpp` patrol constants.
  Those values were imported into `Patrol.yaml`, `PatrolScanSetting`, and
  `Configuration.cpp` fallback validation instead.
- Kept `Behavion`'s config-driven patrol/facemode code and area-transition code.
- Added the `Regional` start-gate `+10 deg` pitch behavior on top of
  `Behavion`'s config-driven `WaitBeforeGame.cpp`.
- Preserved Behavion's Outpost no-target `+15 deg` pitch lift in `GameLoop.cpp`
  despite the conflicting Regional C++ implementation, and recorded that discrepancy
  above.

## Verification Run

- `python3 -m json.tool src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
  passed.
- YAML parse check passed for:
  `src/behavior_tree/config/Base.yaml`,
  `src/behavior_tree/config/Task.yaml`,
  `src/behavior_tree/config/Patrol.yaml`.
- `git diff --check` passed.
- `colcon build --packages-select gimbal_driver behavior_tree navi_tf_bridge`
  passed.
  - `gimbal_driver` emitted existing CMake dev warnings.
  - `behavior_tree` emitted the existing BehaviorTree.CPP pinned-config warning.
- `./scripts/selfcheck.sh sentry --skip-hz` did not pass because no ROS2 nodes were
  active; static checks inside it passed, then runtime graph reported
  `No ROS2 nodes found`.
- `./scripts/selfcheck.sh sentry --static-only` passed with:
  `PASS: 105`, `WARN: 0`, `FAIL: 0`.
