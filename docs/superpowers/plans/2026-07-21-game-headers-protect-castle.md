# Game Headers And ProtectCastle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stamp lower-machine game and Base/outpost health state, then use fresh Base damage and Castle occupancy attribution for bounded ProtectCastle decisions.

**Architecture:** `gimbal_driver` stamps every migrated semantic message at serial receive/decode time. `behavior_tree` owns freshness and Base-damage state; it never changes TypeID wire data. ProtectCastle resolves team occupancy from referee event data, physical self-presence from local RFID/position, and teammate presence from fresh official positions before choosing Castle, perimeter defense, or bounded chase.

**Tech Stack:** ROS 2 Humble C++, `gimbal_driver` messages, behavior-tree CTest, YAML parameters, decision trace JSON.

## Global Constraints

- Keep `/ly/game/is_start` as `std_msgs/Bool`.
- Do not alter lower-machine TypeID layouts or serial protocol payloads.
- `header.stamp` means upper-machine serial receive/decode time, not MCU source time.
- Keep topic names; caller-owned `/ly/game/*`, `/ly/friend/*`, and `/ly/enemy/*` consumers rebuild against the upgraded types.
- Preserve all unrelated dirty documentation changes.

---

### Task 1: Upgrade stamped semantic state contracts

**Files:**
- Modify: `src/gimbal_driver/msg/GameData.msg`
- Create: `src/gimbal_driver/msg/StampedUInt16.msg`
- Create: `src/gimbal_driver/msg/StampedInt16.msg`
- Modify: `src/gimbal_driver/CMakeLists.txt`
- Modify: `src/gimbal_driver/main.cpp`
- Modify: `src/behavior_tree/include/Topic.hpp`
- Modify: `src/behavior_tree/src/SubscribeMessage.cpp`

**Interfaces:**
- Produces `gimbal_driver/msg/StampedUInt16 { std_msgs/Header header; uint16 data; }`.
- Produces `gimbal_driver/msg/StampedInt16 { std_msgs/Header header; int16 data; }`.
- Migrates `/ly/game/time_left`, `/ly/game/damage_difference`, `/ly/friend/base_hp`, `/ly/enemy/base_hp`, `/ly/friend/op_hp`, and `/ly/enemy/op_hp` to those types.

- [ ] Add `std_msgs/Header header` as the first field of `GameData.msg`; create both scalar wrappers and list them in `rosidl_generate_interfaces`.
- [ ] Change `PubGameData`, `PubHealthMyselfData`, `PubHealthEnemyData`, TypeID=1 fallback outpost publishing, and TypeID=10 precise outpost publishing to receive one `rclcpp::Time stamp` per decoded serial frame and assign it to every migrated message.
- [ ] Keep `/ly/game/is_start` as `std_msgs::msg::Bool`; assign its behavior exactly as before.
- [ ] Change behavior-tree aliases/subscriptions to the new types and read scalar payloads through `msg->data`.
- [ ] Add `lastSelfBaseHealthRxTime_`, `hasReceivedSelfBaseHealth_`, a retained previous value, and equivalent enemy timestamps only where needed for diagnostics; initialize the first valid sample without emitting a damage event.
- [ ] Build `gimbal_driver behavior_tree` and run targeted CTest. Commit as `gimbal_driver: stamp lower game health state`.

### Task 2: Make Base damage a fresh tactical source

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/SubscribeMessage.cpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/config/Tactical.yaml`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/include/TacticalProtectionPolicy.hpp`
- Test: `src/behavior_tree/test/test_tactical_protection_policy.cpp`

**Interfaces:**
- Adds `Tactical.ProtectCastle.Base` with default `true`.
- Adds internal `IsProtectCastleBaseDamageActive(now)` which requires a received, fresh nonzero Base HP sample and lasts eight seconds after the most recent strictly lower value.

- [ ] Write failing policy tests for master/Base gates, first sample suppression, a lower sample activating the eight-second window, equal/increased samples not renewing it, and stale data cancelling it.
- [ ] Add `Base: true` to `Tactical.yaml`, parse the parameter through `ApplyTacticalParameterOverrides`, and expose the effective switch in startup decision logging.
- [ ] Implement the Base-damage window with a fixed `8s` expiry renewed only by a strictly lower fresh `selfBaseHealth`; do not infer shield damage and do not activate when Base HP is zero.
- [ ] Feed Base damage into `EvaluateRegionalDefenseThreat` as a hard ProtectCastle source without changing Recovery priority.
- [ ] Run the focused test binary and commit as `behavior_tree: trigger castle defense on fresh base damage`.

### Task 3: Resolve Castle occupancy and bounded pursuit

**Files:**
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/include/TacticalProtectionPolicy.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/config/Tactical.yaml`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Test: `src/behavior_tree/test/test_tactical_protection_policy.cpp`

**Interfaces:**
- Adds `ProtectCastle` settings `OccupancyPositionFreshMs: 2500`, `CastlePositionMarginCm: 60`, and `ArrivalConfirmGraceMs: 3000`.
- Adds an internal occupancy result with team status, self-at-Castle confidence, teammate-at-Castle confidence, and action `ApproachCastle`, `HoldCastle`, or `PerimeterDefense`.

- [ ] Write failing pure-policy tests for referee statuses `0/2 -> ApproachCastle`, `1/3 + self absent -> PerimeterDefense`, and `1/3 + self present -> HoldCastle`.
- [ ] Implement self evidence in this order: fresh `rfid.friend_bastion`, fresh sentry official position inside the Castle polygon inflated by the configured margin, then `Castle` reached only as a grace-window aid. Do not treat reached alone as occupancy.
- [ ] Classify fresh non-sentry teammate positions inside the same inflated polygon as `teammate_likely_at_castle`; never allow stale teammates to override fresh referee status.
- [ ] Retain a three-second arrival-confirm window after self entry. Use it only to label `self_likely_occupant` versus `team_or_ambiguous_occupant`; do not claim referee certainty.
- [ ] Replace current `2/3` Castle hard-lock selection: every active ProtectCastle source uses the occupancy action. Status `0/2` approaches Castle; status `1/3` chooses the four perimeter points unless self is already at Castle; self-at-Castle disables chase.
- [ ] Permit chase only from perimeter, only with a fresh exact enemy official point inside MyBase, and return to the nearest perimeter point on target staleness, unreachable result, or end of the active ProtectCastle source. Do not enable generic cross-map Chase.
- [ ] Run focused policy/decision tests and commit as `behavior_tree: attribute castle occupancy before defense chase`.

### Task 4: Expose and verify runtime evidence

**Files:**
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`
- Modify: `src/behavior_tree/include/DecisionExplain.hpp`
- Modify: `src/simulator/simulator/model.py`
- Modify: `src/simulator/simulator/trace.py`
- Modify: `src/simulator/simulator/validation.py`
- Modify: `src/simulator/config/default.yaml`
- Modify: `docs/sentry/internal/simulator.md`
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/sentry/embedded/serial_data_mapping.md`
- Modify: `docs/sentry/internal/ros2_topic_structure.md`

**Interfaces:**
- Trace records `base_hp`, Base HP freshness/age, Base damage active/remaining time, referee fortress status freshness, self/team occupancy attribution, and selected Castle action.

- [ ] Add trace assertions proving a replay can distinguish Base damage defense, Castle hold, perimeter defense, and bounded chase without reconstructing state from UI fields.
- [ ] Update simulator rendering/model parsing only for these observed fields; preserve existing trace compatibility defaults for older files.
- [ ] Add startup and point-change logs including source (`rfid`, `enemy_pos`, `base_damage`), team status, attribution, and selected action.
- [ ] Document every migrated topic type and receipt-time semantics; update the simulator document `Updated:` line.
- [ ] Run simulator tests, behavior-tree CTest, `./scripts/selfcheck.sh sentry --static-only`, `git diff --check`, and dashboard API checks. Commit as `docs: trace castle defense attribution`.

### Task 5: End-to-end acceptance

**Files:**
- Modify only if verification reveals a defect in Tasks 1-4.

- [ ] Build `auto_aim_common gimbal_driver navi_tf_bridge behavior_tree simulator` with `--symlink-install`.
- [ ] Run `colcon test --packages-select gimbal_driver behavior_tree simulator` and `colcon test-result --verbose`.
- [ ] In a ROS-capable environment, verify `ros2 topic info` shows the upgraded message types and `ros2 topic echo --once` displays `header.stamp` for each migrated topic.
- [ ] Replay a trace with fresh Base HP decline plus fortress status `0`, `1`, `2`, and `3`; verify the actions match Task 3 and the eight-second expiry returns control to normal strategy.
- [ ] Review the final diff for unintentional topic/serial changes and commit any verification-only corrections separately.
