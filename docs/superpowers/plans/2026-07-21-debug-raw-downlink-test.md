# Debug Raw Downlink Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox syntax.

**Goal:** Add an exclusive debug-only raw serial writer for validated full frames on `/ly/download/typeid0x00..05`.

**Architecture:** A pure validator checks existing packed frames. The debug launch extracts one driver parameter from `debug_mode.yaml`; the driver changes download IDs from observers to subscribers only for this mode.

**Tech Stack:** ROS2 Humble C++20, `GimbalRawFrame`, gtest, pytest, YAML.

## Global Constraints

- `raw_downlink_test_mode` defaults to `false` and is available only through `debug_node.launch.py`.
- Raw test mode makes `/ly/download/typeid0x00..05` input-only and never mirrors TX back to them.
- All normal downlink writers are suppressed; RX parsing and feedback remain active.
- Accept only matching `DIRECTION_TX`, topic/type ID, `data[0]=='!'`, exact frame size, 0x02 CRC16, and 0x04 CRC8.
- Do not change formal launches, messages, protocols, or dependencies.

### Task 1: Testable frame validation

**Files:** Create `src/gimbal_driver/include/RawDownlinkTest.hpp`; modify `src/gimbal_driver/test/test_mpc_gimbal_protocol.cpp`.

- [ ] Write gtest cases that build valid 0x00..0x05 frames, then assert reject for wrong expected ID, bad head, byte-1 mismatch, bad length, bad 0x02 CRC16, and bad 0x04 CRC8.
- [ ] Run `cmake --build build/gimbal_driver --target test_mpc_gimbal_protocol -j2 && ./build/gimbal_driver/test_mpc_gimbal_protocol`; it must fail because the validator does not exist.
- [ ] Add `gimbal_driver::IsValidRawDownlinkTestFrame(uint8_t expected_type_id, std::span<const uint8_t> bytes) noexcept`. It switches by type, compares exact `sizeof`, then copies 0x02/0x04 into packed structs for their existing CRC helpers.
- [ ] Rerun the focused test; all tests must pass. Commit `gimbal_driver: validate raw downlink test frames`.

### Task 2: Debug config and exclusive writer

**Files:** Modify `src/gimbal_driver/main.cpp`, `src/gimbal_driver/launch/gimbal_driver.launch.py`, `src/gimbal_driver/launch/debug_node.launch.py`, `src/gimbal_driver/config/debug_mode.yaml`, and `src/gimbal_driver/test/test_debug_control_bridge.py`.

- [ ] Add failing Python assertions for a default-false debug YAML key, debug-launch extraction/routing, and default-false formal launch argument. Run `PYTHONPATH=src/gimbal_driver python3 -m pytest src/gimbal_driver/test/test_debug_control_bridge.py -q`; it must fail.
- [ ] Add `raw_downlink_test_mode: false`; debug launch reads only it and forwards it to driver slash/dot `io_config` aliases. The bridge does not consume this value.
- [ ] In `main.cpp`, when mode is true create six `GimbalRawFrame` subscribers; validate direction/type/data then copy into the matched frame, `Device.WriteRaw`, and `LogDownlinkRaw`.
- [ ] In this mode suppress the control callback, trajectory, posture/sentry command, map/path/custom, coordinate, stale fallback, and retry writers. Do not create serial-mode download publishers, preventing self-loop.
- [ ] Run `PYTHONPATH=src/gimbal_driver python3 -m pytest src/gimbal_driver/test/test_debug_control_bridge.py -q`, build `gimbal_driver_node` and `test_mpc_gimbal_protocol`, and run the gtest. Commit `gimbal_driver: add exclusive raw downlink debug mode`.

### Task 3: Operational contract

**Files:** Modify `docs/modules/2026-05-05_gimbal_driver.md`, `docs/sentry/embedded/serial_data_mapping.md`, and `scripts/selfcheck/sentry.sh`.

- [ ] Document the mode, all six topic packet requirements, exclusive-writer rule, observation-loop rule, and a `ros2 topic pub --once` example. Require its YAML key, launch routing, validator, and suppression in static selfcheck.
- [ ] Run `cmake --build build/gimbal_driver --target gimbal_driver_node test_mpc_gimbal_protocol -j2`, run gtest, run the Python test, run `./scripts/selfcheck.sh sentry --static-only`, then `git diff --check`. Separate known missing external ROS/Aim interfaces from the gimbal contract result.
- [ ] Commit `docs: describe raw downlink debug test mode`.
