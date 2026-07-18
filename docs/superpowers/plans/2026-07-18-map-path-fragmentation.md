# Map Path Fragmentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the full 50-point `0x0307 map_data_t` payload while ensuring every upper-to-lower-machine serial write is at most 64 bytes.

**Architecture:** `MapPathFrame` remains the 107-byte logical data model: `!`, `0x02`, and the 105-byte referee map payload. `gimbal_driver` serializes it into exactly two 64-byte `MapPathFragmentFrame` writes. Each fragment repeats `!` and `0x02`, carries a path sequence, zero-based fragment index, fixed fragment count, actual payload length, payload bytes, and CRC16. The lower machine reassembles only two CRC-valid fragments with the same sequence before emitting its referee `0x0307` frame.

**Tech Stack:** C++20, ROS2 Humble, gtest, Boost.Asio serial write.

## Global Constraints

- Keep `/ly/game/path`, `/ly/control/map_path`, `MapPath.msg`, `intention`, sender ID, and 50-point encoding unchanged.
- Physical downlink frames must be `<= 64B`; this design uses two fixed 64B frames.
- Preserve `DownlinkTypeID=0x02`; lower-machine firmware changes atomically to fragment reassembly.
- Do not add delays between the two writes.
- Keep raw serial observation on `/ly/download/typeid0x02`, now with one 64B raw frame per fragment.

### Task 1: Lock the Fragment Contract with Tests

**Files:**
- Modify: `src/gimbal_driver/test/test_mpc_gimbal_protocol.cpp`
- Modify: `src/gimbal_driver/module/BasicTypes.hpp`

**Interfaces:**
- Produces `LangYa::MakeMapPathFragments(const MapPathFrame&, uint8_t)` returning two CRC-protected fragments.

- [x] Write a failing gtest that fills all 49 `DeltaX_dm`, all 49 `DeltaY_dm`, start coordinates, intention, and sender ID; assert two fragments, each `sizeof <= 64`, same sequence, indices `0/1`, count `2`, lengths `56/49`, valid CRC, and byte-for-byte recovery of the original 105-byte map payload.
- [x] Run a package build to verify the new test does not compile before the helper exists.
- [x] Add the packed 64B fragment frame, CRC16 helper, fragment validator, and deterministic two-part serializer next to `MapPathFrame` in `BasicTypes.hpp`.
- [x] Re-run the targeted test; all gtests pass.

### Task 2: Send Physical Fragments and Keep Observation Accurate

**Files:**
- Modify: `src/gimbal_driver/main.cpp`

**Interfaces:**
- Consumes the tested `MakeMapPathFragments` helper.
- Produces two sequential 64B `WriteRaw` calls for every accepted map path.

- [x] Replace the single `MapPathFrame` serial write in `SendMapPath` with the two generated fragment writes, incrementing one `uint8_t` sequence per logical path.
- [x] Route raw logging and `/ly/download/typeid0x02` publication through the physical fragment type so captures match the bytes actually written.
- [x] Build `gimbal_driver` and run the fragment gtest; build success and all tests passing.

### Task 3: Document the Lower-Machine Contract

**Files:**
- Modify: `docs/sentry/embedded/downlink_control_frame.md`
- Modify: `docs/sentry/embedded/serial_data_mapping.md`
- Modify: `docs/sentry/embedded/referee_serial_integration.md`
- Modify: `docs/plans/2026-07-11_protocol_v2_change_brief.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

- [x] Replace the physical `0x02=107B` claim with `0x02 map path fragment=64B x2`, retaining a separate statement that the reassembled logical map payload stays 107B / 50 points.
- [x] Specify every fragment byte, CRC16 algorithm, reassembly acceptance rules, and no-delay transmission requirement.
- [x] Validate graph JSON and run `git diff --check`.

### Task 4: Full Verification and Commit

- [x] Run targeted `gimbal_driver` build and tests.
- [x] Run `./scripts/selfcheck.sh sentry --static-only` with ROS and `SENTRY_COMMON_ROOT` sourced.
- [x] Re-check both graph JSON files, staged diff, and the absence of staged user-owned lock files.
- [ ] Commit the runtime protocol, tests, docs, and graph together; push `Behavion`.
