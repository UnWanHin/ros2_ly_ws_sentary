# Upper-Lower Protocol Documents Design

Updated: 2026-07-20

## Goal

Add two reader-facing documents for the current gimbal-driver and lower-machine
binary protocol. They describe the protocol contract only; they do not describe
serial device setup, launch commands, or firmware integration steps.

## Sources Of Truth

The current upper-machine contract is derived from:

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/gimbal_driver/main.cpp`
- `src/gimbal_driver/msg/*.msg`

Existing embedded documents remain detailed references. The externally supplied
`云台上位机通信协议总览.md` is comparison evidence, not an authority for the
current upper-machine implementation.

## Documents

### `upper_lower_protocol_overview.md`

This is the concise protocol entry point. It will include:

- the common framing rules and separate uplink/downlink ID spaces;
- an uplink TypeID 0-11 table with the current 12-byte payload meaning and
  semantic ROS output;
- a downlink 0x00-0x05 table with frame size, semantic ROS input, and purpose;
- unit and coordinate rules, including the TypeID 9 official-float-to-fixed-cm
  conversion;
- links to the existing detailed documents.

### `lower_machine_protocol_difference_report.md`

This report compares the supplied lower-machine protocol Markdown file with the
current upper-machine contract. It will:

- mark all downlink frame definitions as matching;
- distinguish matching uplink TypeIDs from incompatible or unverified ones;
- state the exact byte offsets and expected runtime impact for TypeID 0, 1, 2,
  3, 6, and 10;
- avoid claiming firmware behavior without current `pc_serial.c/.h` source or
  captured raw frames.

## Acceptance Criteria

- Neither document changes a ROS topic, message, config key, serial protocol,
  or runtime behavior.
- Every numeric byte-layout claim traces to the current upper-machine source or
  explicitly identifies the lower-machine Markdown as its evidence.
- TypeID 9 explicitly differentiates official referee `float32` metres from
  USB uplink `int16` centimetres.
- The detailed existing documents are linked rather than duplicated in full.
