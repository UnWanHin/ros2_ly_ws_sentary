# Simulator Match Clock Design

Updated: 2026-07-23

## Goal

Expose the existing offline simulator match clock as a clear Flight Deck control surface. Operators must be able to start, pause, reset, advance, rewind, and set the remaining match time without changing simulator state ownership or decision logic.

## Scope

- Display the remaining time and running state in the existing command bar.
- Keep high-frequency actions visible: start/pause, rewind/forward by 10 and 30 seconds, and reset.
- Add an Operations shelf time control with a range input, `mm:ss` numeric entry, and match-phase shortcuts.
- Reuse only `start`, `pause`, `reset`, `rewind`, `forward`, and `set_time_left` through the existing control bus.
- Preserve the current Flight Deck visual language: matte surfaces, `#4DB7FF` operation accent, one 40px control height, keyboard-accessible native controls, and responsive placement that does not cover the battlefield.

## Semantics

The clock is a countdown over the configured `match_duration_sec` (420 seconds by default).

- **Rewind** adds seconds to remaining time, capped at match duration.
- **Forward** subtracts seconds from remaining time, floored at zero.
- **Set time** assigns remaining time directly within the same bounds.
- **Start/Pause/Reset** retain their current mock-input and runtime behavior.
- Rewind, forward, and set-time do **not** rewind robot positions, structure HP, referee flags, scene commands, or decision-trace history. BT receives the revised time through the existing mock referee publisher and continues from the current state.

## Data Flow

```text
Browser clock control
  -> POST /api/control
  -> control JSONL bus
  -> SimulationRuntime local display state
  -> simulator.mock_inputs match state
  -> existing /ly/game timer publisher
  -> behavior_tree
```

There is no second clock state, replay database, or API contract change.

## Verification

1. Runtime test: start advances time; pause stops it; rewind/forward/set-time clamp correctly and retain scene state.
2. Tactical web test: controls send the expected existing command payloads.
3. Browser test: displayed time changes after control input; all buttons and time input remain usable with a persistent Inspector.
4. Run simulator tests, static checks, and `git diff --check`.
