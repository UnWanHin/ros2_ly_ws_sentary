# Simulator Match Clock Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the browser Tactical Board expose the existing match countdown and time-only rewind controls in the Flight Deck workspace.

**Architecture:** `SimulationRuntime` and `simulator.mock_inputs` remain the single authoritative clock pair. The browser emits only existing control-bus commands and renders the shared `match` state from `/api/tactical-state`; it stores no time or replay state locally. Rewind/forward/set-time change remaining time only, never scene or trace state.

**Tech Stack:** Python 3, ROS2 mock publisher, JSONL control bus, dependency-free HTML/CSS/JavaScript, pytest and Playwright.

## Global Constraints

- Do not change behavior-tree, ROS topics, simulator APIs, or scene/state ownership.
- Keep `start`, `pause`, `reset`, `rewind`, `forward`, and `set_time_left` as the only clock commands.
- Rewind means increasing remaining time, capped at `match_duration_sec`; it never rewinds scene or trace facts.
- Keep the Flight Deck palette and the existing 40px command-control height.
- Browser mutations remain unavailable in `manual_ros` and read-only replay modes.

---

### Task 1: Lock Runtime Time Semantics

**Files:**
- Modify: `src/simulator/test/test_runtime.py`
- Modify: `src/simulator/simulator/runtime.py`

**Interfaces:**
- Consumes: existing `SimulationRuntime.apply_local_match_command(command, payload)`.
- Produces: `web_status_metadata()["replay"]` with bounded `match_time_left` and unchanged scene data after a time-only command.

- [ ] **Step 1: Write the failing runtime regression test**

```python
def test_match_clock_rewind_and_forward_only_change_remaining_time(tmp_path: Path) -> None:
    runtime, control_file = make_runtime(tmp_path)
    original_scene = runtime.sim_input_state.snapshot()
    append_command(control_file, "start")
    append_command(control_file, "forward", {"seconds": 30})
    append_command(control_file, "rewind", {"seconds": 45})
    append_command(control_file, "set_time_left", {"seconds": 30})
    runtime.poll_control_commands()

    assert runtime.web_status_metadata()["replay"]["match_time_left"] == 30
    assert runtime.sim_input_state.snapshot() == original_scene
```

- [ ] **Step 2: Run the test before implementation**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_runtime.py::test_match_clock_rewind_and_forward_only_change_remaining_time`

Expected: fail until the test fixture/helper and clock state contract are aligned.

- [ ] **Step 3: Implement only required runtime contract changes**

Keep the existing command switch. Ensure `rewind`, `forward`, and `set_time_left` normalize finite seconds, clamp to `[0, match_duration_sec]`, stop only at zero, and do not call scene mutation paths.

```python
elif cmd in {"rewind", "forward", "set_time_left"}:
    seconds = self._float(payload.get("seconds"), 0.0)
    if cmd == "rewind":
        self.match_time_left_sec += max(0.0, seconds)
    elif cmd == "forward":
        self.match_time_left_sec -= max(0.0, seconds)
    else:
        self.match_time_left_sec = seconds
    self.match_time_left_sec = max(0.0, min(float(self.match_duration_sec), self.match_time_left_sec))
```

- [ ] **Step 4: Run the focused runtime test**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_runtime.py`

Expected: all runtime tests pass.

### Task 2: Add Flight Deck Clock Controls

**Files:**
- Modify: `src/simulator/simulator/tactical_web.py`
- Modify: `src/simulator/test/test_tactical_web.py`

**Interfaces:**
- Consumes: `state.match.duration_sec`, `state.match.time_left`, `state.match.running`, `state.scene.can_edit`.
- Produces: existing `/api/control` payloads `start`, `pause`, `reset`, `rewind`, `forward`, and `set_time_left`.

- [ ] **Step 1: Write failing web-contract tests**

```python
def test_tactical_html_exposes_match_clock_commands() -> None:
    body = build_tactical_html(0).decode("utf-8")
    for identifier in ("matchClock", "matchTimeline", "matchTimeInput", "rewind10", "forward10"):
        assert f'id="{identifier}"' in body

def test_tactical_match_controls_emit_time_commands_when_playwright_available(tmp_path: Path) -> None:
    # Click rewind and forward, then edit the mm:ss input.
    # Assert the JSONL command order and seconds payloads.
    ...
```

- [ ] **Step 2: Run tests to prove the controls are absent**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_tactical_web.py -k 'match_clock or match_controls'`

Expected: fail because the clock/timeline elements do not yet exist.

- [ ] **Step 3: Render the controls from shared state**

Add a small pure browser formatter/parser pair:

```javascript
function clockText(seconds) {
  const whole = Math.max(0, Math.round(Number(seconds) || 0));
  return String(Math.floor(whole / 60)).padStart(2, '0') + ':' + String(whole % 60).padStart(2, '0');
}
function clockSeconds(value, limit) {
  const match = String(value).trim().match(/^(\d{1,2}):(\d{2})$/);
  if (!match || Number(match[2]) >= 60) return null;
  return clamp(Number(match[1]) * 60 + Number(match[2]), 0, limit);
}
```

Render the command-bar clock, start/pause toggle, reset, 10/30 second grouped rewind/forward icon buttons, and a shelf range input plus `mm:ss` input. On input/change, call only `command({command: 'set_time_left', seconds})`; on button click call the matching existing command. Disable all clock mutators when `scene.can_edit` is false.

- [ ] **Step 4: Apply Flight Deck layout rules**

Place the compact clock in the top command bar and the full time controls in the existing Operations shelf. Use existing CSS tokens, 40px control height, grouped flex rows, a range input with visible numeric time, and no new floating panel. Keep shelf collapse behavior unchanged.

- [ ] **Step 5: Run focused web tests**

Run: `PYTHONPATH=src/simulator pytest -q src/simulator/test/test_tactical_web.py -k 'match_clock or match_controls or tactical_workspace'`

Expected: passing static and Playwright cases; Playwright skips only when unavailable.

### Task 3: Document And Verify The Whole Path

**Files:**
- Modify: `docs/sentry/internal/simulator.md`
- Modify: `src/simulator/README.md`

**Interfaces:**
- Documents the existing `match_control` configuration and browser control-bus semantics.

- [ ] **Step 1: Document exact rewind semantics**

State that `rewind` increases remaining match time, `forward` decreases it, `set_time_left` assigns it, all values clamp to configured duration, and none restore scene/trace data.

- [ ] **Step 2: Run complete simulator verification**

Run:

```bash
PYTHONPATH=src/simulator pytest -q src/simulator/test
python3 -m py_compile src/simulator/simulator/runtime.py src/simulator/simulator/tactical_web.py
git diff --check
```

Expected: no failed tests, no syntax errors, and no whitespace errors.

- [ ] **Step 3: Run a browser smoke check**

Start a temporary live board on a non-default port, open it through the existing Playwright test path, click Start, `+10s`, `-10s`, and edit `mm:ss`; assert emitted JSONL commands and a persistent Inspector.

- [ ] **Step 4: Commit the implementation**

```bash
git add src/simulator/simulator/runtime.py src/simulator/simulator/tactical_web.py \
  src/simulator/test/test_runtime.py src/simulator/test/test_tactical_web.py \
  docs/sentry/internal/simulator.md src/simulator/README.md
git commit -m "simulator: add match clock controls"
```

## Plan Self-Review

- Scope coverage: command-bar countdown, shelf timeline, exact input, start/pause/reset, 10/30 second rewind/forward, existing control bus, read-only gating, runtime semantics, documentation, and browser verification are mapped to Tasks 1-3.
- No placeholders: every code-facing task names its files, commands, inputs, and outcome.
- Type consistency: all controls use the existing command strings and `seconds` payload accepted by `control_bus.py`, `SimulationRuntime`, and `mock_inputs.py`.
