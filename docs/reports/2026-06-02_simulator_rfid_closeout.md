# Simulator RFID Closeout

Date: 2026-06-02

Scope: simulator-only RFID scenario/test closeout and headless smoke verification.

## Workspace Check

Commands used before updating this report:

```bash
git status --short
rg -n "RFID|rfid|zero-HP|zero HP|quality gate|headless|smoke|targeted|34 passed|rfid_full_zone_context|WARN|PASS" docs/reports src/simulator src/behavior_tree -S
```

Observed state:

- The worktree already contains broad simulator and behavior-tree changes plus untracked simulator assets/tests/docs.
- `docs/reports` had no existing diff before this report was added.
- RFID-related coverage is present under `src/simulator/test/`, `src/simulator/sample/scenarios/manifest.json`, simulator docs, and behavior-tree trace state.

## RFID Slice Results

Freshly rerun for this closeout:

- RFID targeted tests: `34 passed in 0.13s`.
- Scenario fixture `rfid_full_zone_context`: `PASS`.
- The two zero-HP fixtures, `unit_zero_hp_runtime_mismatch` and `self_zero_hp_runtime_mismatch`, are confirmed expected `WARN` cases, matching current formal behavior-tree subscriber semantics for self/unit HP zero values.

## Closeout Verification

Fresh local runs:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator python3 -m pytest src/simulator/test/test_mock_inputs_cli.py src/simulator/test/test_start_mock_command.py src/simulator/test/test_foxglove_export.py -q
PYTHONPATH=src/simulator python3 -m simulator.quality
SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test --web-port 9121
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/unit_zero_hp_runtime_mismatch.jsonl --validate-only --validate-format json
PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/self_zero_hp_runtime_mismatch.jsonl --validate-only --validate-format json
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check --json
```

Results:

- RFID targeted pytest slice: exit `0`, `34 passed in 0.13s`.
- `simulator.quality`: exit `0`.
  - `pytest`: `150 passed in 11.77s`.
  - Quality steps: `15/15 PASS`.
  - Sample validation: `PASS`.
  - Expected route-churn fixture: `WARN` with `scenario.route_churn`, accepted by the gate.
  - Full-roster visual asset QA: `PASS`, `14/14` units visible.
  - Gate smoke screenshots written:
    - `/tmp/ly-simulator-full-roster-smoke.png`
    - `/tmp/ly-simulator-full-roster-visual-qa.png`
- Standalone headless smoke on port `9121`: exit `0`.
  - Web stream started at `http://127.0.0.1:9121/` during the smoke run.
  - Non-blocking environment warnings observed from pygame/pkg_resources and `fc-list` font lookup timeout.
- `unit_zero_hp_runtime_mismatch`: exit `0`, validation status `WARN`, `errors=0`, `warnings=1`, code `unit.zero_hp_runtime_mismatch`.
- `self_zero_hp_runtime_mismatch`: exit `0`, validation status `WARN`, `errors=0`, `warnings=1`, code `referee.zero_self_hp_runtime_mismatch`.
- Optional browser visual checker: exit `0`, status `skip`, reason `python package playwright is not installed`.

## Risk Notes

- The current workspace includes unrelated existing changes; this report does not attempt to classify or revert them.
- Optional real-browser Playwright/Chromium screenshot coverage is still not available in this environment because the Python `playwright` package is not installed; the simulator's optional checker reports this as a clean `skip`.
- Headless pygame smoke still emits non-blocking pkg_resources/font lookup warnings in this shell.
