from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


MATCH_CONTROL_COMMANDS = {
    "start",
    "pause",
    "reset",
    "rewind",
    "forward",
    "set_time_left",
}

SIMULATOR_INPUT_COMMANDS = {
    "set_self_health",
    "set_ammo",
    "set_posture",
    "set_self_position",
    "set_structure_health",
    "set_structure_hp",
    "set_unit",
    "set_units",
    "set_unit_hp",
    "remove_unit",
    "clear_units",
}

SUPPORTED_CONTROL_COMMANDS = MATCH_CONTROL_COMMANDS | SIMULATOR_INPUT_COMMANDS
SECONDS_COMMANDS = {"rewind", "forward", "set_time_left"}


def command_name(payload: dict[str, Any]) -> str:
    return str(payload.get("command", "")).strip().lower()


def normalize_api_control_payload(
    data: dict[str, Any],
    default_step_sec: int,
) -> tuple[str | None, dict[str, Any], str | None]:
    command = command_name(data)
    if command not in SUPPORTED_CONTROL_COMMANDS:
        return (None, {}, f"unsupported command: {command}")

    payload = {key: value for key, value in data.items() if key not in {"command", "ts"}}
    if command in SECONDS_COMMANDS:
        try:
            payload["seconds"] = float(data.get("seconds", default_step_sec))
        except (TypeError, ValueError):
            return (None, {}, "seconds must be number")
    return (command, payload, None)


def append_command(path: Path, command: str, payload: dict[str, Any] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body: dict[str, Any] = {
        "ts": time.time(),
        "command": command,
    }
    if payload:
        body.update(payload)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(body, ensure_ascii=True, separators=(",", ":")))
        stream.write("\n")
        stream.flush()


def read_commands(path: Path, offset: int) -> tuple[list[dict[str, Any]], int]:
    if not path.exists():
        return ([], offset)
    try:
        with path.open("rb") as stream:
            stream.seek(max(0, int(offset)))
            chunk = stream.read()
    except OSError:
        return ([], offset)
    if not chunk:
        return ([], offset)

    commands: list[dict[str, Any]] = []
    consumed = 0
    for line in chunk.splitlines(keepends=True):
        if not line.endswith(b"\n"):
            break
        consumed += len(line)
        raw = line.strip()
        if not raw:
            continue
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(payload, dict):
            commands.append(payload)
    return (commands, offset + consumed)
