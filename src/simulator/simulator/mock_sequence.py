from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .control_bus import SUPPORTED_CONTROL_COMMANDS, append_command, command_name
from .config import package_share


SUPPORTED_SEQUENCE_COMMANDS = SUPPORTED_CONTROL_COMMANDS
TIME_KEYS = ("at_sec", "at", "elapsed_sec", "match_elapsed_sec")
IGNORED_ACTION_KEYS = {"description", "label", "name", "note"}
SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_SEQUENCE_GLOBS = ("*.json", "*.yaml", "*.yml")


@dataclass(frozen=True)
class MockSequenceAction:
    at_sec: float
    command: str
    payload: dict[str, Any]


@dataclass(frozen=True)
class MockSequenceSample:
    name: str
    path: Path
    description: str
    action_count: int


class MockSequence:
    def __init__(self, actions: list[MockSequenceAction]) -> None:
        self.actions = actions
        self._next_index = 0

    @property
    def action_count(self) -> int:
        return len(self.actions)

    def reset(self) -> None:
        self._next_index = 0

    def due_actions(self, elapsed_sec: float) -> list[MockSequenceAction]:
        try:
            elapsed = float(elapsed_sec)
        except (TypeError, ValueError):
            return []
        if not math.isfinite(elapsed):
            return []

        due: list[MockSequenceAction] = []
        while self._next_index < len(self.actions) and self.actions[self._next_index].at_sec <= elapsed:
            due.append(self.actions[self._next_index])
            self._next_index += 1
        return due


def parse_mock_sequence(raw: Any) -> MockSequence:
    raw_actions = _raw_actions(raw)
    actions = [_parse_action(index, item) for index, item in enumerate(raw_actions)]
    actions.sort(key=lambda action: action.at_sec)
    return MockSequence(actions)


def load_mock_sequence_file(path: str | Path) -> MockSequence:
    raw = load_mock_sequence_raw(path)
    return parse_mock_sequence(raw)


def load_mock_sequence_raw(path: str | Path) -> Any:
    sequence_path = Path(path).expanduser()
    with sequence_path.open("r", encoding="utf-8") as stream:
        if sequence_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError("YAML mock sequences require PyYAML") from exc

            try:
                return yaml.safe_load(stream) or {}
            except yaml.YAMLError as exc:
                raise ValueError(f"invalid YAML mock sequence: {exc}") from exc
        try:
            return json.load(stream)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON mock sequence: {exc}") from exc


def sample_sequence_dir() -> Path:
    source_dir = SOURCE_PACKAGE_ROOT / "sample" / "mock_sequences"
    if source_dir.exists():
        return source_dir
    share = package_share()
    if share is not None:
        return share / "sample" / "mock_sequences"
    return source_dir


def iter_sample_sequence_paths(sample_dir: Path | None = None) -> list[Path]:
    root = sample_dir or sample_sequence_dir()
    if not root.exists():
        return []
    paths: list[Path] = []
    for pattern in SAMPLE_SEQUENCE_GLOBS:
        paths.extend(root.glob(pattern))
    return sorted({path.resolve() for path in paths if path.is_file()})


def load_mock_sequence_sample(path: str | Path) -> MockSequenceSample:
    sequence_path = Path(path).expanduser().resolve()
    raw = load_mock_sequence_raw(sequence_path)
    description = ""
    if isinstance(raw, dict):
        description = str(raw.get("description", "")).strip()
    sequence = parse_mock_sequence(raw)
    return MockSequenceSample(
        name=sequence_path.name,
        path=sequence_path,
        description=description,
        action_count=sequence.action_count,
    )


def iter_sample_sequences(sample_dir: Path | None = None) -> list[MockSequenceSample]:
    return [load_mock_sequence_sample(path) for path in iter_sample_sequence_paths(sample_dir)]


def print_sample_sequences(sample_dir: Path | None = None) -> int:
    samples = iter_sample_sequences(sample_dir)
    root = sample_dir or sample_sequence_dir()
    if not samples:
        print(f"No mock sequence samples found in {root}")
        return 0
    print("Mock sequence samples:")
    for sample in samples:
        print(f"  {sample.name}: {sample.action_count} actions")
        print(f"    path: {sample.path}")
        if sample.description:
            print(f"    description: {sample.description}")
    return 0


def emit_due_actions(sequence: MockSequence, control_path: Path, elapsed_sec: float) -> int:
    count = 0
    for action in sequence.due_actions(elapsed_sec):
        payload = {key: value for key, value in action.payload.items() if key not in {"command", "ts"}}
        append_command(control_path, action.command, payload)
        count += 1
    return count


def run_sequence(sequence: MockSequence, control_path: Path, poll_sec: float = 0.05) -> int:
    start = time.monotonic()
    while True:
        elapsed = time.monotonic() - start
        emit_due_actions(sequence, control_path, elapsed)
        if sequence._next_index >= len(sequence.actions):
            return 0
        time.sleep(max(0.001, float(poll_sec)))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Emit timed simulator control-bus commands from a mock sequence.")
    parser.add_argument("sequence", nargs="?", help="JSON/YAML mock sequence file.")
    parser.add_argument("--control-file", default="", help="JSONL simulator control bus path.")
    parser.add_argument("--poll-sec", type=float, default=0.05, help="Scheduler poll interval in seconds.")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print the sequence without emitting commands.")
    parser.add_argument("--list-samples", action="store_true", help="List bundled mock sequence examples and exit.")
    args = parser.parse_args(argv)
    if args.list_samples:
        return args
    if not str(args.sequence or "").strip():
        parser.error("sequence is required unless --list-samples is used")
    if args.poll_sec <= 0:
        parser.error("--poll-sec must be > 0")
    if not str(args.control_file).strip():
        parser.error("--control-file must not be empty")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.list_samples:
        try:
            return print_sample_sequences()
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"failed to list mock sequence samples: {exc}")
            return 2
    sequence_path = Path(args.sequence).expanduser().resolve()
    control_path = Path(args.control_file).expanduser().resolve()
    try:
        sequence = load_mock_sequence_file(sequence_path)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"failed to load mock sequence {sequence_path}: {exc}")
        return 2
    if args.dry_run:
        print(f"mock sequence: {sequence_path}")
        print(f"control file: {control_path}")
        for action in sequence.actions:
            payload = {key: value for key, value in action.payload.items() if key != "command"}
            print(f"{action.at_sec:.3f}s {action.command} {json.dumps(payload, ensure_ascii=True, sort_keys=True)}")
        return 0
    return run_sequence(sequence, control_path, poll_sec=float(args.poll_sec))


def _raw_actions(raw: Any) -> list[Any]:
    if isinstance(raw, dict):
        actions = raw.get("actions", [])
    else:
        actions = raw
    if not isinstance(actions, list):
        raise ValueError("mock sequence actions must be a list")
    return actions


def _parse_action(index: int, raw: Any) -> MockSequenceAction:
    if not isinstance(raw, dict):
        raise ValueError(f"mock sequence action #{index} must be an object")
    at_sec = _action_time(index, raw)
    payload = _action_payload(raw)
    command = command_name(payload)
    if command not in SUPPORTED_SEQUENCE_COMMANDS:
        if command:
            raise ValueError(f"mock sequence action #{index} has unsupported command: {command}")
        raise ValueError(f"mock sequence action #{index} is missing command")
    payload["command"] = command
    return MockSequenceAction(at_sec=at_sec, command=command, payload=payload)


def _action_time(index: int, raw: dict[str, Any]) -> float:
    value = None
    for key in TIME_KEYS:
        if key in raw:
            value = raw[key]
            break
    try:
        at_sec = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"mock sequence action #{index} has invalid time") from None
    if not math.isfinite(at_sec) or at_sec < 0:
        raise ValueError(f"mock sequence action #{index} time must be >= 0")
    return at_sec


def _action_payload(raw: dict[str, Any]) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    raw_payload = raw.get("payload")
    if isinstance(raw_payload, dict):
        payload.update(raw_payload)
    else:
        for key, value in raw.items():
            if key in TIME_KEYS or key in IGNORED_ACTION_KEYS or key == "payload":
                continue
            payload[key] = value
    if "command" in raw and "command" not in payload:
        payload["command"] = raw["command"]
    return payload


if __name__ == "__main__":
    raise SystemExit(main())
