from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import goals_by_id, load_config, resolve_path
from .field import FieldGeometry
from .interactive_inputs import load_unit_scene_file
from .trace import load_trace


FORMAL_UNIT_INFO_TYPE_IDS = {1, 2, 3, 4, 7}
FORMAL_UNIT_INFO_TYPE_NAMES = {
    1: "Hero",
    2: "Engineer",
    3: "Infantry1",
    4: "Infantry2",
    7: "Sentry",
}


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


@dataclass(frozen=True)
class ExpectedUnitEvidence:
    side: str
    type_id: int
    type_name: str
    hp: int
    x: float
    y: float
    traceable: bool
    skip_reason: str

    @property
    def key(self) -> str:
        return f"{self.side}:{self.type_name}"

    def as_payload(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "type_id": self.type_id,
            "type": self.type_name,
            "hp": int(self.hp),
            "position_cm": {"x": float(self.x), "y": float(self.y)},
            "traceable": self.traceable,
            "skip_reason": self.skip_reason,
        }


def expected_units_from_scene(scene_path: Path) -> list[ExpectedUnitEvidence]:
    units = load_unit_scene_file(scene_path)
    expected: list[ExpectedUnitEvidence] = []
    for unit in units:
        item = as_dict(unit)
        type_id = int(item.get("type_id", 0))
        type_name = str(item.get("type", FORMAL_UNIT_INFO_TYPE_NAMES.get(type_id, f"Unit{type_id}")))
        traceable = type_id in FORMAL_UNIT_INFO_TYPE_IDS
        expected.append(
            ExpectedUnitEvidence(
                side=str(item.get("side", "")),
                type_id=type_id,
                type_name=type_name,
                hp=int(item.get("hp", 0)),
                x=float(item.get("x", 0.0)),
                y=float(item.get("y", 0.0)),
                traceable=traceable,
                skip_reason="" if traceable else "formal_unit_info_excludes_type",
            )
        )
    return expected


def goal_names_from_config(config: dict[str, Any]) -> dict[int, str]:
    return {goal_id: str(goal.get("name", f"Goal{goal_id}")) for goal_id, goal in goals_by_id(config).items()}


def position_error_cm(expected: ExpectedUnitEvidence, observed: Any) -> float | None:
    if observed.position_cm is None:
        return None
    return math.hypot(float(observed.position_cm[0]) - expected.x, float(observed.position_cm[1]) - expected.y)


def observed_unit_payload(unit: Any, record_index: int, pos_error: float | None) -> dict[str, Any]:
    return {
        "record_index": int(record_index),
        "side": unit.side,
        "car_id": int(unit.car_id),
        "type_id": int(unit.type_id),
        "type": unit.type_name,
        "hp": int(unit.hp),
        "has_hp": bool(unit.has_hp),
        "hp_fresh": bool(unit.hp_fresh),
        "position_cm": (
            {"x": float(unit.position_cm[0]), "y": float(unit.position_cm[1])}
            if unit.position_cm is not None
            else None
        ),
        "has_position": bool(unit.has_position),
        "position_fresh": bool(unit.position_fresh),
        "position_source": unit.position_source,
        "position_error_cm": None if pos_error is None else round(pos_error, 3),
        "area_id": int(unit.area_id),
        "area_name": unit.area_name,
        "area_used_nearest_fallback": bool(unit.area_used_nearest_fallback),
    }


def unit_matches(
    expected: ExpectedUnitEvidence,
    observed: Any,
    *,
    max_position_error_cm: float,
    require_fresh: bool,
) -> tuple[bool, list[str], float | None]:
    failures: list[str] = []
    if observed.side != expected.side:
        failures.append("side")
    if observed.type_id != expected.type_id:
        failures.append("type_id")
    if not observed.has_hp:
        failures.append("missing_hp")
    elif observed.hp != expected.hp:
        failures.append("hp")
    if require_fresh and not observed.hp_fresh:
        failures.append("stale_hp")
    if not observed.has_position or observed.position_cm is None:
        failures.append("missing_position")
        pos_error = None
    else:
        pos_error = position_error_cm(expected, observed)
        if pos_error is None or pos_error > max_position_error_cm:
            failures.append("position")
    if require_fresh and not observed.position_fresh:
        failures.append("stale_position")
    return (not failures, failures, pos_error)


def best_observation(
    expected: ExpectedUnitEvidence,
    records: list[Any],
    *,
    max_position_error_cm: float,
    require_fresh: bool,
) -> tuple[dict[str, Any] | None, list[str]]:
    best_payload: dict[str, Any] | None = None
    best_failures: list[str] = []
    best_score = -1

    for record in records:
        for unit in record.unit_info:
            if unit.side != expected.side or unit.type_id != expected.type_id:
                continue
            matched, failures, pos_error = unit_matches(
                expected,
                unit,
                max_position_error_cm=max_position_error_cm,
                require_fresh=require_fresh,
            )
            payload = observed_unit_payload(unit, record.index, pos_error)
            if matched:
                return payload, []
            score = 0
            score += 1 if unit.has_hp else 0
            score += 1 if unit.hp == expected.hp else 0
            score += 1 if unit.has_position and unit.position_cm is not None else 0
            score += 1 if pos_error is not None and pos_error <= max_position_error_cm else 0
            score += 1 if (not require_fresh or unit.hp_fresh) else 0
            score += 1 if (not require_fresh or unit.position_fresh) else 0
            if score > best_score:
                best_score = score
                best_payload = payload
                best_failures = failures
    return best_payload, best_failures


def build_unit_trace_report(
    trace_path: Path,
    scene_path: Path,
    *,
    config_path: Path | None = None,
    team: str = "red",
    max_position_error_cm: float = 1.0,
    require_fresh: bool = True,
) -> dict[str, Any]:
    config = load_config(config_path)
    names = goal_names_from_config(config)
    records, bad_lines = load_trace(trace_path, names)
    expected_units = expected_units_from_scene(scene_path)
    field = FieldGeometry.from_config(config.get("field_cm"))

    matches: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    observations: list[dict[str, Any]] = []

    if bad_lines:
        issues.append(
            {
                "severity": "error",
                "code": "unit_trace.bad_line",
                "unit": "",
                "message": f"trace skipped {bad_lines} bad JSONL line(s)",
                "suggestion": "Regenerate or repair the trace before using it as UnitInfo evidence.",
            }
        )

    for expected in expected_units:
        expected_payload = expected.as_payload()
        if not expected.traceable:
            skipped.append(expected_payload)
            continue
        if not (0 <= expected.x <= field.width and 0 <= expected.y <= field.height):
            issues.append(
                {
                    "severity": "error",
                    "code": "unit_scene.position_bounds",
                    "unit": expected.key,
                    "message": f"expected position is out of field bounds: x={expected.x:.0f} y={expected.y:.0f}",
                    "suggestion": "Fix the unit scene coordinates before using it as decision trace evidence.",
                }
            )
            continue
        observation, failures = best_observation(
            expected,
            records,
            max_position_error_cm=max_position_error_cm,
            require_fresh=require_fresh,
        )
        if observation is not None:
            observations.append(observation)
        if observation is not None and not failures:
            matches.append({"expected": expected_payload, "observed": observation})
            continue
        code = "unit_trace.missing_unit_info" if observation is None else "unit_trace.mismatched_unit_info"
        detail = "not found" if observation is None else ",".join(failures)
        issues.append(
            {
                "severity": "error",
                "code": code,
                "unit": expected.key,
                "message": f"{expected.key} unit_info evidence {detail}",
                "suggestion": (
                    "Run offline decision with the matching --unit-scene, wait for Health and "
                    "PositionData subscriptions to refresh, then re-check the generated trace."
                ),
            }
        )

    error_count = sum(1 for issue in issues if issue["severity"] == "error")
    return {
        "schema": "ly_simulator_unit_trace_report_v1",
        "trace": trace_path.as_posix(),
        "scene": scene_path.as_posix(),
        "team": team,
        "require_fresh": bool(require_fresh),
        "max_position_error_cm": float(max_position_error_cm),
        "summary": {
            "status": "FAIL" if error_count else "PASS",
            "records": len(records),
            "bad_lines": int(bad_lines),
            "expected_units": len(expected_units),
            "traceable_units": sum(1 for unit in expected_units if unit.traceable),
            "matched_units": len(matches),
            "skipped_units": len(skipped),
            "errors": error_count,
        },
        "matches": matches,
        "skipped": skipped,
        "observations": observations,
        "issues": issues,
    }


def print_text_report(report: dict[str, Any]) -> int:
    summary = as_dict(report.get("summary"))
    print("Unit Trace Evidence Report")
    print(f"Status: {summary.get('status', 'FAIL')}")
    print(f"Trace: {Path(str(report.get('trace', ''))).name}")
    print(f"Scene: {Path(str(report.get('scene', ''))).name}")
    print(
        "Units: "
        f"traceable={summary.get('traceable_units', 0)} "
        f"matched={summary.get('matched_units', 0)} "
        f"skipped={summary.get('skipped_units', 0)}"
    )
    for item in report.get("matches", []):
        expected = as_dict(item.get("expected"))
        observed = as_dict(item.get("observed"))
        pos = as_dict(observed.get("position_cm"))
        print(
            "  PASS "
            f"{expected.get('side')}:{expected.get('type')} "
            f"hp={observed.get('hp')} "
            f"pos=({pos.get('x'):.0f},{pos.get('y'):.0f}) "
            f"source={observed.get('position_source')} "
            f"area={observed.get('area_name')}"
        )
    for item in report.get("skipped", []):
        skipped = as_dict(item)
        print(f"  SKIP {skipped.get('side')}:{skipped.get('type')} reason={skipped.get('skip_reason')}")
    for issue in report.get("issues", []):
        item = as_dict(issue)
        print(f"  ERROR {item.get('code')} {item.get('unit')}: {item.get('message')}")
    return 0 if summary.get("status") == "PASS" else 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare a simulator unit scene with behavior_tree trace unit_info evidence."
    )
    parser.add_argument("trace", help="Decision trace JSONL to inspect.")
    parser.add_argument("--unit-scene", required=True, help="JSON/YAML unit scene expected to feed mock inputs.")
    parser.add_argument("--config", default="", help="Optional simulator config YAML.")
    parser.add_argument("--team", choices=("red", "blue"), default="red", help="Friend team used for the scene.")
    parser.add_argument("--max-position-error-cm", type=float, default=1.0)
    parser.add_argument("--allow-stale", action="store_true", help="Accept stale unit_info HP/position evidence.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    args = parser.parse_args(argv)
    if args.max_position_error_cm < 0:
        parser.error("--max-position-error-cm must be >= 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    trace_path = resolve_path(args.trace).resolve()
    scene_path = resolve_path(args.unit_scene).resolve()
    config_path = resolve_path(args.config).resolve() if str(args.config).strip() else None
    try:
        report = build_unit_trace_report(
            trace_path,
            scene_path,
            config_path=config_path,
            team=args.team,
            max_position_error_cm=float(args.max_position_error_cm),
            require_fresh=not bool(args.allow_stale),
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"failed to build unit trace report: {exc}")
        return 2
    if args.json:
        print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
        return 0 if as_dict(report.get("summary")).get("status") == "PASS" else 1
    return print_text_report(report)


if __name__ == "__main__":
    raise SystemExit(main())
