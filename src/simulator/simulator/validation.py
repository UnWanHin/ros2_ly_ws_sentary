from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .model import TraceRecord
from .trace import as_dict


@dataclass(frozen=True)
class ValidationIssue:
    severity: str
    index: int | None
    message: str


def validate_records(records: list[TraceRecord], config: dict[str, Any], bad_lines: int = 0) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    if bad_lines:
        issues.append(ValidationIssue("warning", None, f"skipped {bad_lines} bad trace line(s)"))

    field = as_dict(config.get("field_cm"))
    field_w = float(field.get("width", 2800))
    field_h = float(field.get("height", 1500))
    last_t: float | None = None
    legacy_schema_seen = False

    for record in records:
        try:
            schema_version = int(record.raw.get("schema_version", 1) or 1)
        except (TypeError, ValueError):
            schema_version = 1
        if schema_version < 2:
            legacy_schema_seen = True
        elif not as_dict(record.raw.get("decision_output")):
            issues.append(ValidationIssue("error", record.index, "schema_version >= 2 missing decision_output"))
        elif not as_dict(record.raw.get("decision_intent")):
            issues.append(ValidationIssue("warning", record.index, "schema_version >= 2 missing decision_intent"))

        if last_t is not None and record.t < last_t:
            issues.append(ValidationIssue("error", record.index, "trace time is not monotonic"))
        last_t = record.t

        pos = record.output.goal_pos_cm
        if pos is None and record.output.uses_goal_pos:
            issues.append(ValidationIssue("error", record.index, "goal_pos output has no goal_pos_cm"))
        if pos is not None and not (0 <= pos[0] <= field_w and 0 <= pos[1] <= field_h):
            issues.append(
                ValidationIssue(
                    "error",
                    record.index,
                    f"goal_pos_cm out of field: x={pos[0]:.0f} y={pos[1]:.0f}",
                )
            )
        if record.output.publish_enabled and record.output.publish_allowed and not record.output.output_topic:
            issues.append(ValidationIssue("warning", record.index, "published output has no output_topic"))
        if record.output.kind == "relative_target_bridge" and not record.navi_relative_target.valid:
            issues.append(ValidationIssue("warning", record.index, "relative_target_bridge output has no valid navi_relative_target"))

        for unit in record.units:
            if unit.max_hp > 0 and not (0 <= unit.hp <= unit.max_hp):
                issues.append(
                    ValidationIssue(
                        "warning",
                        record.index,
                        f"{unit.side}:{unit.type_name} hp={unit.hp} outside 0..{unit.max_hp}",
                    )
                )
            if unit.position_cm is None:
                continue
            x, y = unit.position_cm
            if not (0 <= x <= field_w and 0 <= y <= field_h):
                issues.append(
                    ValidationIssue(
                        "warning",
                        record.index,
                        f"{unit.side}:{unit.type_name} position out of field: x={x:.0f} y={y:.0f}",
                    )
                )

    if legacy_schema_seen:
        issues.insert(
            0,
            ValidationIssue(
                "warning",
                None,
                "legacy schema_version < 2 records are readable but lack stable decision_output/decision_intent fields",
            ),
        )

    return issues


def format_validation(records: list[TraceRecord], issues: list[ValidationIssue]) -> str:
    errors = sum(1 for issue in issues if issue.severity == "error")
    warnings = sum(1 for issue in issues if issue.severity == "warning")
    lines = [
        f"records={len(records)} errors={errors} warnings={warnings}",
        f"duration={records[-1].t - records[0].t:.2f}s ticks={records[0].tick}..{records[-1].tick}",
    ]
    for issue in issues[:40]:
        prefix = issue.severity.upper()
        index = "-" if issue.index is None else str(issue.index)
        lines.append(f"{prefix} record={index}: {issue.message}")
    if len(issues) > 40:
        lines.append(f"... {len(issues) - 40} more issue(s)")
    return "\n".join(lines)
