from __future__ import annotations

import json
from pathlib import Path

from simulator.unit_trace import build_unit_trace_report, main


REPO_ROOT = Path(__file__).resolve().parents[3]
TRACE_PATH = REPO_ROOT / "src" / "simulator" / "sample" / "scenarios" / "multi_unit_decision_context.jsonl"
SCENE_PATH = REPO_ROOT / "src" / "simulator" / "sample" / "unit_scenes" / "multi_unit_trace_contract.json"


def test_unit_trace_report_matches_traceable_scene_units_and_skips_visual_only_units() -> None:
    report = build_unit_trace_report(TRACE_PATH, SCENE_PATH)

    assert report["schema"] == "ly_simulator_unit_trace_report_v1"
    assert report["summary"] == {
        "status": "PASS",
        "records": 1,
        "bad_lines": 0,
        "expected_units": 14,
        "traceable_units": 10,
        "matched_units": 10,
        "skipped_units": 4,
        "errors": 0,
    }
    assert not report["issues"]
    skipped = {(unit["side"], unit["type"], unit["skip_reason"]) for unit in report["skipped"]}
    assert skipped == {
        ("friend", "Infantry3", "formal_unit_info_excludes_type"),
        ("friend", "Drone", "formal_unit_info_excludes_type"),
        ("enemy", "Infantry3", "formal_unit_info_excludes_type"),
        ("enemy", "Drone", "formal_unit_info_excludes_type"),
    }

    enemy_infantry2 = [
        item
        for item in report["matches"]
        if item["expected"]["side"] == "enemy" and item["expected"]["type"] == "Infantry2"
    ][0]
    assert enemy_infantry2["observed"]["hp"] == 64
    assert enemy_infantry2["observed"]["position_source"] == "navi_target_official"
    assert enemy_infantry2["observed"]["position_error_cm"] == 0.0


def test_unit_trace_cli_outputs_machine_readable_report(capsys) -> None:
    code = main([str(TRACE_PATH), "--unit-scene", str(SCENE_PATH), "--json"])
    output = capsys.readouterr().out
    payload = json.loads(output)

    assert code == 0
    assert payload["summary"]["status"] == "PASS"
    assert payload["summary"]["matched_units"] == 10


def test_unit_trace_report_fails_when_scene_expectation_does_not_match_trace(tmp_path: Path) -> None:
    scene_path = tmp_path / "wrong_scene.json"
    scene_path.write_text(
        json.dumps(
            {
                "units": [
                    {
                        "side": "friend",
                        "type": "Hero",
                        "hp": 199,
                        "max_hp": 200,
                        "position_cm": {"x": 625, "y": 1080},
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    report = build_unit_trace_report(TRACE_PATH, scene_path)

    assert report["summary"]["status"] == "FAIL"
    assert report["summary"]["errors"] == 1
    assert report["issues"][0]["code"] == "unit_trace.mismatched_unit_info"
    assert report["issues"][0]["unit"] == "friend:Hero"
    assert "hp" in report["issues"][0]["message"]


def test_unit_trace_report_fails_when_trace_contains_bad_jsonl_rows(tmp_path: Path) -> None:
    trace_path = tmp_path / "corrupt_trace.jsonl"
    trace_path.write_text(
        TRACE_PATH.read_text(encoding="utf-8") + "\n{bad json}\n",
        encoding="utf-8",
    )

    report = build_unit_trace_report(trace_path, SCENE_PATH)

    assert report["summary"]["status"] == "FAIL"
    assert report["summary"]["bad_lines"] == 1
    assert report["summary"]["errors"] == 1
    assert report["issues"][0]["code"] == "unit_trace.bad_line"
