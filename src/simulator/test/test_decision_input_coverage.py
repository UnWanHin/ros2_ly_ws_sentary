from __future__ import annotations

import json
from pathlib import Path

from simulator.decision_input_coverage import COVERAGE, coverage_payload, main, validate_coverage


def test_decision_input_coverage_catalog_has_expected_inputs() -> None:
    keys = {item.key for item in COVERAGE}

    assert validate_coverage() == []
    assert {
        "match_state",
        "structure_hp",
        "unit_hp_position",
        "self_position",
        "navigation_status_velocity",
        "referee_event_energy",
        "team_buff",
        "rfid",
        "official_target_fallback",
        "gimbal_fire_posture",
        "external_aim",
        "bullet_state",
    } <= keys


def test_coverage_payload_marks_known_partial_items_without_catalog_issues() -> None:
    payload = coverage_payload()
    by_key = {item["key"]: item for item in payload["inputs"]}

    assert payload["schema"] == "ly_simulator_decision_input_coverage_v1"
    assert payload["issues"] == []
    assert by_key["unit_hp_position"]["status"] == "covered_with_formal_exclusions"
    assert "Drone" in " ".join(by_key["unit_hp_position"]["gaps"])
    assert by_key["self_position"]["status"] == "covered"
    assert "/ly/friend/uwb_pos" in by_key["self_position"]["formal_topics"]
    assert "--mock-publish-uwb-position" in by_key["self_position"]["mock_inputs"]
    assert "uwb-position-fusion" in by_key["self_position"]["workflows"]
    assert by_key["self_position"]["gaps"] == []
    assert by_key["external_aim"]["status"] == "optional_covered"
    assert by_key["bullet_state"]["status"] == "covered"
    assert "bullet_info.initial_speed" in by_key["bullet_state"]["trace_fields"]
    assert "bullet_info_resource" in by_key["bullet_state"]["fixtures"]
    assert "bullet-info-resource-snapshot" in by_key["bullet_state"]["workflows"]
    assert by_key["bullet_state"]["gaps"] == []


def test_coverage_cli_outputs_summary_and_detail(capsys) -> None:
    assert main([]) == 0
    output = capsys.readouterr().out

    assert "Decision input coverage:" in output
    assert "official_target_fallback" in output

    assert main(["official_target_fallback"]) == 0
    detail = capsys.readouterr().out

    assert "/ly/navi/target_official" in detail
    assert "--mock-official-target-valid" in detail


def test_coverage_cli_outputs_json_for_selected_input(capsys) -> None:
    assert main(["rfid", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["schema"] == "ly_simulator_decision_input_coverage_v1"
    assert [item["key"] for item in payload["inputs"]] == ["rfid"]
    assert "/ly/game/rfid" in payload["inputs"][0]["formal_topics"]


def test_fail_on_partial_reports_known_gaps(capsys) -> None:
    assert main(["--fail-on-partial"]) == 1
    output = capsys.readouterr().out

    partial_lines = [line for line in output.splitlines() if line.startswith("partial decision input coverage:")]
    assert partial_lines == ["partial decision input coverage: external_aim"]


def test_coverage_entry_point_is_packaged() -> None:
    setup_py = Path(__file__).resolve().parents[1] / "setup.py"

    assert "simulator-decision-input-coverage = simulator.decision_input_coverage:main" in setup_py.read_text(
        encoding="utf-8"
    )
