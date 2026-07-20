from __future__ import annotations

import json
from pathlib import Path

from simulator.unit_scene import build_scene_summary, iter_unit_scene_samples, main


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SCENE = REPO_ROOT / "src" / "simulator" / "sample" / "unit_scene.json"
FULL_ROSTER_SCENE = REPO_ROOT / "src" / "simulator" / "sample" / "unit_scenes" / "full_roster.json"
TACTICAL_BOARD_SCENE = REPO_ROOT / "src" / "simulator" / "sample" / "unit_scenes" / "tactical_board.yaml"


def test_unit_scene_sample_catalog_includes_default_and_full_roster() -> None:
    samples = iter_unit_scene_samples()
    by_name = {sample.name: sample for sample in samples}

    assert by_name["unit_scene.json"].unit_count == 7
    assert by_name["full_roster.json"].unit_count == 14
    assert "Full red/blue unit-art roster" in by_name["full_roster.json"].description


def test_build_scene_summary_exposes_mock_decision_inputs_and_assets() -> None:
    summary = build_scene_summary(DEFAULT_SCENE, team="red")

    assert summary["schema"] == "ly_simulator_unit_scene_summary_v1"
    assert summary["summary"]["unit_count"] == 7
    assert summary["summary"]["friend_units"] == 3
    assert summary["summary"]["enemy_units"] == 4
    assert summary["health_fields"]["friend"]["hero"] == 190
    assert summary["health_fields"]["friend"]["engineer"] == 230
    assert summary["health_fields"]["friend"]["sentry"] == 400
    assert summary["health_fields"]["enemy"]["hero"] == 200
    assert summary["health_fields"]["enemy"]["infantry2"] == 64
    assert "drone" not in summary["health_fields"]["enemy"]

    units = {(unit["side"], unit["type"]): unit for unit in summary["units"]}
    assert units[("enemy", "Drone")]["health_field"] is None
    assert units[("enemy", "Drone")]["health_published"] is False
    assert units[("enemy", "Drone")]["decision_summary"] == "PUB:POS noUI"
    assert units[("enemy", "Drone")]["position_data"]["car_id"] == 106
    assert units[("enemy", "Drone")]["position_data"]["raw_y"] == 577
    assert units[("enemy", "Drone")]["asset"]["sprite_available"] is True
    assert units[("friend", "Sentry")]["field_side"] == "red"
    assert units[("friend", "Sentry")]["decision_summary"] == "BT:HP,POS,UI"
    assert units[("friend", "Sentry")]["asset"]["sprite"].endswith("/units/red/sentry.png")


def test_full_roster_summary_covers_all_visual_unit_assets() -> None:
    summary = build_scene_summary(FULL_ROSTER_SCENE, team="blue")

    assert summary["summary"]["unit_count"] == 14
    assert summary["summary"]["friend_units"] == 7
    assert summary["summary"]["enemy_units"] == 7
    assert all(unit["asset"]["sprite_available"] for unit in summary["units"])
    friend_sides = {unit["field_side"] for unit in summary["units"] if unit["side"] == "friend"}
    enemy_sides = {unit["field_side"] for unit in summary["units"] if unit["side"] == "enemy"}
    assert friend_sides == {"blue"}
    assert enemy_sides == {"red"}


def test_tactical_board_scene_covers_protection_rehearsal_piece_assets() -> None:
    summary = build_scene_summary(TACTICAL_BOARD_SCENE, team="red")

    assert summary["summary"]["unit_count"] == 6
    assert summary["summary"]["friend_units"] == 3
    assert summary["summary"]["enemy_units"] == 3
    assert all(unit["asset"]["sprite_available"] for unit in summary["units"])
    enemy_hero = next(unit for unit in summary["units"] if unit["side"] == "enemy" and unit["type"] == "Hero")
    assert enemy_hero["position_cm"] == {"x": 640, "y": 749}
    assert enemy_hero["decision_summary"] == "BT:HP,POS,UI"


def test_unit_scene_list_samples_cli(capsys) -> None:
    assert main(["--list-samples"]) == 0
    output = capsys.readouterr().out

    assert "Unit scene samples:" in output
    assert "unit_scene.json: 7 units" in output
    assert "full_roster.json: 14 units" in output


def test_unit_scene_json_cli_outputs_machine_readable_summary(capsys) -> None:
    assert main([str(DEFAULT_SCENE), "--team", "red", "--json"]) == 0
    output = capsys.readouterr().out
    payload = json.loads(output)

    assert payload["schema"] == "ly_simulator_unit_scene_summary_v1"
    assert payload["summary"]["unit_count"] == 7
    assert payload["units"][0]["asset"]["sprite_available"] is True


def test_unit_scene_text_cli_includes_decision_channel_badges(capsys) -> None:
    assert main([str(DEFAULT_SCENE), "--team", "red"]) == 0
    output = capsys.readouterr().out

    assert "Hero#1" in output
    assert "decision=BT:HP,POS,UI" in output
    assert "Drone#6" in output
    assert "Health=visual-only decision=PUB:POS noUI" in output
