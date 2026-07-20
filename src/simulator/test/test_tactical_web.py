from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import pytest

from simulator.tactical_web import tactical_state_from_status
from simulator.web_visual_check import find_sync_playwright
from simulator.web_stream import SimulatorWebStream


def tactical_metadata(*, ownership_mode: str = "mock") -> dict[str, Any]:
    return {
        "current_record": {
            "tick": 42,
            "strategy": "Regional",
            "goal": {"id": 25, "name": "PreRoadland", "side": "red", "pos_cm": [457, 72]},
            "intent": {"layer": "default", "reason": "area_patrol", "priority": 10},
            "gimbal_feedback": {"available": True, "age_ms": 12, "fire_code": {"rotate": 1}},
            "control_output": {
                "available": True,
                "sequence": 9,
                "source": "normal",
                "angles": {"published": True, "yaw": 2.5, "pitch": -1.0},
                "fire_code": {"published": True, "follow_mode": True, "rotate": 0},
                "trajectory": {"published": True, "available": True, "yaw": 2.5, "pitch": -1.0},
            },
            "tactical": {
                "available": True,
                "protect_castle": {
                    "enabled": True,
                    "rfid_enabled": True,
                    "enemy_pos_enabled": True,
                    "rfid_event_raw_active": False,
                    "rfid_event_active": False,
                    "enemy_pos_active": True,
                },
                "protect_hero": {"enabled": True, "active": False},
                "regional_defense": {
                    "threat_active": True,
                    "search_kind": "own_base",
                    "fortress_enemy_count": 0,
                    "own_base_enemy_count": 1,
                },
            },
        },
        "simulator_inputs": {
            "enabled": True,
            "state": {
                "ownership_mode": ownership_mode,
                "field": {"width_cm": 2800, "height_cm": 1500, "frame": "left_bottom_origin_cm"},
                "selected_entity_id": "enemy:hero:a",
                "units": [
                    {
                        "entity_id": "enemy:hero:a",
                        "side": "enemy",
                        "field_side": "blue",
                        "unit_key": "hero",
                        "asset_key": "Hero",
                        "type": "Hero",
                        "hp": 180,
                        "max_hp": 200,
                        "position_cm": {"x": 1200, "y": 700},
                    }
                ],
                "structures": [
                    {
                        "key": "friend_base",
                        "label": "Friend Base",
                        "side": "friend",
                        "kind": "base",
                        "hp": 4800,
                        "max_hp": 5000,
                        "position_cm": {"x": 245, "y": 750},
                    }
                ],
                "palette": [
                    {"side": "enemy", "unit_key": "hero", "asset_key": "Hero", "type": "Hero", "hp": 200, "max_hp": 200}
                ],
            },
        },
    }


def get_json(stream: SimulatorWebStream, path: str) -> tuple[int, dict[str, Any]]:
    url = f"http://127.0.0.1:{stream.port}{path}"
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def get_text(stream: SimulatorWebStream, path: str) -> tuple[int, str]:
    url = f"http://127.0.0.1:{stream.port}{path}"
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return response.status, response.read().decode("utf-8")


def post_json(stream: SimulatorWebStream, path: str, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    url = f"http://127.0.0.1:{stream.port}{path}"
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def started_stream(tmp_path: Path) -> tuple[SimulatorWebStream, Path]:
    control_file = tmp_path / "tactical-control.jsonl"
    stream = SimulatorWebStream(host="127.0.0.1", port=0, control_file=control_file.as_posix())
    stream.start()
    return stream, control_file


def test_tactical_state_keeps_control_output_separate_from_feedback() -> None:
    state = tactical_state_from_status(tactical_metadata())

    assert state["field"] == {"width_cm": 2800, "height_cm": 1500, "frame": "left_bottom_origin_cm"}
    assert state["scene"]["ownership_mode"] == "mock"
    assert state["scene"]["show_trace_units"] is False
    assert state["decision"]["goal"]["id"] == 25
    assert state["gimbal_feedback"]["fire_code"]["rotate"] == 1
    assert state["control_output"]["fire_code"]["rotate"] == 0
    assert state["control_output"]["trajectory"]["available"] is True
    assert state["tactical"]["protect_castle"]["enemy_pos_active"] is True


def test_tactical_state_uses_resolved_goal_position_when_trace_goal_has_no_coordinates() -> None:
    metadata = tactical_metadata()
    metadata["current_record"]["goal"]["pos_cm"] = None
    metadata["decision"] = {
        "goal": {"id": 25, "name": "PreRoadland", "position_cm": [457.0, 72.0]},
        "route_cm": [],
    }

    state = tactical_state_from_status(metadata)

    assert state["decision"]["goal"]["pos_cm"] == [457.0, 72.0]


def test_tactical_routes_use_shared_status_and_reject_manual_ros_mutation(tmp_path: Path) -> None:
    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())

        status, body = get_text(stream, "/tactical")
        assert status == 200
        assert "Tactical board" in body
        assert "pointerdown" in body
        assert "aria-live" in body
        assert 'id="tactical"' in body

        status, state = get_json(stream, "/api/tactical-state")
        assert status == 200
        assert state["scene"]["units"][0]["entity_id"] == "enemy:hero:a"
        assert state["decision"]["goal"]["pos_cm"] == [457, 72]

        status, response = post_json(
            stream,
            "/api/control",
            {
                "command": "place_unit",
                "entity_id": "enemy:hero:b",
                "side": "enemy",
                "unit_key": "hero",
                "x": 1111,
                "y": 722,
            },
        )
        assert status == 200
        assert response["ok"] is True
        assert response["payload"]["x"] == 1111
        written = json.loads(control_file.read_text(encoding="utf-8").strip())
        assert written["command"] == "place_unit"

        stream.update_metadata(tactical_metadata(ownership_mode="manual_ros"))
        status, response = post_json(
            stream,
            "/api/control",
            {"command": "set_structure_health", "side": "friend", "structure": "base", "hp": 3000},
        )
        assert status == 409
        assert response["ok"] is False
        assert response["message"] == "manual_ros_observer_mode"
        assert len(control_file.read_text(encoding="utf-8").splitlines()) == 1
    finally:
        stream.stop()


def test_tactical_browser_interactions_emit_scene_commands_when_playwright_available(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        url = f"http://127.0.0.1:{stream.port}/tactical"

        with factory() as playwright:
            try:
                browser = playwright.chromium.launch(headless=True)
            except Exception as exc:
                pytest.skip(f"Chromium unavailable: {exc}")
            try:
                context = browser.new_context(viewport={"width": 1440, "height": 900})
                try:
                    page = context.new_page()
                    page.goto(url, wait_until="domcontentloaded", timeout=5000)
                    page.wait_for_selector("#fieldBoard", timeout=5000)
                    page.wait_for_function(
                        "document.querySelectorAll('#palette button').length === 1 && "
                        "document.querySelectorAll('.piece').length === 1",
                        timeout=5000,
                    )

                    page.locator("#palette button").click()
                    board_box = page.locator("#fieldBoard").bounding_box()
                    assert board_box is not None
                    page.mouse.click(board_box["x"] + board_box["width"] * 0.40, board_box["y"] + board_box["height"] * 0.55)

                    piece_box = page.locator(".piece[data-entity-id='enemy:hero:a']").bounding_box()
                    assert piece_box is not None
                    page.mouse.move(piece_box["x"] + piece_box["width"] / 2, piece_box["y"] + piece_box["height"] / 2)
                    page.mouse.down()
                    page.mouse.move(board_box["x"] + board_box["width"] * 0.72, board_box["y"] + board_box["height"] * 0.60, steps=4)
                    page.mouse.up()

                    page.get_by_role("button", name="Increase Friend Base health").click()

                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        if control_file.exists() and len(control_file.read_text(encoding="utf-8").splitlines()) >= 3:
                            break
                        page.wait_for_timeout(25)

                    commands = [json.loads(line) for line in control_file.read_text(encoding="utf-8").splitlines()]
                    assert len(commands) == 3
                    assert all(isinstance(command.get("ts"), float) for command in commands)
                    assert commands[0]["command"] == "place_unit"
                    assert commands[0]["side"] == "enemy"
                    assert commands[0]["unit_key"] == "hero"
                    assert 0 <= commands[0]["x"] <= 2800
                    assert 0 <= commands[0]["y"] <= 1500
                    assert {key: value for key, value in commands[1].items() if key != "ts"} == {
                        "command": "place_unit",
                        "entity_id": "enemy:hero:a",
                        "side": "enemy",
                        "unit_key": "hero",
                        "hp": 180,
                        "x": pytest.approx(2016, abs=3),
                        "y": pytest.approx(600, abs=3),
                    }
                    assert {key: value for key, value in commands[2].items() if key != "ts"} == {
                        "command": "set_structure_health",
                        "side": "friend",
                        "structure": "base",
                        "hp": 5000,
                    }
                    page.screenshot(path=(tmp_path / "tactical-interactions.png").as_posix(), full_page=True)
                    assert (tmp_path / "tactical-interactions.png").is_file()
                finally:
                    context.close()
            finally:
                browser.close()
    finally:
        stream.stop()
