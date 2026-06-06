from __future__ import annotations

import json
from pathlib import Path

import pytest

from simulator.assets import load_unit_asset_catalog
from simulator.config import load_config, resolve_path
from simulator.field import FieldGeometry, field_to_screen
from simulator.trace import as_dict
from simulator.visual_asset_qa import build_visual_asset_report, main, map_image_rect, scaled_sprite


REPO_ROOT = Path(__file__).resolve().parents[3]


def render_unit_screenshot(tmp_path: Path, scene_path: Path, team: str = "red") -> Path:
    pygame = pytest.importorskip("pygame")
    pygame.init()
    try:
        config = load_config(None)
        screenshot_path = tmp_path / "sprite-visible.png"
        screenshot = pygame.Surface((1500, 900), pygame.SRCALPHA)
        screenshot.fill((16, 18, 22, 255))

        map_path = resolve_path(as_dict(config.get("paths")).get("default_map", "tools/maps/basemaps/buff_map_field.png"))
        map_image = pygame.image.load(str(map_path))
        image_rect = map_image_rect(config, screenshot.get_size(), map_image.get_size())
        field = FieldGeometry.from_config(config.get("field_cm"))
        catalog = load_unit_asset_catalog(as_dict(config.get("assets")))
        sprite_path = catalog.path_for(team, "Hero")
        assert sprite_path is not None
        sprite = scaled_sprite(pygame, sprite_path, catalog.unit_size_px)
        center = field_to_screen((420.0, 1140.0), image_rect, field)
        screenshot.blit(sprite, sprite.get_rect(center=center))
        pygame.image.save(screenshot, str(screenshot_path))
        return screenshot_path
    finally:
        pygame.quit()


def write_one_unit_scene(tmp_path: Path) -> Path:
    scene_path = tmp_path / "one_unit_scene.json"
    scene_path.write_text(
        json.dumps(
            {
                "units": [
                    {
                        "side": "friend",
                        "type": "Hero",
                        "hp": 200,
                        "max_hp": 200,
                        "position_cm": {"x": 420, "y": 1140},
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    return scene_path


def test_visual_asset_qa_passes_when_sprite_pixels_match(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    scene_path = write_one_unit_scene(tmp_path)
    screenshot_path = render_unit_screenshot(tmp_path, scene_path)

    report = build_visual_asset_report(screenshot_path, scene_path, team="red")

    assert report["schema"] == "ly_simulator_visual_asset_qa_v1"
    assert report["summary"]["status"] == "PASS"
    assert report["summary"]["visible_units"] == 1
    assert report["issues"] == []
    assert report["units"][0]["visible"] is True
    assert report["units"][0]["match_ratio"] > 0.9


def test_visual_asset_qa_fails_when_expected_sprite_is_missing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pygame = pytest.importorskip("pygame")
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    scene_path = write_one_unit_scene(tmp_path)
    screenshot_path = tmp_path / "blank.png"
    pygame.init()
    try:
        blank = pygame.Surface((1500, 900), pygame.SRCALPHA)
        blank.fill((16, 18, 22, 255))
        pygame.image.save(blank, str(screenshot_path))
    finally:
        pygame.quit()

    report = build_visual_asset_report(screenshot_path, scene_path, team="red")

    assert report["summary"]["status"] == "FAIL"
    assert report["summary"]["visible_units"] == 0
    assert report["issues"][0]["code"] == "visual_asset.sprite_not_visible"


def test_visual_asset_qa_cli_outputs_json(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    scene_path = write_one_unit_scene(tmp_path)
    screenshot_path = render_unit_screenshot(tmp_path, scene_path)

    code = main(["--screenshot", str(screenshot_path), "--unit-scene", str(scene_path), "--json"])
    payload = json.loads(capsys.readouterr().out)

    assert code == 0
    assert payload["summary"]["status"] == "PASS"
