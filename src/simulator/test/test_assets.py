from __future__ import annotations

import struct

import pytest

from simulator.assets import load_unit_asset_catalog
from simulator.config import load_config
from simulator.trace import as_dict


PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
REQUIRED_UNIT_KEYS = {
    ("red", "hero"),
    ("red", "engineer"),
    ("red", "infantry"),
    ("red", "sentry"),
    ("red", "drone"),
    ("blue", "hero"),
    ("blue", "engineer"),
    ("blue", "infantry"),
    ("blue", "sentry"),
    ("blue", "drone"),
}


def png_size(data: bytes) -> tuple[int, int]:
    assert data[:8] == PNG_MAGIC
    assert data[12:16] == b"IHDR"
    width, height = struct.unpack(">II", data[16:24])
    return int(width), int(height)


def test_default_unit_asset_manifest_resolves_semantic_units() -> None:
    config = load_config(None)
    catalog = load_unit_asset_catalog(as_dict(config.get("assets")))

    assert catalog.enabled
    assert catalog.manifest_path is not None
    assert catalog.path_for("red", "Hero").name == "hero.png"
    assert catalog.path_for("blue", "Engineer").name == "engineer.png"
    assert catalog.path_for("red", "Infantry1").name == "infantry.png"
    assert catalog.path_for("blue", "Infantry3").name == "infantry.png"
    assert catalog.path_for("red", "Drone").name == "drone.png"
    assert catalog.path_for("green", "Hero") is None
    assert catalog.path_for("red", "Unknown") is None


def test_default_asset_files_are_packaged_pngs() -> None:
    config = load_config(None)
    catalog = load_unit_asset_catalog(as_dict(config.get("assets")))

    assert len(catalog.unit_paths) == 10
    for path in catalog.unit_paths.values():
        data = path.read_bytes()
        width, height = png_size(data)
        assert 96 <= width <= 320
        assert 96 <= height <= 240
    assert set(catalog.armor_paths) == {"armor_strip", "armor_no_background", "armor_fine_edited"}
    for path in catalog.armor_paths.values():
        width, height = png_size(path.read_bytes())
        assert width == 2172
        assert height in {194, 724}
    assert catalog.armor_path_for("armor_no_background").name == "armor_nobackground.png"
    assert catalog.armor_path_for("missing") is None


def test_default_asset_manifest_has_required_semantic_keys_and_provenance() -> None:
    config = load_config(None)
    catalog = load_unit_asset_catalog(as_dict(config.get("assets")))

    assert set(catalog.unit_paths) == REQUIRED_UNIT_KEYS
    assert catalog.aliases == {"infantry1": "infantry", "infantry2": "infantry", "infantry3": "infantry"}
    assert catalog.provenance["archive_name"] == "素材.zip"
    assert catalog.provenance["archive_sha256"] == "8716faeaadce88422023023f923af5679c404241151b81d9a7a96b126fe963f4"
    assert catalog.provenance["license_status"] == "unknown"
    assert catalog.provenance["redistribution"] == "local_project_only_until_license_confirmed"
    assert catalog.provenance["attribution"] == "not_provided_in_archive"


def test_asset_catalog_can_be_disabled() -> None:
    catalog = load_unit_asset_catalog({"enabled": False})

    assert not catalog.enabled
    assert catalog.provenance == {}
    assert catalog.path_for("red", "Hero") is None
    assert catalog.armor_path_for("armor_no_background") is None


def test_default_assets_decode_scale_and_blit_with_pygame(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    pygame.init()
    try:
        pygame.display.set_mode((1, 1))
        config = load_config(None)
        catalog = load_unit_asset_catalog(as_dict(config.get("assets")))
        canvas = pygame.Surface((640, 360), pygame.SRCALPHA)

        x = 18
        y = 18
        for (_side, _unit_type), path in sorted(catalog.unit_paths.items()):
            raw = pygame.image.load(str(path)).convert_alpha()
            sprite = pygame.transform.smoothscale(raw, (48, 48))
            assert sprite.get_bounding_rect(min_alpha=1).width > 0
            canvas.blit(sprite, (x, y))
            x += 58
            if x > 560:
                x = 18
                y += 58

        y += 70
        for path in catalog.armor_paths.values():
            raw = pygame.image.load(str(path)).convert_alpha()
            scale = min(170 / raw.get_width(), 46 / raw.get_height())
            target_size = (
                max(1, int(raw.get_width() * scale)),
                max(1, int(raw.get_height() * scale)),
            )
            sprite = pygame.transform.smoothscale(raw, target_size)
            assert sprite.get_bounding_rect(min_alpha=1).width > 0
            canvas.blit(sprite, (18, y))
            y += 56

        rendered = canvas.get_bounding_rect(min_alpha=1)
        assert rendered.width > 0
        assert rendered.height > 0
    finally:
        pygame.quit()
