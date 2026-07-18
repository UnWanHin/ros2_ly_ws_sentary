from __future__ import annotations

from simulator.config import load_config


def test_default_overlay_keeps_pre_and_ready_roadland_main_areas() -> None:
    config = load_config(None)
    structures = config["structures"]
    items = {str(item["name"]): item for item in structures["items"]}

    assert items["ReadyRoadland.Red"]["kind"] == "main_area"
    assert items["ReadyRoadland.Red"]["polygon"] == [[510, 235], [510, 19], [1251, 17], [1333, 221]]
    assert items["ReadyRoadland.Blue"]["polygon"] == [[2290, 1265], [2290, 1481], [1549, 1483], [1467, 1279]]
    assert items["PreRoadland.Red"]["kind"] == "main_area"
    assert items["PreRoadland.Red"]["polygon"] == [
        [687, 380], [758, 235], [510, 235], [510, 205], [510, 19], [389, 15], [391, 373]
    ]
    assert items["PreRoadland.Blue"]["polygon"] == [
        [2113, 1120], [2042, 1265], [2290, 1265], [2290, 1295], [2290, 1481], [2411, 1485], [2409, 1127]
    ]
    assert "RoadlandFollow.Red" not in items
