from __future__ import annotations

import html
import mimetypes
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import quote, unquote

from .assets import UnitAssetCatalog, load_unit_asset_catalog


TACTICAL_STATE_SCHEMA = "ly_simulator_tactical_state_v1"


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


@dataclass(frozen=True)
class TacticalAssetRegistry:
    """Expose only manifest-backed PNGs to the dependency-free tactical page."""

    root: Path | None
    catalog: UnitAssetCatalog

    @classmethod
    def load(cls) -> "TacticalAssetRegistry":
        catalog = load_unit_asset_catalog()
        root = catalog.manifest_path.parent.resolve() if catalog.manifest_path is not None else None
        return cls(root=root, catalog=catalog)

    def unit_url(self, field_side: str, asset_key: str) -> str | None:
        path = self.catalog.path_for(field_side, asset_key)
        if path is None or self.root is None:
            return None
        try:
            relative = path.resolve().relative_to(self.root)
        except ValueError:
            return None
        return "/assets/" + quote(relative.as_posix())

    def resolve_public_path(self, encoded_relative: str) -> tuple[Path | None, str | None]:
        if self.root is None:
            return None, None
        relative = Path(unquote(encoded_relative))
        if relative.is_absolute() or ".." in relative.parts or relative.suffix.lower() != ".png":
            return None, None
        candidate = (self.root / relative).resolve()
        try:
            candidate.relative_to(self.root)
        except ValueError:
            return None, None
        if not candidate.is_file():
            return None, None
        return candidate, mimetypes.guess_type(candidate.name)[0] or "image/png"


def _field(scene: Mapping[str, Any]) -> dict[str, Any]:
    raw = as_dict(scene.get("field"))
    width = _positive_int(raw.get("width_cm"), 2800)
    height = _positive_int(raw.get("height_cm"), 1500)
    return {
        "width_cm": width,
        "height_cm": height,
        "frame": str(raw.get("frame", "left_bottom_origin_cm")),
    }


def _positive_int(value: Any, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0 else default


def _scene_item(value: Any, registry: TacticalAssetRegistry | None) -> dict[str, Any]:
    item = dict(as_dict(value))
    field_side = str(item.get("field_side", "")).strip().lower()
    asset_key = str(item.get("asset_key", item.get("unit_key", item.get("type", "")))).strip()
    if registry is not None and field_side in {"red", "blue"} and asset_key:
        asset_url = registry.unit_url(field_side, asset_key)
        if asset_url is not None:
            item["asset_url"] = asset_url
    return item


def tactical_state_from_status(
    status: Mapping[str, Any],
    *,
    asset_registry: TacticalAssetRegistry | None = None,
) -> dict[str, Any]:
    """Extract the browser contract from the generic stream status snapshot.

    The browser deliberately receives only normalized scene facts and trace
    evidence. It never infers a control command from mutable input state.
    """

    root = as_dict(status)
    simulator_inputs = as_dict(root.get("simulator_inputs"))
    raw_scene = as_dict(simulator_inputs.get("state"))
    record = as_dict(root.get("current_record"))
    ownership_mode = str(raw_scene.get("ownership_mode", "mock")).strip().lower()
    if ownership_mode not in {"mock", "manual_ros"}:
        ownership_mode = "mock"

    enabled = bool(simulator_inputs.get("enabled", False))
    control_enabled = bool(root.get("control_enabled", False))
    units = [_scene_item(value, asset_registry) for value in as_list(raw_scene.get("units"))]
    palette = [_scene_item(value, asset_registry) for value in as_list(raw_scene.get("palette"))]
    structures = [dict(as_dict(value)) for value in as_list(raw_scene.get("structures"))]
    can_edit = enabled and control_enabled and ownership_mode == "mock"
    goal = dict(as_dict(record.get("goal")))
    decision = as_dict(root.get("decision"))
    resolved_goal = as_dict(decision.get("goal"))
    if goal.get("pos_cm") is None:
        resolved_position = resolved_goal.get("position_cm")
        if resolved_position is None:
            resolved_position = resolved_goal.get("pos_cm")
        if resolved_position is not None:
            goal["pos_cm"] = resolved_position
    intent = dict(as_dict(record.get("intent")))

    return {
        "schema": TACTICAL_STATE_SCHEMA,
        "ready": bool(root.get("ready", False)),
        "field": _field(raw_scene),
        "scene": {
            "enabled": enabled,
            "ownership_mode": ownership_mode,
            "can_edit": can_edit,
            "show_trace_units": not bool(units),
            "selected_entity_id": raw_scene.get("selected_entity_id"),
            "units": units,
            "structures": structures,
            "palette": palette,
            "projection": dict(as_dict(raw_scene.get("projection"))),
        },
        "decision": {
            "tick": record.get("tick"),
            "strategy": record.get("strategy"),
            "aim": record.get("aim"),
            "goal": goal,
            "intent": intent,
            "route_cm": list(as_list(decision.get("route_cm"))),
        },
        "gimbal_feedback": dict(as_dict(record.get("gimbal_feedback"))),
        "control_output": dict(as_dict(record.get("control_output"))),
        "tactical": dict(as_dict(record.get("tactical"))),
        "match": {
            "time_left": as_dict(root.get("replay")).get("match_time_left"),
            "running": bool(as_dict(root.get("replay")).get("match_running", False)),
        },
        "stream": {
            "frame_url": "/frame.jpg",
            "map_url": "/tactical-map.png",
            "frame_id": root.get("frame_id"),
        },
    }


def is_scene_mutation(command: str) -> bool:
    return str(command).strip().lower() in {
        "place_unit",
        "set_unit",
        "set_units",
        "set_unit_hp",
        "remove_unit",
        "clear_units",
        "set_structure_health",
        "set_structure_hp",
        "set_self_health",
        "set_ammo",
        "set_posture",
        "set_self_position",
    }


def build_tactical_html(port: int) -> bytes:
    """Return the complete standalone tactical map editor shell."""

    body = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LY Tactical Board</title>
<style>
:root {{
  --ink: #101417;
  --chrome: #171d21;
  --surface: #222a2e;
  --line: #445158;
  --text: #eff4f5;
  --muted: #a9b5b9;
  --red: #e85d5d;
  --blue: #5d8ee8;
  --gold: #f5c542;
  --green: #4cc38a;
  --danger: #ef8174;
}}
* {{ box-sizing: border-box; }}
body {{ margin: 0; min-width: 320px; background: var(--ink); color: var(--text); font: 14px/1.35 Inter, "Noto Sans", sans-serif; }}
button {{ font: inherit; color: inherit; }}
button:focus-visible, [tabindex]:focus-visible {{ outline: 2px solid var(--gold); outline-offset: 2px; }}
#topbar {{ display: flex; align-items: center; justify-content: space-between; gap: 12px; min-height: 54px; padding: 10px 16px; border-bottom: 1px solid var(--line); background: var(--chrome); }}
#topbar h1 {{ margin: 0; font-size: 17px; font-weight: 700; letter-spacing: 0; }}
#topbar h1 span {{ color: var(--gold); font-weight: 600; }}
.status-strip {{ display: flex; align-items: center; flex-wrap: wrap; justify-content: flex-end; gap: 6px; }}
.pill {{ padding: 3px 7px; border: 1px solid var(--line); border-radius: 4px; color: var(--muted); font: 12px/1.1 "DejaVu Sans Mono", monospace; }}
.pill.good {{ color: var(--green); border-color: var(--green); }}
.pill.warn {{ color: var(--gold); border-color: var(--gold); }}
.pill.bad {{ color: var(--danger); border-color: var(--danger); }}
.shell {{ display: grid; grid-template-columns: minmax(0, 1fr) 360px; min-height: calc(100vh - 54px); }}
.workspace {{ padding: 12px; min-width: 0; }}
.board-toolbar {{ display: flex; justify-content: space-between; align-items: center; gap: 10px; min-height: 38px; padding: 0 2px 8px; color: var(--muted); }}
.board-toolbar strong {{ color: var(--text); font-weight: 600; }}
#fieldBoard {{ position: relative; width: 100%; max-width: 1180px; aspect-ratio: 28 / 15; overflow: hidden; touch-action: none; background: #30373a; border: 1px solid var(--line); }}
#fieldImage {{ position: absolute; inset: 0; width: 100%; height: 100%; object-fit: fill; user-select: none; pointer-events: none; }}
#routeOverlay {{ position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; overflow: visible; }}
#pieceLayer {{ position: absolute; inset: 0; }}
.piece {{ position: absolute; width: 48px; height: 48px; margin: -24px 0 0 -24px; padding: 0; border: 0; border-radius: 50%; background: transparent; cursor: grab; touch-action: none; }}
.piece.dragging {{ cursor: grabbing; z-index: 8; }}
.piece.selected .piece-ring {{ border-color: var(--gold); box-shadow: 0 0 0 2px #101417; }}
.piece-ring {{ position: absolute; inset: 1px; border: 2px solid var(--blue); border-radius: 50%; background: #101417; overflow: hidden; }}
.piece.enemy .piece-ring {{ border-color: var(--red); }}
.piece img {{ width: 100%; height: 100%; object-fit: contain; display: block; }}
.piece-fallback {{ display: grid; width: 100%; height: 100%; place-items: center; color: var(--text); background: var(--surface); font-weight: 700; }}
.piece-hp {{ position: absolute; right: 0; bottom: -5px; left: 0; height: 5px; border: 1px solid #101417; background: #353f44; }}
.piece-hp > i {{ display: block; height: 100%; background: var(--green); }}
.piece.low .piece-hp > i {{ background: var(--danger); }}
#goalMarker {{ position: absolute; width: 36px; height: 36px; margin: -18px 0 0 -18px; border: 3px solid var(--gold); border-radius: 50%; box-shadow: 0 0 0 2px #101417; pointer-events: none; }}
#goalMarker::after {{ content: ""; position: absolute; top: 9px; left: 9px; width: 12px; height: 12px; background: var(--gold); border-radius: 50%; }}
#goalLabel {{ position: absolute; padding: 3px 5px; color: #101417; background: var(--gold); border: 1px solid #101417; font: 12px/1.2 "DejaVu Sans Mono", monospace; pointer-events: none; white-space: nowrap; }}
.side {{ padding: 12px; border-left: 1px solid var(--line); background: var(--chrome); overflow-y: auto; }}
.section {{ padding: 10px 0 14px; border-bottom: 1px solid var(--line); }}
.section:last-child {{ border-bottom: 0; }}
.section h2 {{ margin: 0 0 8px; color: var(--gold); font-size: 13px; font-weight: 700; letter-spacing: 0; }}
.section h3 {{ margin: 8px 0 4px; color: var(--muted); font-size: 12px; font-weight: 600; }}
.palette {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 6px; }}
.palette button, .structure-row button, .action-row button {{ min-height: 32px; padding: 5px 7px; border: 1px solid var(--line); border-radius: 4px; background: var(--surface); text-align: left; cursor: pointer; }}
.palette button[aria-pressed="true"] {{ border-color: var(--gold); color: var(--gold); }}
.palette button:disabled, .structure-row button:disabled, .action-row button:disabled {{ cursor: not-allowed; opacity: .45; }}
.palette-chip {{ display: flex; align-items: center; min-width: 0; gap: 6px; }}
.palette-chip img {{ width: 26px; height: 26px; object-fit: contain; flex: 0 0 auto; }}
.palette-chip span {{ min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
.structure-row {{ display: grid; grid-template-columns: minmax(0, 1fr) auto auto; align-items: center; gap: 6px; margin: 6px 0; }}
.structure-row button {{ min-width: 34px; text-align: center; }}
.meter {{ height: 5px; margin-top: 3px; overflow: hidden; background: #353f44; border: 1px solid #101417; }}
.meter > i {{ display: block; height: 100%; background: var(--green); }}
.kv {{ display: grid; grid-template-columns: 92px minmax(0, 1fr); gap: 4px 8px; font: 12px/1.35 "DejaVu Sans Mono", monospace; }}
.kv dt {{ color: var(--muted); }} .kv dd {{ min-width: 0; margin: 0; overflow-wrap: anywhere; }}
.action-row {{ display: flex; flex-wrap: wrap; gap: 6px; }}
.action-row button {{ text-align: center; }}
.action-row button.danger {{ color: var(--danger); }}
.read-only {{ color: var(--gold); }}
#liveRegion {{ position: fixed; width: 1px; height: 1px; overflow: hidden; clip: rect(1px, 1px, 1px, 1px); white-space: nowrap; }}
@media (max-width: 900px) {{ .shell {{ grid-template-columns: 1fr; }} .side {{ border-top: 1px solid var(--line); border-left: 0; max-height: none; }} }}
@media (max-width: 520px) {{ #topbar {{ align-items: flex-start; flex-direction: column; }} .status-strip {{ justify-content: flex-start; }} .workspace {{ padding: 8px; }} .side {{ padding: 10px; }} .piece {{ width: 40px; height: 40px; margin: -20px 0 0 -20px; }} }}
</style>
</head>
<body>
<header id="topbar">
  <h1>LY <span>Tactical board</span></h1>
  <div class="status-strip" aria-label="board status">
    <span id="ownerPill" class="pill">owner -</span>
    <span id="decisionPill" class="pill">goal -</span>
    <span id="controlPill" class="pill">control -</span>
  </div>
</header>
<main class="shell">
  <section class="workspace" aria-label="tactical field">
    <div class="board-toolbar"><strong id="goalSummary">Waiting for decision trace</strong><span id="coordinateReadout">-</span></div>
    <div id="fieldBoard" role="application" aria-label="Tactical field. Choose a unit from the palette then click the field to place it. Drag an existing piece to move it.">
      <img id="fieldImage" alt="Competition field map">
      <svg id="routeOverlay" aria-hidden="true"><polyline id="routeLine" fill="none" stroke="#f5c542" stroke-width="4" stroke-linejoin="round" stroke-linecap="round"></polyline></svg>
      <div id="pieceLayer"></div>
      <div id="goalMarker" hidden></div><div id="goalLabel" hidden></div>
    </div>
  </section>
  <aside class="side" aria-label="tactical controls and decision evidence">
    <section class="section"><h2>Pieces</h2><div id="ownerMessage" class="read-only" hidden></div><div id="palette" class="palette"></div></section>
    <section class="section"><h2>Structures</h2><div id="structures"></div></section>
    <section class="section"><h2>Selected piece</h2><div id="selected" class="kv"></div><div id="selectedActions" class="action-row"></div></section>
    <section class="section"><h2>Decision</h2><dl id="decision" class="kv"></dl></section>
    <section class="section"><h2>Tactical</h2><dl id="tactical" class="kv"></dl></section>
    <section class="section"><h2>Final control output</h2><dl id="control" class="kv"></dl><h3>Lower-machine feedback</h3><dl id="feedback" class="kv"></dl></section>
    <section class="section"><h2>Match</h2><div id="matchActions" class="action-row"><button data-command="start">Start</button><button data-command="pause">Pause</button><button data-command="reset">Reset</button></div></section>
  </aside>
</main>
<div id="liveRegion" role="status" aria-live="polite"></div>
<script>
const board = document.getElementById('fieldBoard');
const mapImage = document.getElementById('fieldImage');
const pieceLayer = document.getElementById('pieceLayer');
const palette = document.getElementById('palette');
const structures = document.getElementById('structures');
const selected = document.getElementById('selected');
const selectedActions = document.getElementById('selectedActions');
const goalMarker = document.getElementById('goalMarker');
const goalLabel = document.getElementById('goalLabel');
const routeLine = document.getElementById('routeLine');
const liveRegion = document.getElementById('liveRegion');
let state = null;
let paletteChoice = null;
let drag = null;
let placementPointer = null;
function text(value, fallback='-') {{ return value === null || value === undefined || value === '' ? fallback : String(value); }}
function numeric(value, digits=1) {{ return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '-'; }}
function escapeText(value) {{ return text(value).replace(/[&<>\"]/g, (c) => ({{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}}[c])); }}
function clamp(value, low, high) {{ return Math.max(low, Math.min(high, value)); }}
function pct(point) {{ const f = state.field; return {{ left: (100 * point.x / f.width_cm), top: (100 * (1 - point.y / f.height_cm)) }}; }}
function command(payload) {{
  if (!state || !state.scene.can_edit) return Promise.resolve({{ok:false, message:'manual_ros_observer_mode'}});
  return fetch('/api/control', {{method:'POST', headers:{{'Content-Type':'application/json'}}, body:JSON.stringify(payload)}})
    .then((r) => r.json().then((body) => ({{status:r.status, body}})))
    .then((result) => {{ announce(result.body.ok ? 'Command accepted' : ('Command rejected: ' + text(result.body.message))); return result.body; }})
    .catch((error) => {{ announce('Command failed'); return {{ok:false, message:String(error)}}; }});
}}
function announce(message) {{ liveRegion.textContent = message; }}
function setPill(id, value, kind='') {{ const el = document.getElementById(id); el.textContent = value; el.className = 'pill ' + kind; }}
function addKv(container, rows) {{ container.textContent = ''; for (const [key, value] of rows) {{ const dt=document.createElement('dt'); dt.textContent=key; const dd=document.createElement('dd'); dd.textContent=text(value); container.append(dt,dd); }} }}
function readPoint(event) {{ const rect = board.getBoundingClientRect(); const f = state.field; return {{ x: Math.round(clamp((event.clientX - rect.left) * f.width_cm / rect.width, 0, f.width_cm)), y: Math.round(clamp((rect.bottom - event.clientY) * f.height_cm / rect.height, 0, f.height_cm)) }}; }}
function setPosition(el, point) {{ const p=pct(point); el.style.left=p.left+'%'; el.style.top=p.top+'%'; }}
function unitId(choice) {{ return choice.side + ':' + choice.unit_key + ':' + Date.now().toString(36); }}
function isLow(unit) {{ return Number(unit.max_hp) > 0 && Number(unit.hp) / Number(unit.max_hp) < .35; }}
function renderPalette() {{
  palette.textContent=''; const editable=Boolean(state.scene.can_edit);
  for (const item of state.scene.palette || []) {{
    const button=document.createElement('button'); button.type='button'; button.disabled=!editable; button.setAttribute('aria-pressed', String(paletteChoice && paletteChoice.side===item.side && paletteChoice.unit_key===item.unit_key));
    const chip=document.createElement('span'); chip.className='palette-chip';
    if (item.asset_url) {{ const img=document.createElement('img'); img.src=item.asset_url; img.alt=''; chip.append(img); }}
    const label=document.createElement('span'); label.textContent=text(item.side)+' '+text(item.type); chip.append(label); button.append(chip);
    button.addEventListener('click', () => {{ paletteChoice=item; renderPalette(); announce('Selected '+text(item.type)); }}); palette.append(button);
  }}
}}
function renderPieces() {{
  pieceLayer.textContent=''; const editable=Boolean(state.scene.can_edit); const selectedId=state.scene.selected_entity_id;
  for (const unit of state.scene.units || []) {{
    const point=unit.position_cm; if (!point) continue; const button=document.createElement('button'); button.type='button'; button.className='piece '+text(unit.side)+(unit.entity_id===selectedId?' selected':'')+(isLow(unit)?' low':''); button.dataset.entityId=unit.entity_id; button.setAttribute('aria-label', text(unit.side)+' '+text(unit.type)+' '+text(unit.hp)+' of '+text(unit.max_hp)); button.disabled=!editable; setPosition(button, point);
    const ring=document.createElement('span'); ring.className='piece-ring'; if (unit.asset_url) {{ const img=document.createElement('img'); img.src=unit.asset_url; img.alt=''; ring.append(img); }} else {{ const fallback=document.createElement('span'); fallback.className='piece-fallback'; fallback.textContent=text(unit.type).slice(0,2); ring.append(fallback); }}
    const meter=document.createElement('span'); meter.className='piece-hp'; const fill=document.createElement('i'); fill.style.width=(100*clamp(Number(unit.hp)/Math.max(1,Number(unit.max_hp)),0,1))+'%'; meter.append(fill); button.append(ring,meter);
    button.addEventListener('pointerdown', (event) => {{ if (!editable) return; event.stopPropagation(); drag={{unit, pointerId:event.pointerId, element:button}}; button.classList.add('dragging'); board.setPointerCapture(event.pointerId); }}); pieceLayer.append(button);
  }}
}}
function renderStructures() {{
  structures.textContent=''; const editable=Boolean(state.scene.can_edit);
  for (const item of state.scene.structures || []) {{ const row=document.createElement('div'); row.className='structure-row'; const detail=document.createElement('div'); const title=document.createElement('div'); title.textContent=text(item.label)+' '+text(item.hp)+'/'+text(item.max_hp); const meter=document.createElement('div'); meter.className='meter'; const fill=document.createElement('i'); fill.style.width=(100*clamp(Number(item.hp)/Math.max(1,Number(item.max_hp)),0,1))+'%'; meter.append(fill); detail.append(title,meter); const minus=document.createElement('button'); minus.type='button'; minus.textContent='-'; minus.disabled=!editable; minus.setAttribute('aria-label','Decrease '+text(item.label)+' health'); const plus=document.createElement('button'); plus.type='button'; plus.textContent='+'; plus.disabled=!editable; plus.setAttribute('aria-label','Increase '+text(item.label)+' health'); const step=item.kind==='base'?500:10; minus.addEventListener('click',()=>command({{command:'set_structure_health',side:item.side,structure:item.kind,hp:Math.max(0,Number(item.hp)-step)}})); plus.addEventListener('click',()=>command({{command:'set_structure_health',side:item.side,structure:item.kind,hp:Math.min(Number(item.max_hp),Number(item.hp)+step)}})); row.append(detail,minus,plus); structures.append(row); }}
}}
function renderSelected() {{
  const unit=(state.scene.units || []).find((item)=>item.entity_id===state.scene.selected_entity_id); selectedActions.textContent=''; if (!unit) {{ addKv(selected, [['State','No piece selected']]); return; }}
  addKv(selected, [['Piece',text(unit.side)+' '+text(unit.type)],['HP',text(unit.hp)+'/'+text(unit.max_hp)],['Position',text(unit.position_cm.x)+', '+text(unit.position_cm.y)],['ROS',text(unit.decision_summary,'catalog mapping')]]);
  const editable=Boolean(state.scene.can_edit); for (const [label,next] of [['-10',Math.max(0,Number(unit.hp)-10)],['+10',Math.min(Number(unit.max_hp),Number(unit.hp)+10)]]) {{ const button=document.createElement('button'); button.type='button'; button.textContent=label; button.disabled=!editable; button.addEventListener('click',()=>command({{command:'set_unit_hp',entity_id:unit.entity_id,hp:next}})); selectedActions.append(button); }} const remove=document.createElement('button'); remove.type='button'; remove.textContent='Remove'; remove.className='danger'; remove.disabled=!editable; remove.addEventListener('click',()=>command({{command:'remove_unit',entity_id:unit.entity_id}})); selectedActions.append(remove);
}}
function renderDecision() {{
  const decision=state.decision || {{}}; const goal=decision.goal || {{}}; addKv(document.getElementById('decision'), [['Strategy',decision.strategy],['Layer',decision.intent && decision.intent.layer],['Reason',decision.intent && decision.intent.reason],['Priority',decision.intent && decision.intent.priority],['Goal',text(goal.name)+' #'+text(goal.id)],['Position',goal.pos_cm ? text(goal.pos_cm[0])+', '+text(goal.pos_cm[1])+' cm' : '-']]);
  document.getElementById('goalSummary').textContent=goal.name ? ('Target: '+text(goal.name)+' / '+text(decision.intent && decision.intent.reason)) : 'Waiting for decision trace'; setPill('decisionPill', goal.name ? ('goal '+text(goal.id)) : 'goal -', goal.name ? 'good' : 'warn');
  if (Array.isArray(goal.pos_cm) && goal.pos_cm.length>=2) {{ const point={{x:Number(goal.pos_cm[0]),y:Number(goal.pos_cm[1])}}; setPosition(goalMarker,point); goalMarker.hidden=false; const p=pct(point); goalLabel.style.left=p.left+'%'; goalLabel.style.top='calc('+p.top+'% - 34px)'; goalLabel.textContent=text(goal.name)+' #'+text(goal.id); goalLabel.hidden=false; }} else {{ goalMarker.hidden=true; goalLabel.hidden=true; }}
  const f=state.field; const points=(decision.route_cm || []).map((p)=>Array.isArray(p)&&p.length>=2?{{x:Number(p[0]),y:Number(p[1])}}:null).filter(Boolean).map((p)=>{{const q=pct(p); return q.left+','+q.top;}}); routeLine.setAttribute('points',points.join(' '));
}}
function renderControl() {{ const output=state.control_output || {{}}; const fire=output.fire_code || {{}}; const trajectory=output.trajectory || {{}}; addKv(document.getElementById('control'), [['Sequence',output.sequence],['Source',output.source],['Angles',output.angles && output.angles.published ? 'yaw '+numeric(output.angles.yaw)+' pitch '+numeric(output.angles.pitch) : 'not published'],['Fire',fire.published ? 'follow '+text(fire.follow_mode)+' rotate '+text(fire.rotate) : 'not published'],['Trajectory',trajectory.available ? 'yaw '+numeric(trajectory.yaw)+' pitch '+numeric(trajectory.pitch) : text(trajectory.unavailable_reason,'unavailable')]]); const feedback=state.gimbal_feedback || {{}}; addKv(document.getElementById('feedback'), [['Available',feedback.available],['Age',feedback.age_ms===null||feedback.age_ms===undefined?'-':text(feedback.age_ms)+' ms'],['Rotate',feedback.fire_code && feedback.fire_code.rotate]]); setPill('controlPill', output.available ? ('control '+text(output.sequence)) : 'control -', output.available ? 'good' : 'warn'); }}
function renderTactical() {{ const tactical=state.tactical || {{}}; const castle=tactical.protect_castle || {{}}; const hero=tactical.protect_hero || {{}}; const defense=tactical.regional_defense || {{}}; addKv(document.getElementById('tactical'), [['Evidence',tactical.available ? 'BT trace' : 'not recorded'],['Castle',castle.enabled],['RFID source',castle.rfid_event_active],['Enemy in base',castle.enemy_pos_active],['Hero protection',hero.active],['Defense threat',defense.threat_active],['Defense search',defense.search_kind],['Base enemies',defense.own_base_enemy_count]]); }}
function renderOwner() {{ const owner=state.scene.ownership_mode; const readonly=owner==='manual_ros'||!state.scene.can_edit; setPill('ownerPill',owner,readonly?'warn':'good'); const message=document.getElementById('ownerMessage'); message.hidden=!readonly; message.textContent=readonly?'Manual ROS observer mode: Foxglove or ROS CLI owns scene facts.':''; for (const button of document.querySelectorAll('#matchActions button')) button.disabled=readonly; }}
function render() {{ if (!state) return; mapImage.src=state.stream.map_url+'?frame='+encodeURIComponent(text(state.stream.frame_id,'0')); renderOwner(); renderPalette(); renderPieces(); renderStructures(); renderSelected(); renderDecision(); renderTactical(); renderControl(); }}
function refresh() {{ return fetch('/api/tactical-state',{{cache:'no-store'}}).then((response)=>response.json()).then((next)=>{{ state=next; render(); }}).catch(()=>announce('Tactical state unavailable')); }}
board.addEventListener('pointerdown',(event)=>{{ if (!state || !state.scene.can_edit || !paletteChoice || event.target.closest('.piece')) return; placementPointer={{pointerId:event.pointerId}}; }});
board.addEventListener('pointermove',(event)=>{{ if (!state) return; const point=readPoint(event); document.getElementById('coordinateReadout').textContent=point.x+', '+point.y+' cm'; if (drag) setPosition(drag.element,point); }});
board.addEventListener('pointerup',(event)=>{{
  if (drag && event.pointerId===drag.pointerId) {{
    const moved=drag; drag=null; moved.element.classList.remove('dragging'); const point=readPoint(event);
    command({{command:'place_unit',entity_id:moved.unit.entity_id,side:moved.unit.side,unit_key:moved.unit.unit_key,hp:moved.unit.hp,x:point.x,y:point.y}});
    return;
  }}
  if (!placementPointer || event.pointerId!==placementPointer.pointerId) return;
  placementPointer=null; const point=readPoint(event);
  command({{command:'place_unit',entity_id:unitId(paletteChoice),side:paletteChoice.side,unit_key:paletteChoice.unit_key,hp:paletteChoice.hp,x:point.x,y:point.y}});
}});
board.addEventListener('pointercancel',()=>{{ placementPointer=null; if (drag) {{ drag.element.classList.remove('dragging'); drag=null; }} }});
for (const button of document.querySelectorAll('#matchActions button')) button.addEventListener('click',()=>command({{command:button.dataset.command}}));
refresh(); setInterval(refresh,500);
</script>
</body>
</html>"""
    return body.encode("utf-8")


def safe_html_label(value: str) -> str:
    return html.escape(value, quote=True)
