"""수동 입력 화면: 작업대 평면에서 클릭으로 물체를 추가·이동하고, 표로 본다 (gui-spec 3절). 작업 공간용 작은 평면도 여기 있다."""

import html

from nicegui import run, ui

from ...core.config import WorkspaceConfig
from .. import theme
from ..backend import MOTION_TEXT
from ..pages.common import HEIGHT_HINT, Controls, nudge_pad


def plane_svg(table_mm: dict, objects, zones: dict, base, reach_mm: dict, selected: str | None = None) -> str:
    """위에서 본 작업대. SVG 좌표는 mm이고 y만 뒤집는다(작업대는 y가 위로 간다)."""
    w, h = table_mm["x"], table_mm["y"]
    parts = [f'<rect x="0" y="0" width="{w}" height="{h}" fill="{theme.CARD}" stroke="{theme.CONTROL_BORDER}" stroke-width="2"/>']
    for x in range(100, int(w), 100):
        parts.append(f'<line x1="{x}" y1="0" x2="{x}" y2="{h}" stroke="{theme.LINE}"/>')
    for y in range(100, int(h), 100):
        parts.append(f'<line x1="0" y1="{y}" x2="{w}" y2="{y}" stroke="{theme.LINE}"/>')
    bx, by = base.x_mm, h - base.y_mm
    for r in (reach_mm["min"], reach_mm["max"]):
        parts.append(f'<circle cx="{bx}" cy="{by}" r="{r}" fill="none" stroke="{theme.MUTED}" stroke-dasharray="8 6"/>')
    parts.append(f'<circle cx="{bx}" cy="{by}" r="10" fill="{theme.MUTED}"/>')
    parts.append(f'<text x="{bx + 16}" y="{by + 6}" font-size="18" fill="{theme.MUTED}">팔</text>')
    ws = WorkspaceConfig(table_mm, {"x_mm": base.x_mm, "y_mm": base.y_mm, "yaw_deg": base.yaw_deg}, zones, reach_mm=reach_mm)
    for name, zone in zones.items():
        x, y = zone["x_mm"], h - zone["y_mm"]
        zw, zh = zone.get("w_mm", 80), zone.get("h_mm", 80)  # 옛 점 형식은 80x80 표시(11.3.5)
        parts.append(f'<rect x="{x - zw / 2}" y="{y - zh / 2}" width="{zw}" height="{zh}" fill="{theme.OK}" fill-opacity="0.2" stroke="{theme.OK}"/>')
        parts.append(f'<text x="{x - zw / 2}" y="{y - zh / 2 - 6}" font-size="16" fill="{theme.OK}">배출</text>')
        for i, (sx, sy) in enumerate(ws.zone_slots(name) or []):  # 팔이 놓는 칸(가까운 순 번호)
            parts.append(f'<circle cx="{sx}" cy="{h - sy}" r="6" fill="{theme.OK}"/><text x="{sx + 8}" y="{h - sy + 5}" font-size="13" fill="{theme.OK}">{i + 1}</text>')
    for o in objects:
        x, y = o.x_mm, h - o.y_mm
        color = theme.ACCENT if o.id == selected else theme.MUTED
        angle = -(o.angle_deg or 0.0)  # SVG 회전은 시계 방향이다
        parts.append(
            f'<g transform="translate({x:.1f} {y:.1f}) rotate({angle:.1f})">'
            f'<rect x="{-o.length_mm / 2:.1f}" y="{-o.width_mm / 2:.1f}" width="{o.length_mm}" height="{o.width_mm}" fill="{color}" fill-opacity="0.75" stroke="{color}"/></g>'
        )
        parts.append(f'<text x="{x:.1f}" y="{y - o.width_mm / 2 - 8:.1f}" text-anchor="middle" font-size="18" fill="{theme.TEXT}">{html.escape(o.id)}</text>')
    return "".join(parts)


def reachable_text(backend, o) -> str:
    return "닿음" if backend.object_reachable(o) else "안 닿음"


def _plane(backend, on_mouse=None, selected=lambda: None, classes: str = "w-full plane") -> ui.interactive_image:
    table = backend.arm.ws.table_mm
    image = ui.interactive_image(
        size=(table["x"], table["y"]), on_mouse=on_mouse, events=["click"] if on_mouse else [], cross=bool(on_mouse), sanitize=False
    ).classes(classes).mark("plane")

    def redraw() -> None:
        image.content = plane_svg(table, backend.objects(), backend.arm.ws.drop_zones, backend.arm.ws.base, backend.arm.ws.reach_mm, selected())

    redraw()
    ui.timer(0.5, redraw)
    return image


def mini_view(backend) -> None:
    _plane(backend, selected=lambda: backend.state.chosen)


def page(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id, block_during_task=False)  # 실행 중에도 물체를 더할 수 있다
    table_h = backend.arm.ws.table_mm["y"]
    state: dict = {"selected": None}

    columns = [
        {"name": "id", "label": "물체", "field": "id", "align": "left"},
        {"name": "x", "label": "x (mm)", "field": "x"},
        {"name": "y", "label": "y (mm)", "field": "y"},
        {"name": "angle", "label": "각도 (°)", "field": "angle"},
        {"name": "width", "label": "폭 (mm)", "field": "width"},
        {"name": "length", "label": "길이 (mm)", "field": "length"},
        {"name": "height", "label": "높이 (mm)", "field": "height"},
        {"name": "reach", "label": "닿는지", "field": "reach", "align": "left"},
    ]

    def rows() -> list[dict]:
        return [
            {"id": o.id, "x": round(o.x_mm), "y": round(o.y_mm), "angle": round(o.angle_deg or 0), "width": round(o.width_mm),
             "length": round(o.length_mm), "height": "높이 필요" if o.id in needed else (round(o.height_mm) or "-"), "reach": reachable_text(backend, o)}
            for needed in [backend.height_needed()] for o in backend.objects()
        ]

    def refresh_table() -> None:
        table.rows = rows()
        kept = [r for r in table.rows if r["id"] == state["selected"]]
        table.selected = kept
        remove_button.enabled = bool(controls.allowed and kept)

    def selected_changed(e) -> None:
        state["selected"] = e.selection[0]["id"] if e.selection else None
        refresh_table()

    async def clicked(e) -> None:
        x, y = round(e.image_x, 1), round(table_h - e.image_y, 1)
        if mode.value == "move" and state["selected"]:
            backend.move_object(state["selected"], x, y)
        else:
            state["selected"] = backend.add_object(x, y, float(angle.value or 0), float(width.value or 30), float(length.value or 100), float(height.value or 0))
        refresh_table()

    async def plane_clicked(e) -> None:  # guard는 이벤트 인자를 넘기지 않는다
        await controls.guard(lambda: clicked(e))()

    async def apply_height() -> None:
        if state["selected"] and (height.value or 0) > 0:
            backend.set_object_height(state["selected"], float(height.value))
            refresh_table()

    async def remove() -> None:
        if state["selected"]:
            backend.remove_object(state["selected"])
            state["selected"] = None
            refresh_table()

    async def reset() -> None:
        await run.io_bound(backend.reset_objects)
        state["selected"] = None
        refresh_table()

    with actions:
        controls.add(ui.button("처음 물체로", on_click=controls.guard(reset)).props("outline no-caps").mark("reset-objects"))
        remove_button = ui.button("지우기", on_click=controls.guard(remove)).props("outline no-caps").mark("remove-object")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-2"):
            _plane(backend, on_mouse=plane_clicked, selected=lambda: state["selected"], classes="w-full plane plane-large")
            ui.label("작업대를 클릭해 물체를 놓습니다. 점선은 팔이 닿는 범위입니다").classes("page-body")
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("클릭하면").classes("card-title")
                mode = controls.add(
                    ui.toggle({"add": "새 물체", "move": "고른 물체 옮기기"}, value="add").props("no-caps spread").classes("w-full").mark("plane-mode")
                )
                with ui.row().classes("w-full no-wrap gap-2"):
                    angle = controls.add(ui.number("각도 (°)", value=0, min=0, max=180, step=15).props("outlined dense").classes("flex-1").mark("new-angle"))
                    width = controls.add(ui.number("폭 (mm)", value=30, min=5, max=110, step=5).props("outlined dense").classes("flex-1").mark("new-width"))
                    length = controls.add(ui.number("길이 (mm)", value=100, min=5, max=300, step=5).props("outlined dense").classes("flex-1").mark("new-length"))
                    height = controls.add(ui.number("높이 (mm)", value=0, min=0, max=200, step=5).props("outlined dense").classes("flex-1")
                                          .tooltip(HEIGHT_HINT).mark("new-height"))
                need_text = ui.label("").classes("page-body text-negative").mark("height-needed")
                apply_button = controls.add(ui.button("고른 물체에 이 높이", on_click=controls.guard(lambda: apply_height())).props("outline no-caps dense").mark("apply-height"))
            with ui.card().classes("w-full gap-2"):
                ui.label("고른 물체 조금씩").classes("card-title")

                def nudge(dx: float, dy: float, dangle: float) -> None:
                    if state["selected"] and controls.allowed:
                        backend.nudge_object(state["selected"], dx, dy, dangle)
                        refresh_table()

                nudge_pad(nudge, add=controls.add)
                ui.label("표에서 물체를 고르고 누르세요. 누르고 있으면 계속 움직입니다").classes("page-body")
            if backend.task_running():
                ui.label("작업 실행 중에는 다음 스캔부터 반영됩니다").classes("page-body")

    table = ui.table(rows=rows(), columns=columns, row_key="id", selection="single", on_select=selected_changed).props('flat bordered dense no-data-label="물체가 없습니다. 위 작업대를 클릭해 놓으세요"').classes("w-full").mark("object-table")

    def tick() -> None:
        controls.refresh()
        table.rows = rows()
        remove_button.enabled = bool(controls.allowed and state["selected"])
        apply_button.enabled = bool(controls.allowed and state["selected"] and (height.value or 0) > 0)
        needed = sorted(backend.height_needed())  # 결정 0052: 수직으로 집는데 높이를 몰라 집지 않은 물체 (REQ-057)
        need_text.text = f"{', '.join(needed)}: {MOTION_TEXT['height_required']}. 표에서 고르고 높이를 넣은 뒤 [고른 물체에 이 높이]" if needed else ""
        need_text.visible = bool(needed)
        height.props(add="error" if needed else "", remove="" if needed else "error")

    ui.timer(0.5, tick)


from . import register

register("manual", "수동 입력", "/input", page, mini_view)
