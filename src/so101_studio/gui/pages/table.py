"""작업대·좌표 페이지 (gui-spec 3절): 위에서 본 작업대, 팔 받침과 배출 구역 배치, 숫자 입력, 저장. 저장하면 팔·평면 그림·3D에 바로 적용된다(REQ-034)."""

from nicegui import run, ui

from ...core.frames import Pose2D
from ..sensors.manual_view import plane_svg
from .common import Controls, nudge_pad


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id)
    ws = backend.arm.ws
    zone = next(iter(ws.drop_zones))
    values = {
        "table_x": ws.table_mm["x"], "table_y": ws.table_mm["y"],
        "base_x": ws.base.x_mm, "base_y": ws.base.y_mm, "base_yaw": ws.base.yaw_deg,
        "zone_x": ws.drop_zones[zone]["x_mm"], "zone_y": ws.drop_zones[zone]["y_mm"],
        "margin": ws.safety_margin_mm,
    }
    fields: dict = {}

    def redraw() -> None:
        table = {"x": values["table_x"], "y": values["table_y"]}
        plane.content = plane_svg(
            table, [], {zone: {**ws.drop_zones[zone], "x_mm": values["zone_x"], "y_mm": values["zone_y"]}},
            Pose2D(values["base_x"], values["base_y"], values["base_yaw"]), ws.reach_mm,
        )
        plane._props["size"] = [table["x"], table["y"]]
        plane.update()

    def number(label: str, key: str, step: float = 5) -> None:
        def changed(e) -> None:
            if e.value is not None:
                values[key] = float(e.value)
                redraw()

        fields[key] = controls.add(
            ui.number(label, value=values[key], step=step, on_change=changed).props("outlined dense").classes("flex-1").mark(f"ws-{key}")
        )

    async def plane_clicked(e) -> None:
        await controls.guard(lambda: _place(e))()

    async def _place(e) -> None:
        x, y = round(e.image_x), round(values["table_y"] - e.image_y)
        prefix = "base" if target.value == "base" else "zone"
        fields[prefix + "_x"].value, fields[prefix + "_y"].value = x, y

    async def save() -> None:
        reason = await run.io_bound(backend.save_workspace, dict(values), zone)
        ui.notify(reason or "저장했습니다. 팔과 3D에 바로 적용했습니다", type="negative" if reason else "positive")

    with actions:
        ui.badge("실측 전 값", color="warning").classes("page-state").mark("unmeasured")
        controls.add(ui.button("저장", on_click=controls.guard(save)).props("no-caps").mark("ws-save"))

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-2"):
            plane = ui.interactive_image(
                size=(values["table_x"], values["table_y"]), on_mouse=plane_clicked, events=["click"], cross=True, sanitize=False
            ).classes("w-full plane plane-large").mark("ws-plane")
            ui.label("작업대를 클릭하면 고른 것이 그 자리로 옮겨집니다").classes("page-body")
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("옮길 것 (클릭이나 패드로)").classes("card-title")
                target = controls.add(
                    ui.toggle({"base": "팔 받침", "zone": "배출 구역"}, value="zone").props("no-caps spread").classes("w-full").mark("ws-target")
                )
                def nudge(dx: float, dy: float, dangle: float) -> None:
                    if not controls.allowed:
                        return
                    prefix = "base" if target.value == "base" else "zone"
                    fields[prefix + "_x"].value = values[prefix + "_x"] + dx
                    fields[prefix + "_y"].value = values[prefix + "_y"] + dy
                    if prefix == "base" and dangle:
                        fields["base_yaw"].value = (values["base_yaw"] + dangle) % 360

                nudge_pad(nudge, add=controls.add)
                ui.label("누를 때마다 5mm·5°, 누르고 있으면 계속. 돌리기는 팔 받침만. 저장은 따로 누릅니다").classes("page-body")
            with ui.card().classes("w-full gap-2"):
                ui.label("작업대 (mm)").classes("card-title")
                with ui.row().classes("w-full no-wrap gap-2"):
                    number("가로", "table_x", 10)
                    number("세로", "table_y", 10)
                number("가장자리 여유", "margin", 1)
            with ui.card().classes("w-full gap-2"):
                ui.label("팔 받침").classes("card-title")
                with ui.row().classes("w-full no-wrap gap-2"):
                    number("x", "base_x")
                    number("y", "base_y")
                    number("방향 (°)", "base_yaw", 15)
            with ui.card().classes("w-full gap-2"):
                ui.label(f"배출 구역 ({zone})").classes("card-title")
                with ui.row().classes("w-full no-wrap gap-2"):
                    number("x", "zone_x")
                    number("y", "zone_y")
            ui.label("저장하면 팔과 3D에 바로 적용됩니다").classes("page-body")

    redraw()
    ui.timer(0.5, controls.refresh)
