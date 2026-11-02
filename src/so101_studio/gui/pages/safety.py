"""안전 페이지 (gui-spec 3절): 모드(끔은 확인 창과 빨간 배지), 관절 여유, XYZ 허용 상자(3D 표시), 항상 켜진 검사."""

from nicegui import run, ui

from ..backend import JOINT_TEXT
from ..state import SAFETY_TEXT
from .common import Controls, scene_frame

AXES = ("x", "y", "z")


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id)
    current = backend.get_safety()
    box = current["box_mm"] or {}
    fields: dict = {}
    state = {"shown_mode": current["mode"]}

    def read_box() -> dict | None:
        """비어 있는 칸은 기본 범위를 쓴다. 다 비어 있으면 None."""
        out = {}
        for axis in AXES:
            lo, hi = fields[axis, 0].value, fields[axis, 1].value
            if lo is not None or hi is not None:
                out[axis] = [lo, hi]
        return out or None

    async def apply(mode: str | None = None) -> None:
        reason = await run.io_bound(backend.set_safety, mode or state["shown_mode"], float(margin.value or 0), read_box())
        if reason:
            ui.notify(reason, type="negative")
        else:
            ui.notify("안전 설정을 적용했습니다", type="positive")
        sync()

    def sync() -> None:
        now = backend.get_safety()
        state["shown_mode"] = now["mode"]
        mode_toggle.value = now["mode"]
        off_badge.visible = now["mode"] == "off"
        limits.clear()
        with limits:
            effective = now["effective_limits_deg"]
            if effective:
                for name, (lo, hi) in effective.items():
                    ui.label(f"{JOINT_TEXT[name]}: {lo:.0f}° ~ {hi:.0f}°").classes("event-line")
            else:
                ui.label("관절 한계 꺼짐").classes("event-line")

    async def confirm_off() -> None:
        dialog.close()
        await apply("off")

    def cancel_off() -> None:
        dialog.close()
        mode_toggle.value = state["shown_mode"]

    def mode_changed(e) -> None:
        if e.value == state["shown_mode"]:
            return
        if e.value == "off":
            dialog.open()
        else:
            ui.timer(0.01, lambda: apply(e.value), once=True)

    async def reset() -> None:
        for field in fields.values():
            field.value = None
        margin.value = 0
        await apply("normal")

    with ui.dialog() as dialog, ui.card().classes("gap-3"):
        ui.label("안전 검사를 끌까요?").classes("card-title")
        ui.label("관절 한계, 작업 영역, 바닥 검사가 모두 꺼집니다. 한 주기 이동량 한계와 과열·과부하 감시만 남습니다.").classes("page-body")
        with ui.row().classes("w-full justify-end gap-2"):
            ui.button("취소", on_click=cancel_off).props("outline no-caps").mark("off-cancel")
            ui.button("끄기", color="negative", on_click=controls.guard(confirm_off)).props("no-caps").mark("off-confirm")
    dialog.props("persistent")

    with actions:
        off_badge = ui.badge("안전 검사 꺼짐", color="negative").classes("page-state").mark("safety-off-badge")
        off_badge.visible = current["mode"] == "off"
        controls.add(ui.button("기본값으로", on_click=controls.guard(reset)).props("outline no-caps").mark("safety-reset"))
        controls.add(ui.button("적용", on_click=controls.guard(apply)).props("no-caps").mark("safety-apply"))

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-2"):
            scene_frame(backend)
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("모드").classes("card-title")
                mode_toggle = controls.add(
                    ui.toggle(SAFETY_TEXT, value=current["mode"], on_change=mode_changed).props("no-caps spread").classes("w-full").mark("safety-mode")
                )
                ui.label("보통: 관절 한계, 작업대, 바닥 · 자유: 작업대 x, y만 끔 · 끔: 모두 끔").classes("page-body")
            with ui.card().classes("w-full gap-2"):
                ui.label("관절 여유 (°)").classes("card-title")
                margin = controls.add(
                    ui.number(value=current["joint_margin_deg"], min=-15, max=30, step=1).props("outlined dense").classes("w-full").mark("safety-margin")
                )
                ui.label("양수는 한계를 안쪽으로 줄이고, 음수는 실제 가동 범위까지 넓힙니다").classes("page-body")
            with ui.card().classes("w-full gap-2"):
                ui.label("허용 상자 (mm)").classes("card-title")
                for axis in AXES:
                    with ui.row().classes("w-full no-wrap items-center gap-2"):
                        ui.label("높이" if axis == "z" else axis).classes("axis-label")
                        lo, hi = box.get(axis, (None, None))
                        fields[axis, 0] = controls.add(ui.number("최소", value=lo).props("outlined dense").classes("flex-1").mark(f"box-{axis}-min"))
                        fields[axis, 1] = controls.add(ui.number("최대", value=hi).props("outlined dense").classes("flex-1").mark(f"box-{axis}-max"))
                ui.label("비워 두면 기본 범위를 씁니다. 3D의 반투명 상자가 지금 적용되는 범위입니다").classes("page-body")
            with ui.card().classes("w-full gap-1"):
                ui.label("항상 켜진 검사").classes("card-title")
                ui.label(f"한 주기 이동량 {current['max_step_deg']:.0f}° 이하").classes("event-line")
                ui.label("과열 · 과부하 · 통신 끊김 감시").classes("event-line")
            with ui.card().classes("w-full gap-1"):
                ui.label("지금 관절 한계").classes("card-title")
                limits = ui.column().classes("gap-0 w-full").mark("safety-limits")

    sync()
    if backend.scene:
        backend.show_safety_box()
    ui.timer(0.5, controls.refresh)
