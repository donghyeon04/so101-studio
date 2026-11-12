"""안전 페이지 (gui-spec 3절): 모드(끔은 확인 창과 빨간 배지), 관절 여유, XYZ 허용 상자(3D 표시), 항상 켜진 검사."""

from nicegui import run, ui

from ..backend import JOINT_TEXT
from ..state import SAFETY_TEXT
from .common import Controls, meta_number, scene_frame

AXES = ("x", "y", "z")


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id)
    current = backend.get_safety()
    box = current["box_mm"] or {}
    fields: dict = {}
    metas: list = []  # 여유 칸 + 상자 칸 여섯: 적용·모드 토글·끄기 확인이 모두 읽는다 (12.7.4)
    state = {"shown_mode": current["mode"]}

    def fields_ok() -> bool:
        """허용 밖 칸이 있으면 알리고 토글을 지금 적용된 모드로 되돌린다."""
        bad = next((m for m in metas if not m.valid), None)
        if bad is None:
            return True
        ui.notify(f"{bad.name}: 허용 밖이라 적용하지 않습니다", type="warning")
        mode_toggle.value = state["shown_mode"]
        return False

    def read_box() -> dict | None:
        """비어 있는 칸은 기본 범위를 쓴다. 다 비어 있으면 None."""
        out = {}
        for axis in AXES:
            lo, hi = fields[axis, 0].value, fields[axis, 1].value
            if lo is not None or hi is not None:
                out[axis] = [lo, hi]
        return out or None

    async def apply(mode: str | None = None) -> None:
        if not fields_ok():
            return
        reason = await run.io_bound(backend.set_safety, mode or state["shown_mode"], float(margin.value), read_box())
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
        if not fields_ok():  # 거부되면 확인 창을 닫지 않는다
            return
        dialog.close()
        await apply("off")

    def cancel_off() -> None:
        dialog.close()
        mode_toggle.value = state["shown_mode"]

    def mode_changed(e) -> None:
        if e.value == state["shown_mode"]:
            return
        if e.value == "off":
            if fields_ok():  # 허용 밖 칸이 있으면 확인 창을 열지 않고 되돌린다
                dialog.open()
        else:
            ui.timer(0.01, lambda: apply(e.value), once=True)

    async def reset() -> None:
        for field in fields.values():
            field.set(None, quiet=True)
        margin.set(0, quiet=True)
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
        controls.add(ui.button("적용", on_click=controls.guard(apply, needs=metas)).props("no-caps").mark("safety-apply"), needs=metas)

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
                margin = meta_number("safety.joint_margin_deg", current["joint_margin_deg"], label="관절 여유", mark="safety-margin", add=controls.add)
                metas.append(margin)
                ui.label("양수는 한계를 안쪽으로 줄이고, 음수는 실제 가동 범위까지 넓힙니다").classes("page-body")
            with ui.card().classes("w-full gap-2"):
                ui.label("허용 상자 (mm)").classes("card-title")
                for axis in AXES:
                    lo, hi = box.get(axis, (None, None))
                    name = "높이" if axis == "z" else axis
                    for i, (text, value) in enumerate((("최소", lo), ("최대", hi))):
                        ui.label(f"{name} {text}").classes("stat-label")
                        fields[axis, i] = meta_number("safety.box.z" if axis == "z" else "safety.box.xy", value, label=f"상자 {name} {text}",
                                                      mark=f"box-{axis}-{'min' if i == 0 else 'max'}", add=controls.add)
                        metas.append(fields[axis, i])
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
