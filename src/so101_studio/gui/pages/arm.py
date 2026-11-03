"""장치 > 팔 페이지 (gui-spec 3절): 서보 6개 표, 전압과 전류, 진단 버튼(큰 동작은 확인 창)."""

import json
import threading

from nicegui import run, ui

from ..backend import BIG_MOVES, DIAGNOSTICS, JOINT_TEXT
from .common import Controls

COLUMNS = [
    {"name": "joint", "label": "서보", "field": "joint", "align": "left"},
    {"name": "position", "label": "위치", "field": "position"},
    {"name": "temperature", "label": "온도 (°C)", "field": "temperature"},
    {"name": "load", "label": "부하 (%)", "field": "load"},
    {"name": "state", "label": "응답", "field": "state", "align": "left"},
]


def servo_rows(status) -> list[dict]:
    if status is None:
        return []
    joints = vars(status.joints)
    state = "오류" if status.fault else "정상"
    return [
        {"joint": f"{i + 1}. {text}", "position": f"{joints[name]:.1f}", "temperature": f"{status.temperature_c.get(name, 0):.0f}",
         "load": f"{status.load_pct.get(name, 0):.1f}", "state": state}
        for i, (name, text) in enumerate(JOINT_TEXT.items())
    ]


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id)
    lines: list[str] = []
    stop_flag = threading.Event()
    state: dict = {"running": None, "pending": None}

    async def run_diagnostic(name: str) -> None:
        lines.clear()
        stop_flag.clear()
        state["running"] = name
        try:
            result, reason = await run.io_bound(backend.run_diagnostic, name, lines.append, stop_flag.is_set)
        finally:
            state["running"] = None
        if reason:
            lines.append(reason)
            ui.notify(reason, type="negative")
            return
        lines.append(json.dumps(result, ensure_ascii=False, default=str)[:4000])
        ui.notify("진단을 마쳤습니다", type="positive")

    def press(name: str):
        async def pressed() -> None:
            if name in BIG_MOVES:
                state["pending"] = name
                confirm_text.text = f"{DIAGNOSTICS[name]}: 팔을 크게 움직입니다. 주변을 비우고 전원 스위치 옆에서 지켜보세요."
                dialog.open()
            else:
                await run_diagnostic(name)

        return controls.guard(pressed)

    async def confirmed() -> None:
        dialog.close()
        await run_diagnostic(state["pending"])

    with ui.dialog() as dialog, ui.card().classes("gap-3"):
        ui.label("큰 동작을 시작할까요?").classes("card-title")
        confirm_text = ui.label("").classes("page-body").mark("diag-confirm-text")
        with ui.row().classes("w-full justify-end gap-2"):
            ui.button("취소", on_click=dialog.close).props("outline no-caps").mark("diag-cancel")
            ui.button("시작", color="negative", on_click=controls.guard(confirmed)).props("no-caps").mark("diag-confirm")
    dialog.props("persistent")

    with actions:
        ui.button("진단 중단", color="negative", on_click=stop_flag.set).props("outline no-caps").mark("diag-stop")

    # REQ-015 임시: doctor가 오면 정상/주의/실패, 고른 팔 파일과 이유, 팔 바꾸기로 채운다
    with ui.card().classes("w-full gap-1").mark("doctor-card"):
        ui.label("설치·연결 점검").classes("card-title")
        with ui.row().classes("items-center gap-1 no-wrap"):
            ui.icon("schedule", color="grey-8")
            ui.label("준비 중").classes("page-body").mark("doctor-state")
        ui.label("점검 기능이 들어오면 환경, 포트, 팔 파일, 캘리브레이션, 서보 응답 결과를 여기에 보여 줍니다").classes("page-body")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-3"):
            table = ui.table(rows=servo_rows(backend.state.status), columns=COLUMNS, row_key="joint").props("flat bordered dense hide-bottom").classes("w-full").mark("servo-table")
            with ui.card().classes("w-full gap-1"):
                ui.label("진단 출력").classes("card-title")
                output = ui.column().classes("w-full gap-0 diag-output").mark("diag-output")
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-1"):
                ui.label("전원").classes("card-title")
                power = ui.label("").classes("stat-value").mark("power")
            with ui.card().classes("w-full gap-2"):
                ui.label("진단").classes("card-title")
                buttons = {}
                for name, text in DIAGNOSTICS.items():
                    buttons[name] = ui.button(text, on_click=press(name)).props("outline no-caps").classes("w-full").mark(f"diag-{name}")
                ui.label("linkwatch는 실제 팔의 포트가 필요해 시뮬레이션에서는 꺼져 있습니다").classes("page-body")

    shown = [0]

    def tick() -> None:
        controls.refresh()
        status = backend.state.status
        table.rows = servo_rows(status)
        if status:
            power.text = f"{status.voltage_v:.1f} V · {status.current_ma / 1000:.2f} A"
        idle = bool(status and status.state == "idle") and state["running"] is None
        for name, button in buttons.items():
            button.enabled = controls.allowed and idle and backend.diagnostic_available(name)
        if len(lines) != shown[0]:
            shown[0] = len(lines)
            output.clear()
            with output:
                for line in lines[-30:]:
                    ui.label(line).classes("event-line")

    ui.timer(0.5, tick)
