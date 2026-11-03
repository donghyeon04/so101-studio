"""장치 > 팔 페이지 (gui-spec 3절): 점검 카드와 팔 파일(decisions/0027), 서보 6개 표, 전압과 전류, 진단 버튼(큰 동작은 확인 창)."""

import json
import threading

from nicegui import run, ui

from .. import theme
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


# 움직이는 명령은 사람이 직접 실행한다 (decisions/0027)
NEW_ARM_STEPS = (
    "1. lerobot-setup-motors로 서보 ID를 정합니다 (모터에 씁니다)",
    "2. lerobot-calibrate --robot.id=<id>로 캘리브레이션합니다 (팔이 움직입니다)",
    "3. so101 doctor로 보드 시리얼을 확인합니다",
    "4. config/arms/_template.yaml을 복사해 값을 채웁니다",
)


def doctor_summary(report: dict) -> tuple[str, str]:
    """(가장 나쁜 상태, 요약 글). 건너뛴 항목은 문제가 아니다."""
    counts = {state: sum(c["state"] == state for c in report["checks"]) for state in theme.DOCTOR_STYLES}
    worst = "fail" if counts["fail"] else "warn" if counts["warn"] else "ok"
    return worst, " · ".join(f"{theme.DOCTOR_STYLES[s][2]} {n}" for s, n in counts.items() if n)


def _check_row(check: dict) -> None:
    icon, color, text = theme.DOCTOR_STYLES[check["state"]]
    with ui.row().classes("w-full items-start gap-1 no-wrap"):
        ui.icon(icon, color=color)
        with ui.column().classes("gap-0 min-w-0"):
            ui.label(f"{text} · {check['name']}: {check['text']}").classes("page-body").style("white-space: pre-line")
            if check["fix"] and check["state"] != "ok":
                ui.label(f"고치기: {check['fix']}").classes("page-body")


def _doctor_card(backend, controls: Controls) -> None:
    async def check() -> None:
        state_text.text = "점검 중"
        report = await run.io_bound(backend.run_doctor)
        worst, summary = doctor_summary(report)
        state_icon.name, color, _ = theme.DOCTOR_STYLES[worst]
        state_icon.props(f"color={color}")
        state_text.text = summary
        problems.clear()
        details.clear()
        with problems:
            for item in report["checks"]:
                if item["state"] in ("warn", "fail"):
                    _check_row(item)
        with details:
            for item in report["checks"]:
                _check_row(item)

    async def choose() -> None:
        reason = backend.choose_arm(picker.value)
        ui.notify(reason or f"{picker.value}: 다시 시작하면 적용됩니다", type="negative" if reason else "positive")

    info, names = backend.arm_info, backend.arm_names()
    with ui.card().classes("w-full gap-2").mark("doctor-card"):
        with ui.row().classes("w-full items-center"):
            ui.label("설치·연결 점검").classes("card-title")
            ui.space()
            ui.button("다시 점검", on_click=check).props("outline no-caps").mark("doctor-run")
        with ui.row().classes("items-center gap-1 no-wrap"):
            state_icon = ui.icon("schedule", color="grey-8").mark("doctor-icon")
            state_text = ui.label("점검 중").classes("page-body").mark("doctor-state")
        ui.label("포트를 열지 않고 확인합니다. 서보 응답과 전압은 터미널에서 so101 doctor로 봅니다").classes("page-body")
        problems = ui.column().classes("w-full gap-1").mark("doctor-problems")
        with ui.expansion("전체 항목").classes("w-full").mark("doctor-all"):
            details = ui.column().classes("w-full gap-1").mark("doctor-details")
        ui.separator()
        ui.label(f"팔 파일: {info['arm'] or '없음'} ({info['reason']})").classes("page-body").mark("arm-file")
        if len(names) > 1:
            with ui.row().classes("items-center gap-2"):
                picker = ui.select(names, value=info["arm"] or names[0], label="팔").classes("w-64").mark("arm-picker")
                controls.add(ui.button("팔 바꾸기", on_click=controls.guard(choose)).props("outline no-caps").mark("arm-change"))
        if not info["arm"] or backend.move_lock():
            with ui.column().classes("gap-0").mark("new-arm-steps"):
                ui.label("새 팔이면 이 순서로 준비합니다").classes("card-title")
                for step in NEW_ARM_STEPS:
                    ui.label(step).classes("page-body")
    ui.timer(0.1, check, once=True)


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

    _doctor_card(backend, controls)

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
            button.enabled = controls.allowed and idle and backend.diagnostic_available(name) and not (name in BIG_MOVES and backend.move_lock())
        if len(lines) != shown[0]:
            shown[0] = len(lines)
            output.clear()
            with output:
                for line in lines[-30:]:
                    ui.label(line).classes("event-line")

    ui.timer(0.5, tick)
