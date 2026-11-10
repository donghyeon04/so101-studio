"""장치 > 팔 페이지 (gui-spec 3절): 점검 카드와 팔 파일(decisions/0027), 서보 6개 표, 전압과 전류, 진단 버튼(큰 동작은 확인 창)."""

import json
import threading
import time

from nicegui import run, ui

from .. import theme
from ..backend import BIG_MOVES, DIAGNOSTICS, JOINT_TEXT
from ..health import EVENT_KINDS, LABELS, LEVEL_TEXT, LIMITS
from .common import Controls

COLUMNS = [
    {"name": "joint", "label": "서보", "field": "joint", "align": "left"},
    {"name": "position", "label": "위치", "field": "position"},
    {"name": "temperature", "label": "온도 °C (최고)", "field": "temperature"},
    {"name": "load", "label": "부하 % (최고)", "field": "load"},
    {"name": "state", "label": "상태", "field": "state", "align": "left"},
]
CHARTS = {"temperature": "온도", "load": "부하", "power": "전압·전류"}


def servo_rows(status, health=None) -> list[dict]:
    """서보 6개 표. health가 있으면 이번 실행 최고값과 3초 이어진 주의·오류 판정을 함께 보인다 (결정 0043-5)."""
    if status is None:
        return []
    joints = vars(status.joints)
    rows = []
    for i, (name, text) in enumerate(JOINT_TEXT.items()):
        temp, load = status.temperature_c.get(name, 0), status.load_pct.get(name, 0)
        row = {"joint": f"{i + 1}. {text}", "position": f"{joints[name]:.1f}", "temperature": f"{temp:.0f}", "load": f"{load:.1f}", "state": "오류" if status.fault else "정상"}
        if health is not None:
            row["temperature"] += f" ({health.peak.get(f'temperature:{name}', temp):.0f})"
            row["load"] += f" ({health.peak.get(f'load:{name}', load):.0f})"
            if not status.fault:
                levels = {m: health.level(f"{m}:{name}") for m in ("temperature", "load")}
                worst = "error" if "error" in levels.values() else "warn" if "warn" in levels.values() else "ok"
                what = [LABELS[m] for m, lv in levels.items() if lv != "ok"]
                row["state"] = LEVEL_TEXT[worst] + (f": {', '.join(what)}" if what else "")
        rows.append(row)
    return rows


def chart_options(kind: str, series: dict) -> dict:
    """최근 5분 추이. 관절별 계열은 관절 이름으로, 전원은 전압과 전류 두 축으로."""
    axis = {"axisLine": {"lineStyle": {"color": theme.CONTROL_BORDER}}, "axisLabel": {"color": theme.MUTED},
            "splitLine": {"lineStyle": {"color": theme.LINE}}, "nameTextStyle": {"color": theme.MUTED}}
    t = series["t"]
    if kind == "power":
        y = [{"type": "value", "name": "V", "scale": True, **axis}, {"type": "value", "name": "A", "min": 0, **axis, "splitLine": {"show": False}}]
        lines = [("전압", series["voltage"], 0, theme.SERIES[0]), ("전류 합", series["current"], 1, theme.SERIES[1])]
    else:
        y = [{"type": "value", "name": "°C" if kind == "temperature" else "%", "scale": True, **axis}]
        lines = [(JOINT_TEXT.get(j, j), v, 0, theme.SERIES[i % len(theme.SERIES)]) for i, (j, v) in enumerate(series[kind].items())]
    return {
        "animation": False,
        "tooltip": {"trigger": "axis"},
        "legend": {"top": 0, "textStyle": {"color": theme.TEXT}},
        "grid": {"left": 48, "right": 48, "top": 36, "bottom": 28},
        "xAxis": {"type": "value", "name": "초", "min": -300, "max": 0, **axis},
        "yAxis": y,
        "series": [{"name": name, "type": "line", "showSymbol": False, "lineStyle": {"width": 2}, "color": color, "yAxisIndex": ax,
                    "data": [[a, b] for a, b in zip(t, values)]} for name, values, ax, color in lines],
    }


def _health_card(backend) -> None:
    """요약(가장 나쁜 판정과 항목), 이벤트 횟수, 추이 그래프 (결정 0043-5)."""
    health = backend.health
    with ui.card().classes("w-full gap-2").mark("health-card"):
        with ui.row().classes("w-full items-center"):
            ui.label("서보 건강").classes("card-title")
            ui.space()
            pick = ui.toggle(CHARTS, value="temperature").props("no-caps dense").mark("health-chart-pick")
        with ui.row().classes("items-center gap-1 no-wrap"):
            icon = ui.icon("check_circle", color="positive").mark("health-icon")
            summary = ui.label("").classes("page-body").mark("health-summary")
        limits = " · ".join(f"{LABELS[m]} {'≥' if hi else '<'} {warn:g}{unit}" for m, (unit, warn, _, hi) in LIMITS.items())
        ui.label(f"주의 기준(3초 이어질 때): {limits}. 오류 기준에 닿으면 팔이 멈춥니다").classes("page-body")
        events = ui.label("").classes("page-body").mark("health-events")
        chart = ui.echart(chart_options(pick.value, health.series())).classes("w-full run-chart").mark("health-chart")

    def refresh() -> None:
        level, texts = health.worst()
        icon_name, color, text = theme.COMPONENT_STYLES[level]
        icon.name = icon_name
        icon.props(f"color={color}")
        summary.text = f"{text}: {', '.join(texts)}" if texts else "모든 값이 주의 기준 안입니다"
        counts = backend.health_events()
        events.text = "이번 실행: " + " · ".join(f"{EVENT_KINDS[k]} {n}회" + (f"(마지막 {_clock(t)}{', ' + why if why else ''})" if n else "") for k, (n, t, why) in counts.items())
        chart.options.clear()
        chart.options.update(chart_options(pick.value, health.series()))
        chart.update()

    pick.on_value_change(lambda _: refresh())
    refresh()
    ui.timer(2.0, refresh)


def _clock(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t))


GRIPPER_TEXT = {"single_jaw": "단동식(한쪽 턱만 움직임, 기본)", "parallel": "병렬(두 턱이 대칭으로 움직임)"}


def gripper_text(info: dict) -> str:
    """지금 그리퍼 한 줄."""
    return f"그리퍼: {GRIPPER_TEXT.get(info['type'], info['type'])} · 최대 벌림 {info['max_open_mm']:g} mm"


# 움직이는 명령은 사람이 직접 실행한다 (decisions/0027)
ZERO_CHECK_STEP = "so101 calibrate --verify-zero로 0° 자세를 확인합니다 (팔이 움직입니다)"
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
        zero = report["zero_check"]
        zero_icon.name, color, _ = theme.DOCTOR_STYLES["ok" if zero["state"] == "checked" else "warn"]
        zero_icon.props(f"color={color}")
        zero_text.text = f"0° 자세 확인: 확인됨 ({zero['date']})" if zero["state"] == "checked" else f"0° 자세 확인: 필요 ({zero['reason']})"
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
        probe_note = "서보 응답과 전압은 앱을 시작할 때 읽은 값입니다. 지금 값은 아래 표에서 봅니다" if backend.real else "포트를 열지 않고 확인합니다. 서보 응답과 전압은 터미널에서 so101 doctor로 봅니다"
        ui.label(probe_note).classes("page-body")
        problems = ui.column().classes("w-full gap-1").mark("doctor-problems")
        with ui.expansion("전체 항목").classes("w-full").mark("doctor-all"):
            details = ui.column().classes("w-full gap-1").mark("doctor-details")
        ui.separator()
        ui.label(f"팔 파일: {info['arm'] or '없음'} ({info['reason']})").classes("page-body").mark("arm-file")
        with ui.row().classes("items-center gap-1 no-wrap"):  # decisions/0029
            zero_icon = ui.icon("schedule", color="grey-8").mark("zero-check-icon")
            zero_text = ui.label("0° 자세 확인: 준비 중").classes("page-body").mark("zero-check")
        if len(names) > 1:
            with ui.row().classes("items-center gap-2"):
                picker = ui.select(names, value=info["arm"] or names[0], label="팔").classes("w-64").mark("arm-picker")
                controls.add(ui.button("팔 바꾸기", on_click=controls.guard(choose)).props("outline no-caps").mark("arm-change"))
        grip = backend.gripper_info()
        if grip:  # 결정 0043-1: 종류는 local.yaml에 적고 다시 시작하면 적용된다
            ui.label(gripper_text(grip)).classes("page-body").mark("gripper-type")
            with ui.row().classes("items-center gap-2"):
                grip_pick = ui.select({g: GRIPPER_TEXT.get(g, g) for g in grip["available"]}, value=grip["type"], label="그리퍼").classes("w-64").mark("gripper-picker")

                async def choose_gripper() -> None:
                    reason = backend.choose_gripper(grip_pick.value)
                    ui.notify(reason or f"{GRIPPER_TEXT.get(grip_pick.value, grip_pick.value)}: 다시 시작하면 적용됩니다", type="negative" if reason else "positive")

                controls.add(ui.button("그리퍼 바꾸기", on_click=controls.guard(choose_gripper)).props("outline no-caps").mark("gripper-change"))
        lock = backend.setup_lock()
        if lock and lock.startswith("0° 자세"):
            ui.label(f"터미널에서 {ZERO_CHECK_STEP}").classes("page-body").mark("zero-check-step")
        elif not info["arm"] or lock:
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
            table = ui.table(rows=servo_rows(backend.state.status, backend.health), columns=COLUMNS, row_key="joint").props("flat bordered dense hide-bottom").classes("w-full").mark("servo-table")
            _health_card(backend)
            with ui.card().classes("w-full gap-1"):
                ui.label("진단 출력").classes("card-title")
                output = ui.column().classes("w-full gap-0 diag-output").mark("diag-output")
                with output:
                    ui.label("오른쪽 진단 버튼을 누르면 결과가 여기에 나옵니다").classes("page-body")
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-1"):
                ui.label("전원").classes("card-title")
                power = ui.label("").classes("stat-value").mark("power")
            with ui.card().classes("w-full gap-2"):
                ui.label("진단").classes("card-title")
                buttons = {}
                for name, text in DIAGNOSTICS.items():
                    buttons[name] = ui.button(text, on_click=press(name)).props("outline no-caps").classes("w-full").mark(f"diag-{name}")
                note = "linkwatch는 팔 연결을 잠시 끊고 포트를 직접 엽니다. 토크는 유지됩니다. [진단 중단]으로 끝내면 다시 연결합니다" if backend.real else "linkwatch는 실제 팔의 포트가 필요해 시뮬레이션에서는 꺼져 있습니다"
                ui.label(note).classes("page-body")

    shown = [0]

    def tick() -> None:
        controls.refresh()
        status = backend.state.status
        table.rows = servo_rows(status, backend.health)
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
