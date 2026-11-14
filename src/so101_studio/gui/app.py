"""NiceGUI 앱: 공통 틀(위쪽 상태줄, 사이드바, 아래쪽 상태줄, E-STOP)과 페이지 등록."""

import logging
import os
import sys
import time

from nicegui import app, background_tasks, run, ui

from ..cli import MUJOCO_MISSING_TEXT, make_driver
from ..core.config import ArmSelectionError, load_robot_ex
from ..core.ports import PortChoiceError, list_serial_ports, pick_port
from ..doctor import doctor
from . import port_check, scene3d, sensors, theme
from .backend import RUN_MODES, Backend
from .health import LEVEL_TEXT, describe
from .pages import (
    arm,
    calibration,
    esp32,
    logs,
    manual,
    operator,
    params,
    program,
    runs,
    safety,
    start,
    table,
    teach,
    workspace,
)
from .pages.common import Controls, jog_pads
from .param_meta import (
    META,  # 막대 슬라이더는 자리가 좁아 숫자 칸 없이 한계·눈금만 쓴다 (12.7.2)
)
from .share import LOCAL_HOST, SHARE_HOST, is_remote, share_warning
from .state import EMPTY, component_states, page_state
from .view_cube import VIEW_CUBE_JS
from .watchdog_link import PORT as WATCHDOG_PORT
from .watchdog_link import WatchdogLink, browser_hb_js

HOST = LOCAL_HOST  # NiceGUI 기본값 0.0.0.0은 같은 네트워크 어디서나 팔을 조작할 수 있게 한다 (decisions/0023). --share일 때만 0.0.0.0(결정 0047)
DEFAULT_PORT = 8080
REFRESH_S = 0.1

# (그룹, 페이지들). 페이지는 (경로, 제목, 채우는 작업). gui-spec 3절
PAGES = (
    ("안내", (("/start", "시작", "G12"),)),
    ("작업", (("/", "작업 공간", "G6"), ("/manual", "직접 조작", "G4"), ("/teach", "교시", "G5"), ("/program", "프로그램", "결정 0044"))),
    ("인식", sensors.menu()),
    ("장치", (("/arm", "팔", "G8"), ("/calibration", "캘리브레이션", "결정 0044"), ("/esp32", "ESP32 센서 보드", "3단계"))),
    ("설정", (("/params", "파라미터·프리셋", "G7"), ("/safety", "안전", "G7"), ("/table", "작업대·좌표", "G7"))),
    ("시스템", (("/runs", "실행 기록", "G8"), ("/logs", "로그", "G8"))),
)
# 페이지 제목 아래 한 줄 설명. 시작 화면 페이지 안내도 같은 문구를 쓴다 (gui-spec 11절 D11)
DESCRIPTIONS = {
    "/": "인식한 물체를 차례로 집어 배출 구역으로 옮기는 자동 작업을 실행하고 지켜봅니다.",
    "/manual": "팔을 관절이나 손끝 좌표로 직접 움직입니다.",
    "/teach": "손으로 움직인 동작을 기록하고 다시 재생합니다.",
    "/program": "저장한 자세와 이동·집기·그리퍼·대기·반복을 순서대로 엮어 저장하고 실행합니다.",
    "/ultrasonic": "레이더와 손목 센서가 찾은 물체를 봅니다. 지금은 가상 실행만 됩니다.",
    "/camera": "카메라로 물체를 찾는 화면입니다.",
    "/input": "센서 대신 사람이 물체 위치를 알려 주는 인식 방식입니다. 작업 공간 자동 실행이 이 물체들을 집습니다.",
    "/arm": "팔 연결 상태와 점검 결과, 팔 파일, 진단을 봅니다.",
    "/calibration": "자동 캘리브레이션을 단계별로 측정·비교하고, 합격하면 저장한 뒤 0° 자세를 확인합니다. 실제 팔에서만 켜집니다.",
    "/esp32": "초음파 센서 보드(ESP32)의 연결과 최근 메시지를 봅니다.",
    "/params": "움직임·집기·교시·실행 방식 값을 바꾸고 프리셋으로 저장합니다.",
    "/safety": "안전 모드, 관절 여유, 허용 상자를 정합니다.",
    "/table": "팔 받침, 배출 구역, 센서가 작업대 어디에 있는지 정합니다.",
    "/runs": "지난 실행의 이벤트와 전압·온도 그래프를 봅니다.",
    "/logs": "앱과 감시 프로세스의 기록을 한 시간축으로 봅니다.",
    "/start": "지금 모드와 처음 할 순서, 페이지 안내를 봅니다.",
}
BUILDERS = {"/": workspace.build, "/manual": manual.build, "/teach": teach.build, "/program": program.build, "/params": params.build, "/safety": safety.build, "/table": table.build, "/arm": arm.build, "/calibration": calibration.build, "/esp32": esp32.build, "/runs": runs.build, "/logs": logs.build, "/start": lambda b, c, a: start.build(b, c, a, PAGES[1:], DESCRIPTIONS), **sensors.pages()}  # 만든 페이지. 나머지는 자리만 있다
COMPONENTS = ("버스", "서보")  # 감시 칸과 센서 칸은 따로 있다 (decisions/0030)
TOP_STATS = ("센서", "전압", "최고 온도", "속도", "조작권")  # 감시 칸은 아이콘이 있어 따로 그린다. 실행 방식은 오른쪽 고르기 칸이 보인다
WIDE_ONLY = {"센서", "최고 온도"}  # 1440px 아래에서는 숨긴다. 과열은 서보 칸이 경고로 보인다
BOTTOM_STATS = ("손끝 좌표", "닿는지·오차", "안전 모드", "토크", "기록 폴더")


def _stat(label: str, value: str = EMPTY, wide_only: bool = False) -> ui.label:
    with ui.column().classes("gap-0 shrink-0" + (" gt-md" if wide_only else "")):
        ui.label(label).classes("stat-label")
        return ui.label(value).classes("stat-value")


def make_root(backend: Backend):
    def root() -> None:
        """모든 페이지가 공유하는 틀. /operator는 틀 없이 운영자 화면만 그린다(gui-spec 12.4)."""
        client = ui.context.client
        remote = is_remote(client.ip)  # 다른 기기(LAN IP로 연 같은 PC 포함)는 늘 보기 전용이다(결정 0047, gui-spec 12.6)
        if client.request is not None and client.request.url.path == "/operator":
            operator.build(backend, client.id)
            return
        theme.apply()
        nav: dict[str, ui.button] = {}
        components: dict[str, tuple] = {}
        stats: dict[str, ui.label] = {}
        badge: list[ui.badge] = []
        resume: list[ui.button] = []
        reconnect: list[ui.button] = []
        lock_row: list = []
        beating: list[bool] = []  # 이 브라우저가 지금 감시 프로세스에 heartbeat를 보내는가
        health_seen = [time.time()]  # 이 시각 뒤에 바뀐 서보 건강 판정만 알린다
        log_seen = [backend.log_loss()]  # 이 브라우저가 열린 뒤 새로 빠진 기록만 알린다
        ui.add_head_html(browser_hb_js(getattr(backend.link, "port", WATCHDOG_PORT)))
        ui.add_head_html(VIEW_CUBE_JS)

        def show(path: str, title: str, task: str) -> None:
            for btn in nav.values():
                btn.classes(remove="nav-active").props(remove="aria-current")
            nav[path].classes(add="nav-active").props("aria-current=page")
            ui.page_title(f"{title} · SO-101 Studio")
            backend.clear_plan()
            with ui.row().classes("w-full items-center"):
                ui.label(title).classes("page-title").mark("page-title")
                text = page_state(backend.state.status)
                badge[:] = [ui.badge(text, color=theme.STATE_COLORS[text]).classes("page-state").mark("page-state")]
                reconnect[:] = [ui.button("다시 연결", color="negative", on_click=reconnect_arm).props("no-caps").mark("reconnect")]
                reconnect[0].visible = False
                ui.space()
                actions = ui.row().classes("items-center gap-2")
            if path in DESCRIPTIONS:
                ui.label(DESCRIPTIONS[path]).classes("page-body page-desc").mark("page-desc")
            with ui.row().classes("w-full items-center gap-1 no-wrap").mark("lock-banner") as row:
                ui.icon("lock", color="negative")
                label = ui.label(f"움직이는 버튼이 꺼져 있습니다: {backend.move_lock()}").classes("page-body")
            row.visible = bool(backend.move_lock())
            lock_row[:] = [row, label]
            if path in BUILDERS:
                BUILDERS[path](backend, client.id, actions)
                return
            ui.label("아직 준비 중인 화면입니다. 지금은 초음파나 수동 입력으로 물체를 알려 주세요.").classes("page-body").mark("page-pending")

        def refresh() -> None:
            for name, value in backend.readout().items():
                if name in stats:
                    stats[name].text = value
            stats["조작권"].text = "이 브라우저" if backend.owner == client.id else ("보기 전용(다른 기기)" if remote else "보기 전용")
            take.visible = backend.owner is None and not remote
            if backend.pending and backend.owner == client.id and not pending_dialog.value:
                pending_text.text = f"시뮬레이션에서 '{backend.pending[0]}'을 마쳤습니다. 실제 팔로 실행할까요?"
                pending_dialog.open()
            for t, key, level, value in list(backend.health.changes):
                if t > health_seen[0]:
                    health_seen[0] = t
                    if level == "ok":
                        ui.notify(f"서보 건강: {describe(key)} 정상으로 돌아옴 ({value:.1f})", type="positive")
                    else:
                        ui.notify(f"서보 {LEVEL_TEXT[level]}: {describe(key, level)} ({value:.1f})", type="warning" if level == "warn" else "negative")
            lost = backend.log_loss()
            if lost > log_seen[0]:
                ui.notify(f"실행 기록 {lost - log_seen[0]}줄을 쓰지 못했습니다. 디스크 공간과 runs 폴더 권한을 확인하세요 (지금까지 {lost}줄)", type="warning")
                log_seen[0] = lost
            bar.refresh()
            if backend.arm.speed != shown_speed[0]:  # 다른 화면에서 바뀐 속도만 따라간다. 끄는 중인 값은 덮지 않는다
                shown_speed[0] = bar_speed.value = backend.arm.speed
            comm_lost = backend.comm_lost()
            for name, (level, detail) in component_states(backend.state.status, time.time(), comm_lost).items():
                icon_name, icon_color, text = theme.COMPONENT_STYLES[level]
                icon, tooltip = components[name]
                if stats[name].text != text:
                    stats[name].text = text
                    icon.name = icon_name
                    icon.props(f"color={icon_color}")
                tooltip.text = detail
            icon_name, icon_color = theme.WATCH_STYLES[stats["감시"].text]
            watch_icon.name = icon_name
            watch_icon.props(f"color={icon_color}")
            mine = backend.owner == client.id
            stats["안전 모드"].classes(add="stat-danger" if stats["안전 모드"].text == "끔" else "", remove="" if stats["안전 모드"].text == "끔" else "stat-danger")
            if beating != [mine]:
                beating[:] = [mine]
                ui.run_javascript(f"so101Hb({str(mine).lower()})")
            text = "오류" if comm_lost or backend.physics_failed() else page_state(backend.state.status)
            if lock_row and not lock_row[0].is_deleted:
                lock = backend.move_lock()
                lock_row[0].visible = bool(lock)
                lock_row[1].text = f"움직이는 버튼이 꺼져 있습니다: {lock}"
            if badge and not badge[0].is_deleted and badge[0].text != text:
                badge[0].text = text
                badge[0].props(f"color={theme.STATE_COLORS[text]}")
            if resume and not resume[0].is_deleted:
                resume[0].visible = mine and text in ("정지", "오류") and not comm_lost and not backend.physics_failed()
            if reconnect and not reconnect[0].is_deleted:
                reconnect[0].visible = mine and comm_lost
            if top_mode.value != backend.mode:
                top_mode.set_value(backend.mode)
            top_mode.enabled = mine and not backend.task_running()
            top_reconnect.enabled = mine and backend.real is not None and not backend.task_running()
            top_grip.enabled = mine
            if top_grip.value != backend.arm.cfg.gripper_speed_pct:  # 파라미터 페이지에서 바꿔도 따라간다
                top_grip.set_value(backend.arm.cfg.gripper_speed_pct)

        async def resume_arm() -> None:
            if backend.owner == client.id:
                reason = await run.io_bound(backend.resume)
                if reason:
                    ui.notify(reason, type="negative")

        async def reconnect_arm() -> None:
            if backend.owner != client.id:
                return
            reason, torque_off = await run.io_bound(backend.reconnect)
            if reason:
                ui.notify(reason, type="negative")
            elif torque_off:
                ui.notify(f"다시 연결했습니다. 토크가 꺼진 서보: {', '.join(torque_off)}. 팔을 받쳐 주세요", type="warning")
            else:
                ui.notify("다시 연결했습니다. 모든 서보의 토크가 켜져 있습니다", type="positive")

        async def run_pending() -> None:
            pending_dialog.close()
            reason = await run.io_bound(backend.confirm_pending)
            ui.notify(reason or "실제 팔로 실행했습니다", type="negative" if reason else "positive")

        def skip_pending() -> None:
            backend.cancel_pending()
            pending_dialog.close()

        with ui.dialog() as pending_dialog, ui.card().classes("gap-3"):
            ui.label("실제 팔로 실행").classes("card-title")
            pending_text = ui.label("").classes("page-body").mark("pending-text")
            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("취소", on_click=skip_pending).props("outline no-caps").mark("pending-cancel")
                ui.button("실행", color="negative", on_click=run_pending).props("no-caps").mark("pending-confirm")
        pending_dialog.props("persistent")

        def set_mode(mode: str) -> None:
            if mode == backend.mode:
                return
            if backend.owner != client.id:
                ui.notify("보기 전용입니다", type="warning")
                return
            reason = backend.set_run_mode(mode)
            if reason:
                ui.notify(reason, type="negative")

        def set_grip_speed(pct: float) -> None:
            if pct == backend.arm.cfg.gripper_speed_pct or backend.owner != client.id:
                return
            reason, _ = backend.set_params({"gripper_speed_pct": float(pct)})  # 바로 적용 (결정 0042)
            if reason:
                ui.notify(reason, type="negative")

        async def show_doctor() -> None:
            report = await run.io_bound(backend.run_doctor)
            marks = {"ok": "정상", "warn": "주의", "fail": "실패", "skip": "건너뜀"}
            with ui.dialog() as dialog, ui.card().classes("gap-1").mark("doctor-dialog"):
                ui.label("점검 (읽기 전용)").classes("card-title")
                for c in report["checks"]:
                    ui.label(f'[{marks.get(c["state"], c["state"])}] {c["name"]}: {c["text"]}').classes("page-body")
                ui.button("닫기", on_click=dialog.close).props("outline no-caps")
            dialog.open()

        bar = Controls(backend, client.id, moves=True)  # 아래 막대와 조그 패널의 움직이는 조작

        async def go_home() -> None:
            reason = await run.io_bound(backend.home)
            if reason:
                ui.notify(reason, type="negative")

        async def grip(value: float) -> None:
            reason = await run.io_bound(backend.gripper, value)
            if reason:
                ui.notify(reason, type="negative")

        async def estop() -> None:  # 보기 전용 브라우저도 누를 수 있다. 세우는 일은 조작권과 상관없다
            failed = await run.io_bound(backend.estop)
            ui.notify(failed or "비상 정지했습니다", type="negative", **({"timeout": 0, "close_button": "닫기"} if failed else {}))

        # value=None: 넓은 화면에서만 열린 채 시작(좁으면 닫힌 채, 메뉴 버튼으로 연다). True는 좁은 화면에서 내용을 덮었다
        drawer = ui.left_drawer(value=None, bordered=False).props(f"width={theme.DRAWER_W}").classes("p-0 gap-0")

        with ui.header().classes("frame-bar frame-top items-center no-wrap gap-4 px-3 py-1"):
            ui.button(icon="menu", on_click=drawer.toggle).props("flat round color=dark aria-label=메뉴").mark("menu")
            ui.link("SO-101 Studio", "/start").classes("text-lg font-semibold no-underline text-inherit").props('aria-label="시작 화면으로"').mark("brand")
            # 상태 칸은 좁은 화면에서 줄바꿈 대신 가로로 스크롤한다. 오른쪽 조작(그리퍼 속도·실행 방식·다시 연결·점검)은 항상 보인다
            with ui.row().classes("items-center gap-4 no-wrap min-w-0 flex-1 overflow-x-auto top-stats"):
                stats["팔"] = _stat("팔")
                with ui.row().classes("items-center gap-1 no-wrap"):
                    watch_icon = ui.icon("remove_circle_outline", size="20px").mark("watch-icon")
                    stats["감시"] = _stat("감시")
                for name in COMPONENTS:
                    with ui.row().classes("items-center gap-1 no-wrap"):
                        icon = ui.icon("sync_problem", size="20px", color="grey-8").mark(f"component-{name}-icon")
                        stats[name] = _stat(name)
                        stats[name].mark(f"component-{name}")
                        components[name] = (icon, ui.tooltip(""))
                for label in TOP_STATS:
                    stats[label] = _stat(label, wide_only=label in WIDE_ONLY)
                stats["조작권"].text = "보기 전용"
                take = ui.button("조작권 받기", on_click=lambda: backend.claim(client.id, remote)).props("outline no-caps").mark("take")
                take.visible = False
            # 자주 쓰는 것을 상단에서 바로 (gui-spec 11절 A2). 원래 페이지의 같은 기능과 같은 상태를 쓴다
            if backend.share:  # 다른 기기에서 보는 중일 수 있다(결정 0047). 누르면 주소
                ui.badge("공유 중", color="warning").props('aria-label="공유 중: 누르면 주소"').classes("cursor-pointer").mark("share-badge").on(
                    "click", lambda: ui.notify(share_warning(backend.web_port), timeout=0, close_button="닫기"))
            with ui.row().classes("items-center gap-2 no-wrap"):
                ui.label("그리퍼 속도").classes("stat-label")
                top_grip = ui.slider(min=META["gripper_speed_pct"].hard[0], max=META["gripper_speed_pct"].hard[1], step=META["gripper_speed_pct"].step, value=backend.arm.cfg.gripper_speed_pct,
                                     on_change=lambda e: set_grip_speed(e.value)).props('dense label aria-label="그리퍼 속도"').classes("w-28").mark("top-grip-speed")
            top_mode = ui.select(RUN_MODES, value=backend.mode, on_change=lambda e: set_mode(e.value)).props(
                'dense outlined options-dense aria-label="실행 방식"').classes("w-40").mark("top-run-mode")
            top_reconnect = ui.button(icon="sync", on_click=reconnect_arm).props('flat round color=dark aria-label="다시 연결"').mark("top-reconnect")
            ui.button(icon="fact_check", on_click=lambda: show_doctor()).props('flat round color=dark aria-label="점검"').mark("top-doctor")

        with drawer, ui.column().classes("w-full gap-0").mark("nav"):
            for group, pages in PAGES:
                ui.label(group).classes("nav-group")
                for path, title, _ in pages:
                    nav[path] = ui.button(title, color=None, on_click=lambda p=path: ui.navigate.to(p)).props(
                        "flat no-caps align=left"
                    ).classes("nav-item")

        # 어디서나 여는 조그 패널 (결정 0044, PolyScope X 멀티태스크 패널)
        with ui.right_drawer(value=False, bordered=True).props(f"width={theme.JOG_PANEL_W}").classes("gap-3").mark("jog-panel") as jog_panel:
            ui.label("조그").classes("card-title")
            jog_pads(backend, bar, prefix="bar-")
            with ui.row().classes("w-full gap-2 no-wrap"):
                bar.add(ui.button("그리퍼 열기", on_click=bar.guard(lambda: grip(100.0))).props("outline no-caps").classes("flex-1").mark("bar-grip-open"))
                bar.add(ui.button("닫기", on_click=bar.guard(lambda: grip(0.0))).props("outline no-caps").classes("flex-1").mark("bar-grip-close"))
            ui.label("직접 조작 화면과 같은 속도·안전 검사를 씁니다").classes("page-body")

        # 아래 팔 조작 막대 (결정 0044, PolyScope X 로봇 제어 막대): 상태 값 | 속도·정지 해제·홈·조그 패널·E-STOP
        with ui.footer().classes("frame-bar frame-bottom items-center gap-4 px-3 py-1 no-wrap"):
            with ui.row().classes("items-center gap-6 no-wrap min-w-0 flex-1 overflow-x-auto top-stats"):
                for label in BOTTOM_STATS:
                    stats[label] = _stat(label)
            with ui.row().classes("items-center gap-2 no-wrap shrink-0"):
                ui.label("속도").classes("stat-label")
                bar_speed = bar.add(ui.slider(min=META["speed_pct"].hard[0], max=META["speed_pct"].hard[1], step=META["speed_pct"].step, value=backend.arm.speed).props('dense label aria-label="속도"').classes("w-28").mark("bar-speed"))
                shown_speed = [backend.arm.speed]
                bar_speed.on("change", bar.guard(lambda: run.io_bound(backend.set_speed, float(bar_speed.value))))
                resume[:] = [ui.button("정지 해제", on_click=resume_arm).props("outline no-caps").mark("resume")]
                resume[0].visible = False
                bar.add(ui.button("홈", on_click=bar.guard(go_home)).props("outline no-caps").mark("bar-home"))
                ui.button(icon="open_with", on_click=jog_panel.toggle).props('flat round color=dark aria-label="조그 패널"').mark("bar-jog")
                ui.button("E-STOP", color="negative", on_click=estop).props("round no-caps").classes("estop").mark("estop")

        ui.sub_pages(
            {path: lambda p=path, t=title, k=task: show(p, t, k) for _, pages in PAGES for path, title, task in pages}
        ).classes("w-full")

        # 페이지를 그릴 때 받으면 소켓을 열지 않는 HTTP 요청(curl, 미리 읽기)이 조작권을 가져간다
        client.on_connect(lambda: backend.claim(client.id, remote))
        client.on_disconnect(lambda: backend.disconnected(client.id))
        ui.timer(REFRESH_S, refresh)

    return root


def real_arm_args(arm_name: str | None) -> dict:
    """연결된 보드로 팔과 포트를 고른다(decisions/0027). 목록만 읽고 포트는 열지 않는다. Backend에 넘길 인자를 돌려준다."""
    ports = list_serial_ports()
    serials = [p["serial"] for p in ports]
    cfg, info = load_robot_ex(arm=arm_name, serials=serials)
    port, _ = pick_port(ports, cfg.board_serial, None, info["local"].get("port"), cfg.port)
    return {"real_driver": make_driver(False, cfg), "serials": serials, "port": port, "first_report": _first_report(arm_name, port)}


def _first_report(arm_name: str | None, port: str) -> dict | None:
    """연결 전에만 포트를 열 수 있어 시작할 때 한 번 읽는다. PING/READ만 보낸다(decisions/0027). 실패해도 앱은 뜬다."""
    try:
        return doctor(probe=True, port=port, arm=arm_name)
    except Exception:
        logging.getLogger(__name__).exception("연결 전 점검 실패")
        return None


def main(args) -> None:
    """`python -m so101_studio.gui`와 `so101 gui`(REQ-005)의 입구. --sim이 없으면 실제 팔에 연결한다(토크는 그대로, 움직이지 않는다)."""
    sim = getattr(args, "sim", False)
    engine, viewer = getattr(args, "sim_engine", "kinematic"), getattr(args, "sim_viewer", False)
    if getattr(args, "sim_scenario", None):  # 두 입구(so101 gui, python -m so101_studio.gui)를 함께 덮는다(gui-spec 12.8)
        sys.exit("GUI는 config/manual_objects.yaml 물체만 씁니다")
    if engine == "mujoco" and not sim:
        sys.exit("--sim-engine은 --sim과 함께 쓴다")
    if viewer and not (sim and engine == "mujoco"):
        sys.exit("--sim-viewer는 --sim --sim-engine mujoco와 함께 쓴다")
    share = bool(getattr(args, "share", False))
    host = SHARE_HOST if share else LOCAL_HOST
    # so101.cmd gui의 --port는 서보 시리얼 포트다. 웹 포트는 python -m so101_studio.gui --port(web_port)로만 바꾼다(gui-spec 12.6)
    web_port = getattr(args, "web_port", None) or DEFAULT_PORT
    wd_port = getattr(args, "watchdog_port", None) or WATCHDOG_PORT  # 시험용 서버는 다른 포트로 띄워 실제 팔 GUI의 감시 프로세스와 겹치지 않게
    viser_port = getattr(args, "viser_port", None) or scene3d.DEFAULT_PORT
    if why := port_check.check_ports({web_port, viser_port, wd_port}, {wd_port + 1}, url=f"http://127.0.0.1:{web_port}"):  # 팔을 연결하기 전에: 이전 GUI가 남아 있으면 포트 오류로 죽었다
        sys.exit(why)
    link = WatchdogLink(sim=sim, port=wd_port, stop_port=wd_port + 1)
    try:
        arm_name = getattr(args, "arm", None)
        backend = Backend(
            speed=getattr(args, "speed", None), link=link, viser_port=viser_port,
            arm_name=arm_name, viser_host=host, share=share, web_port=web_port, **({} if sim else real_arm_args(arm_name)),
            **({"sim_engine": engine, "sim_viewer": viewer} if engine == "mujoco" else {}),
        )
    except ImportError:  # mujoco가 없다. 기구학 엔진으로 바꿔 띄우지 않는다
        link.stop()
        if engine != "mujoco":
            raise
        print(MUJOCO_MISSING_TEXT, flush=True)
        sys.exit(2)
    except (RuntimeError, ValueError, ArmSelectionError, PortChoiceError) as e:  # ValueError: 병렬 그리퍼·물체 겹침·scene.yaml 오류(MuJoCo)
        link.stop()
        others = [] if sim else port_check.so101_processes()  # 시리얼 포트를 다른 so101 프로그램이 쥐고 있는 경우가 많다
        hint = ("\n지금 켜진 SO-101 프로그램(이 중 하나가 팔 포트를 쓰고 있을 수 있습니다. 끈 뒤 다시 실행하세요):\n  " + "\n  ".join(others)) if others else ""
        sys.exit(f"연결 실패: {e}{hint}")
    link.start(backend.run_dir)  # 실행 폴더는 연결한 뒤에 생긴다
    app.on_startup(lambda: background_tasks.create(link.run(), name="watchdog heartbeat"))
    app.on_shutdown(backend.close)
    if share:
        print(share_warning(web_port), flush=True)
    ui.run(
        make_root(backend), host=host, port=web_port, title="SO-101 Studio",
        language="ko-KR", reload=False,
        show=False if (getattr(args, "no_browser", False) or os.environ.get("SO101_GUI_NO_BROWSER")) else "/start",  # 세션이 띄울 때는 기본 브라우저에 탭을 열지 않는다(먼저 붙은 탭이 조작권을 가져간다)
    )
