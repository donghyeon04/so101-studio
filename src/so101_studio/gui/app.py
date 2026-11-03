"""NiceGUI 앱: 공통 틀(위쪽 상태줄, 사이드바, 아래쪽 상태줄, E-STOP)과 페이지 등록."""

import sys

from nicegui import app, background_tasks, run, ui

from . import scene3d, sensors, theme
from .backend import Backend
from .pages import arm, logs, manual, params, runs, safety, table, teach, workspace
from .state import EMPTY, page_state
from .view_cube import VIEW_CUBE_JS
from .watchdog_link import BROWSER_HB_JS, WatchdogLink

HOST = "127.0.0.1"  # NiceGUI 기본값 0.0.0.0은 같은 네트워크 어디서나 팔을 조작할 수 있게 한다 (decisions/0023)
DEFAULT_PORT = 8080
REFRESH_S = 0.1

# (그룹, 페이지들). 페이지는 (경로, 제목, 채우는 작업). gui-spec 3절
PAGES = (
    ("작업", (("/", "작업 공간", "G6"), ("/manual", "직접 조작", "G4"), ("/teach", "교시", "G5"))),
    ("인식", sensors.menu()),
    ("장치", (("/arm", "팔", "G8"), ("/esp32", "ESP32 센서 보드", "3단계"))),
    ("설정", (("/params", "파라미터·프리셋", "G7"), ("/safety", "안전", "G7"), ("/table", "작업대·좌표", "G7"))),
    ("시스템", (("/runs", "실행 기록", "G8"), ("/logs", "로그", "G8"))),
)
BUILDERS = {"/": workspace.build, "/manual": manual.build, "/teach": teach.build, "/params": params.build, "/safety": safety.build, "/table": table.build, "/arm": arm.build, "/runs": runs.build, "/logs": logs.build, **sensors.pages()}  # 만든 페이지. 나머지는 자리만 있다
TOP_STATS = ("센서", "실행 방식", "전압", "최고 온도", "속도", "조작권")  # 감시 칸은 아이콘이 있어 따로 그린다
BOTTOM_STATS = ("손끝 좌표", "닿는지·오차", "안전 모드", "토크", "기록 폴더")


def _stat(label: str, value: str = EMPTY) -> ui.label:
    with ui.column().classes("gap-0"):
        ui.label(label).classes("stat-label")
        return ui.label(value).classes("stat-value")


def make_root(backend: Backend):
    def root() -> None:
        """모든 페이지가 공유하는 틀."""
        theme.apply()
        client = ui.context.client
        nav: dict[str, ui.button] = {}
        stats: dict[str, ui.label] = {}
        badge: list[ui.badge] = []
        resume: list[ui.button] = []
        beating: list[bool] = []  # 이 브라우저가 지금 감시 프로세스에 heartbeat를 보내는가
        ui.add_head_html(BROWSER_HB_JS)
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
                resume[:] = [ui.button("정지 해제", on_click=resume_arm).props("outline no-caps").mark("resume")]
                resume[0].visible = False
                ui.space()
                actions = ui.row().classes("items-center gap-2")
            if path in BUILDERS:
                BUILDERS[path](backend, client.id, actions)
                return
            ui.label(f"{task}에서 만든다.").classes("page-body")

        def refresh() -> None:
            for name, value in backend.readout().items():
                stats[name].text = value
            stats["조작권"].text = "이 브라우저" if backend.owner == client.id else "보기 전용"
            take.visible = backend.owner is None
            if backend.pending and backend.owner == client.id and not pending_dialog.value:
                pending_text.text = f"시뮬레이션에서 '{backend.pending[0]}'을 마쳤습니다. 실제 팔로 실행할까요?"
                pending_dialog.open()
            icon_name, icon_color = theme.WATCH_STYLES[stats["감시"].text]
            watch_icon.name = icon_name
            watch_icon.props(f"color={icon_color}")
            mine = backend.owner == client.id
            stats["안전 모드"].classes(add="stat-danger" if stats["안전 모드"].text == "끔" else "", remove="" if stats["안전 모드"].text == "끔" else "stat-danger")
            if beating != [mine]:
                beating[:] = [mine]
                ui.run_javascript(f"so101Hb({str(mine).lower()})")
            text = page_state(backend.state.status)
            if badge and not badge[0].is_deleted and badge[0].text != text:
                badge[0].text = text
                badge[0].props(f"color={theme.STATE_COLORS[text]}")
            if resume and not resume[0].is_deleted:
                resume[0].visible = mine and text in ("정지", "오류")

        async def resume_arm() -> None:
            if backend.owner == client.id:
                await run.io_bound(backend.resume)

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

        async def estop() -> None:  # 보기 전용 브라우저도 누를 수 있다. 세우는 일은 조작권과 상관없다
            await run.io_bound(backend.estop)
            ui.notify("비상 정지했습니다", type="negative")

        drawer = ui.left_drawer(value=True, bordered=False).props(f"width={theme.DRAWER_W}").classes("p-0 gap-0")

        with ui.header().classes("frame-bar frame-top items-center gap-6 px-3 py-1"):
            ui.button(icon="menu", on_click=drawer.toggle).props("flat round color=dark aria-label=메뉴").mark("menu")
            ui.label("SO-101 Studio").classes("text-lg font-semibold")
            stats["팔"] = _stat("팔")
            with ui.row().classes("items-center gap-1 no-wrap"):
                watch_icon = ui.icon("remove_circle_outline", size="20px").mark("watch-icon")
                stats["감시"] = _stat("감시")
            for label in TOP_STATS:
                stats[label] = _stat(label)
            stats["조작권"].text = "보기 전용"
            take = ui.button("조작권 받기", on_click=lambda: backend.claim(client.id)).props("outline no-caps").mark("take")
            take.visible = False

        with drawer, ui.column().classes("w-full gap-0").mark("nav"):
            for group, pages in PAGES:
                ui.label(group).classes("nav-group")
                for path, title, _ in pages:
                    nav[path] = ui.button(title, color=None, on_click=lambda p=path: ui.navigate.to(p)).props(
                        "flat no-caps align=left"
                    ).classes("nav-item")

        with ui.footer().classes("frame-bar frame-bottom items-center gap-6 px-3 py-1"):
            for label in BOTTOM_STATS:
                stats[label] = _stat(label)

        with ui.page_sticky("bottom-right", x_offset=theme.GAP, y_offset=theme.GAP):
            ui.button("E-STOP", color="negative", on_click=estop).props("round no-caps").classes("estop").mark("estop")

        ui.sub_pages(
            {path: lambda p=path, t=title, k=task: show(p, t, k) for _, pages in PAGES for path, title, task in pages}
        ).classes("w-full")

        # 페이지를 그릴 때 받으면 소켓을 열지 않는 HTTP 요청(curl, 미리 읽기)이 조작권을 가져간다
        client.on_connect(lambda: backend.claim(client.id))
        client.on_disconnect(lambda: backend.disconnected(client.id))
        ui.timer(REFRESH_S, refresh)

    return root


def main(args) -> None:
    """`python -m so101_studio.gui`와 `so101 gui`(REQ-005)의 입구."""
    if not getattr(args, "sim", False):
        # shortcut: 실제 팔 연결이 아직 없다, G9·G10에서 backend가 연결하면 지운다
        sys.exit("아직 시뮬레이션만 지원한다: --sim을 붙여 실행한다")
    link = WatchdogLink(sim=True)
    link.start()
    try:
        backend = Backend(
            speed=getattr(args, "speed", None), link=link, viser_port=getattr(args, "viser_port", scene3d.DEFAULT_PORT)
        )
    except RuntimeError as e:
        link.stop()
        sys.exit(f"연결 실패: {e}")
    app.on_startup(lambda: background_tasks.create(link.run(), name="watchdog heartbeat"))
    app.on_shutdown(backend.close)
    ui.run(
        make_root(backend), host=HOST, port=getattr(args, "port", DEFAULT_PORT), title="SO-101 Studio",
        language="ko-KR", reload=False,
    )
