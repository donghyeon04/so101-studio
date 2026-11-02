"""NiceGUI 앱: 공통 틀(위쪽 상태줄, 사이드바, 아래쪽 상태줄, E-STOP)과 페이지 등록."""

import sys

from nicegui import app, run, ui

from . import theme
from .backend import Backend
from .state import EMPTY, page_state

HOST = "127.0.0.1"  # NiceGUI 기본값 0.0.0.0은 같은 네트워크 어디서나 팔을 조작할 수 있게 한다 (decisions/0023)
DEFAULT_PORT = 8080
REFRESH_S = 0.1

# (그룹, 페이지들). 페이지는 (경로, 제목, 채우는 작업). gui-spec 3절
PAGES = (
    ("작업", (("/", "작업 공간", "G6"), ("/manual", "직접 조작", "G4"), ("/teach", "교시", "G5"))),
    ("인식", (("/ultrasonic", "초음파 A·B", "3단계"), ("/camera", "카메라", "4단계"), ("/input", "수동 입력", "G6"))),
    ("장치", (("/arm", "팔", "G8"), ("/esp32", "ESP32 센서 보드", "3단계"))),
    ("설정", (("/params", "파라미터·프리셋", "G7"), ("/safety", "안전", "G7"), ("/table", "작업대·좌표", "G7"))),
    ("시스템", (("/runs", "실행 기록", "G8"), ("/logs", "로그", "G8"))),
)
TOP_STATS = ("센서", "실행 방식", "전압", "최고 온도", "속도", "조작권")
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
        owner = backend.claim(client.id)

        def show(path: str, title: str, task: str) -> None:
            for btn in nav.values():
                btn.classes(remove="nav-active").props(remove="aria-current")
            nav[path].classes(add="nav-active").props("aria-current=page")
            ui.page_title(f"{title} · SO-101 Studio")
            with ui.row().classes("w-full items-center"):
                ui.label(title).classes("page-title").mark("page-title")
                text = page_state(backend.state.status)
                badge[:] = [ui.badge(text, color=theme.STATE_COLORS[text]).classes("page-state").mark("page-state")]
            ui.label(f"{task}에서 만든다.").classes("page-body")

        def refresh() -> None:
            for name, value in backend.readout().items():
                stats[name].text = value
            stats["조작권"].text = "이 브라우저" if backend.owner == client.id else "보기 전용"
            take.visible = backend.owner is None
            text = page_state(backend.state.status)
            if badge and not badge[0].is_deleted and badge[0].text != text:
                badge[0].text = text
                badge[0].props(f"color={theme.STATE_COLORS[text]}")

        async def estop() -> None:  # 보기 전용 브라우저도 누를 수 있다. 세우는 일은 조작권과 상관없다
            await run.io_bound(backend.estop)
            ui.notify("비상 정지했습니다", type="negative")

        drawer = ui.left_drawer(value=True, bordered=False).props(f"width={theme.DRAWER_W}").classes("p-0 gap-0")

        with ui.header().classes("frame-bar frame-top items-center gap-6 px-3 py-1"):
            ui.button(icon="menu", on_click=drawer.toggle).props("flat round color=dark aria-label=메뉴").mark("menu")
            ui.label("SO-101 Studio").classes("text-lg font-semibold")
            _stat("팔", "시뮬레이션")
            for label in TOP_STATS:
                stats[label] = _stat(label)
            stats["조작권"].text = "이 브라우저" if owner else "보기 전용"
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

        client.on_connect(lambda: backend.claim(client.id))
        client.on_disconnect(lambda: backend.disconnected(client.id))
        client.on_delete(lambda: backend.disconnected(client.id))  # 소켓이 붙기 전에 사라진 브라우저
        ui.timer(REFRESH_S, refresh)

    return root


def main(args) -> None:
    """`python -m so101_studio.gui`와 `so101 gui`(REQ-005)의 입구."""
    if not getattr(args, "sim", False):
        # shortcut: 실제 팔 연결이 아직 없다, G9·G10에서 backend가 연결하면 지운다
        sys.exit("아직 시뮬레이션만 지원한다: --sim을 붙여 실행한다")
    try:
        backend = Backend(speed=getattr(args, "speed", None))
    except RuntimeError as e:
        sys.exit(f"연결 실패: {e}")
    app.on_shutdown(backend.close)
    ui.run(
        make_root(backend), host=HOST, port=getattr(args, "port", DEFAULT_PORT), title="SO-101 Studio",
        language="ko-KR", reload=False,
    )
