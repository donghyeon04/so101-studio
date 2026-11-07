"""페이지들이 같이 쓰는 조각: 조작권 게이트, 3D 칸."""

import asyncio

from nicegui import run, ui

from .. import theme
from ..scene3d import LAYERS
from ..view_cube import view_cube


class Controls:
    """조작권이 있어야 켜지는 요소를 모으고, 눌렸을 때 서버에서 한 번 더 막는다."""

    def __init__(self, backend, client_id: str, block_during_task: bool = True, moves: bool = False):
        self.backend, self.client_id, self.block_during_task, self.moves = backend, client_id, block_during_task, moves
        self.elements: list = []

    def add(self, element):
        self.elements.append(element)
        return element

    @property
    def mine(self) -> bool:
        return self.backend.owner == self.client_id

    @property
    def allowed(self) -> bool:
        """작업 실행 중에는 팔을 직접 움직이는 조작을 막는다."""
        return self.mine and not self.locked and not (self.block_during_task and self.backend.task_running())

    @property
    def locked(self) -> str | None:
        """팔을 움직이는 페이지(moves)는 실제 팔이 잠기면 함께 꺼진다 (decisions/0027)."""
        return self.backend.move_lock() if self.moves else None

    def guard(self, fn):
        async def call(*args):
            if not self.mine:
                ui.notify("보기 전용입니다", type="warning")
                return None
            if self.locked:
                ui.notify(self.locked, type="warning")
                return None
            if not self.allowed:
                ui.notify("작업 실행 중입니다", type="warning")
                return None
            return await fn(*args)

        return call

    def refresh(self) -> None:
        allowed = self.allowed
        for element in self.elements:
            element.enabled = allowed


def _layer_menu(scene) -> None:
    """3D에 보일 것을 고르는 메뉴. 3D 칸 왼쪽 위에 얹는다."""
    with ui.button(icon="layers").props('round dense size=sm color=white text-color=dark aria-label="3D에 보일 것 고르기"').classes("scene-layers").mark("scene-layers"), ui.menu(), ui.column().classes("gap-0 p-2"):
        for name, text in LAYERS.items():
            ui.checkbox(text, value=scene.layers[name], on_change=lambda e, n=name: scene.set_layer(n, e.value)).mark(f"layer-{name}")


def _request_host() -> str | None:
    """이 브라우저가 앱을 연 호스트 이름(시험처럼 요청이 없으면 None)."""
    try:
        return ui.context.client.request.url.hostname
    except (RuntimeError, AttributeError):
        return None


def scene_frame(backend, classes: str = "scene3d") -> None:
    with ui.element("div").classes("scene-wrap"):
        ui.element("iframe").props(f'src={backend.scene.url_for(_request_host())} title="3D 보기"').classes(classes).mark("scene3d")
        view_cube(backend.scene)
        _layer_menu(backend.scene)


def offer_free_edge(backend, retry) -> None:
    """작업대 가장자리에 걸려 멈췄을 때 그 자리에서 묻는다: 가장자리 검사만 끄고(자유 모드) 다시 하기."""
    with ui.dialog() as dialog, ui.card().classes("gap-2").mark("edge-dialog"):
        ui.label("작업대 가장자리 밖이라 움직이지 않았습니다").classes("card-title")
        ui.label("작업대 가장자리 검사만 끄고(자유 모드) 다시 할까요? 관절 한계와 바닥 검사는 그대로 켜져 있습니다. 안전 페이지에서 다시 켤 수 있습니다.").classes("page-body")

        async def go() -> None:
            dialog.close()
            reason = await run.io_bound(backend.free_table_edge)
            if reason:
                ui.notify(reason, type="negative")
                return
            result = retry()  # 다시 할 동작(비동기 함수면 기다린다)
            if asyncio.iscoroutine(result):
                await result

        with ui.row().classes("gap-2"):
            ui.button("가장자리 검사 끄고 다시 하기", on_click=go).props("no-caps").mark("edge-free")
            ui.button("취소", on_click=dialog.close).props("outline no-caps")
    dialog.open()


NUDGE_MM, NUDGE_DEG, NUDGE_REPEAT_S = 5.0, 5.0, 0.15


def nudge_pad(on_step, add=lambda e: e, rotate: bool = True) -> None:
    """고른 것을 옮기고 돌리는 패드 (gui-spec 11절 D11, G16). 누를 때마다 5mm·5°, 누르고 있으면 계속. on_step(dx, dy, dangle).
    위 = 작업대 +y(평면 그림의 위쪽). add는 조작권에 따라 켜고 끄는 Controls.add."""
    repeat = {"timer": None}

    def start(step) -> None:
        stop()
        on_step(*step)
        repeat["timer"] = ui.timer(NUDGE_REPEAT_S, lambda: on_step(*step))

    def stop() -> None:
        if repeat["timer"] is not None:
            repeat["timer"].cancel()
            repeat["timer"] = None

    def button(icon: str, label: str, step) -> None:
        b = add(ui.button(icon=icon).props(f'outline dense aria-label="{label}"').classes("nudge-button").mark(f"nudge-{label}"))
        b.on("mousedown", lambda: start(step))
        b.on("touchstart", lambda: start(step))
        for event in ("mouseup", "mouseleave", "touchend"):
            b.on(event, stop)

    m, d = NUDGE_MM, NUDGE_DEG
    with ui.grid(columns=3).classes("gap-1 nudge-pad").mark("nudge-pad"):
        button("rotate_left", "왼쪽으로 돌리기", (0, 0, d)) if rotate else ui.element()
        button("arrow_upward", "위로", (0, m, 0))
        button("rotate_right", "오른쪽으로 돌리기", (0, 0, -d)) if rotate else ui.element()
        button("arrow_back", "왼쪽으로", (-m, 0, 0))
        ui.label(f"{m:.0f}mm").classes("stat-label self-center text-center")
        button("arrow_forward", "오른쪽으로", (m, 0, 0))
        ui.element()
        button("arrow_downward", "아래로", (0, -m, 0))
        ui.element()


def jog_pads(backend, gate: Controls, prefix: str = "") -> None:
    """G13 조이스틱 두 개(앞뒤·좌우, 높이): 누르는 동안 기울인 만큼의 속도로, 떼면 멈춘다. 위 = 팔 앞쪽(+y).
    직접 조작 화면과 어디서나 여는 조그 패널(결정 0044)이 같이 쓴다. 가만히 쥐고 있으면 이벤트가 안 와서 0.1초마다 이어 보낸다(데드맨)."""
    pad: dict = {"v": None}

    def move(key: str, e) -> None:
        if not gate.allowed or e.x is None:
            return
        v = list(pad["v"] or [0.0, 0.0, 0.0])
        if key == "xy":
            v[0], v[1] = e.x, e.y
        else:
            v[2] = e.y
        pad["v"] = v
        backend.jog_vector(v)

    def end() -> None:
        if pad["v"] is not None:
            pad["v"] = None
            backend.jog_release("cartesian")

    def keep() -> None:
        if pad["v"] and gate.allowed:
            backend.jog_vector(pad["v"])

    with ui.row().classes("w-full no-wrap items-end justify-around gap-2 mb-2").mark(f"{prefix}joystick"):
        for key, label, opts in (("xy", "앞뒤·좌우", {}), ("z", "높이", {"lockY": True})):
            with ui.column().classes("items-center gap-1"):
                ui.joystick(color=theme.ACCENT, size=96, on_move=lambda e, k=key: move(k, e),
                            on_end=lambda _e: end(), **opts).classes("joystick").mark(f"{prefix}pad-{key}")
                ui.label(f"{label} (눌러 끌기)").classes("stat-label")
    ui.timer(0.1, keep)
