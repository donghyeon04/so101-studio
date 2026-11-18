"""시점 페이지 (gui-spec 12.10 ①): 탑뷰·손목 카메라, 초음파 작은 화면, 3D를 2×2 칸으로. 칸을 누르면 그 칸만 크게 본다.
카메라는 늘 시뮬레이션 쪽 서비스(backend.sim)의 driver.render로 그린다."""

from nicegui import ui

from ...service.policy_runner import CAMERAS
from .. import sensors
from .common import camera_tile, scene_frame, watch_visibility
from .policy import CAMERA_TITLES

NO_RENDER_TEXT = "MuJoCo 시뮬레이션에서만 보입니다"


def build(backend, client_id: str, actions) -> None:
    driver = backend.sim.driver
    can_render = callable(getattr(driver, "render", None))  # 실제 팔·기구학 엔진에는 카메라가 없다
    visible = watch_visibility()
    cards: list = []
    tiles: list = []  # (카드, 그리기 함수)
    grid = ui.grid(columns=2).classes("w-full gap-4")

    def toggle(card) -> None:
        big = "col-span-2" not in card.classes
        for other in cards:
            other.set_visibility(not big or other is card)
        if big:
            card.classes(add="col-span-2")
        else:
            card.classes(remove="col-span-2")

    def tile(name: str):
        card = ui.card().classes("min-w-0 gap-1 cursor-pointer").mark(f"view-{name}")
        card.on("click", lambda: toggle(card))
        cards.append(card)
        return card

    with grid:
        for cam in CAMERAS:
            with tile(cam) as card:
                if can_render:
                    tiles.append((card, camera_tile(driver, cam, CAMERA_TITLES[cam])))
                else:
                    ui.label(CAMERA_TITLES[cam]).classes("card-title")
                    ui.label(NO_RENDER_TEXT).classes("page-body").mark(f"camera-unavailable-{cam}")
        with tile("ultrasonic"):
            for view in sensors.mini_views():
                ui.label(view.title).classes("card-title")
                view.mini_view_fn(backend)
        with tile("scene"):
            ui.label("3D").classes("card-title")  # 3D 안을 누르면 iframe이 받으므로 크게 보기는 제목 줄을 누른다
            scene_frame(backend, "scene3d scene3d-short")

    async def draw() -> None:
        if visible():
            for card, update in tiles:
                if card.visible:
                    await update()

    if tiles:
        ui.timer(0.1, draw)  # 10fps
