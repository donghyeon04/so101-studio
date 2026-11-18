"""시점 페이지 (gui-spec 12.10 ①): render가 없으면 안내, 있으면 두 카메라를 그리고 칸을 누르면 크게 본다. MuJoCo로 팔을 움직이면 그림이 바뀐다."""

import asyncio
import base64
import io

import numpy as np
import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app

CARDS = ("top", "wrist", "ultrasonic", "scene")


def open_views(backend, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/")
            with user.scope(marker="nav"):
                user.find("시점").click()
            await check(user)

    asyncio.run(go())


def one(user, marker):
    return next(iter(user.find(marker=marker).elements))


def pixels(source: str) -> np.ndarray:
    from PIL import Image

    return np.asarray(Image.open(io.BytesIO(base64.b64decode(source.split(",", 1)[1]))), dtype=np.float32)


def test_without_render_shows_text_and_other_tiles(backend):
    async def check(user):
        await user.should_see(marker="camera-unavailable-top")
        await user.should_see(marker="camera-unavailable-wrist")
        await user.should_see(marker="us-mini")
        await user.should_see(marker="scene3d")
        await user.should_not_see(marker="camera-top")

    open_views(backend, check)


def test_cameras_draw_and_tile_click_enlarges(backend, monkeypatch):
    calls = []

    def render(camera, width=320, height=240):
        calls.append(camera)
        return np.full((height, width, 3), 100, np.uint8)

    monkeypatch.setattr(backend.sim.driver, "render", render, raising=False)

    async def check(user):
        await user.should_see(marker="camera-top")
        for _ in range(40):
            if one(user, "camera-top").source and one(user, "camera-wrist").source:
                break
            await asyncio.sleep(0.05)
        assert one(user, "camera-top").source.startswith("data:image/jpeg")
        user.find(marker="view-wrist").click()
        await user.should_see(marker="view-wrist")
        for c in ("top", "ultrasonic", "scene"):
            await user.should_not_see(marker=f"view-{c}")
        await asyncio.sleep(0.3)  # 누르기 전에 시작한 그리기가 끝나게
        calls.clear()
        await asyncio.sleep(0.4)
        assert calls and set(calls) == {"wrist"}  # 숨긴 칸은 그리지 않는다
        user.find(marker="view-wrist").click()
        for c in CARDS:
            await user.should_see(marker=f"view-{c}")
        user.find(marker="view-scene").click()  # 3D 칸은 안을 눌러도(view cube 등) 그대로, 제목을 눌러야 크게 본다
        await user.should_see(marker="view-top")
        user.find(marker="view-title-3D").click()
        await user.should_not_see(marker="view-top")

    open_views(backend, check)


@pytest.mark.slow
def test_mujoco_cameras_change_when_the_arm_moves(backend, monkeypatch):
    """12.10 시험 1: 관절 하나를 30° 넘게 움직인 전·후 두 카메라 그림의 픽셀 차이 평균이 0보다 크다."""
    pytest.importorskip("mujoco")
    from so101_studio.arm.mujoco_driver import MuJoCoDriver
    from so101_studio.core.scene import load_scene
    from so101_studio.core.types import JointState

    svc = backend.sim
    mj = MuJoCoDriver(svc.cfg, svc.ws, scene=load_scene(), realtime=False)
    mj.connect()
    monkeypatch.setattr(svc.driver, "render", mj.render, raising=False)
    home = list(svc.cfg.poses["home"])

    async def frames(user, previous=None) -> dict:
        """두 칸 모두 그림이 있고(처음 Renderer를 만드는 데 0.5초쯤 걸린다) previous와 다를 때까지, 10초까지 기다린다."""
        for _ in range(100):
            await asyncio.sleep(0.1)
            got = {c: one(user, f"camera-{c}").source for c in ("top", "wrist")}
            if all(got.values()) and (previous is None or all(got[c] != previous[c] for c in got)):
                return got
        raise AssertionError(f"카메라 그림이 바뀌지 않았다: {[c for c in got if not got[c]]}")

    async def check(user):
        await user.should_see(marker="camera-top")
        mj.write_joints(JointState(*home))
        mj.advance(1.0)
        before = await frames(user)
        moved = list(home)
        moved[0] += 40  # shoulder_pan
        mj.write_joints(JointState(*moved))
        mj.advance(1.5)
        after = await frames(user, before)
        for c in ("top", "wrist"):
            assert np.abs(pixels(after[c]) - pixels(before[c])).mean() > 0, c

    try:
        open_views(backend, check)
    finally:
        mj.disconnect()
