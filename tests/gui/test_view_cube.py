"""뷰큐브 (Fusion 360 ViewCube 모양): 방향 계산과 페이지 연결. 실제 끌기·클릭은 브라우저에서 확인한다."""

import asyncio
import math
import re

import numpy as np
import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.scene3d import (
    ORBIT_DEG_PER_PX,
    ORBIT_MAX_ELEVATION_DEG,
    Z_UP,
    orbit_offset,
    up_for,
)
from so101_studio.gui.view_cube import (
    DIRECTIONS,
    FACES,
    camera_axes,
    cube_matrix,
    faces_html,
    parse_direction,
    triad_svg,
)


def _css(matrix: str) -> np.ndarray:
    """matrix3d 글자를 4x4 행렬로. CSS는 열 순서로 쓴다."""
    return np.array([float(v) for v in re.findall(r"-?\d+\.\d+", matrix)]).reshape(4, 4).T


def test_faces_are_not_mirrored_and_point_outward():
    for right, down, out in FACES.values():
        assert np.linalg.det(np.array([right, down, out]).T) == pytest.approx(-1)


def test_cells_cover_all_26_directions():
    dirs = {parse_direction(d) for d in re.findall(r'data-dir="([^"]+)"', faces_html())}
    assert dirs == DIRECTIONS and len(dirs) == 26
    assert faces_html().count('tabindex="0"') == 6  # 키보드로는 면만


@pytest.mark.parametrize("text", ["0,0,0", "2,0,0", "1,0", "a,b,c", "", None])
def test_bad_direction_from_browser_is_refused(text):
    assert parse_direction(text) is None


def test_front_view_shows_front_face_with_top_up():
    axes = camera_axes((0, -1, 0), (0, 0, 0), Z_UP)
    rotate = _css(cube_matrix(axes))[:3, :3]
    assert rotate @ (0, -1, 0) == pytest.approx((0, 0, 1))  # 앞면 바깥쪽이 화면에서 나를 향한다
    assert rotate @ (0, 0, 1) == pytest.approx((0, -1, 0))  # z가 화면 위(CSS y는 아래로 증가)
    assert rotate @ (1, 0, 0) == pytest.approx((1, 0, 0))  # x가 화면 오른쪽
    assert np.linalg.det(rotate) == pytest.approx(-1)


@pytest.mark.parametrize("direction", sorted(DIRECTIONS))
def test_every_direction_has_a_usable_up_and_faces_the_camera(direction):
    unit = np.array(direction, float) / np.linalg.norm(direction)
    axes = camera_axes(unit, (0, 0, 0), up_for(direction))
    assert axes is not None
    assert _css(cube_matrix(axes))[:3, :3] @ unit == pytest.approx((0, 0, 1), abs=1e-3)


def test_camera_straight_along_up_has_no_axes():
    assert camera_axes((0, 0, 1), (0, 0, 0), Z_UP) is None


def test_drag_right_moves_camera_left_and_drag_down_raises_it():
    start = np.array((0.0, -1.0, 0.0))
    assert orbit_offset(start, Z_UP, 30, 0)[0] < 0
    assert orbit_offset(start, Z_UP, 0, 30)[2] > 0
    moved = orbit_offset(start, Z_UP, 30, 20)
    assert np.linalg.norm(moved) == pytest.approx(1)  # 거리는 그대로
    assert math.degrees(math.asin(moved[2])) == pytest.approx(20 * ORBIT_DEG_PER_PX)


def test_drag_stops_before_the_pole():
    moved = orbit_offset((0.0, -1.0, 0.0), Z_UP, 0, 10000)
    assert math.degrees(math.asin(moved[2])) == pytest.approx(ORBIT_MAX_ELEVATION_DEG)


@pytest.mark.parametrize("z", [1, -1])
def test_drag_from_straight_above_or_below_keeps_the_front_toward_the_viewer(z):
    moved = orbit_offset((0.0, 0.0, float(z)), up_for((0, 0, z)), 0, 0)
    assert moved[1] < 0 and abs(moved[0]) < 1e-9  # 앞쪽(-y)으로 살짝 기운다. 화면이 돌아가지 않는다


def test_triad_labels_every_axis():
    svg = triad_svg(camera_axes((1, -1, 1), (0, 0, 0), Z_UP))
    assert all(f">{name}</text>" in svg for name in "XYZ")


def _client(camera):
    from types import SimpleNamespace

    class Atomic:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

    return SimpleNamespace(client_id=1, camera=camera, atomic=Atomic)


def test_scene_aims_every_ready_camera_and_keeps_distance(backend, monkeypatch):
    from types import SimpleNamespace

    scene = backend.scene
    camera = SimpleNamespace(position=np.array((0.3, -0.2, 0.2)), look_at=np.array((0.3, 0.2, 0.0)), up_direction=np.array(Z_UP))
    distance = np.linalg.norm(camera.position - camera.look_at)
    monkeypatch.setattr(scene.server, "get_clients", lambda: {1: _client(camera)})
    monkeypatch.setattr(scene, "_camera_ready", {1})
    scene.look_from((0, 0, 1))
    assert camera.position == pytest.approx(camera.look_at + (0, 0, distance))
    assert tuple(camera.up_direction) == (0.0, 1.0, 0.0)
    scene.orbit(10, 10)
    assert tuple(camera.up_direction) == Z_UP
    assert np.linalg.norm(camera.position - camera.look_at) == pytest.approx(distance)
    scene.home()
    assert tuple(camera.position) == scene._home[0] and tuple(camera.look_at) == scene._home[1]
    assert scene.camera == scene._home


def test_scene_skips_a_camera_not_ready_yet(backend, monkeypatch):
    scene = backend.scene

    class NotReady:
        @property
        def look_at(self):
            raise AssertionError("첫 소식 전에는 읽지 않는다")

    monkeypatch.setattr(scene.server, "get_clients", lambda: {1: _client(NotReady())})
    monkeypatch.setattr(scene, "_camera_ready", set())
    scene.look_from((1, 0, 0))
    scene.orbit(1, 1)
    scene.home()


@pytest.mark.parametrize("path", ["/manual", "/", "/teach", "/safety"])
def test_every_3d_page_has_the_cube(backend, path):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await user.should_see(marker="scene3d")
            await user.should_see(marker="viewcube")
            await user.should_see(marker="viewcube-home")

    asyncio.run(go())
