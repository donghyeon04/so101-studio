"""뷰큐브 (Fusion 360 ViewCube 모양): 방향 계산과 페이지 연결. 실제 끌기·클릭은 브라우저에서 확인한다."""

import asyncio
import math
import re

import numpy as np
import pytest

pytest.importorskip("nicegui")

import itertools

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui import scene3d as scene_module
from so101_studio.gui import theme
from so101_studio.gui.scene3d import (
    ORBIT_DEG_PER_PX,
    ORBIT_MAX_ELEVATION_DEG,
    VIEW_MIN_S,
    VIEW_S_PER_180,
    Z_UP,
    camera_rotation,
    orbit_offset,
    up_for,
    view_ease,
    view_path,
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
    monkeypatch.setattr(scene, "_settling", {})
    scene.look_from((0, 0, 1))
    scene._flight_thread.join(3)
    assert camera.position == pytest.approx(camera.look_at + (0, 0, distance))
    assert tuple(camera.up_direction) == (0.0, 1.0, 0.0)
    scene.orbit(10, 10)
    assert tuple(camera.up_direction) == Z_UP
    assert np.linalg.norm(camera.position - camera.look_at) == pytest.approx(distance)
    scene.home()
    scene._flight_thread.join(3)
    assert tuple(camera.position) == scene._home[0] and tuple(camera.look_at) == scene._home[1]
    assert scene.camera == scene._home


def test_view_ease_starts_and_stops_slowly():
    assert view_ease(0) == 0 and view_ease(1) == 1 and view_ease(0.5) == pytest.approx(0.5)
    assert view_ease(0.1) < 0.1 and view_ease(0.9) > 0.9
    samples = [view_ease(i / 100) for i in range(101)]
    assert all(b >= a for a, b in itertools.pairwise(samples))
    assert max(b - a for a, b in itertools.pairwise(samples)) * 100 == pytest.approx(1.5, abs=0.01)


def _view(direction, up=Z_UP, look_at=(0.0, 0.0, 0.0), distance=1.0):
    unit = np.array(direction, float) / np.linalg.norm(direction)
    return np.array(look_at) + unit * distance, np.array(look_at), up


def _turned(a, b) -> float:
    return math.degrees(np.linalg.norm((camera_rotation(*a).inverse() @ camera_rotation(*b)).log()))


@pytest.mark.parametrize(("goal", "degrees"), [((1, 0, 0), 90), ((1, -1, 0), 45), ((0, 1, 0), 180)])
def test_duration_follows_the_angle(goal, degrees):
    duration, _ = view_path(_view((0, -1, 0)), _view(goal))
    assert duration == pytest.approx(max(VIEW_MIN_S, VIEW_S_PER_180 * degrees / 180))


def test_small_turn_keeps_the_minimum_time():
    assert view_path(_view((0, -1, 0)), _view((0.01, -1, 0)))[0] == VIEW_MIN_S


def test_path_ends_exactly_and_keeps_distance_and_a_level_camera():
    start, goal = _view((0, -1, 0)), _view((1, -1, 1))
    _, at = view_path(start, goal)
    assert at(1.0) is goal
    assert at(0.0)[0] == pytest.approx(start[0])
    for t in np.linspace(0, 0.99, 12):
        position, look, up = at(t)
        assert np.linalg.norm(position - look) == pytest.approx(1)
        assert up @ (position - look) == pytest.approx(0, abs=1e-9)  # 위쪽이 시선과 수직이다
        assert up[2] > 0  # 도중에 화면이 뒤집히지 않는다


@pytest.mark.parametrize(("up", "z"), [((0.0, 1.0, 0.0), 1), ((0.0, -1.0, 0.0), -1)])
def test_path_through_the_pole_turns_the_short_way(up, z):
    start, goal = _view((0, 0, z), up), _view((0, -1, 0))
    _, at = view_path(start, goal)
    total = _turned(start, goal)
    assert total == pytest.approx(90)
    previous = 0.0
    for t in np.linspace(0.1, 0.9, 9):
        done = _turned(start, at(t))
        assert previous < done < total  # 항상 같은 쪽으로 간다
        previous = done
    assert _turned(start, at(0.5)) == pytest.approx(total / 2)


def test_home_path_moves_the_look_point_too():
    start, goal = _view((1, 0, 0), distance=1.0), _view((0, -1, 1), look_at=(0.3, 0.2, 0.0), distance=0.5)
    _, at = view_path(start, goal)
    position, look, _ = at(0.5)
    assert look == pytest.approx((0.15, 0.1, 0.0))
    assert np.linalg.norm(position - look) == pytest.approx(0.75)


def test_new_click_and_drag_stop_the_running_flight(backend, monkeypatch):
    from types import SimpleNamespace

    scene = backend.scene
    camera = SimpleNamespace(position=np.array((0.3, -0.2, 0.2)), look_at=np.array((0.3, 0.2, 0.0)), up_direction=np.array(Z_UP))
    monkeypatch.setattr(scene.server, "get_clients", lambda: {1: _client(camera)})
    monkeypatch.setattr(scene, "_camera_ready", {1})
    monkeypatch.setattr(scene, "_settling", {})
    scene.look_from((0, 1, 0))
    first = scene._flight_thread
    scene.look_from((1, 0, 0))
    first.join(1)
    assert not first.is_alive()
    second = scene._flight_thread
    assert scene._flying
    scene.orbit(1, 0)
    second.join(1)
    assert not second.is_alive() and not scene._flying
    assert tuple(camera.up_direction) == Z_UP


real_view_path = view_path


def _echoing_scene(backend, monkeypatch):
    """브라우저가 값을 돌려주는 흉내. (장면, 카메라, 돌려주기 함수)."""
    from types import SimpleNamespace

    scene = backend.scene
    callbacks = []
    camera = SimpleNamespace(
        position=np.array((0.3, -0.2, 0.2)), look_at=np.array((0.3, 0.2, 0.0)), up_direction=np.array(Z_UP), on_update=callbacks.append
    )
    scene._watch_camera(_client(camera))
    monkeypatch.setattr(scene.server, "get_clients", lambda: {1: _client(camera)})
    monkeypatch.setattr(scene, "_settling", {})
    monkeypatch.setattr(scene, "_sent", {})
    monkeypatch.setattr(scene, "camera", scene.camera)
    monkeypatch.setattr(scene, "_camera_ready", {1})

    def echo(position):
        camera.position = np.array(position, float)
        callbacks[-1](camera)

    return scene, camera, echo


def test_late_echoes_do_not_turn_the_cube_back(backend, monkeypatch):
    scene, camera, echo = _echoing_scene(backend, monkeypatch)
    start = camera.position.copy()
    scene.look_from((0, 0, 1))
    scene._flight_thread.join(3)
    goal = scene.camera
    echo(start)  # 전환 시작 무렵의 값이 끝난 뒤에 도착한다
    assert scene.camera is goal
    echo(goal[0])  # 브라우저가 따라왔다
    assert not scene._settling
    echo(start)  # 이제부터는 사람이 3D를 직접 돌린 것이다
    assert scene.camera[0] == pytest.approx(start)


def test_person_can_turn_the_3d_after_the_settle_time(backend, monkeypatch):
    scene, camera, echo = _echoing_scene(backend, monkeypatch)
    start = camera.position.copy()
    scene.look_from((0, 0, 1))
    scene._flight_thread.join(3)
    monkeypatch.setattr(scene, "_settling", {1: (scene._settling[1][0], 0.0)})  # 기한이 지났다
    echo(start)
    assert scene.camera[0] == pytest.approx(start)


def test_next_click_starts_from_what_was_sent_not_the_late_echo(backend, monkeypatch):
    scene, camera, echo = _echoing_scene(backend, monkeypatch)
    start = camera.position.copy()
    scene.look_from((0, 0, 1))
    scene._flight_thread.join(3)
    sent = scene._sent[1]
    echo(start)  # 늦은 값
    seen = []
    monkeypatch.setattr(scene_module, "view_path", lambda a, b: seen.append(a) or real_view_path(a, b))
    scene.look_from((1, 0, 0))
    scene._flight_thread.join(3)
    assert seen[0] is sent


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


def test_cube_has_no_css_transition():
    # 큐브 행렬은 거울상이라 브라우저 보간이 위 시점 근처에서 축을 0으로 찌그러뜨린다(뒤집혀 보임)
    assert "vc-cube" in theme._CSS
    assert "transition" not in next(line for line in theme._CSS.splitlines() if line.startswith(".vc-cube"))


@pytest.mark.parametrize("path", ["/manual", "/", "/teach", "/safety"])
def test_every_3d_page_has_the_cube(backend, path):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await user.should_see(marker="scene3d")
            await user.should_see(marker="viewcube")
            await user.should_see(marker="viewcube-home")

    asyncio.run(go())
