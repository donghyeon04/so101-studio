"""직접 조작 (gui-spec G4). 시뮬레이션 팔로 닿는 점, 닿지 않는 점, 조그, 그리퍼를 본다."""

import asyncio
import time
from dataclasses import fields

import numpy as np
import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.backend import JOG_JOINTS
from so101_studio.gui.scene3d import VIEWS, table_hit
from so101_studio.service.targets import tip_vector

REACHABLE = (300.0, 200.0, 40.0, 90.0)
UNREACHABLE = (30.0, 380.0, 40.0, 90.0)


def _tip(backend) -> np.ndarray:
    st = backend.arm.status()
    t = st.tool
    tip = tip_vector(backend.arm.kin, t.pitch_deg, t.yaw_deg, st.joints.gripper, "tip")
    return np.array([t.x_mm + tip[0], t.y_mm + tip[1], t.z_mm + tip[2] - backend.arm.cfg.table_z_mm])


def _wait_for(check, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.02)
    return False


async def _until(check, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def in_app(backend, check, path="/manual") -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await check(user)

    asyncio.run(go())


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


def test_table_hit_finds_table_point():
    assert table_hit((0.3, 0.0, 0.5), (0.0, 0.5, -0.5)) == pytest.approx((300.0, 500.0))
    assert table_hit((0.3, 0.0, 0.5), (0.0, 0.0, 1.0)) is None  # 위를 향한 광선
    assert table_hit((0.3, 0.0, -0.1), (0.0, 0.0, -1.0)) is None  # 작업대 아래


def test_plan_reachable_point_shows_path_and_preview_without_moving(backend):
    before = backend.arm.status().joints
    plan = backend.plan(*REACHABLE)
    assert plan.ok and plan.text.startswith("닿습니다")
    assert backend.reach == "닿음"
    assert backend.arm.status().joints == before
    assert backend.scene._preview_root.visible is True
    assert "/path" in backend.scene.server.scene._handle_from_node_name
    backend.scene.clear_target()


def test_plan_unreachable_point_gives_reason_and_no_preview(backend):
    plan = backend.plan(*UNREACHABLE)
    assert not plan.ok and plan.text
    assert backend.reach == "안 닿음"
    assert backend.scene._preview_root.visible is False
    assert "/path" not in backend.scene.server.scene._handle_from_node_name
    backend.scene.clear_target()


def test_move_to_reaches_the_point(backend):
    assert backend.move_to(*REACHABLE) is None
    assert _tip(backend) == pytest.approx(np.array(REACHABLE[:3]), abs=5)


def test_jog_step_joint_moves_only_that_joint(backend):
    before = backend.arm.status().joints
    assert backend.jog_step("joint", 0, 5) is None
    after = backend.arm.status().joints
    assert after.shoulder_pan == pytest.approx(before.shoulder_pan + 5, abs=0.5)
    assert after.elbow_flex == pytest.approx(before.elbow_flex, abs=0.5)


def test_jog_step_cartesian_moves_tip_along_axis(backend):
    assert backend.move_to(*REACHABLE) is None
    before = _tip(backend)
    assert backend.jog_step("cartesian", 2, 10) is None
    assert _tip(backend) - before == pytest.approx(np.array([0, 0, 10]), abs=2)


def test_hold_jog_moves_then_release_stops(backend):
    q = backend.arm.status().joints
    sign = 1 if q.shoulder_pan < 0 else -1
    for _ in range(5):
        assert backend.jog("joint", 0, sign)
        time.sleep(0.1)
    backend.jog_release("joint")
    assert _wait_for(lambda: backend.arm.ctl.state == "idle")
    assert sign * (backend.arm.status().joints.shoulder_pan - q.shoulder_pan) > 1


def test_hold_jog_stops_when_input_stops(backend):
    q = backend.arm.status().joints
    backend.jog("joint", 0, 1 if q.shoulder_pan < 0 else -1)
    assert _wait_for(lambda: backend.arm.ctl.state == "idle", 2.0)  # 데드맨 0.3초


def test_gripper_value_and_width(backend):
    assert backend.gripper(50) is None
    assert backend.arm.status().joints.gripper == pytest.approx(50, abs=1)
    assert backend.gripper_width(54) is None
    assert 20 < backend.arm.status().joints.gripper < 40


def test_set_speed_shows_in_readout(backend):
    before = backend.arm.speed
    backend.set_speed(20)
    try:
        assert backend.readout()["속도"] == "20 %"
    finally:
        backend.set_speed(before)


def test_stopped_arm_explains_and_resume_recovers(backend):
    backend.estop()
    q = backend.arm.status().joints
    assert backend.home() == "정지 상태입니다. 정지 해제를 눌러 주세요"
    backend.resume()
    assert backend.arm.status().joints == q  # 해제는 팔을 움직이지 않는다
    assert backend.jog_step("joint", 0, 1) is None


def test_views_cover_top_front_side(backend):
    assert set(VIEWS) == {"위", "앞", "옆"}
    for name in VIEWS:
        backend.scene.set_view(name)  # 열린 3D 칸이 없어도 오류가 없어야 한다


def test_page_blocks_unreachable_point_and_moves_to_reachable(backend):
    async def check(user):
        move = _one(user, "move")
        x, y = _one(user, "target-x"), _one(user, "target-y")
        assert await _until(lambda: _one(user, "reach").text.startswith("닿습니다") and move.enabled)
        x.value, y.value = UNREACHABLE[0], UNREACHABLE[1]
        assert await _until(lambda: not _one(user, "reach").text.startswith("닿습니다") and _one(user, "reach").text != "계산 중")
        assert not move.enabled
        x.value, y.value = REACHABLE[0], REACHABLE[1]
        assert await _until(lambda: _one(user, "reach").text.startswith("닿습니다") and move.enabled)
        user.find(marker="move").click()
        assert await _until(lambda: np.linalg.norm(_tip(backend) - np.array(REACHABLE[:3])) < 5, 15.0)

    in_app(backend, check)


def test_picked_point_from_3d_fills_the_inputs(backend):
    async def check(user):
        x = _one(user, "target-x")
        assert await _until(lambda: _one(user, "reach").text != "계산 중")
        backend.scene.pick(320.0, 210.0, 55.0)
        assert await _until(lambda: x.value == 320.0)
        assert _one(user, "target-y").value == 210.0 and _one(user, "target-h").value == 55.0

    in_app(backend, check)


def test_handle_drag_becomes_a_picked_point(backend):
    scene = backend.scene
    seq = scene.picked[0]
    scene._handle.position = (0.25, 0.18, 0.06)
    scene._on_drag(None)
    assert scene.picked[0] == seq + 1
    assert scene.picked[1:] == pytest.approx((250.0, 180.0, 60.0))
    scene._handle.position = (0.25, 0.18, -0.02)
    scene._on_drag(None)
    assert scene.picked[3] == 0.0  # 작업대 아래로는 못 내려간다


def test_view_only_browser_has_controls_off(backend):
    backend.owner = "다른 브라우저"

    async def check(user):
        await user.should_see("보기 전용")
        for marker in ("move", "home", "park", "jog-joint-0-1", "jog-joint-0--1", "speed"):
            assert await _until(lambda m=marker: not _one(user, m).enabled, 2.0), marker

    in_app(backend, check)


def test_resume_button_appears_only_when_stopped(backend):
    async def check(user):
        assert await _until(lambda: backend.owner is not None)
        await user.should_not_see(marker="resume")
        backend.estop()
        await user.should_see(marker="resume")
        user.find(marker="resume").click()
        assert await _until(lambda: backend.arm.ctl.state == "idle")
        await user.should_not_see(marker="resume")

    in_app(backend, check)


def test_jog_joint_table_matches_jointstate_order(backend):
    names = [f.name for f in fields(backend.arm.status().joints)]
    assert list(JOG_JOINTS) == names[:5]
