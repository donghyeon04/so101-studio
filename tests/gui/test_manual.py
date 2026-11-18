"""직접 조작 (gui-spec G4). 시뮬레이션 팔로 닿는 점, 닿지 않는 점, 조그, 그리퍼를 본다."""

import asyncio
import time
from dataclasses import fields

import numpy as np
import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.backend import CART_AXES, JOG_JOINTS
from so101_studio.gui.scene3d import table_hit
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
    assert "/marks/path" in backend.scene.server.scene._handle_from_node_name
    backend.scene.clear_target()


def test_plan_unreachable_point_gives_reason_and_no_preview(backend):
    plan = backend.plan(*UNREACHABLE)
    assert not plan.ok and plan.text
    assert backend.reach == "안 닿음"
    assert backend.scene._preview_root.visible is False
    assert "/marks/path" not in backend.scene.server.scene._handle_from_node_name
    backend.scene.clear_target()


@pytest.mark.slow
def test_horizontal_tip_plans_moves_and_draws_reach_rings(backend):
    """REQ-037: 손끝 각도 0(수평)으로 앞쪽 점에 닿고, 집기 범위와 움직임 범위 호를 3D에 그린다."""
    nodes = backend.scene.server.scene._handle_from_node_name
    b = backend.arm.ws.base
    plan = backend.plan(b.x_mm, b.y_mm + 300.0, 120, 90, 0)  # 팔 정면 반경 300
    assert plan.ok, plan.text
    assert "/marks/reach/pick" in nodes and "/marks/reach/motion" in nodes
    backend.scene.clear_target()
    assert "/marks/reach/pick" not in nodes
    assert backend.move_to(b.x_mm, b.y_mm + 300.0, 120, 90, 0) is None
    assert backend.arm.status().tool.pitch_deg == pytest.approx(0, abs=5)
    assert backend.jog_step("cartesian", CART_AXES.index("pitch"), -15) is None  # 아래로 15°
    assert backend.arm.status().tool.pitch_deg == pytest.approx(-15, abs=5)
    assert backend.home() is None


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


def test_page_blocks_unreachable_point_and_moves_to_reachable(backend):
    async def check(user):
        move = _one(user, "move")
        x, y = _one(user, "target-x"), _one(user, "target-y")
        assert await _until(lambda: _one(user, "reach").text.startswith("닿습니다") and move.enabled)
        x.meta.set(UNREACHABLE[0]), y.meta.set(UNREACHABLE[1])
        assert await _until(lambda: not _one(user, "reach").text.startswith("닿습니다") and _one(user, "reach").text != "계산 중")
        assert not move.enabled
        x.meta.set(REACHABLE[0]), y.meta.set(REACHABLE[1])
        assert await _until(lambda: _one(user, "reach").text.startswith("닿습니다") and move.enabled)
        user.find(marker="move").click()
        assert await _until(lambda: np.linalg.norm(_tip(backend) - np.array(REACHABLE[:3])) < 5, 15.0)

    in_app(backend, check)


def test_picked_point_from_3d_fills_the_inputs(backend, monkeypatch):
    from so101_studio.gui.pages import manual as manual_page

    toasts = []
    monkeypatch.setattr(manual_page.ui, "notify", lambda *a, **k: toasts.append(a))

    async def check(user):
        x = _one(user, "target-x")
        assert await _until(lambda: _one(user, "reach").text != "계산 중")
        for dx in range(0, 25, 5):  # 손잡이를 끄는 것처럼 점이 여러 번 바뀐다
            backend.scene.pick(300.0 + dx, 210.0, 55.0)
            await asyncio.sleep(0.15)
        assert await _until(lambda: x.value == 320.0)
        assert _one(user, "target-y").value == 210.0 and _one(user, "target-h").value == 55.0
        assert toasts == []  # 닿는 점을 옮기는 동안 알림이 쌓여 화면을 가리지 않는다

    in_app(backend, check)


def test_target_undo_restores_typed_and_picked_points_without_moving(backend):
    """목표 좌표 되돌리기(Ctrl+Z·[되돌리기], 사용자 11-14): 입력칸으로 바꾼 값과 3D에서 고른 점을 차례로 되돌리고, 팔은 움직이지 않는다. 3D 밑에 현재 좌표가 보인다."""
    async def check(user):
        x, y, h = _one(user, "target-x"), _one(user, "target-y"), _one(user, "target-h")
        assert await _until(lambda: _one(user, "reach").text != "계산 중")
        assert await _until(lambda: _one(user, "current-pose").text.startswith("현재 좌표  x "))
        before = _tip(backend)
        x.meta.set(320.0)
        assert await _until(lambda: x.value == 320.0)
        await asyncio.sleep(0.7)  # 0.6초 안의 변경은 한 번으로 친다(12.9 가)
        backend.scene.pick(280.0, 230.0, 60.0)
        assert await _until(lambda: (x.value, y.value, h.value) == (280.0, 230.0, 60.0))
        backend.scene.pick(285.0, 235.0, 60.0)  # 끄는 중의 중간 점: 따로 되돌리지 않는다
        assert await _until(lambda: x.value == 285.0)
        user.find(marker="target-undo").click()
        assert await _until(lambda: (x.value, y.value, h.value) == (320.0, 200.0, 40.0))
        user.find(marker="target-undo").click()
        assert await _until(lambda: x.value == 300.0)
        assert np.linalg.norm(_tip(backend) - before) < 1.0  # 되돌리기는 목표만 바꾼다
        assert not user.notify.contains("되돌릴")  # 되돌릴 것이 있는 동안은 알림이 없다(12.9 가)

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
        await user.should_see(marker="resume", retries=30)  # 기본 0.3초는 부하가 있으면 상태줄 갱신보다 짧다
        user.find(marker="resume").click()
        assert await _until(lambda: backend.arm.ctl.state == "idle")
        await user.should_not_see(marker="resume", retries=30)

    in_app(backend, check)


def test_jog_joint_table_matches_jointstate_order(backend):
    names = [f.name for f in fields(backend.arm.status().joints)]
    assert list(JOG_JOINTS) == names[:5]


def test_joint_to_moves_one_joint_to_the_typed_angle(backend):
    """결정 0042 7번: 관절 하나를 입력한 각도로."""
    backend.set_speed(100)
    assert backend.joint_to(JOG_JOINTS.index("wrist_roll"), 20.0) is None
    assert abs(backend.arm.status().joints.wrist_roll - 20.0) < 1.0
    assert backend.joint_to(JOG_JOINTS.index("wrist_roll"), 0.0) is None


@pytest.mark.slow
def test_pick_place_moves_object_to_the_place_point(backend):
    """gui-spec 11절 B7: 집기 점에서 집어 놓기 점에 놓는다. 범위 밖이면 이유를 돌려준다."""
    backend.set_speed(100)
    assert backend.pick_place((215, 160), 0, 30, 30, (215, 250)) is None  # 팔 정면(결정 0067 판). 놓기 점은 기본 수동 물체(반지름 50)와 겹치지 않게(#229부터 알려진 물체를 피한다)
    assert backend.pick_place((215, 160), 0, 58, 58, (215, 250), height_mm=39) is None  # REQ-034 높이 칸
    assert "집기 점이 집기 범위 밖" in (backend.pick_place((215, 30), 0, 30, 30, None) or "")


def test_pick_place_passes_place_target_and_keeps_tilt(backend, monkeypatch):
    """arm 6.6.1: 집기에 놓을 곳을 같이 넘겨야 병렬 auto가 기울여 집기를 고른다. 고른 기울기는 알림에 쓰도록 남긴다."""
    from so101_studio.core.types import MotionResult, ToolPose

    calls = []

    def fake(arm):
        def pick_and_place(obj, zone=None, place_at=None):
            calls.append((zone, place_at))
            arm.skills.last_pick_info = {"approach": {"beta": 35.0, "phi": 90.0, "gamma": 0.0, "face": "width"}}
            return MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))
        return pick_and_place

    for arm in backend._targets():
        monkeypatch.setattr(arm, "pick_and_place", fake(arm))
    assert backend.pick_place((300, 200), 0, 30, 30, (420, 120), height_mm=39) is None
    assert calls[-1] == (None, (420, 120)) and backend.last_pick_approach["beta"] == 35.0
    assert backend.pick_place((300, 200), 0, 30, 30, None, height_mm=39) is None
    assert calls[-1] == (next(iter(backend.arm.ws.drop_zones)), None)


def test_joystick_vector_moves_then_stops(backend):
    """G13: 조이스틱 비율을 이어 보내는 동안 움직이고, 끊기면 0.3초 데드맨으로 선다."""
    assert _wait_for(lambda: backend.arm.ctl.state == "idle")  # 앞 시험의 동작이 끝나기를 기다린다
    assert backend.move_to(*REACHABLE) is None
    before = _tip(backend)
    for _ in range(5):
        assert backend.jog_vector([0.0, 0.5, 0.0])
        time.sleep(0.1)
    backend.jog_release("cartesian")
    assert _wait_for(lambda: backend.arm.ctl.state == "idle")
    moved = _tip(backend) - before
    assert moved[1] > 5 and abs(moved[0]) < 3


def test_height_required_and_low_lift_get_their_own_text(backend):
    """REQ-057: 결정 0052의 height_required와 놓은 뒤 상승 최소 미달(stopped, reason low_lift)은 일반 문구 대신 원인과 할 일을 보인다."""
    from so101_studio.core.types import MotionResult, ToolPose
    from so101_studio.gui.backend import MOTION_TEXT, Backend

    zero = ToolPose(0, 0, 0, 0)
    assert Backend._run_on(backend.sim, lambda arm: MotionResult("height_required", 0.0, zero, reason="높이가 필요합니다")) == MOTION_TEXT["height_required"]
    text = Backend._run_on(backend.sim, lambda arm: MotionResult("stopped", 0.0, zero, reason="low_lift: 49mm 이상 올라가야 하는데 15mm만"))
    assert "충분히 올라가지 못해" in text and "49mm" in text and text != MOTION_TEXT["stopped"]
    for status, reason, words in (("ik_fail", "known_object: 놓을 자리가 알려진 물체와 겹친다 (object@(350,200))", "겹칩니다"),
                                  ("stopped", "drop_zone_full: 칸 3개가 모두 찼다", "칸이 모두 차서"),
                                  ("stopped", "low_hover: hover가 55mm 이상이어야", "10mm를 지키며"),  # REQ-058
                                  ("stopped", "blocked: wrist_flex 추종 오차 2.3 > 문턱 1.9 (empty)", "막힘 감지로"),
                                  ("stopped", "home_blocked: 방금 놓은 물체와 0.1mm로 지나간다", "손을 먼저 위로")):  # 결정 0059: 정지 해제 안내 대신
        text = Backend._run_on(backend.sim, lambda arm, s=status, r=reason: MotionResult(s, 0.0, zero, reason=r))
        assert words in text and reason.split(":", 1)[1].strip() in text  # 팔이 준 세부 내용도 그대로 보인다


def test_manual_input_marks_objects_that_need_a_height_and_applies_it(backend):
    """작업 실행이 height_required로 건너뛴 물체는 표에 '높이 필요', 이유 문장, [고른 물체에 이 높이]로 고친다 (REQ-057)."""
    from so101_studio.core.types import TaskEvent

    backend.reset_objects()
    target = backend.add_object(260.0, 240.0, 45.0, 30.0, 100.0, 0.0)  # 높이 없는 물체(기본 물체는 #244부터 높이가 있다)
    backend.state.events.append(TaskEvent(time.time(), "PICK", target, "height_required"))
    try:
        assert backend.height_needed() == {target}

        async def check(user):
            await user.should_see(marker="height-needed", retries=30)
            assert await _until(lambda: target in _one(user, "height-needed").text)
            assert await _until(lambda: any(r["height"] == "높이 필요" for r in _one(user, "object-table").rows if r["id"] == target))
            backend.set_object_height(target, 40.0)
            await user.should_not_see(marker="height-needed", retries=30)

        in_app(backend, check, "/input")
        assert backend.height_needed() == set() and next(o for o in backend.objects() if o.id == target).height_mm == 40.0
    finally:
        backend.state.events.clear()
        backend.reset_objects()


def test_plane_draws_rectangle_zone_and_its_slots():
    """사각형 배출 구역(11.3.5)은 크기대로 그리고, 팔이 쓰는 칸(WorkspaceConfig.zone_slots)을 번호와 함께 찍는다. 옛 점 형식은 80x80."""
    from so101_studio.core.frames import Pose2D
    from so101_studio.gui.sensors.manual_view import plane_svg

    base, reach = Pose2D(300, 50, 90), {"min": 90, "max": 250}
    svg = plane_svg({"x": 600, "y": 400}, [], {"right": {"x_mm": 480, "y_mm": 150, "w_mm": 100, "h_mm": 80, "slot_mm": 75}}, base, reach)
    assert 'width="100" height="80"' in svg and svg.count('r="6"') == 3  # 기본 4칸 중 집기 범위 안 3칸
    old = plane_svg({"x": 600, "y": 400}, [], {"right": {"x_mm": 500, "y_mm": 150}}, base, reach)
    assert 'width="80" height="80"' in old and 'r="6"' not in old


def test_reset_button_puts_the_point_back_without_moving(backend):
    """3D 손잡이를 멀리 끌었을 때 [점 처음 자리로]가 입력칸을 기본 점으로 되돌린다. 팔은 움직이지 않는다."""
    from so101_studio.gui.pages.manual import DEFAULT_TARGET

    async def check(user):
        assert await _until(lambda: _one(user, "reach").text != "계산 중")
        backend.scene.pick(520.0, 380.0, 150.0)  # 멀리 보낸 점
        assert await _until(lambda: _one(user, "target-x").value == 520.0)
        before = backend.arm.status().joints
        user.find(marker="target-reset").click()
        assert await _until(lambda: _one(user, "target-x").value == DEFAULT_TARGET["x"])
        assert _one(user, "target-y").value == DEFAULT_TARGET["y"] and _one(user, "target-h").value == DEFAULT_TARGET["h"]
        assert backend.arm.status().joints == before

    in_app(backend, check)
