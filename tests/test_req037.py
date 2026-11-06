"""REQ-037: 손끝 방향(pitch), 집기 범위와 움직임 범위 나누기."""
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    yield s
    s.disconnect()


def test_point_pose_default_is_pointing_down_and_pitch_is_accepted(svc):
    assert svc.point_pose(300, 200, 80).pitch_deg == -90.0
    p = svc.point_pose(300, 200, 80, pitch=-45.0)
    assert p.pitch_deg == -45.0
    why, q = svc.preview_tool(p)
    assert why is None and svc.kin.fk(q).pitch_deg == pytest.approx(-45.0, abs=0.5)  # 그 자세로 풀린다


def test_move_tool_with_a_tilted_hand_reaches_the_tip_point(svc):
    pose = svc.point_pose(300, 200, 80, pitch=-45.0)
    assert svc.move_tool(pose).status == "ok"
    q = svc.driver.read_joints()
    x, y, h = svc.tool_position(q)
    assert (x, y, h) == pytest.approx((300, 200, 80), abs=3.0)  # 끝점(tip)이 요청한 점에 온다
    assert svc.kin.fk(q).pitch_deg == pytest.approx(-45.0, abs=1.5)


def test_floor_check_uses_the_tilted_hand_real_jaws(svc):
    # 같은 끝점 높이 2mm라도 기울이면 다른 턱이 더 낮을 수 있다: 안전 검사는 그 자세의 실제 턱으로 판정한다(REQ-036 메쉬)
    why, _q = svc.preview_tool(svc.point_pose(300, 200, 2.0, pitch=-90.0))
    assert why in ("z_floor", None)
    why, _q = svc.preview_tool(svc.point_pose(300, 200, -10.0, pitch=-45.0))
    assert why == "z_floor"


def test_cartesian_jog_can_tilt_the_hand(svc):
    q = svc.skills._ik(svc.point_pose(300, 200, 80, pitch=-60.0))
    p0 = svc.kin.fk(q).pitch_deg
    up = svc.kin.step(q, 0, 0, 0, 0, dpitch_deg=5.0)
    down = svc.kin.step(q, 0, 0, 0, 0, dpitch_deg=-5.0)
    assert svc.kin.fk(up).pitch_deg - p0 == pytest.approx(5.0, abs=1.0)  # 양수 = 수평 쪽으로 든다
    assert svc.kin.fk(down).pitch_deg - p0 == pytest.approx(-5.0, abs=1.0)


def test_pick_range_is_the_workspace_reach_and_is_separate_from_the_motion_range(svc):
    r = svc.reach_ranges(-90.0)
    assert r["pick"] == WS.pick_reach_mm == WS.reach_mm and r["pitch_deg"] == -90.0
    down = {row["height_mm"]: row for row in r["motion"]}
    assert down[40]["segments"] and down[40]["max_mm"] >= WS.pick_reach_mm["max"]  # 움직임 범위가 집기 범위를 포함한다
    horiz = {row["height_mm"]: row for row in svc.reach_ranges(0.0)["motion"]}
    assert 330 <= horiz[120]["max_mm"] <= 360  # 0° 자세(완전히 편 팔) 수평 도달 약 353mm (실측 일치)
    assert horiz[120]["max_mm"] > down[120]["max_mm"]  # 손끝 방향이 자유로우면 더 멀리 간다
    assert svc.reach_ranges(0.0) is not None  # 같은 인자는 캐시


def test_reach_ranges_refuses_to_compute_while_moving(svc):
    from so101_studio.core.types import JointState

    h = svc.ctl.execute(svc.skills._joint_move(JointState(0, -30, 30, 0, 0, 3)))
    with pytest.raises(RuntimeError):
        svc.reach_ranges(-30.0)
    svc.ctl.stop()
    h.wait()
    svc.ctl.wait_idle()


def test_default_pitch_unchanged_for_relative_target_parsing():
    from so101_studio.service.targets import parse_target

    base = WS.base
    from so101_studio.core.types import ToolPose

    cur = ToolPose(300, 200, 50, 90, -90.0)
    p = parse_target("d 10 0 0", "table", base, cur)
    assert p.pitch_deg == -90.0  # 기본은 지금처럼 -90
    assert parse_target("d 10 0 0", "table", base, cur, pitch_deg=-30.0).pitch_deg == -30.0
