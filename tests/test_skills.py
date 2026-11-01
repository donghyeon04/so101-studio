import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.skills import Skills
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


class Recorder(SimDriver):
    def __init__(self, *a):
        super().__init__(*a)
        self.log = []

    def write_joints(self, q):
        super().write_joints(q)
        self.log.append(q)


@pytest.fixture(scope="module")
def kin():
    return Kinematics(CFG.urdf, WS.base)


def rig(kin):
    d = Recorder(CFG.poses["home"])
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin), CFG.control_hz)
    return d, ctl, Skills(ctl, kin, CFG, WS, speed_pct=100)


def obj(x=300, y=200, angle=0, width=30):
    return DetectedObject("t", x, y, angle, width, width, 1.0, "manual")


def test_pick_sequence_open_descend_close_rise(kin):
    d, ctl, sk = rig(kin)
    r = sk.pick(obj())
    ctl.close()
    assert r.status == "ok"
    z = [kin.fk(q).z_mm for q in d.log]
    g = [q.gripper for q in d.log]
    low = z.index(min(z))
    assert min(z) == pytest.approx(CFG.grasp_height_mm, abs=3)
    assert max(g[:low]) > 20 and g[-1] == pytest.approx(CFG.joint_limits_deg["gripper"][0], abs=1)  # 내려가기 전에 열고 올라온 뒤엔 닫혀 있다
    assert z[-1] == pytest.approx(CFG.grasp_height_mm + CFG.approach_height_mm, abs=3)  # 접근 높이까지 다시 올라온다


def test_pick_unreachable_object_does_not_move(kin):
    d, ctl, sk = rig(kin)
    r = sk.pick(obj(x=300, y=500))  # 작업대 밖(팔 도달 범위 밖)
    ctl.close()
    assert r.status == "ik_fail" and d.log == []


def test_place_opens_gripper_at_zone(kin):
    d, ctl, sk = rig(kin)
    r = sk.place("right")
    ctl.close()
    assert r.status == "ok" and d.log[-1].gripper == pytest.approx(CFG.gripper_open, abs=1)


def test_gripper_for_width_is_clamped_and_monotonic(kin):
    _d, ctl, sk = rig(kin)
    ctl.close()
    assert sk.gripper_for_width(0) == CFG.joint_limits_deg["gripper"][0]
    assert sk.gripper_for_width(40) < sk.gripper_for_width(60) < sk.gripper_for_width(500) + 1e-9
    assert sk.gripper_for_width(500) == CFG.joint_limits_deg["gripper"][1]


def run_pick(kin, **cfg_kw):
    from dataclasses import replace

    d = Recorder(CFG.poses["home"])
    cfg = replace(CFG, **cfg_kw)
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin), CFG.control_hz)
    r = Skills(ctl, kin, cfg, WS, speed_pct=100).pick(obj(x=300, y=210, angle=0, width=30))
    ctl.close()
    assert r.status == "ok"
    return d


def at_lowest(d, kin):
    q = min(d.log, key=lambda q: kin.fk(q).z_mm)
    return q, kin.fk(q)


def test_jaws_open_perpendicular_to_the_long_axis(kin):
    # 물체 각도 0(긴 변이 작업대 x축)이면 턱은 작업대 y축을 따라 벌어져야 한다 (가동 턱 쪽 = +y)
    d = run_pick(kin)
    q, _ = at_lowest(d, kin)
    mx, my = kin.moving_jaw_dir(q)
    assert (mx, my) == pytest.approx((0, 1), abs=0.1)


def test_jaw_axis_follows_the_object_angle(kin):
    import math

    d = Recorder(CFG.poses["home"])
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin), CFG.control_hz)
    r = Skills(ctl, kin, CFG, WS, speed_pct=100).pick(obj(x=300, y=210, angle=45, width=30))
    ctl.close()
    assert r.status == "ok"
    q, _ = at_lowest(d, kin)
    mx, my = kin.moving_jaw_dir(q)
    assert (mx, my) == pytest.approx((math.cos(math.radians(135)), math.sin(math.radians(135))), abs=0.1)


def test_fixed_jaw_face_ends_at_the_object_edge(kin):
    # FK 기준점 = 고정 턱 안쪽 면. 닫기 직전에는 그 면이 물체의 고정 턱 쪽 모서리(물체 중심에서 폭/2 만큼 -m 쪽)에 와야 한다
    d = run_pick(kin, gripper_margin_mm=20)
    top = max(q.gripper for q in d.log)
    last_open = max(i for i, q in enumerate(d.log) if q.gripper == top)  # 열린 상태의 마지막 샘플 = 닫기 시작
    p = kin.fk(d.log[last_open])
    mx, my = kin.moving_jaw_dir(d.log[last_open])
    along = (p.x_mm - 300) * mx + (p.y_mm - 210) * my  # 물체 중심 기준 m 방향 좌표
    assert along == pytest.approx(-15, abs=3)  # 폭 30 / 2
    across = (p.x_mm - 300) * -my + (p.y_mm - 210) * mx
    assert across == pytest.approx(0, abs=3)


def test_grasp_bias_shifts_toward_the_moving_jaw(kin):
    a = at_lowest(run_pick(kin, grasp_bias_mm=0), kin)[1]
    b = at_lowest(run_pick(kin, grasp_bias_mm=6), kin)[1]
    assert b.y_mm - a.y_mm == pytest.approx(6, abs=2)


@pytest.mark.parametrize("angle", [0, 45, 90, 135])
def test_pick_returns_to_hover_for_every_angle(kin, angle):
    d = Recorder(CFG.poses["home"])
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin), CFG.control_hz)
    r = Skills(ctl, kin, CFG, WS, speed_pct=100).pick(obj(x=300, y=200, angle=angle, width=30))
    ctl.close()
    assert r.status == "ok"
    assert kin.fk(d.log[-1]).z_mm == pytest.approx(CFG.grasp_height_mm + CFG.approach_height_mm, abs=3)


def test_width_table_follows_urdf_geometry(kin):
    _d, ctl, sk = rig(kin)
    ctl.close()
    assert sk.gripper_for_width(50) == pytest.approx(30.4, abs=1.0)  # 벌림 50mm는 값 약 30 (기하 계산)
    assert sk.gripper_for_width(94) == pytest.approx(63.6, abs=1.5)
