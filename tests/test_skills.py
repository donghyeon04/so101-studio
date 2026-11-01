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


def lowest_xy(d, kin):
    z = [kin.fk(q) for q in d.log]
    low = min(z, key=lambda p: p.z_mm)
    return low.x_mm, low.y_mm


def run_pick(kin, **cfg_kw):
    from dataclasses import replace

    d = Recorder(CFG.poses["home"])
    cfg = replace(CFG, **cfg_kw)
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin), CFG.control_hz)
    r = Skills(ctl, kin, cfg, WS, speed_pct=100).pick(obj(x=300, y=210, angle=0))
    ctl.close()
    assert r.status == "ok"
    return d


def test_grasp_center_offset_shifts_the_target_along_the_jaw_axis(kin):
    # angle 0 -> tool yaw 0 -> 턱 축은 작업대 +y. offset 10이면 목표는 물체에서 -10mm(y)
    base = lowest_xy(run_pick(kin, grasp_center_offset_mm=0, moving_jaw_sign=1, gripper_margin_mm=0), kin)
    shifted = lowest_xy(run_pick(kin, grasp_center_offset_mm=10, moving_jaw_sign=1, gripper_margin_mm=0), kin)
    assert shifted[1] - base[1] == pytest.approx(-10, abs=2.5)
    assert shifted[0] - base[0] == pytest.approx(0, abs=2.5)


def test_nudge_toward_moving_jaw_before_closing(kin):
    # 여유 폭 20mm -> 닫기 직전에 가동 턱 쪽으로 10mm. sign에 따라 방향이 바뀐다
    def y_at_close_start(d):
        top = max(q.gripper for q in d.log)
        last_open = max(i for i, q in enumerate(d.log) if q.gripper == top)  # 열린 상태의 마지막 샘플 = 닫기 시작
        return kin.fk(d.log[last_open]).y_mm

    plus = y_at_close_start(run_pick(kin, moving_jaw_sign=1, gripper_margin_mm=20))
    minus = y_at_close_start(run_pick(kin, moving_jaw_sign=-1, gripper_margin_mm=20))
    assert plus - minus == pytest.approx(20, abs=4)
