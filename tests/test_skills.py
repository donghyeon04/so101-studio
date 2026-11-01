import math
from dataclasses import replace

import numpy as np
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
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)
WIDTH = 30.0
BLOCK = float(np.interp(WIDTH, [p[0] for p in TABLE], [p[1] for p in TABLE]))  # 폭 30mm 물체가 있을 때 그리퍼가 멈추는 값


def gap_of(v):
    pts = sorted(CFG.gripper_width_mm)
    return float(np.interp(v, [p[0] for p in pts], [p[1] for p in pts]))


class Recorder(SimDriver):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.log = []

    def write_joints(self, q):
        super().write_joints(q)
        self.log.append(self._q)


@pytest.fixture(scope="module")
def kin():
    return Kinematics(CFG.urdf, WS.base)


def rig(kin, block=None, **cfg_kw):
    d = Recorder(CFG.poses["home"], grip_block=block)
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin), CFG.control_hz)
    return d, ctl, Skills(ctl, kin, replace(CFG, **cfg_kw), WS, speed_pct=100)


def obj(x=300, y=210, angle=0, width=0.0):
    return DetectedObject("t", x, y, angle, width, width, 1.0, "manual")


def run_pick(kin, block=BLOCK, **kw):
    d, ctl, sk = rig(kin, block)
    r = sk.pick(obj(**kw))
    ctl.close()
    return d, r


def test_pick_unknown_size_closes_until_contact(kin):
    d, r = run_pick(kin)
    assert r.status == "ok"
    assert d.log[-1].gripper == pytest.approx(BLOCK, abs=0.6)  # 물체에서 멈춘다(끝까지 닫히지 않는다)


def test_pick_without_an_object_reports_no_object(kin):
    _d, r = run_pick(kin, block=None)
    assert r.status == "no_object"


def test_jaw_gap_stays_centered_on_the_object_while_closing(kin):
    # 닫는 동안 내내 틈의 중심이 물체 중심(300, 210)에 있어야 한다 = 가운데 있는 물체를 양쪽에서 잡는 모양
    d, r = run_pick(kin)
    assert r.status == "ok"
    top = max(q.gripper for q in d.log)
    first = next(i for i, q in enumerate(d.log) if q.gripper == top)
    closing = [q for q in d.log[first:] if BLOCK + 0.5 < q.gripper < top - 0.5]
    assert len(closing) > 20
    for q in closing[:: max(1, len(closing) // 30)]:
        p, (mx, my) = kin.fk(q), kin.moving_jaw_dir(q)
        center = (p.x_mm - 300) * mx + (p.y_mm - 210) * my + gap_of(q.gripper) / 2
        across = (p.x_mm - 300) * -my + (p.y_mm - 210) * mx
        assert center == pytest.approx(0, abs=4)
        assert across == pytest.approx(0, abs=4)


def test_open_gap_is_the_configured_maximum(kin):
    d, _r = run_pick(kin)
    assert gap_of(max(q.gripper for q in d.log)) == pytest.approx(CFG.grasp_max_open_mm, abs=3)


def test_jaws_open_perpendicular_to_the_long_axis(kin):
    d, _ = run_pick(kin)
    low = min(d.log, key=lambda q: kin.fk(q).z_mm)
    assert kin.moving_jaw_dir(low) == pytest.approx((0, 1), abs=0.1)  # 각도 0 -> 턱은 작업대 y축을 따라


def test_jaw_axis_follows_the_object_angle(kin):
    d, _ = run_pick(kin, angle=45)
    low = min(d.log, key=lambda q: kin.fk(q).z_mm)
    assert kin.moving_jaw_dir(low) == pytest.approx((math.cos(math.radians(135)), math.sin(math.radians(135))), abs=0.1)


@pytest.mark.parametrize("angle", [0, 45, 90, 135])
def test_pick_returns_to_hover_for_every_angle(kin, angle):
    d, r = run_pick(kin, x=300, y=200, angle=angle)
    assert r.status == "ok"
    assert kin.fk(d.log[-1]).z_mm == pytest.approx(CFG.grasp_height_mm + CFG.approach_height_mm, abs=3)


def test_known_width_with_fit_policy_opens_only_as_much_as_needed(kin):
    d, ctl, sk = rig(kin, BLOCK, grasp_open_policy="fit")
    r = sk.pick(obj(width=WIDTH))
    ctl.close()
    assert r.status == "ok"
    assert gap_of(max(q.gripper for q in d.log)) == pytest.approx(WIDTH + CFG.gripper_margin_mm, abs=3)


def test_grasp_bias_shifts_toward_the_moving_jaw(kin):
    def start_of_closing(bias):
        d, ctl, sk = rig(kin, BLOCK, grasp_bias_mm=bias)
        sk.pick(obj())
        ctl.close()
        top = max(q.gripper for q in d.log)
        return kin.fk(d.log[max(i for i, q in enumerate(d.log) if q.gripper == top)])  # 하강이 끝나고 닫기 시작하는 자세

    assert start_of_closing(6).y_mm - start_of_closing(0).y_mm == pytest.approx(6, abs=2)


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


def test_width_table_follows_urdf_geometry(kin):
    _d, ctl, sk = rig(kin)
    ctl.close()
    assert sk.gripper_for_width(54) == pytest.approx(30.0, abs=1.5)  # 카드(54mm) 시험: 값 27.7~32.0에서 접촉
    assert sk.gripper_for_width(94) == pytest.approx(58.0, abs=1.5)
