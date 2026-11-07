import math
import time
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
    return Kinematics(CFG.urdf, WS.base, CFG.joint_limits_deg)  # 실제 ArmService처럼 풀이기가 관절 한계 안에서 푼다


def rig(kin, block=None, **cfg_kw):
    d = Recorder(CFG.poses["home"], grip_block=block)
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm), CFG.control_hz)
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


@pytest.mark.slow
def test_pick_without_an_object_reports_no_object(kin):
    _d, r = run_pick(kin, block=None)
    assert r.status == "no_object"


def tips_mid_and_low(kin, q):
    """FK 자세와 그리퍼 값으로 계산한 두 턱 끝의 중간점(작업대 x, y)과 낮은 턱 끝의 높이."""
    p = kin.fk(q)
    f, m = kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, q.gripper)
    return (p.x_mm + (f[0] + m[0]) / 2, p.y_mm + (f[1] + m[1]) / 2), p.z_mm + min(f[2], m[2])


def test_jaw_tips_stay_centered_on_the_object_while_closing(kin):
    # 닫는 동안 내내 두 턱 끝의 중간점이 물체 중심(300, 210)에 있어야 한다 = 가운데 있는 물체를 양쪽에서 잡는 모양
    d, r = run_pick(kin)
    assert r.status == "ok"
    top = max(q.gripper for q in d.log)
    first = next(i for i, q in enumerate(d.log) if q.gripper == top)
    closing = [q for q in d.log[first:] if BLOCK + 0.5 < q.gripper < top - 0.5]
    assert len(closing) > 20
    for q in closing[:: max(1, len(closing) // 30)]:
        (cx, cy), low = tips_mid_and_low(kin, q)
        assert (cx, cy) == pytest.approx((300, 210), abs=5)
        assert low == pytest.approx(CFG.table_z_mm + CFG.grasp_height_mm, abs=6)


def test_hand_tilts_when_open_and_straightens_while_closing(kin):
    d, r = run_pick(kin)
    assert r.status == "ok"
    top = max(q.gripper for q in d.log)
    i_open = max(i for i, q in enumerate(d.log) if q.gripper == top)
    opened = d.log[i_open]
    expected = min(CFG.grasp_tilt_deg, 0.607 * top)  # θ/2, 손목 한계에서 자른다
    assert kin.fk(opened).pitch_deg == pytest.approx(-90 - expected, abs=4)  # 벌린 상태: 가동 턱 반대쪽으로 기울임
    at_contact = next(q for q in d.log[i_open:] if q.gripper <= BLOCK + 1.0)
    assert abs(kin.fk(at_contact).pitch_deg + 90) < abs(kin.fk(opened).pitch_deg + 90) - 5  # 닫을수록 기울기가 줄어든다


@pytest.mark.slow
def test_no_tilt_when_the_jaw_axis_is_sideways(kin):
    d, _r = run_pick(kin, angle=90)  # 턱이 좌우로 벌어짐: 5축 팔은 옆으로 기울일 수 없다
    top = max(q.gripper for q in d.log)
    opened = d.log[max(i for i, q in enumerate(d.log) if q.gripper == top)]
    assert kin.fk(opened).pitch_deg == pytest.approx(-90, abs=2)


def test_open_gap_is_the_configured_maximum(kin):
    d, _r = run_pick(kin)
    assert gap_of(max(q.gripper for q in d.log)) == pytest.approx(CFG.grasp_max_open_mm, abs=3)


def test_jaws_open_perpendicular_to_the_long_axis(kin):
    d, _ = run_pick(kin)
    low = min(d.log, key=lambda q: kin.fk(q).z_mm)
    assert kin.moving_jaw_dir(low) == pytest.approx((0, 1), abs=0.1)  # 각도 0 -> 턱은 작업대 y축을 따라


@pytest.mark.slow
def test_jaw_axis_follows_the_object_angle(kin):
    d, _ = run_pick(kin, angle=45)
    low = min(d.log, key=lambda q: kin.fk(q).z_mm)
    assert kin.moving_jaw_dir(low) == pytest.approx((math.cos(math.radians(135)), math.sin(math.radians(135))), abs=0.1)


@pytest.mark.parametrize("angle", [0, 45, 90, 135])
@pytest.mark.slow
def test_pick_returns_to_hover_for_every_angle(kin, angle):
    d, r = run_pick(kin, x=300, y=200, angle=angle)
    assert r.status == "ok"
    low = min(kin.fk(q).z_mm for q in d.log)
    assert kin.fk(d.log[-1]).z_mm >= low + CFG.approach_height_mm - 6  # 접촉한 자리에서 접근 높이만큼 올라온다


def test_known_width_with_fit_policy_opens_only_as_much_as_needed(kin):
    d, ctl, sk = rig(kin, BLOCK, grasp_open_policy="fit")
    r = sk.pick(obj(width=WIDTH))
    ctl.close()
    assert r.status == "ok"
    assert gap_of(max(q.gripper for q in d.log)) == pytest.approx(WIDTH + CFG.gripper_margin_mm, abs=3)


@pytest.mark.slow
def test_grasp_bias_shifts_toward_the_moving_jaw(kin):
    def start_of_closing(bias):
        d, ctl, sk = rig(kin, BLOCK, grasp_bias_mm=bias)
        sk.pick(obj())
        ctl.close()
        top = max(q.gripper for q in d.log)
        return kin.fk(d.log[max(i for i, q in enumerate(d.log) if q.gripper == top)])  # 하강이 끝나고 닫기 시작하는 자세

    assert start_of_closing(6).y_mm - start_of_closing(0).y_mm == pytest.approx(6, abs=2)


@pytest.mark.slow
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


def test_symmetric_tilt_is_half_the_jaw_opening_angle(kin):
    _d, ctl, sk = rig(kin)
    ctl.close()
    for v in (10.0, 20.0, 40.0):
        assert sk._tilt(v) == pytest.approx(min(CFG.grasp_tilt_deg, 0.607 * v), abs=0.2)  # θ = 1.214 v, 한쪽 턱이 ±θ/2
    assert sk._tilt(0.0) == 0.0 and sk._tilt(100.0) == CFG.grasp_tilt_deg  # 다 닫으면 서고, 많이 벌리면 손목 한계에서 멈춘다


# ---- REQ-020 (결정 0033): 예압, RAM 조임 한계, no_object 때 열기, 그리퍼 부하 두 단계, 놓기 보정

@pytest.mark.slow
def test_contact_sets_goal_to_position_plus_preload_and_torque_limit_goes_300_200_then_original(kin):
    d, ctl, sk = rig(kin, BLOCK)
    d.torque_limit = 777  # 원래 값
    r = sk.pick(obj())
    assert r.status == "ok"
    assert ctl.grip_hold == pytest.approx(d.read_joints().gripper - CFG.grip_preload_units, abs=0.3)  # 실제보다 예압만큼만 닫힌 쪽
    assert ctl._last.gripper == pytest.approx(ctl.grip_hold, abs=0.3)  # 위로 올라간 뒤에도 목표가 예압 값
    assert d.torque_limit_writes == [CFG.gripper_squeeze_torque, CFG.gripper_hold_torque]  # 들고 있는 동안은 200
    assert sk.place("right").status == "ok"
    ctl.close()
    assert d.torque_limit_writes == [300, 200, 777] and d.torque_limit == 777  # place가 끝난 뒤 원래 값
    assert ctl.grip_hold is None


def test_torque_limit_is_ram_not_eeprom():
    import types

    from so101_studio.arm.driver import ArmDriver

    class FakeBus:
        def __init__(self):
            self.motors = {"gripper": None}
            self.writes = []

        def write(self, register, motor, value, **kw):
            self.writes.append((register, motor, value))

    bus, d = FakeBus(), ArmDriver()
    d._robot = types.SimpleNamespace(bus=bus)
    d.set_gripper_torque_limit(300)
    assert bus.writes == [("Torque_Limit", "gripper", 300)]  # RAM(Torque_Limit)만. Max_Torque_Limit(EEPROM)은 쓰지 않는다


@pytest.mark.slow
def test_torque_limit_is_restored_when_place_raises(kin):
    d, ctl, sk = rig(kin, BLOCK)
    d.torque_limit = 900
    assert sk.pick(obj()).status == "ok"
    sk._opening_path = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
    with pytest.raises(RuntimeError):
        sk.place("right")
    ctl.close()
    assert d.torque_limit == 900 and ctl.grip_hold is None


@pytest.mark.slow
def test_no_object_opens_only_after_rising_to_the_approach_height(kin):
    d, ctl, sk = rig(kin, None)
    d.torque_limit = 900
    r = sk.pick(obj())
    ctl.close()
    assert r.status == "no_object" and d.torque_limit == 900  # 접촉이 없으면 한계도 바로 되돌린다
    tail = d.log[-1]
    assert tail.gripper == pytest.approx(CFG.gripper_open, abs=1)  # 마지막에 열려 있다
    gmin = min(q.gripper for q in d.log)
    i_open = max(i for i, q in enumerate(d.log) if q.gripper <= gmin + 0.3) + 1  # 닫힌 채 마지막 틱 다음부터 열린다
    z_at_open_start = kin.fk(d.log[i_open - 1]).z_mm
    z_min = min(kin.fk(q).z_mm for q in d.log)
    assert z_at_open_start > z_min + CFG.approach_height_mm * 0.8  # 바닥(가장 낮은 곳)이 아니라 올라온 높이에서 열기 시작


@pytest.mark.slow
def test_grip_load_two_stages_hold_then_open_and_fault(kin):
    d, ctl, sk = rig(kin, BLOCK)
    assert sk.pick(obj()).status == "ok"
    events = []
    ctl.on_event = lambda *a: events.append(a)
    d.grip_load = 80.0  # 기준(60%) 위
    t0 = time.perf_counter()
    while ctl.state != "fault" and time.perf_counter() - t0 < 6:
        time.sleep(0.05)
    ctl.close()
    codes = [e[2] for e in events]
    assert "ARM-GRIP-OVERLOAD-WARN" in codes and "ARM-GRIP-OVERLOAD" in codes and codes.index("ARM-GRIP-OVERLOAD-WARN") < codes.index("ARM-GRIP-OVERLOAD")
    assert ctl.fault == "grip_overload" and d.read_joints().gripper == pytest.approx(CFG.gripper_open, abs=1)  # 그리퍼를 열었다


@pytest.mark.slow
def test_grip_load_below_limit_does_nothing(kin):
    d, ctl, sk = rig(kin, BLOCK)
    assert sk.pick(obj()).status == "ok"
    d.grip_load = 40.0
    time.sleep(2.5)
    ctl.close()
    assert ctl.state != "fault"


@pytest.mark.slow
def test_place_moves_the_hand_so_the_jaw_midpoint_stays_while_opening(kin):
    d, ctl, sk = rig(kin, BLOCK)
    assert sk.pick(obj()).status == "ok"
    d.log.clear()
    assert sk.place("right").status == "ok"
    ctl.close()
    top = max(q.gripper for q in d.log)
    start = next(i for i, q in enumerate(d.log) if q.gripper > min(x.gripper for x in d.log) + 3)

    def mid(q):
        p = kin.fk(q)
        f, m = kin.jaw_tip_offsets(-90.0, p.yaw_deg, q.gripper)
        c = (f[:2] + m[:2]) / 2
        return np.array([p.x_mm + c[0], p.y_mm + c[1]])

    opened = [q for q in d.log[start:] if q.gripper >= top - 0.5]
    assert opened, "열린 구간이 있어야 한다"
    m0 = mid(d.log[start - 1])
    assert max(np.linalg.norm(mid(q) - m0) for q in d.log[start:start + len(opened)]) < 4.0  # 열리는 동안 중간점이 거의 제자리
    a0, a1 = kin.fk(d.log[start - 1]), kin.fk(opened[-1])
    assert np.hypot(a1.x_mm - a0.x_mm, a1.y_mm - a0.y_mm) > 1.0  # 손은 실제로 움직였다(보정이 있었다)


@pytest.mark.slow
def test_opening_command_after_pick_releases_hold_and_restores_limit_but_estop_keeps_them(kin):
    from so101_studio.service.arm_service import ArmService

    d, ctl, sk = rig(kin, BLOCK)
    d.torque_limit = 800
    assert sk.pick(obj()).status == "ok"
    hold = ctl.grip_hold
    ctl.estop()  # 정지: 고정과 200을 유지한다
    assert ctl.grip_hold == hold and d.torque_limit == CFG.gripper_hold_torque and ctl._last.gripper == pytest.approx(hold, abs=0.01)  # E-STOP이 쓴 목표도 예압 값
    ctl.resume()
    svc = ArmService(d, CFG, WS, speed_pct=100, log_root=None)
    svc.ctl, svc.skills, svc.kin, svc.safety = ctl, sk, kin, ctl.safety
    svc._connected = True
    r = svc.set_gripper(60)  # 지금보다 여는 값
    ctl.close()
    assert r.status == "ok" and d.read_joints().gripper == pytest.approx(60, abs=0.6)  # 실제로 열렸다
    assert d.torque_limit == 800 and ctl.grip_hold is None  # 원래 값으로


@pytest.mark.slow
def test_place_restores_limit_before_opening(kin):
    d, ctl, sk = rig(kin, BLOCK)
    d.torque_limit = 800
    assert sk.pick(obj()).status == "ok"
    assert sk.place("right").status == "ok"
    ctl.close()
    assert d.torque_limit_writes[-1] == 800 and d.torque_limit_writes.count(800) == 1
