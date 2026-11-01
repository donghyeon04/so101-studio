import time
from dataclasses import replace

import pytest

from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.teach import Teach
from so101_studio.core.config import load_robot

CFG = load_robot("config/robot.yaml")
LIMITS = CFG.joint_limits_deg
HOME = CFG.poses["home"]


def rig():
    d = SimDriver(HOME)
    return d, MotionController(d, None, Safety(LIMITS), CFG.control_hz)


def settle(seconds=0.2):
    time.sleep(seconds)


def push(d, shoulder_pan=0.0):
    """손으로 민 것을 흉내: 한 주기에 3°씩(한 주기 이동량 한계 5° 안) 실제 위치를 목표와 다르게 만든다."""
    n = max(1, int(abs(shoulder_pan) / 3.0 + 0.999))
    for _ in range(n):
        d._q = replace(d._q, shoulder_pan=d._q.shoulder_pan + shoulder_pan / n)
        time.sleep(0.05)


def test_follow_moves_the_goal_when_pushed_past_the_deadband_but_not_for_small_noise():
    d, ctl = rig()
    assert ctl.follow(True) and ctl.state == "follow"
    start = ctl._last.shoulder_pan
    push(d, shoulder_pan=0.4)  # 데드밴드(1°) 안: 무시
    settle()
    assert ctl._last.shoulder_pan == pytest.approx(start)
    push(d, shoulder_pan=6.0)  # 손으로 밂
    settle()
    ctl.follow(False)
    ctl.close()
    assert ctl._last.shoulder_pan == pytest.approx(d._q.shoulder_pan, abs=0.5) and ctl._last.shoulder_pan > start + 3


def test_follow_does_not_follow_past_a_joint_limit():
    hi = LIMITS["shoulder_pan"][1]
    d = SimDriver([hi - 1.5, *HOME[1:]])  # 한계 바로 앞에서 시작
    ctl = MotionController(d, None, Safety(LIMITS), CFG.control_hz)
    ctl.follow(True)
    push(d, shoulder_pan=6.0)  # 한계 밖으로 밂
    settle()
    block, last = ctl.follow_block, ctl._last.shoulder_pan
    ctl.follow(False)
    ctl.close()
    assert last <= hi and block is not None and "shoulder_pan" in block


def test_follow_cannot_start_while_a_motion_runs_or_after_estop():
    _d, ctl = rig()
    ctl.estop()
    assert ctl.follow(True) is False
    ctl.resume()
    assert ctl.follow(True) is True and ctl.jog("joint", [10, 0, 0, 0, 0, 0]) is False  # 추종 중에는 조그를 받지 않는다
    ctl.stop()
    assert ctl.state == "idle"
    ctl.close()


def test_follow_recording_keeps_torque_lowers_limits_and_restores_them(tmp_path):
    d, ctl = rig()
    d.set_torque(True)
    teach = Teach(ctl, CFG, tmp_path)
    before, p_before, d_before = d.get_torque_limits(), d.get_p(), d.get_d()
    teach.record_start(follow=True, stiffness=0.5)
    during = d.get_torque_limits()
    assert d.get_p()["shoulder_pan"] == max(1, round(CFG.teach_p["shoulder_pan"] * 0.5)) < p_before["shoulder_pan"]  # P도 낮춘다
    assert ctl.follow_dead_deg == CFG.teach_dead_deg
    assert d.get_d()["shoulder_pan"] == max(0, round(CFG.teach_d["shoulder_pan"] * 0.5)) < d_before["shoulder_pan"]  # D도 낮춘다
    assert d.torque_enabled() is True  # 토크는 켜져 있다
    assert during["shoulder_lift"] == int(CFG.teach_torque["shoulder_lift"] * 0.5) and during["shoulder_lift"] < before["shoulder_lift"]
    for step in (0, 8, 16):
        push(d, shoulder_pan=8.0 if step else 0.0)
        time.sleep(0.25)
    path = teach.record_stop("hand")
    ctl.close()
    assert path.exists() and d.get_torque_limits() == before and d.get_p() == p_before and d.get_d() == d_before and ctl.state == "idle"  # 토크 제한과 P를 되돌렸다
    q, _ = teach.load("hand")
    assert q[-1][0] > q[0][0] + 10  # 밀린 만큼 기록됨


def test_arm_driver_torque_limit_helpers_write_to_the_bus():
    from types import SimpleNamespace

    from so101_studio.arm.driver import ArmDriver

    names = ["shoulder_pan", "gripper"]

    class Bus:
        motors = names

        def __init__(self):
            self.writes = []

        def read(self, name, motor, normalize=True):
            return {"shoulder_pan": 1000, "gripper": 500}[motor]

        def write(self, name, motor, value):
            self.writes.append((name, motor, value))

    drv = ArmDriver()
    drv._robot = SimpleNamespace(bus=Bus())
    assert drv.get_torque_limits() == {"shoulder_pan": 1000, "gripper": 500}
    drv.set_torque_limits({"shoulder_pan": 250.0})
    assert drv._robot.bus.writes == [("Torque_Limit", "shoulder_pan", 250)]
