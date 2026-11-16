"""servo_joints: 정책이 흘려 넣는 관절 목표 (arm-software-spec 6.10)."""
import math
import time
from dataclasses import replace

import pytest

from so101_studio.arm.motion import SERVO_TIMEOUT_S
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import move_j
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.logread import read_events
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
HOME = JointState(*CFG.poses["home"])


def make(tmp_path, speed=100, **sim_kw):
    s = ArmService(SimDriver(sim_kw.pop("home", CFG.poses["home"]), **sim_kw), CFG, WS, speed_pct=speed, log_root=str(tmp_path))
    ok, msg = s.connect()
    assert ok, msg
    return s


@pytest.fixture
def svc(tmp_path):
    s = make(tmp_path)
    yield s
    s.disconnect()


def stream(svc, goal_at, seconds, hz=25):
    """goal_at(t)를 hz로 servo_joints에 넣는다. 모두 받았는지 돌려준다."""
    t0, ok = time.perf_counter(), True
    while (t := time.perf_counter() - t0) < seconds:
        ok = svc.servo_joints(goal_at(t)) and ok
        time.sleep(1 / hz)
    return ok


def test_out_of_range_goal_is_rejected_and_the_arm_stays(svc):
    bad = replace(HOME, shoulder_pan=CFG.joint_limits_deg["shoulder_pan"][1] + 20)
    assert not svc.servo_joints(bad) and not svc.servo_joints(bad)
    time.sleep(0.2)
    assert svc.ctl.servo_reason == "joint_limit:shoulder_pan"
    assert svc.driver.read_joints() == HOME and svc.ctl.state == "idle"
    rej = [e for e in read_events(svc.logger.dir) if e["code"] == "ARM-SERVO-REJECT"]
    assert len(rej) == 1 and rej[0]["data"] == {"reason": "joint_limit:shoulder_pan", "count": 1}  # 같은 이유는 1초에 한 번


def test_stops_where_it_is_when_goals_stop_coming(svc):
    far = replace(HOME, shoulder_pan=HOME.shoulder_pan + 80)  # 90°/s로 0.5초에 닿지 못하는 거리
    assert svc.servo_joints(far)
    time.sleep(SERVO_TIMEOUT_S + 0.2)
    p1 = svc.driver.read_joints().shoulder_pan
    time.sleep(0.3)
    assert svc.driver.read_joints().shoulder_pan == p1 and svc.ctl.state == "idle"
    assert 20 < p1 - HOME.shoulder_pan < 70


def test_busy_while_a_trajectory_runs(svc):
    goal = replace(HOME, shoulder_pan=HOME.shoulder_pan + 30)
    h = svc.ctl.execute(move_j(HOME, goal, 30, 100))
    assert not svc.servo_joints(HOME) and svc.ctl.servo_reason == "busy"
    assert h.wait(5).status == "ok"


def test_real_arm_driver_is_refused(svc):
    svc.driver.simulated = False  # 실제 팔 드라이버인 척
    assert not svc.servo_joints(HOME) and svc.ctl.servo_reason == "real_arm"


def test_25hz_goal_stream_ends_within_1_deg_of_the_last_goal(svc):
    def goal_at(t):
        k = math.sin(t / 2.0 * math.pi / 2)  # 2초 동안 0 -> 1로 부드럽게
        return replace(HOME, shoulder_pan=HOME.shoulder_pan + 40 * k, elbow_flex=HOME.elbow_flex - 30 * k, wrist_roll=HOME.wrist_roll + 20 * k)

    assert stream(svc, goal_at, 2.0)
    last = goal_at(2.0)
    time.sleep(0.2)
    q = svc.driver.read_joints()
    assert max(abs(getattr(q, j) - getattr(last, j)) for j in vars(q)) < 1.0
    assert svc.status().cmd == svc.ctl._last


def test_closing_gripper_goal_turns_on_contact_and_holds_at_contact_plus_preload(tmp_path):
    s = make(tmp_path, speed=30, home=[*CFG.poses["home"][:5], 60.0], grip_block=30.0)  # 30 아래로는 닫히지 않는다(물체)
    try:
        lo = CFG.joint_limits_deg["gripper"][0]
        start = s.driver.read_joints()
        stream(s, lambda t: replace(start, gripper=lo), 3.0)
        assert s.ctl.grip_hold == pytest.approx(30.0 - CFG.grip_preload_units)
        assert any(e["code"] == "ARM-BLOCKED" and e["data"]["joint"] == "gripper" for e in read_events(s.logger.dir))
        assert s.servo_joints(replace(start, gripper=60.0)) and s.ctl.grip_hold is None  # 여는 목표는 고정을 푼다
    finally:
        s.disconnect()
