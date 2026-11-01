import time
from dataclasses import replace

import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import HealthMonitor, Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import move_j
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
HOME = JointState(*CFG.poses["home"])


@pytest.fixture(scope="module")
def kin():
    return Kinematics(CFG.urdf, WS.base)


@pytest.fixture(scope="module")
def safety(kin):
    return Safety(CFG.joint_limits_deg, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm)


def test_home_is_allowed(safety):
    assert safety.check_goal(HOME) is None


@pytest.mark.parametrize("joint", list(CFG.joint_limits_deg))
@pytest.mark.parametrize("side", [0, 1])
def test_every_joint_limit_is_rejected_on_both_sides(safety, joint, side):
    limit = CFG.joint_limits_deg[joint][side]
    bad = replace(HOME, **{joint: limit + (-1 if side == 0 else 1)})
    assert safety.check_goal(bad) == f"joint_limit:{joint}"


def test_workspace_and_floor_are_rejected(kin):
    s = Safety({}, WS, kin)
    assert s.check_goal(HOME) is None
    assert s.check_goal(JointState(0, 0, 0, 0, 0, 0)) == "workspace"  # 팔을 곧게 뻗으면 작업대 앞쪽 경계(y=390)를 넘는다
    assert s.check_goal(JointState(0, 40, 40, 60, 0, 0)) == "z_floor"  # 작업대 아래로 내려간 자세


def test_floor_check_uses_tool_height(kin):
    ws_wide = replace(WS, table_mm={"x": 2000, "y": 2000}, robot_base_in_table={"x_mm": 1000, "y_mm": 1000, "yaw_deg": 0})
    s = Safety({}, ws_wide, Kinematics(CFG.urdf, ws_wide.base))
    assert s.check_goal(JointState(0, 40, 40, 60, 0, 0)) == "z_floor"
    assert s.check_goal(HOME) is None


def test_step_limit(safety):
    assert safety.check_step(HOME, replace(HOME, shoulder_pan=4.9)) is None
    assert safety.check_step(HOME, replace(HOME, shoulder_pan=5.1)) == "step"


def test_path_check_rejects_goal_outside_limits_and_allows_resting_start(safety):
    t = move_j(replace(HOME, elbow_flex=97.4), replace(HOME, shoulder_pan=10), 30, 100)  # elbow 시작값은 검사하지 않는다
    assert safety.check_path(t) is None
    assert safety.check_path(move_j(HOME, replace(HOME, shoulder_pan=150), 30, 100)) == "joint_limit:shoulder_pan"


def health(temp=35, load=10):
    return {"temperature_c": {"elbow_flex": temp}, "load_pct": {"elbow_flex": load}, "voltage_v": 12.0}


def test_overheat_needs_consecutive_readings():
    m = HealthMonitor()
    assert m.update_health(health(temp=65), 0) is None  # 튄 값 1~2회는 무시 (T2 관찰)
    assert m.update_health(health(temp=65), 1) is None
    assert m.update_health(health(temp=35), 2) is None  # 중간에 정상값이 끼면 세기를 처음부터
    assert [m.update_health(health(temp=65), t) for t in (3, 4, 5)] == [None, None, "overheat:elbow_flex"]


def test_overload_only_after_3_seconds():
    m = HealthMonitor()
    assert m.update_health(health(load=95), 0) is None
    assert m.update_health(health(load=95), 2.9) is None
    assert m.update_health(health(load=95), 3.0) == "overload:elbow_flex"
    m = HealthMonitor()
    m.update_health(health(load=95), 0)
    m.update_health(health(load=10), 1)  # 부하가 내려가면 시간이 초기화
    assert m.update_health(health(load=95), 3.5) is None


def test_comm_loss_after_half_second():
    m = HealthMonitor()
    assert m.comm_failed(10.0) is None
    assert m.comm_failed(10.4) is None
    assert m.comm_failed(10.5) == "comm_lost"
    m.comm_ok()
    assert m.comm_failed(11.0) is None


def test_controller_rejects_unsafe_goal_without_moving(safety):
    d = SimDriver(CFG.poses["home"])
    c = MotionController(d, safety=safety)
    r = c.execute(move_j(HOME, replace(HOME, shoulder_pan=150), 30, 100)).wait(1)
    c.close()
    assert r.status == "out_of_bounds" and d.read_joints() == HOME


class FailingDriver(SimDriver):
    def write_joints(self, q):
        raise ConnectionError("no status packet")


def test_controller_faults_on_comm_loss():
    d = FailingDriver(CFG.poses["home"])
    c = MotionController(d)
    r = c.execute(move_j(HOME, replace(HOME, shoulder_pan=20), 30, 100)).wait(3)
    c.close()
    assert r.status == "fault" and c.fault == "comm_lost"
    assert c.execute(move_j(HOME, HOME, 30, 100)).wait(1).status == "fault"  # resume 전에는 거부


class OverheatDriver(SimDriver):
    def read_health(self):
        h = super().read_health()
        h["temperature_c"]["elbow_flex"] = 70
        return h


def test_controller_faults_on_overheat_and_holds_position():
    d = OverheatDriver(CFG.poses["home"])
    c = MotionController(d)
    h = c.execute(move_j(HOME, replace(HOME, shoulder_pan=40), 10, 100))
    r = h.wait(5)
    time.sleep(0.1)
    c.close()
    assert r.status == "fault" and c.fault == "overheat:elbow_flex"
    assert d.read_joints().shoulder_pan < 40  # 도착하지 못하고 그 자리에서 멈춘다


def test_floor_is_measured_at_the_lowest_jaw_tip(kin):
    from so101_studio.core.types import JointState

    s = Safety({}, WS, kin, z_floor_mm=4.0)
    q = JointState(0, -5, 38, 76, 0, 0)  # 수직 하강, 턱을 닫음
    p = kin.fk(q)
    f, m = kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, q.gripper)
    tip = p.z_mm + min(f[2], m[2])
    assert tip < p.z_mm - 5  # 턱 끝은 기준점보다 낮다
    assert (s.check_goal(q) == "z_floor") == (tip < 4.0)
