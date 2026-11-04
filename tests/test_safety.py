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


def test_relaxed_restores_the_checks_even_when_the_block_raises():
    from so101_studio.arm.safety import Safety

    s = Safety({})
    try:
        with s.relaxed(check_xy=False, max_step_deg=9.0):
            assert s.check_xy is False and s.max_step_deg == 9.0
            raise RuntimeError("시험 중 오류")
    except RuntimeError:
        pass
    assert s.check_xy is True and s.max_step_deg == 5.0


def test_path_to_a_goal_no_further_out_than_a_start_beyond_the_limit_is_allowed():
    from so101_studio.arm.safety import Safety
    from so101_studio.arm.trajectory import move_j
    from so101_studio.core.types import JointState

    s = Safety({"elbow_flex": [-96, 95]})
    rest, inside = JointState(0, -90, 97.2, 0, 0, 3), JointState(0, -90, 93, 0, 0, 3)
    assert s.check_path(move_j(rest, rest, 20, 100)) is None  # 받침 자세에 머무는 동작
    assert s.check_path(move_j(rest, inside, 20, 100)) is None
    assert s.check_path(move_j(inside, rest, 20, 100)) is not None  # 안에서 밖으로 새로 나가는 것은 막는다
    assert s.check_path(move_j(rest, JointState(0, -90, 99, 0, 0, 3), 20, 100)) is not None  # 시작보다 더 바깥도 막는다


def test_floor_check_lets_the_tip_leave_a_pose_already_below_the_floor_but_not_go_lower():
    from types import SimpleNamespace

    from so101_studio.arm.safety import Safety
    from so101_studio.core.types import JointState, ToolPose

    kin = SimpleNamespace(fk=lambda q: ToolPose(100, 100, q.shoulder_lift, 90, -90), jaw_tip_offsets=lambda *a: ((0, 0, 0), (0, 0, 0)))
    ws = SimpleNamespace(table_mm={"x": 600, "y": 400}, safety_margin_mm=10)
    s = Safety({}, ws, kin, z_floor_mm=5, table_z_mm=0)
    at = lambda h: JointState(0, h, 0, 0, 0, 0)  # 높이 = shoulder_lift 값(mm)
    assert s.check_goal(at(3)) == "z_floor"  # 처음부터 낮은 목표는 거부
    assert s.check_goal(at(3), ref=at(2)) is None  # 2mm에서 출발해 3mm로 올라가는 것은 허용
    assert s.check_goal(at(1.5), ref=at(2)) == "z_floor"  # 출발보다 더 낮아지는 것은 거부


def test_safety_settings_margin_box_and_off_mode():
    from types import SimpleNamespace

    from so101_studio.arm.safety import Safety
    from so101_studio.core.types import JointState, ToolPose

    base = {"shoulder_pan": [-118, 118], "gripper": [1, 99]}
    phys = {"shoulder_pan": [-119.5, 119.5]}
    kin = SimpleNamespace(fk=lambda q: ToolPose(300, 200, q.shoulder_lift, 90, -90), jaw_tip_offsets=lambda *a: ((0, 0, 0), (0, 0, 0)))
    ws = SimpleNamespace(table_mm={"x": 600, "y": 400}, safety_margin_mm=10)
    s = Safety(base, ws, kin, z_floor_mm=5, table_z_mm=0, physical_limits=phys)
    q = lambda pan=0, h=50: JointState(pan, h, 0, 0, 0, 3)  # 높이 = shoulder_lift 값(mm)
    s.settings.joint_margin_deg = 10  # 안쪽으로 줄이면
    assert s.limits["shoulder_pan"] == [-108, 108] and s.check_goal(q(110)) == "joint_limit:shoulder_pan"
    s.settings.joint_margin_deg = -5  # 넓히면 실제 가동 범위(119.5)까지만
    assert s.limits["shoulder_pan"] == [-119.5, 119.5] and s.limits["gripper"] == [1, 99]
    assert s.check_goal(q(119)) is None and s.check_goal(q(121)) == "joint_limit:shoulder_pan"
    s.settings.joint_margin_deg = 0
    s.settings.box_mm = {"x": [250, 350], "z": [5, 100]}  # x 범위를 좁히고 z 상한을 둠
    assert s.check_goal(q()) is None and s.check_goal(q(h=150)) == "z_ceiling"
    s.settings.box_mm = {"x": [400, 500]}
    assert s.check_goal(q()) == "workspace"
    s.settings.mode = "free"  # 가장자리(x, y)만 끈다
    assert s.check_goal(q()) is None and s.check_goal(q(h=1)) == "z_floor"
    s.settings.mode = "off"  # 전부 끄지만 한 주기 이동량 한계는 남는다
    assert s.limits == {} and s.check_goal(q(500, 1)) is None
    assert s.check_step(q(0), q(30)) == "step"


def test_health_monitor_trips_on_sustained_overcurrent_but_not_a_single_spike():
    from so101_studio.arm.safety import HealthMonitor

    base = {"temperature_c": {"a": 30}, "load_pct": {"a": 10}}
    h = HealthMonitor(current_limit_ma=4000, current_count=2)
    assert h.update_health({**base, "current_ma": 4500}, 0.0) is None  # 한 번 튄 값은 무시
    assert h.update_health({**base, "current_ma": 1000}, 0.5) is None
    assert h.update_health({**base, "current_ma": 4500}, 1.0) is None
    assert h.update_health({**base, "current_ma": 4600}, 1.5) == "overcurrent"  # 연속이면 fault


def test_xy_check_lets_the_tip_return_from_outside_the_table_but_not_go_further_out(kin):
    s = Safety({}, WS, kin)
    out = JointState(0, 0, 0, 0, 0, 0)  # 0° 자세: 손끝이 작업대 앞쪽 경계(y=390) 밖
    assert s.check_goal(out) == "workspace"
    assert s.check_goal(HOME, out) is None  # 밖에서 안쪽(home)으로는 허용
    assert s.check_goal(out, out) is None  # 같은 자리에 머무는 것은 허용
    mid = next(JointState(0, -d, -d, 0, 0, 0) for d in range(1, 30) if 395 < kin.fk(JointState(0, -d, -d, 0, 0, 0)).y_mm < 402)  # 밖이지만 0°보다 덜 바깥
    assert s.check_goal(mid, out) is None  # 밖에서 덜 바깥으로는 허용
    assert s.check_goal(out, mid) == "workspace"  # 밖에서 더 바깥은 거부
    assert s.check_goal(out, HOME) == "workspace"  # 안에서 밖은 거부(ref가 안이면 지금과 같다)
