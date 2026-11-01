import time
from dataclasses import astuple

import numpy as np
import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import DEADMAN_S, MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.teach import Teach, smooth
from so101_studio.arm.teleop import keyboard_loop, robot_to_table
from so101_studio.arm.trajectory import move_j
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
HOME = JointState(*CFG.poses["home"])
LIMITS = CFG.joint_limits_deg


@pytest.fixture(scope="module")
def kin():
    return Kinematics(CFG.urdf, WS.base)


def rig(kin=None, **safety_kw):
    d = SimDriver(CFG.poses["home"])
    return d, MotionController(d, kin, Safety(LIMITS, WS, kin, **safety_kw) if kin else Safety(LIMITS), 100)


# ---------------- T8: jog + 데드맨
def jog_for(ctl, mode, v, seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        ctl.jog(mode, v)
        time.sleep(0.05)


def test_joint_jog_moves_while_input_continues():
    d, ctl = rig()
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.5)
    ctl.close()
    assert d.read_joints().shoulder_pan - HOME.shoulder_pan == pytest.approx(10, abs=3)  # 20°/s x 0.5s


def test_deadman_stops_within_a_third_of_a_second_after_input_ends():
    d, ctl = rig()
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.3)
    time.sleep(DEADMAN_S + 0.1)
    p1 = d.read_joints().shoulder_pan
    time.sleep(0.3)
    p2 = d.read_joints().shoulder_pan
    state = ctl.state
    ctl.close()
    assert p1 == p2 and state == "idle"  # 입력이 끊기자 더 움직이지 않는다


def test_joint_jog_stops_at_the_limit():
    d, ctl = rig()
    jog_for(ctl, "joint", [1000, 0, 0, 0, 0, 0], 0.6)  # 한계 이상으로 밀어도
    ctl.close()
    assert d.read_joints().shoulder_pan <= LIMITS["shoulder_pan"][1] + 1e-6


def test_jog_is_ignored_while_a_trajectory_runs():
    d, ctl = rig()
    h = ctl.execute(move_j(HOME, JointState(20, -90, 93, 0, 0, 3), 20, 60))
    assert ctl.jog("joint", [0, 0, 0, 0, 30, 0]) is False
    h.wait(5)
    ctl.close()
    assert d.read_joints().wrist_roll == pytest.approx(0, abs=1e-6)


def test_cartesian_jog_moves_the_tip_forward(kin):
    d, ctl = rig(kin)
    q0 = d.read_joints()
    p0 = kin.fk(q0)
    f, left, up, yaw = robot_to_table((30.0, 0, 0, 0), WS.base.yaw_deg)  # 로봇 앞쪽 30mm/s
    jog_for(ctl, "cartesian", [f, left, up, yaw], 0.5)
    ctl.close()
    p1 = kin.fk(d.read_joints())
    assert np.hypot(p1.x_mm - p0.x_mm, p1.y_mm - p0.y_mm) > 5  # 움직였다
    assert (p1.x_mm - p0.x_mm, p1.y_mm - p0.y_mm) == pytest.approx((f * 0.5, left * 0.5), abs=8)


def test_keyboard_loop_maps_keys_and_exits():
    keys = iter(["q"] * 8 + ["x"])
    d, ctl = rig()
    keyboard_loop(ctl, "joint", WS.base.yaw_deg, 20.0, lambda: next(keys), out=lambda *_: None, tick_s=0.02)
    ctl.close()
    assert d.read_joints().shoulder_pan > HOME.shoulder_pan + 1  # q 키 = 1번 관절 +


# ---------------- T11: retarget
def test_retarget_replaces_the_move_smoothly_without_a_jump():
    d, ctl = rig()
    log = []
    orig = d.write_joints
    d.write_joints = lambda q: (log.append(astuple(q)), orig(q))[1]
    h1 = ctl.execute(move_j(HOME, JointState(40, -90, 93, 0, 0, 3), 40, 120, profile="ruckig", j_max_deg_s3=400))
    time.sleep(0.4)
    h2 = ctl.retarget(JointState(-30, -90, 93, 0, 0, 3), 40, 120, 400)  # 반대쪽으로 목표를 바꾼다
    assert h1.wait(2).status == "preempted"
    assert h2.wait(8).status == "ok"
    ctl.close()
    q = d.read_joints()
    assert q.shoulder_pan == pytest.approx(-30, abs=0.01)
    steps = np.abs(np.diff(np.array(log), axis=0)).max()
    assert steps < 5.0  # 한 주기 이동량 상한(안전)을 넘지 않는다 = 속도가 이어진다


def test_retarget_when_idle_just_starts_a_move():
    d, ctl = rig()
    r = ctl.retarget(JointState(10, -90, 93, 0, 0, 3), 40, 120, 400).wait(5)
    ctl.close()
    assert r.status == "ok" and d.read_joints().shoulder_pan == pytest.approx(10, abs=0.01)


def test_retarget_outside_limits_is_rejected_and_the_old_move_continues():
    _d, ctl = rig()
    h1 = ctl.execute(move_j(HOME, JointState(30, -90, 93, 0, 0, 3), 30, 100))
    time.sleep(0.2)
    h2 = ctl.retarget(JointState(500, -90, 93, 0, 0, 3), 40, 120, 400)
    assert h2.wait(1).status == "out_of_bounds"
    assert h1.wait(5).status == "ok"
    ctl.close()


# ---------------- T9: 녹화·재생
def test_smooth_keeps_endpoints():
    q = np.array([[0.0, 0], [10, 0], [0, 0], [10, 0], [0, 0], [5, 0]])
    s = smooth(q, 3)
    assert (s[0] == q[0]).all() and (s[-1] == q[-1]).all() and s[2, 0] < 8


def test_record_save_and_replay(tmp_path):
    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    teach.record_start()
    assert d.torque_enabled() is False  # 토크를 끄고 기록한다
    for pan in (0, 8, 16, 24, 30):
        d.write_joints(JointState(pan, -90, 93, 0, 0, 3))  # 사람이 손으로 움직이는 것을 흉내
        time.sleep(0.25)
    path = teach.record_stop("wave")
    assert path.exists() and d.torque_enabled() is True and teach.names() == ["wave"]
    d.write_joints(HOME)  # 다른 자리로 옮겨 놓고 재생
    ctl.sync()
    r = teach.replay("wave", speed=4.0)
    ctl.close()
    assert r.status == "ok" and d.read_joints().shoulder_pan == pytest.approx(30, abs=1.0)


def test_replay_rejects_a_sequence_outside_the_limits(tmp_path):
    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    (tmp_path / "bad.json").write_text('{"hz": 20, "joints": [[0,-90,95,0,0,3],[300,-90,95,0,0,3]]}', encoding="utf-8")
    r = teach.replay("bad")
    ctl.close()
    assert r.status == "out_of_bounds" and d.read_joints().shoulder_pan == pytest.approx(HOME.shoulder_pan, abs=0.01)


def test_cartesian_jog_works_with_the_wrist_rolled(kin):
    d = SimDriver(CFG.poses["home"])
    d.write_joints(JointState(0, -90, 93, 0, 17, 3))  # 손목이 돌아가 있는 자세
    ctl = MotionController(d, kin, Safety(LIMITS, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm), 100)
    y0 = kin.fk(d.read_joints()).y_mm
    f, left, up, yaw = robot_to_table((30.0, 0, 0, 0), WS.base.yaw_deg)
    jog_for(ctl, "cartesian", [f, left, up, yaw], 0.5)
    ctl.close()
    assert kin.fk(d.read_joints()).y_mm - y0 == pytest.approx(0.5 * 30, abs=6)


def test_jog_speed_ramps_up_and_down_without_a_velocity_step():
    d, ctl = rig()
    log = []
    orig = d.write_joints
    d.write_joints = lambda q: (log.append(astuple(q)), orig(q))[1]
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.4)
    ctl.jog_stop()
    time.sleep(0.3)
    ctl.close()
    pan = np.array(log)[:, 0]
    step = np.diff(pan)
    assert abs(step[0]) < 0.2 and np.abs(np.diff(step)).max() < 0.2  # 시작·정지에서 틱당 속도 변화가 가속도 한계(0.16°) 안
    assert step[-1] == 0 or abs(step[-1]) < 0.2
    assert ctl.state == "idle"
