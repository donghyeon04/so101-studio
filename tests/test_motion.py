import time

import numpy as np

from so101_studio.arm.motion import MotionController
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import move_j
from so101_studio.core.types import JointState

A = JointState(0, 0, 0, 0, 0, 0)
B = JointState(30, -20, 10, 40, 0, 50)
LIMITS = {"shoulder_pan": [-100, 100], "gripper": [0, 100]}


def test_move_j_endpoints_and_zero_end_velocity():
    t = move_j(A, B, v_max_deg_s=30, a_max_deg_s2=100)
    assert t.sample(0) == A and t.sample(t.duration_s) == B
    assert np.allclose(t.velocity(0), 0) and np.allclose(t.velocity(t.duration_s), 0)


def test_move_j_respects_speed_limit():
    t = move_j(A, B, v_max_deg_s=30, a_max_deg_s2=1000)
    peak = max(np.abs(t.velocity(x * t.duration_s / 200)).max() for x in range(201))
    assert peak <= 30 * 1.01  # 가장 많이 움직이는 관절(gripper 50)이 한계 이내


def test_move_j_is_monotonic():
    t = move_j(A, B, 30, 100)
    xs = [t.sample(i * t.duration_s / 50).shoulder_pan for i in range(51)]
    assert xs == sorted(xs)


def test_controller_reaches_goal():
    d = SimDriver()
    c = MotionController(d, limits=LIMITS, hz=200)
    r = c.execute(move_j(A, JointState(10, 0, 0, 0, 0, 20), 200, 1000)).wait(5)
    c.close()
    assert r.status == "ok" and d.read_joints() == JointState(10, 0, 0, 0, 0, 20)


def test_out_of_bounds_is_rejected_without_moving():
    d = SimDriver()
    c = MotionController(d, limits=LIMITS)
    r = c.execute(move_j(A, JointState(150, 0, 0, 0, 0, 0), 30, 100)).wait(1)
    c.close()
    assert r.status == "out_of_bounds" and d.read_joints() == A


def test_stop_halts_within_300ms():
    d = SimDriver()
    c = MotionController(d, limits=LIMITS)
    h = c.execute(move_j(A, JointState(60, 0, 0, 0, 0, 0), 30, 100))
    time.sleep(0.5)
    t0 = time.perf_counter()
    c.stop()
    assert h.wait(1).status == "stopped"
    for _ in range(40):  # 감속이 끝날 때까지 기다린다
        if c._job is None:
            break
        time.sleep(0.01)
    assert time.perf_counter() - t0 < 0.4
    pos = d.read_joints().shoulder_pan
    time.sleep(0.2)
    c.close()
    assert d.read_joints().shoulder_pan == pos and 0 < pos < 60


def test_estop_blocks_until_resume():
    d = SimDriver()
    c = MotionController(d, limits=LIMITS)
    c.execute(move_j(A, JointState(60, 0, 0, 0, 0, 0), 30, 100))
    time.sleep(0.2)
    c.estop()
    assert c.execute(move_j(A, B, 30, 100)).wait(1).status == "stopped"
    c.resume()
    assert c.execute(move_j(d.read_joints(), A, 100, 1000)).wait(5).status == "ok"
    c.close()


def test_big_jump_between_ticks_is_a_fault():
    d = SimDriver()
    c = MotionController(d, hz=50)
    r = c.execute(move_j(A, JointState(60, 0, 0, 0, 0, 0), 5000, 1e6)).wait(2)  # 0.2s에 60° → 주기당 5° 초과
    c.close()
    assert r.status == "fault"


def test_goal_at_resting_pose_beyond_old_limit_is_allowed():
    from so101_studio.core.config import load_robot

    cfg = load_robot("config/robot.yaml")
    d = SimDriver()
    rest = JointState(0, -89, 97.4, 0, 0, 3)
    d.write_joints(rest)
    c = MotionController(d, limits=cfg.joint_limits_deg)
    r = c.execute(move_j(rest, JointState(10, -89, 97.4, 0, 0, 3), 200, 1000)).wait(5)
    c.close()
    assert r.status == "ok"
