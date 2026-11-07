import time
from dataclasses import astuple

import numpy as np
import pytest

from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
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
    c = MotionController(d, safety=Safety(LIMITS), hz=200)
    r = c.execute(move_j(A, JointState(10, 0, 0, 0, 0, 20), 200, 1000)).wait(5)
    c.close()
    assert r.status == "ok" and d.read_joints() == JointState(10, 0, 0, 0, 0, 20)


def test_out_of_bounds_is_rejected_without_moving():
    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS))
    r = c.execute(move_j(A, JointState(150, 0, 0, 0, 0, 0), 30, 100)).wait(1)
    c.close()
    assert r.status == "out_of_bounds" and d.read_joints() == A


def test_stop_halts_within_300ms():
    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS))
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
    c = MotionController(d, safety=Safety(LIMITS))
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


def test_goal_at_resting_pose_beyond_limit_is_allowed_as_start():
    from so101_studio.core.config import load_robot

    cfg = load_robot("config/robot.yaml")
    d = SimDriver()
    rest = JointState(0, -89, 97.4, 0, 0, 3)
    d.write_joints(rest)
    c = MotionController(d, safety=Safety(cfg.joint_limits_deg))
    r = c.execute(move_j(rest, JointState(10, -89, 90, 0, 0, 3), 200, 1000)).wait(5)
    c.close()
    assert r.status == "ok"


def test_new_trajectory_may_start_far_from_last_command():
    d = SimDriver()
    c = MotionController(d)
    c.execute(move_j(A, JointState(10, 0, 0, 0, 0, 0), 200, 1000)).wait(5)
    # 서보가 뒤처져 실제 위치가 마지막 명령에서 8° 떨어져 있어도 새 동작은 거부되지 않는다
    r = c.execute(move_j(JointState(2, 0, 0, 0, 0, 0), JointState(20, 0, 0, 0, 0, 0), 200, 1000)).wait(5)
    c.close()
    assert r.status == "ok"


def test_ruckig_move_j_respects_limits_and_ends_at_rest():
    t = move_j(A, B, 27, 60, profile="ruckig", j_max_deg_s3=200)
    ts = [t.duration_s * i / 400 for i in range(401)]
    assert t.sample(0) == A
    end = t.sample(t.duration_s)
    assert np.allclose(np.array(list(vars(end).values())), np.array(list(vars(B).values())), atol=1e-6)
    vel = np.array([t.velocity(x) for x in ts])
    assert np.abs(vel).max() <= 27 * 1.001 and np.allclose(vel[0], 0) and np.allclose(vel[-1], 0, atol=1e-6)
    acc = np.abs(np.diff(vel, axis=0)).max() / (t.duration_s / 400)
    assert acc <= 60 * 1.1
    assert all(abs(t.velocity(x)).max() <= 27.001 for x in ts)


def test_ruckig_is_faster_than_quintic_at_the_same_speed_limit():
    q = move_j(A, B, 27, 120)
    r = move_j(A, B, 27, 120, profile="ruckig", j_max_deg_s3=600)
    assert r.duration_s < q.duration_s


def test_controller_runs_a_ruckig_trajectory_in_sim():
    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS), hz=200)
    r = c.execute(move_j(A, JointState(10, 0, 0, 0, 0, 20), 200, 800, profile="ruckig", j_max_deg_s3=8000)).wait(5)
    c.close()
    assert r.status == "ok"
    assert np.allclose(np.array(list(vars(d.read_joints()).values())), [10, 0, 0, 0, 0, 20], atol=1e-6)


def test_close_during_a_move_releases_the_waiting_handle():
    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS))
    h = c.execute(move_j(A, JointState(60, 0, 0, 0, 0, 0), 10, 100))  # 오래 걸리는 동작
    time.sleep(0.1)
    c.close()
    r = h.wait(1)
    assert r is not None and r.status == "stopped"


def test_a_late_tick_slows_the_move_instead_of_faulting_on_step(monkeypatch):
    import types

    from so101_studio.arm import motion

    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS))
    c._quit.set()
    c._thread.join(2)  # 제어 스레드를 끄고 틱을 직접 부른다
    now = [1000.0]
    monkeypatch.setattr(motion, "time", types.SimpleNamespace(perf_counter=lambda: now[0], sleep=time.sleep))
    h = c.execute(move_j(A, JointState(60, 0, 0, 0, 0, 0), 200, 1000))
    now[0] += c.dt
    c._tick()
    before = d.read_joints().shoulder_pan
    now[0] += 0.5  # 틱이 0.5초 늦었다: 예전에는 위치가 한 번에 수십 도 건너뛰어 'step' fault
    with c._lock:
        c._tick()
    after = d.read_joints().shoulder_pan
    assert c.state == "moving" and c.fault is None and h.result is None
    assert 0 < after - before <= c.safety.max_step_deg and c.time_lost_s > 0.4



def test_late_ticks_catch_up_only_within_the_step_limit_on_a_fast_replay(monkeypatch):
    """213°/s 재생(한 주기 4.3°)을 틱이 23ms 넘게 늦는 가짜 시계로 돌려도 step fault 없이 끝까지 가고, 늦어진 시간이 쌓인다 (REQ-034)."""
    import types

    import numpy as np

    from so101_studio.arm import motion
    from so101_studio.arm.trajectory import SequenceTrajectory

    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS))
    c._quit.set()
    c._thread.join(2)
    now = [1000.0]
    monkeypatch.setattr(motion, "time", types.SimpleNamespace(perf_counter=lambda: now[0], sleep=time.sleep))
    t = np.arange(0, 0.5001, 0.05)
    q = np.tile(np.array(astuple(A), float), (len(t), 1))
    q[:, 0] = -50.0 + 213.0 * t  # 관절 한계 안(-50 -> 56.5)
    h = c.execute(SequenceTrajectory(t, q))
    delays = [0.02, 0.045, 0.02, 0.06, 0.03, 0.02, 0.04, 0.025]
    for i in range(400):
        if h.result is not None:
            break
        now[0] += delays[i % len(delays)]
        with c._lock:
            c._tick()
    assert c.state != "fault" and c.fault is None  # 늦어도 step fault가 아니다
    assert h.result is not None and h.result.status == "ok"
    assert d.read_joints().shoulder_pan == pytest.approx(-50.0 + 213.0 * 0.5, abs=0.5)
    assert c.time_lost_s > 0.05  # 늦어진 시간이 기록된다


def test_non_comm_exception_in_a_tick_is_an_internal_error_fault_not_comm_lost():
    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS), hz=100)
    events = []
    c.on_event = lambda kind, level, code, data: events.append((code, data))

    def bug(q):
        raise KeyError("코드 오류")

    d.write_joints = bug
    r = c.execute(move_j(A, JointState(10, 0, 0, 0, 0, 0), 30, 100)).wait(2)
    c.close()
    assert r.status == "fault" and c.fault == "internal_error" and "KeyError" in c.fault_detail["traceback"]
    assert not any(code == "ARM-COMM-LOST" for code, _ in events)


def test_estop_returns_the_write_failure_or_none():
    d = SimDriver()
    c = MotionController(d, safety=Safety(LIMITS))
    assert c.estop() is None
    d.write_joints = lambda q: (_ for _ in ()).throw(ConnectionError("no status packet"))
    err = c.estop()
    c.close()
    assert err and "ConnectionError" in err
