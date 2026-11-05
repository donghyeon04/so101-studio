"""REQ-026: 읽기만 실패하고 쓰기는 되는 통신(서보 하나가 빠지거나 버스 잡음, sync_write는 응답을 안 받는다)에서
동작이 영원히 돌아오지 않는 문제. 어떤 동작이든 제한 시간 안에 돌아오고, 상태는 fault(comm_lost)로 이어져야 한다."""
import threading
import time

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import SequenceTrajectory
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
LIMIT_S = 8.0  # 이 안에 돌아오지 않으면 멈춘 것으로 본다


class ReadFails(SimDriver):
    """쓰기는 되는데 읽기(위치·건강)만 실패하는 장치."""
    fail = False

    def read_joints(self):
        if self.fail:
            raise ConnectionError("read timeout")
        return super().read_joints()

    def read_health(self):
        if self.fail:
            raise ConnectionError("read timeout")
        return super().read_health()


@pytest.fixture
def svc():
    s = ArmService(ReadFails(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    assert s.connect()[0]
    assert s.move_joints(JointState(25.0, -60.0, 60.0, 20.0, 0.0, 10.0), 100).status == "ok"  # 홈이 아닌 곳에서 시작
    yield s
    s.driver.fail = False
    s.disconnect()


def _call_with_read_failure(svc, fn, at: float):
    """fn을 스레드로 돌리고, 동작이 at(0~1) 비율만큼 진행됐을 때 읽기를 끊는다. 돌아온 결과(또는 None)와 스레드 생존을 돌려준다."""
    out = {}
    th = threading.Thread(target=lambda: out.update(r=fn()), daemon=True)
    th.start()
    end = time.time() + 5
    while svc.ctl._job is None and time.time() < end:
        time.sleep(0.005)
    job = svc.ctl._job
    assert job is not None, "동작이 시작되지 않았다"
    traj, _, t0, _ = job
    while time.perf_counter() - t0 < at * traj.duration_s:
        time.sleep(0.005)
    svc.driver.fail = True
    th.join(LIMIT_S)
    return out.get("r"), th.is_alive()


def _expect_fault_soon(svc):
    end = time.time() + 2.0  # 끊김 감시는 0.5초 안팎이다
    while svc.ctl.state != "fault" and time.time() < end:
        time.sleep(0.02)
    assert svc.ctl.state == "fault" and svc.ctl.fault == "comm_lost"


@pytest.mark.parametrize("at", [0.3, 0.97])
def test_home_returns_and_faults(svc, at):
    r, alive = _call_with_read_failure(svc, svc.home, at)
    assert not alive, "home이 돌아오지 않는다"
    assert r.status == "fault"
    _expect_fault_soon(svc)


@pytest.mark.parametrize("at", [0.3, 0.97])
def test_move_tool_returns_and_faults(svc, at):
    pose = svc.point_pose(300.0, 200.0, 55.0)
    r, alive = _call_with_read_failure(svc, lambda: svc.move_tool(pose), at)
    assert not alive, "move_tool이 돌아오지 않는다"
    assert r.status == "fault"
    _expect_fault_soon(svc)


@pytest.mark.parametrize("at", [0.3, 0.97])
def test_recorded_sequence_replay_returns_and_faults(svc, at):
    q0 = svc.driver.read_joints()
    q = [np.array([q0.shoulder_pan + 20 * k / 10, q0.shoulder_lift, q0.elbow_flex, q0.wrist_flex, q0.wrist_roll, q0.gripper]) for k in range(11)]
    seq = SequenceTrajectory(np.linspace(0, 1.5, 11), np.array(q))  # 교시 재생과 같은 궤적
    r, alive = _call_with_read_failure(svc, lambda: svc.ctl.execute(seq).wait(), at)
    assert not alive, "재생이 돌아오지 않는다"
    assert r.status == "fault"
    _expect_fault_soon(svc)


@pytest.mark.parametrize("at", [0.3, 0.97])
def test_pick_and_place_returns_and_faults(svc, at):
    obj = DetectedObject("obj-1", 300.0, 170.0, 0.0, 30.0, 30.0, 1.0, "manual")
    r, alive = _call_with_read_failure(svc, lambda: svc.pick_and_place(obj, "right"), at)
    assert not alive, "작업 동작이 돌아오지 않는다"
    assert r.status != "ok"
    _expect_fault_soon(svc)


def test_stop_does_not_raise_when_the_bus_cannot_be_read(svc):
    th = threading.Thread(target=lambda: svc.home(), daemon=True)
    th.start()
    end = time.time() + 5
    while svc.ctl._job is None and time.time() < end:
        time.sleep(0.005)
    svc.driver.fail = True
    svc.stop()  # 예외 없이 끝나야 한다(결과를 읽지 못해도)
    th.join(LIMIT_S)
    assert not th.is_alive()
