"""PolicyRunner 1차: 시작·정지 (arm-software-spec 6.10)."""
import time

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.logread import read_events
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService
from so101_studio.service.policy_runner import PolicyRunner

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
HOME = CFG.poses["home"]
HOME_Q = JointState(*HOME)


class CamSim(SimDriver):
    def render(self, camera, width=320, height=240):
        return np.zeros((height, width, 3), np.uint8)


class HomePolicy:
    def __init__(self):
        self.calls, self.resets = 0, 0

    def select_action(self, obs):
        assert obs["observation.state"].shape == (6,) and obs["observation.images.wrist"].shape == (240, 320, 3)
        self.calls += 1
        return list(HOME)

    def reset(self):
        self.resets += 1


def make(tmp_path, driver):
    s = ArmService(driver, CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    return s


def policy_events(s):
    return [e for e in read_events(s.logger.dir) if e["code"].startswith("ARM-POLICY")]


def test_start_then_stop_leaves_two_events_and_judge_outcome(tmp_path, monkeypatch):
    monkeypatch.setattr("so101_studio.service.policy_runner.JUDGE_DELAY_S", 0.1)
    s = make(tmp_path, CamSim(HOME))
    try:
        pol = HomePolicy()
        r = PolicyRunner(s, s.driver, policy=pol, judge=lambda d: {"outcome": "placed_ok"})
        assert r.start() is None
        time.sleep(0.4)
        r.stop()
        ev = policy_events(s)
        assert [e["code"] for e in ev] == ["ARM-POLICY-START", "ARM-POLICY-STOP"]
        assert ev[0]["data"]["fps"] == 25
        assert ev[1]["data"]["reason"] == "stop" and ev[1]["data"]["outcome"] == "placed_ok" and ev[1]["data"]["steps"] >= 1
        assert pol.resets == 1 and pol.calls >= 1
        assert s.ctl.state == "idle" and s.driver.read_joints() == s.ctl._last
    finally:
        s.disconnect()


def test_ends_when_servo_is_refused(tmp_path):
    s = make(tmp_path, CamSim(HOME))
    try:
        s.estop()  # 정지 상태: servo_joints가 False
        r = PolicyRunner(s, s.driver, policy=HomePolicy(), max_s=5)
        assert r.start() is None
        r._thread.join(2)
        assert r.result["reason"] == "servo:stopped"
    finally:
        s.disconnect()


class ScriptPolicy(HomePolicy):
    """bad(n)이 참인 호출에서 관절 한계 밖 목표를 낸다. delay_s만큼 늦게 낸다."""

    def __init__(self, bad, delay_s=0.0):
        super().__init__()
        self.bad, self.delay_s = bad, delay_s

    def select_action(self, obs):
        time.sleep(self.delay_s)
        q = super().select_action(obs)
        return [CFG.joint_limits_deg["shoulder_pan"][1] + 20, *q[1:]] if self.bad(self.calls) else q


def test_a_short_safety_rejection_does_not_end_the_run(tmp_path):
    s = make(tmp_path, CamSim(HOME))
    try:
        r = PolicyRunner(s, s.driver, policy=ScriptPolicy(lambda n: n <= 3), max_s=5)  # 처음 세 번(0.12초)만 거부
        assert r.start() is None
        time.sleep(0.8)
        r.stop()
        assert r.result["reason"] == "stop"
    finally:
        s.disconnect()


def test_a_safety_rejection_that_lasts_ends_the_run_after_half_a_second(tmp_path):
    s = make(tmp_path, CamSim(HOME))
    try:
        r = PolicyRunner(s, s.driver, policy=ScriptPolicy(lambda n: True), max_s=5)
        t0 = time.perf_counter()
        assert r.start() is None
        r._thread.join(3)
        assert r.result["reason"] == "servo:joint_limit:shoulder_pan" and time.perf_counter() - t0 >= 0.5
    finally:
        s.disconnect()


def test_inference_slower_than_a_cycle_ends_as_too_slow(tmp_path):
    s = make(tmp_path, CamSim(HOME))
    try:
        r = PolicyRunner(s, s.driver, policy=ScriptPolicy(lambda n: False, delay_s=0.06), max_s=5)  # 늘 40ms 넘김
        assert r.start() is None
        r._thread.join(3)
        assert r.result["reason"] == "too_slow" and r.result["skipped"] >= 10
    finally:
        s.disconnect()


def test_does_not_start_without_render(tmp_path):
    s = make(tmp_path, SimDriver(HOME))
    try:
        why = PolicyRunner(s, s.driver, policy=HomePolicy()).start()
        assert why and "MuJoCo" in why and policy_events(s) == []
    finally:
        s.disconnect()


@pytest.mark.slow
def test_missing_checkpoint_returns_a_message(tmp_path):
    s = make(tmp_path, CamSim(HOME))
    try:
        why = PolicyRunner(s, s.driver, checkpoint_dir=tmp_path / "없음").start()
        assert why and why.startswith("정책을 불러오지 못했다") and policy_events(s) == []
    finally:
        s.disconnect()


class FakeSvc:
    """시계로 돌리기 시험용: servo_joints는 늘 받고, 보낸 목표와 이벤트만 센다."""

    def __init__(self, clock):
        from types import SimpleNamespace
        self.clock, self.sent, self.codes = clock, [], []
        self.ctl = SimpleNamespace(servo_reason=None)

    def servo_joints(self, q):
        self.sent.append(q)
        return True

    def stop(self):
        pass

    def log_event(self, kind, data, level="INFO", code=""):
        self.codes.append((code, data))


def test_clock_driven_run_begin_step_finish():
    """REQ-086: 시계(StepClock.every)로 돌리는 공개 경로. ManualClock으로 걸음마다 1/fps씩 흘리면 max_s에서 끝나고 too_slow는 보지 않는다."""
    from types import SimpleNamespace

    from test_clock import ManualClock

    clk = ManualClock()
    svc = FakeSvc(clk)
    drv = SimpleNamespace(render=lambda cam, width=320, height=240: np.zeros((height, width, 3), np.uint8), read_joints=lambda: HOME_Q)
    r = PolicyRunner(svc, drv, policy=HomePolicy(), fps=25, max_s=1.0)
    assert r.begin(clock="step") is None
    n = 0
    while (why := r.due_stop(slow=False) or r.step()) is None:
        n += 1
        clk.t = 1000.0 + n / 25  # 시뮬레이션 시간만 흐른다(더해 쌓으면 0.04×25가 1.0에 못 미친다)
    r.finish(why)
    assert why == "max_s" and n == 25 and len(svc.sent) == 25
    assert [c for c, _ in svc.codes] == ["ARM-POLICY-START", "ARM-POLICY-STOP"] and svc.codes[0][1]["clock"] == "step"
    assert r.result["reason"] == "max_s" and r.result["steps"] == 25 and r.result["skipped"] == 0


def test_pause_sends_nothing_and_max_s_and_too_slow_skip_the_paused_time(tmp_path):
    """REQ-108(gui-spec 12.10 [일시 정지]): 멈춘 동안 목표를 안 보내고 팔은 그 자리, max_s·too_slow는 멈춘 시간을 빼고 센다."""
    s = make(tmp_path, CamSim(HOME))
    try:
        pol = HomePolicy()
        r = PolicyRunner(s, s.driver, policy=pol, max_s=1.0)
        assert r.start() is None
        time.sleep(0.3)
        r.pause()
        assert r.paused
        time.sleep(0.1)
        n, q = pol.calls, s.driver.read_joints()
        time.sleep(1.5)  # max_s(1.0)와 SLOW_S(1.0)를 넘게 멈춘다
        assert r._thread.is_alive() and r.result is None and pol.calls == n and s.driver.read_joints() == q
        r.resume()
        assert not r.paused
        time.sleep(0.3)
        assert pol.calls > n and r.result is None  # 이어서 돌고, 멈춘 시간 때문에 too_slow·max_s로 끝나지 않았다
        r._thread.join(2)
        assert r.result["reason"] == "max_s"
        codes = [e["code"] for e in policy_events(s)]
        assert codes == ["ARM-POLICY-START", "ARM-POLICY-PAUSE", "ARM-POLICY-RESUME", "ARM-POLICY-STOP"]
    finally:
        s.disconnect()


def test_stop_while_paused_ends_at_once(tmp_path):
    s = make(tmp_path, CamSim(HOME))
    try:
        r = PolicyRunner(s, s.driver, policy=HomePolicy(), max_s=10)
        assert r.start() is None
        time.sleep(0.2)
        r.pause()
        t0 = time.monotonic()
        r.stop()
        assert not r._thread.is_alive() and time.monotonic() - t0 < 0.5 and r.result["reason"] == "stop"
    finally:
        s.disconnect()
