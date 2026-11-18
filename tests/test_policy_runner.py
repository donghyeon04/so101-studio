"""PolicyRunner 1차: 시작·정지 (arm-software-spec 6.10)."""
import time

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.logread import read_events
from so101_studio.service.arm_service import ArmService
from so101_studio.service.policy_runner import PolicyRunner

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
HOME = CFG.poses["home"]


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
