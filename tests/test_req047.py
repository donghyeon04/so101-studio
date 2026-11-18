"""REQ-047: 진단(powertest, shaketest)은 끝나거나 중단되거나 실패해도 쉬는 자세로 돌아가고, 한계 밖에서 시작하면 먼저 home으로 옮긴다."""
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli import powertest, shaketest
from so101_studio.cli.rest import go_rest, prepare_rest, warn_if_loaded
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
HOME = JointState(*CFG.poses["home"])
RESTING = JointState(0, -89, 96.4, 0, 0, 3)  # 쉬는 자세: 팔꿈치 96.4° > 소프트 한계 95° (2026-11-07 실물 powertest의 시작 자세)


@pytest.fixture
def svc(tmp_path):
    d = SimDriver(CFG.poses["home"])
    s = ArmService(d, CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    yield s
    s.disconnect()


def near(a: JointState, b: JointState, tol=1.0) -> bool:
    return all(abs(getattr(a, j) - getattr(b, j)) <= tol for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"))


def test_resting_pose_start_is_moved_to_home_first(svc):
    svc.driver.write_joints(RESTING)
    out = []
    assert svc.safety.check_goal(RESTING) is not None  # 한계 밖
    start = prepare_rest(svc, out.append)
    assert near(start, HOME) and any("home으로" in m for m in out)


def test_in_limit_start_is_kept(svc):
    out = []
    assert near(prepare_rest(svc, out.append), HOME) and out == []


def test_go_rest_falls_back_to_home_when_the_start_pose_is_refused(svc):
    svc.driver.write_joints(JointState(0, -30, 30, 50, 0, 3))
    out = []
    assert go_rest(svc, RESTING, out.append) == "ok"  # RESTING은 한계 밖이라 거절 -> home
    assert near(svc.driver.read_joints(), HOME) and any("거절" in m for m in out)


def test_go_rest_warns_and_does_not_move_when_stopped(svc):
    svc.driver.write_joints(JointState(0, -30, 30, 50, 0, 3))
    svc.ctl.estop()
    out = []
    here = svc.driver.read_joints()
    go_rest(svc, HOME, out.append)
    assert svc.driver.read_joints() == here and any("경고" in m for m in out)


@pytest.mark.slow
def test_powertest_from_the_resting_pose_ends_at_home(svc, monkeypatch):
    monkeypatch.setattr(powertest, "SETTLE_S", 0.2)
    svc.driver.write_joints(RESTING)
    out = []
    res = powertest.run_powertest(svc, repeats=1, out=out.append)
    assert res["return"] == "ok" and near(svc.driver.read_joints(), HOME)  # 편 자세에 남지 않는다


@pytest.mark.slow
def test_powertest_aborted_still_returns(svc, monkeypatch):
    monkeypatch.setattr(powertest, "SETTLE_S", 0.2)
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] >= 3  # 세 번째 이동 앞에서 중단

    res = powertest.run_powertest(svc, repeats=1, out=lambda m: None, stop=stop)
    assert res.get("return") == "ok" and near(svc.driver.read_joints(), HOME)


def test_shaketest_aborted_returns_to_start(svc, monkeypatch):
    monkeypatch.setattr(shaketest, "SPEEDS", (200.0,))
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] >= 2  # 갈 때 한 번 간 뒤 중단
    shaketest.run_shaketest(svc, reps=1, out=lambda m: None, accels=(254,), stop=stop)
    assert near(svc.driver.read_joints(), HOME)


class _Health:
    def __init__(self, load):
        self.load = load

    def read_health(self):
        return {"load_pct": self.load}


def test_warn_if_loaded_only_when_the_load_is_high():
    out = []
    warn_if_loaded(type("S", (), {"driver": _Health({"shoulder_lift": 55.0, "gripper": 3.0})})(), out.append)
    assert len(out) == 1 and "shoulder_lift" in out[0] and "home" in out[0]
    out.clear()
    warn_if_loaded(type("S", (), {"driver": _Health({"shoulder_lift": 12.0})})(), out.append)
    assert out == []
