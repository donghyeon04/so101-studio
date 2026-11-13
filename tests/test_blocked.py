"""REQ-064 (결정 0059): 추종 오차(명령 - 실제)로 막힘·접촉을 잡는다. 기구학 SimDriver 위에서 판정 논리만 본다(MuJoCo는 test_blocked_mujoco.py)."""
import time
from dataclasses import replace

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
HOME = JointState(*CFG.poses["home"])


class Wall(SimDriver):
    """shoulder_pan이 wall_deg 위로는 가지 못한다(장애물에 막힌 것처럼, 명령은 더 나가고 실제는 멈춘다)."""

    def __init__(self, *a, wall_deg=None, **k):
        super().__init__(*a, **k)
        self.wall_deg = wall_deg

    def write_joints(self, q):
        if self.wall_deg is not None and q.shoulder_pan > self.wall_deg:
            q = replace(q, shoulder_pan=self.wall_deg)
        super().write_joints(q)


def make(tmp_path, drv, speed=100):
    s = ArmService(drv, CFG, WS, speed_pct=speed, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((level, code, data))
    return s, ev


def blocked(ev):
    return [e for e in ev if e[1] == "ARM-BLOCKED"]


@pytest.mark.slow
def test_move_stopped_by_a_wall_holds_at_the_actual_position(tmp_path):
    s, ev = make(tmp_path, Wall(CFG.poses["home"], wall_deg=20.0))
    try:
        r = s.move_joints(replace(HOME, shoulder_pan=80.0))
        q = s.driver.read_joints()
        assert r.status == "stopped" and r.reason.startswith("blocked: shoulder_pan")
        assert len(blocked(ev)) == 1 and blocked(ev)[0][0] == "WARN" and blocked(ev)[0][2]["joint"] == "shoulder_pan"
        assert q.shoulder_pan == pytest.approx(20.0) and s.ctl._last.shoulder_pan == pytest.approx(20.0)  # 목표를 지금 위치로 맞춘다
        assert s.ctl.state == "idle" and s.ctl._job is None
    finally:
        s.disconnect()


@pytest.mark.slow
def test_normal_moves_never_trigger_blocked(tmp_path):
    s, ev = make(tmp_path, SimDriver(CFG.poses["home"]))
    try:
        for goal in (JointState(*CFG.poses["radar_clear"]), replace(HOME, shoulder_pan=40.0), HOME):
            assert s.move_joints(goal).status == "ok"
        s.ctl.hold_gripper(30.0)  # 쥔 채 이동
        assert s.move_joints(replace(HOME, shoulder_pan=-30.0, gripper=30.0)).status == "ok"
    finally:
        s.disconnect()
    assert not blocked(ev)


def test_held_profile_uses_its_own_threshold(tmp_path):
    """같은 오차라도 쥔 채 문턱이 더 커서(8°/5° 대 6°/4°) 빈 이동 문턱만 넘는 오차는 쥔 채에서는 막힘이 아니다."""
    s, ev = make(tmp_path, Wall(CFG.poses["home"]))
    try:
        c = s.ctl
        c.blocked_cfg = replace(CFG, blocked_empty_accel_deg=1.0, blocked_empty_cruise_deg=1.0, blocked_held_accel_deg=60.0, blocked_held_cruise_deg=60.0)
        s.driver.wall_deg = 20.0
        c.hold_gripper(30.0)
        r = s.move_joints(replace(HOME, shoulder_pan=45.0, gripper=30.0))
    finally:
        s.disconnect()
    assert r.status == "ok" and not blocked(ev)


def test_gripper_closing_on_something_stops_at_actual_plus_preload(tmp_path):
    """집기 밖의 닫기(gripper 명령)도 접촉으로 본다(결정 0033: 지금 위치 + 예압으로 멈춤)."""
    s, ev = make(tmp_path, SimDriver(CFG.poses["home"], grip_block=30.0), speed=100)
    try:
        s.set_gripper(60.0)
        r = s.set_gripper(0.0)
        for _ in range(60):  # 끝난 직후에도 본다: 짧은 닫기는 동작이 끝난 뒤 상태 읽기 2번에 잡힌다
            if blocked(ev):
                break
            time.sleep(0.05)
        assert r.status in ("stopped", "ok")
        assert s.ctl.grip_hold == pytest.approx(30.0 - CFG.grip_preload_units)
        assert blocked(ev)[0][2]["joint"] == "gripper" and blocked(ev)[0][2]["profile"] == "grip_close"
    finally:
        s.disconnect()


def test_one_noisy_read_is_not_enough():
    """연속 2번 넘어야 막힘이다."""
    from so101_studio.arm.motion import BLOCKED_N
    assert BLOCKED_N == 2


def test_each_finished_move_logs_its_tracking_error_maxima(tmp_path):
    """REQ-067: 정상 동작이 끝나면 ARM-TRACKING INFO(프로필, 속도 비율, 출발·정지/정속 오차 최대와 관절)가 남는다. 기구학 SimDriver는 명령을 그대로 따르므로 값은 0 근처."""
    s, ev = make(tmp_path, SimDriver(CFG.poses["home"]), speed=60)
    try:
        assert s.move_joints(replace(HOME, shoulder_pan=40.0), speed_pct=60).status == "ok"
    finally:
        s.disconnect()
    tracking = [d for _, c, d in ev if c == "ARM-TRACKING"]
    assert len(tracking) == 1 and not blocked(ev)
    d = tracking[0]
    assert d["profile"] == "empty" and d["speed_ratio"] == pytest.approx(0.6) and set(d) >= {"accel", "cruise"}
    assert d["accel"]["err_deg"] < 1.0 and d["cruise"]["err_deg"] < 1.0 and d["accel"]["joint"] and d["cruise"]["joint"]
