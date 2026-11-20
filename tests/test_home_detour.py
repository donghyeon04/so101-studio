"""REQ-104(피드백 F4): 극한 자세에서 [홈]이 막히면 먼저 수직으로 올린 뒤 홈. 안전 검사는 끄지 않는다."""
from dataclasses import astuple

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG, WS = load_robot("config/robot.yaml", gripper="parallel"), load_workspace("config/workspace.yaml")
LOW = [-84.5154, -58.9084, 61.8415, 87.0669, -84.5159, 3.0]  # 사용자 사례: 턱 가운데 (132, 30), 면 위 7mm, 수직 아래(베이스 바로 앞 바닥 근처)


def svc(tmp_path, start):
    s = ArmService(SimDriver(start), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    return s, ev


def test_low_pose_near_the_base_goes_home_by_lifting_first(tmp_path):
    s, ev = svc(tmp_path, LOW)
    try:
        assert s.safety.check_path(s.skills._joint_move(JointState(*CFG.poses["home"]))) == "z_floor"  # 바로 가면 바닥에 걸린다
        r = s.home()
        assert r.status == "ok", r
        assert max(abs(a - b) for a, b in zip(astuple(s.driver.read_joints()), CFG.poses["home"], strict=True)) < 0.5
        (data,) = [d for c, d in ev if c == "ARM-HOME-DETOUR"]
        assert data["first_reason"] == "z_floor" and 10 <= data["lift_to_mm"] <= CFG.home_lift_mm
    finally:
        s.disconnect()


def test_free_pose_goes_home_in_one_move_without_detour(tmp_path):
    s, ev = svc(tmp_path, CFG.poses["radar_clear"])
    try:
        assert s.home().status == "ok"
        assert not [c for c, _ in ev if c == "ARM-HOME-DETOUR"]
    finally:
        s.disconnect()


def test_detour_also_blocked_stops_with_a_reason_and_does_not_move(tmp_path):
    s, ev = svc(tmp_path, LOW)
    try:
        box = {"x": [None, None], "y": [None, None], "z": [4, 12]}  # 높이 상한 12mm: 홈도, 올리기도 막힌다
        s.set_safety(box_mm=box)
        q0 = s.driver.read_joints()
        r = s.home()
        assert r.status == "stopped" and "home_blocked" in r.reason and "직접 조작" in r.reason
        assert s.driver.read_joints() == q0 and not [c for c, _ in ev if c == "ARM-HOME-DETOUR"]
    finally:
        s.disconnect()
