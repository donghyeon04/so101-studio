"""11-12 실물 버그 2건: (1) 집은 물체가 known_objects에 있어 놓기 호버가 막힌다, (2) 쥔 채 막히면 새 동작이 그리퍼 step fault."""
import numpy as np
import pytest

from so101_studio.arm.fault_text import describe
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)
GUI_KNOWN = [(300.0, 190.0, 50.0), (250.0, 210.0, 50.0), (350.0, 200.0, 50.0)]  # GUI 자동 실행이 넘기는 기본 수동 물체(집는 obj-1 자신 포함, 길이 100 -> 반지름 50)


def make(tmp_path, width=30):
    block = float(np.interp(width, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((level, code, data))
    return s, ev


@pytest.mark.slow
def test_place_from_the_real_lifted_pose_is_not_blocked_by_the_object_being_carried(tmp_path):
    # 실물 로그(runs/20261112-004018): 들어올린 자세가 sim보다 낮아(낮은 턱 끝 47.9mm) 알려진 물체 obj-1(300,190)과 이미 겹쳐 있었다 -> 칸 3개 호버가 모두 막혀 FAULT
    s, ev = make(tmp_path)
    try:
        sk = s.skills
        assert sk._run([sk._joint_move(JointState(-0.3, -14.9, 31.1, 85.6, -1.0, 15.8))]).status == "ok"
        sk.grip = {"tilted": True, "tip_h": 20.0, "obj_h": 40.0, "obj_w": 30.0, "held_gap": 28.0}
        sk.known_objects = list(GUI_KNOWN)
        r = sk.place("right")
    finally:
        s.disconnect()
    assert r.status == "ok" and not [1 for _, c, _ in ev if c == "ARM-PLACE-LEGACY"]


@pytest.mark.slow
def test_pick_forgets_the_known_object_it_picks_up(tmp_path):
    s, _ = make(tmp_path)
    try:
        s.skills.known_objects = list(GUI_KNOWN)
        assert s.skills.pick(DetectedObject("obj-1", 300.0, 190.0, 0.0, 30, 100, 1.0, "manual", 40.0)).status == "ok"
        assert (300.0, 190.0, 50.0) not in s.skills.known_objects and len(s.skills.known_objects) == 2
    finally:
        s.disconnect()


def test_an_object_overlapping_at_the_start_is_ignored_but_a_new_overlap_is_still_a_collision(tmp_path):
    s, _ = make(tmp_path)
    try:
        sk = s.skills
        sk.known_objects = [(300.0, 190.0, 50.0)]
        real = JointState(-0.3, -14.9, 31.1, 85.6, -1.0, 15.8)  # 실물 들어올린 자세: 낮은 턱 끝 47.9mm라 (300,190) 원과 겹친다
        home = JointState(*CFG.poses["home"])
        assert sk._hits_objects([real, real], held_r=14.0) is None  # 시작부터 겹친 물체: 새 충돌이 아니다
        assert sk._hits_objects([home, real], held_r=14.0) == "object@(300,190)"  # 겹치지 않다가 겹치면 충돌
    finally:
        s.disconnect()


@pytest.mark.slow
def test_a_new_motion_while_holding_a_blocked_gripper_does_not_step_fault(tmp_path):
    # 물체에 막혀 실제 그리퍼가 24.4인데 hold 명령은 그보다 9단위 낮다(쥔 압력): 새 동작 첫 틱이 13° 한 주기로 잡히던 것
    s, ev = make(tmp_path)
    try:
        sk = s.skills
        actual = sk._q().gripper
        sk.ctl.hold_gripper(actual - 9.0)
        sk.ctl._last = sk.ctl._last.__class__(*[getattr(sk.ctl._last, f) for f in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")], actual - 9.0)
        r = sk.home()
        state = sk.ctl.state
    finally:
        s.disconnect()
    assert r.status == "ok" and state != "fault" and not [1 for _, c, _ in ev if c == "ARM-FAULT"]


def test_gripper_step_fault_text_says_the_gripper_is_holding_an_object():
    text = describe("step:gripper", {"joint": "gripper", "delta_deg": 13.2, "limit_deg": 5.0, "gripper_held": True})
    assert "물체를 쥔 채" in text and "더 느린 속도로" not in text
    assert "더 느린 속도로" in describe("step:shoulder_lift", {"joint": "shoulder_lift", "delta_deg": 9.0, "limit_deg": 5.0})
