"""쥠 확인(arm-software-spec 6.6, REQ-076): 접촉으로 멈췄어도 턱이 벌린 폭에서 거의 닫히지 못했으면(윗면에 얹힘) no_object."""
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.logread import read_events
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
OBJ = DetectedObject("b", 300, 200, 0, 0, 0, 1.0, "manual", 39)  # 폭을 모름: 최대로 벌린다


def pick(tmp_path, closed_below_open_mm):
    """벌린 폭보다 closed_below_open_mm만큼 좁은 곳에서 그리퍼가 막히는 집기."""
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        sk = s.skills
        opened = sk._gap_mm(sk.gripper_for_width(CFG.grasp_max_open_mm))
        s.driver.grip_block = sk.gripper_for_width(opened - closed_below_open_mm)
        sk.known_objects = [(OBJ.x_mm, OBJ.y_mm, 0.0)]
        r = sk.pick(OBJ)
        s.logger.flush()
        return r, sk, [e for e in read_events(s.logger.dir) if e["code"] == "ARM-PICK-NO-GRIP"]
    finally:
        s.disconnect()


@pytest.mark.slow
def test_jaws_resting_on_the_top_are_no_object(tmp_path):
    r, sk, ev = pick(tmp_path, 2.0)
    assert r.status == "no_object" and sk.grip is None
    assert sk.known_objects == [(OBJ.x_mm, OBJ.y_mm, 0.0)]  # 장애물로 그대로 남는다
    info = sk.last_pick_info
    assert info["no_grip"] is True and info["closed_mm"] > info["open_mm"] - CFG.grip_min_close_mm
    assert len(ev) == 1 and ev[0]["level"] == "WARN" and ev[0]["data"] == {"open_mm": info["open_mm"], "closed_mm": info["closed_mm"]}
    assert sk._grip_orig is None and sk.ctl.grip_hold is None  # 조임 한계·고정을 되돌렸다


@pytest.mark.slow
def test_a_real_grasp_is_still_ok(tmp_path):
    r, sk, ev = pick(tmp_path, 40.0)
    assert r.status == "ok" and sk.grip and ev == [] and "no_grip" not in sk.last_pick_info
