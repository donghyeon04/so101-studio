"""REQ-106(피드백 F3): ArmService.held는 집기가 닫아 쥘 때 채워지고 놓기가 벌리기 시작할 때 비워진다. ARM-HELD(held·released, 놓은 자리)."""
import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG, WS = load_robot("config/robot.yaml"), load_workspace("config/workspace.yaml")
OBJ = DetectedObject("a", 215.0, 160.0, 0.0, 30.0, 100.0, 1.0, "manual", 40.0)  # config/manual_objects.yaml 첫 물체


@pytest.mark.slow
def test_held_is_filled_while_carrying_and_cleared_when_opening(tmp_path):
    d = SimDriver(CFG.poses["home"])
    s = ArmService(d, CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    d.grip_block = s.skills.gripper_for_width(30.0)
    ev, during = [], []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    real_place = s.skills.place

    def place(zone):  # 놓기 직전(옮기는 중)의 held를 본다
        during.append(s.held)
        return real_place(zone)

    s.skills.place = place
    try:
        assert s.held is None
        r = s.pick_and_place(OBJ, zone="right")
        assert r.status == "ok", r
        (h,) = during
        assert h is not None and h["object"].id == "a"
        rel = np.array(h["obj_in_tool_mm"])
        assert np.linalg.norm(rel[:2]) < 15 and h["tool"].z_mm > CFG.table_z_mm  # 물체 중심이 손끝 가까이(턱 사이)
        assert s.held is None
        held = [dt for c, dt in ev if c == "ARM-HELD"]
        assert [x["state"] for x in held] == ["held", "released"] and held[1]["reason"] == "placed" and held[1]["at"] == [355.0, 80.0]
    finally:
        s.disconnect()


def test_release_is_quiet_when_nothing_is_held(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append(code)
    try:
        s.skills._release("new_pick")
        assert s.held is None and "ARM-HELD" not in ev
    finally:
        s.disconnect()
