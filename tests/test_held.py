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


@pytest.mark.slow
def test_kinematic_sim_with_object_grip_fills_held_without_a_test_grip_block(tmp_path):
    """REQ-114: GUI 기구학 엔진(SimDriver(object_grip=True), grip_block 없음)도 집기가 알려 준 폭에서 멈춰 접촉 → held가 채워지고 놓으면 비워진다."""
    d = SimDriver(CFG.poses["home"], object_grip=True)
    s = ArmService(d, CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    during = []
    real_place = s.skills.place
    s.skills.place = lambda zone: (during.append(s.held), real_place(zone))[1]
    try:
        r = s.pick_and_place(OBJ, zone="right")
        assert r.status == "ok", r  # no_object가 아니라 쥠
        assert during[0] is not None and during[0]["object"].id == "a" and s.held is None
        assert d._expected is None  # 놓은 뒤 막힘을 비운다: 그리퍼를 끝까지 닫을 수 있다
        lo = CFG.joint_limits_deg["gripper"][0]
        assert s.skills._run([s.skills._gripper(lo)]).status == "ok" and d.read_joints().gripper == pytest.approx(lo, abs=0.5)
    finally:
        s.disconnect()


def test_unknown_width_or_plain_sim_driver_closes_fully_as_before(tmp_path):
    d = SimDriver(CFG.poses["home"])  # object_grip 기본 False: 시험·CLI 기구학은 그대로
    d.expect_grip(40.0)
    d.write_joints(d.read_joints().__class__(*CFG.poses["home"][:5], 0.0))
    assert d.read_joints().gripper == 0.0
    d2 = SimDriver(CFG.poses["home"], object_grip=True)
    d2.expect_grip(None)
    d2.write_joints(d2.read_joints().__class__(*CFG.poses["home"][:5], 0.0))
    assert d2.read_joints().gripper == 0.0


@pytest.mark.slow
def test_dry_run_task_with_object_grip_still_succeeds_and_reports_held(tmp_path):
    """dry_run 성공 판정은 그대로(이제 no_object 대신 ok로 집는다). 물체마다 ARM-HELD held → released."""
    from so101_studio.perception.manual import ManualSource
    from so101_studio.service.studio import Studio

    s = ArmService(SimDriver(CFG.poses["home"], object_grip=True), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    src = ManualSource("config/manual_objects.yaml")
    n = len(src.scan().objects)
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    try:
        res = Studio(s, ManualSource("config/manual_objects.yaml"), dry_run=True).start(blocking=True)
    finally:
        s.disconnect()
    assert res["end"] == "no_objects" and len(res["picked"]) == n and res["failed"] == [], res
    states = [d["state"] for c, d in ev if c == "ARM-HELD"]
    assert states == ["held", "released"] * n, states
