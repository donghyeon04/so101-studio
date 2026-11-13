"""REQ-055: 기울인 집기 계획이 안 풀리면 수직으로 다시 시도, CLI pick의 선택 인자 높이."""
import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, MotionResult, ToolPose
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


@pytest.fixture(autouse=True)
def _symmetric_mode(monkeypatch):
    monkeypatch.setattr(CFG, "grasp_tilt_mode", "symmetric")  # "기울이기 실패 -> 수직 대체"는 symmetric에서 본다. equal_tips는 기울기가 작아 250mm에서도 풀린다 (결정 0060)


def make(tmp_path, width):
    block = float(np.interp(width, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    return s, ev


@pytest.mark.slow
def test_far_wide_object_falls_back_to_a_vertical_grasp_and_remembers_it(tmp_path):
    # 반경 250mm·폭 58mm: 기울인 집기는 손목 한계로 안 풀린다(리뷰 준비 세션 sim: 250mm에서는 모든 폭) -> 수직으로 다시 계산해 ok
    s, ev = make(tmp_path, 58)
    try:
        obj = DetectedObject("o", 300.0, 300.0, 0.0, 58, 58, 1.0, "manual", 39.0)
        assert s.skills._tilt_ok(obj, s.skills._jaw_yaw(90.0))  # 턱 축이 팔 방향이라 원래는 기울이는 후보
        r = s.skills.pick(obj)
        grip = s.skills.grip
    finally:
        s.disconnect()
    assert r.status == "ok"
    assert [d["reason"] for c, d in ev if c == "ARM-PICK-TILT-UNREACHABLE"] == ["tilt_unreachable"]
    assert grip is not None and grip["tilted"] is False  # 실제로 쓴 값을 기억한다


@pytest.mark.slow
def test_a_reachable_tilted_grasp_is_not_changed(tmp_path):
    s, ev = make(tmp_path, 30)
    try:
        r = s.skills.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0))
        grip = s.skills.grip
    finally:
        s.disconnect()
    assert r.status == "ok" and grip["tilted"] is True and not [1 for c, _ in ev if c == "ARM-PICK-TILT-UNREACHABLE"]


def test_cli_pick_takes_an_optional_height(monkeypatch):
    from so101_studio import cli

    seen = []
    monkeypatch.setattr(ArmService, "pick_and_place", lambda self, obj, zone=None, place_at=None: seen.append((obj, zone)) or MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0)))
    cli.main(["--sim", "pick", "300", "150", "0", "30", "40"])
    cli.main(["--sim", "pick", "300", "150", "0", "30"])
    assert [o.height_mm for o, _ in seen] == [40.0, 0.0] and [o.width_mm for o, _ in seen] == [30.0, 30.0]
    with pytest.raises(SystemExit):
        cli.main(["--sim", "pick", "300", "150", "0"])


@pytest.mark.slow
def test_far_wide_object_with_unknown_height_is_not_retried_vertically(tmp_path):
    # 수직 + 높이 모름은 위 모서리를 치는 조합(sim 틈 -12mm): 바꾸지 않고 height_required로 움직이지 않는다 (결정 0052)
    s, ev = make(tmp_path, 58)
    try:
        before = s.driver.read_joints()
        r = s.skills.pick(DetectedObject("o", 300.0, 300.0, 0.0, 58, 58, 1.0, "manual", 0.0))
        after = s.driver.read_joints()
    finally:
        s.disconnect()
    assert r.status == "height_required" and after == before
    assert [d["reason"] for c, d in ev if c == "ARM-PICK-TILT-UNREACHABLE"] == ["tilt_unreachable_height_unknown"]
