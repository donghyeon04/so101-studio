"""놓을 때 opening path의 시작·끝 틈과 손 이동량을 INFO 이벤트(ARM-OPENING-PATH)로 남긴다 (실물 2-2 1회차 요청)."""
import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


@pytest.mark.slow
def test_place_logs_the_opening_path_gap_and_the_hand_move(tmp_path):
    block = float(np.interp(58.0, [p[0] for p in TABLE], [p[1] for p in TABLE]))  # 58mm 물체를 쥔 값
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, level, data))
    try:
        assert s.pick_and_place(DetectedObject("o", 300, 200, 0, 58, 58, 1.0, "manual", 39), "right").status == "ok"
    finally:
        s.disconnect()
    hits = [e for e in ev if e[0] == "ARM-OPENING-PATH"]
    assert len(hits) == 1 and hits[0][1] == "INFO" and not any(e[0] == "ARM-OPENING-PATH-FALLBACK" for e in ev)
    d = hits[0][2]
    assert d["gap_start_mm"] == pytest.approx(58.0, abs=1.0) and d["gap_end_mm"] > d["gap_start_mm"] + 30  # 틈이 58mm에서 약 95mm로
    assert d["hand_move_xy_mm"] == pytest.approx((d["gap_end_mm"] - d["gap_start_mm"]) / 2, abs=3.0)  # 두 턱 가운데를 지키려고 손이 벌어진 양의 절반 움직인다(고정 턱이 눈에 보일 만큼)
    assert d["steps"] > 5 and d["duration_s"] > 0.5
