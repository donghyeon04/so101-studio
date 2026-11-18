"""REQ-050 (arm-software-spec 11.3): 놓을 때도 두 턱이 크고 분명하게 양쪽으로 벌어진다."""
import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)
ZONE = {"x_mm": 500.0, "y_mm": 150.0}  # 칸으로 나누기 전 기본 점(11.3.5 이전)


def run(tmp_path, width, known=()):
    """폭 width mm 물체를 쥔 채 배출 구역 right에 놓고, (결과, ARM-OPENING-PATH 이벤트 데이터, 폴백 데이터)를 돌려준다."""
    block = float(np.interp(width, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    s.skills.known_objects = list(known)
    try:
        assert s.skills.pick(DetectedObject("o", 300, 200, 0, width, width, 1.0, "manual", 39)).status == "ok"
        s.skills.grip = None  # REQ-053 이후 기울여 집은 기억이 있으면 대칭 V자로 가므로, 여기서는 첫판(③ fixed_tilt: 수직 손으로 옆으로만)을 시험한다
        r = s.skills.place_at(ZONE["x_mm"], ZONE["y_mm"])
        final = s.driver.read_joints()
    finally:
        s.disconnect()
    path = [d for c, d in ev if c == "ARM-OPENING-PATH"]
    fallback = [d for c, d in ev if c == "ARM-OPENING-PATH-FALLBACK"]
    return r, path, fallback, final


@pytest.mark.slow
@pytest.mark.parametrize("width", [30, 45, 58])
def test_both_jaws_open_wide_and_fast_for_every_demo_width(tmp_path, width):
    r, path, fallback, _ = run(tmp_path, width)
    assert r.status == "ok" and len(path) == 1 and not fallback
    d = path[0]
    assert d["actual_fixed_tip_move_mm"] >= 25.0  # 고정 턱 끝이 눈에 보일 만큼 비켜 간다
    assert d["actual_mid_move_mm"] <= 3.0  # 두 턱 가운데(물체 자리)는 수평으로 제자리
    assert 0.8 <= d["actual_s"] <= 1.2 and d["duration_s"] == pytest.approx(CFG.place_open_s)
    cap = float(np.interp(CFG.joint_limits_deg["gripper"][1], [v for _, v in TABLE], [mm for mm, _ in TABLE]))
    assert d["gap_end_mm"] == pytest.approx(min(width + 2 * CFG.place_clearance_mm, cap), abs=1.5)  # 쥔 틈 + 양쪽 30mm (최대 벌림까지)
    assert d["clearance_mm"] == CFG.place_clearance_mm


@pytest.mark.slow
def test_an_obstacle_on_one_side_shrinks_the_clearance(tmp_path):
    # 폭 45mm를 쥐고 놓는 자리 옆(가동 턱 쪽) 62mm에 반지름 10mm 물체: 여유 30이면 턱 끝(+52.5)이 닿고, 20이면(+42.5) 피한다
    obstacle = (ZONE["x_mm"] + 62.0, ZONE["y_mm"] + 1.4, 10.0)
    r, path, fallback, _ = run(tmp_path, 45, known=[obstacle])
    assert r.status == "ok" and not fallback and len(path) == 1
    assert path[0]["clearance_mm"] < CFG.place_clearance_mm and path[0]["clearance_mm"] >= 10.0
    assert path[0]["requested_clearance_mm"] == CFG.place_clearance_mm


@pytest.mark.slow
def test_boxed_in_on_both_sides_falls_back_with_reason_clearance_and_still_opens(tmp_path):
    boxed = [(ZONE["x_mm"] + dx, ZONE["y_mm"] + 1.4, 15.0) for dx in (-45.0, 45.0)]
    r, path, fallback, final = run(tmp_path, 45, known=boxed)
    assert r.status == "ok" and not path  # 계획 벌림은 없다
    assert len(fallback) == 1 and fallback[0]["reason"] == "clearance" and fallback[0]["blocked_by"].startswith("object@")
    assert final.gripper == pytest.approx(CFG.gripper_open, abs=1.0)  # 그 자리에서 열었다


@pytest.mark.slow
def test_a_placed_object_is_remembered_and_forgotten_when_picked_again(tmp_path):
    block = float(np.interp(45, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        assert s.pick_and_place(DetectedObject("o", 300, 200, 0, 45, 45, 1.0, "manual", 39), "right").status == "ok"
        assert len(s.skills.placed) == 1 and s.skills.placed[0][2] == pytest.approx(22.5, abs=1.0)  # 놓은 물체: 자리와 반지름(쥔 틈/2)
        s.skills.placed = [(300.0, 200.0, 20.0)]  # 이번에 집을 자리 근처 기록은 집으면 지워진다
        s.skills.pick(DetectedObject("o2", 300, 200, 0, 45, 45, 1.0, "manual", 39))
        assert s.skills.placed == []
    finally:
        s.disconnect()
