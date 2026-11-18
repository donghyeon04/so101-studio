"""REQ-058 (arm-software-spec 11.3.3·11.3.5): 배출 구역 칸, 낮은 hover 규칙(low_hover)."""
import math

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import WorkspaceConfig, load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


def ws_with(zone: dict) -> WorkspaceConfig:
    return WorkspaceConfig(table_mm=WS.table_mm, robot_base_in_table=WS.robot_base_in_table, drop_zones={"z": zone}, safety_margin_mm=10, reach_mm=WS.reach_mm)


def test_default_zone_has_three_slots_nearest_to_the_robot_first():
    slots = WS.zone_slots("right")
    b = WS.base
    dist = [math.hypot(x - b.x_mm, y - b.y_mm) for x, y in slots]
    assert len(slots) == 3 and dist == sorted(dist) and max(dist) <= WS.reach_mm["max"]  # 4칸 중 반경 250mm 밖의 한 칸은 뺐다
    assert slots[0] == pytest.approx((442.5, 112.5))


def test_slot_grid_is_centered_on_the_rectangle_and_clipped_by_the_reach():
    slots = ws_with({"x_mm": 300, "y_mm": 150, "w_mm": 150, "h_mm": 75, "slot_mm": 75}).zone_slots("z")
    # 3x2 격자 6칸 가운데 로봇 원점(300,50)에서 90mm 안인 (300, 112.5)는 집기 범위 밖이라 빠진다
    assert len(slots) == 5 and (300.0, 112.5) not in slots and {slots[0], slots[1]} == {(225.0, 112.5), (375.0, 112.5)}
    far = ws_with({"x_mm": 300, "y_mm": 400, "w_mm": 100, "h_mm": 50, "slot_mm": 75})
    assert far.zone_slots("z") == []  # 전부 집기 범위 밖


def test_old_point_format_has_no_slots_and_ignore_zones_cover_the_rectangle():
    assert ws_with({"x_mm": 500, "y_mm": 150}).zone_slots("z") is None
    (x, y, r), = WS.ignore_zones()
    assert (x, y) == (480.0, 150.0) and r == pytest.approx(math.hypot(50, 40) + WS.safety_margin_mm)  # 반 대각선 + 안전 여유
    (_, _, r_old), = ws_with({"x_mm": 500, "y_mm": 150}).ignore_zones()
    assert r_old == WS.drop_zone_ignore_radius_mm


def make(tmp_path, width=30):
    block = float(np.interp(width, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((level, code, data))
    return s, ev


def holding(s, h=40.0):
    assert s.skills.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", h)).status == "ok"


@pytest.mark.slow
def test_second_object_goes_to_the_next_free_slot_and_a_full_zone_stops_without_moving(tmp_path):
    s, ev = make(tmp_path)
    slots = WS.zone_slots("right")
    try:
        holding(s)
        assert s.skills.place("right").status == "ok"
        assert len(s.skills.placed) == 1 and math.hypot(s.skills.placed[0][0] - slots[0][0], s.skills.placed[0][1] - slots[0][1]) < 1.0  # 가까운 칸부터
        s.skills.placed = [(x, y, 15.0) for x, y in slots]  # 모든 칸이 찼다
        holding(s)
        before = s.driver.read_joints()
        r = s.skills.place("right")
        after = s.driver.read_joints()
    finally:
        s.disconnect()
    assert r.status == "stopped" and r.reason.startswith("drop_zone_full") and after == before
    assert [lv for lv, c, _ in ev if c == "ARM-DROP-ZONE-FULL"] == ["ERROR"]


@pytest.mark.slow
def test_old_point_zone_stacks_and_warns(tmp_path):
    s, ev = make(tmp_path)
    s.ws = s.skills.ws = ws_with({"x_mm": 420, "y_mm": 150})
    try:
        holding(s)
        s.skills.placed = [(420.0, 150.0, 15.0)]
        assert s.skills.place("z").status == "ok"
    finally:
        s.disconnect()
    assert [lv for lv, c, _ in ev if c == "ARM-PLACE-SAME-SPOT"] == ["WARN"]


@pytest.mark.slow
def test_a_tall_neighbour_near_the_place_point_demands_a_higher_hover(tmp_path):
    s, _ = make(tmp_path)
    try:
        holding(s)
        sk = s.skills
        need = sk._hover_need((300.0, 200.0))
        assert need == 0.0  # 이웃이 없다
        sk.known_objects = [(340.0, 200.0, 20.0)]  # 100mm 안, 높이 모름 -> 40mm 가정: 바닥이 50mm 위여야 한다 -> hover >= 47
        assert sk._hover_need((300.0, 200.0)) == pytest.approx(NEIGHBOUR + 10 - CFG.place_drop_mm)
        sk.known_objects = [(500.0, 200.0, 20.0)]
        assert sk._hover_need((300.0, 200.0)) == 0.0  # 100mm 밖
    finally:
        s.disconnect()


@pytest.mark.slow
def test_place_stops_with_low_hover_when_no_hover_keeps_over_the_neighbour(tmp_path, monkeypatch):
    s, ev = make(tmp_path)
    try:
        holding(s)
        sk = s.skills
        sk.known_objects = [(340.0, 200.0, 20.0)]
        monkeypatch.setattr(sk, "_ik", lambda pose: None)  # 어떤 hover에도 닿지 않는다
        r = sk.place_at(300.0, 200.0)
    finally:
        s.disconnect()
    assert r.status == "stopped" and r.reason.startswith("low_hover")
    assert [lv for lv, c, _ in ev if c == "ARM-PLACE-LOW-HOVER"] == ["ERROR"]


NEIGHBOUR = 40.0
