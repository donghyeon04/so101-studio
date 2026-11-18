"""결정 0067(H-2, REQ-079): 새 작업대 값과 배출 통 (arm-software-spec 11.3.5 "통 구역"). config/workspace.yaml 실물 배치로 본다."""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from test_skills import Recorder

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.skills import Skills
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.scene import BIN_WALL_T_MM, load_scene
from so101_studio.core.types import DetectedObject

WS = load_workspace("config/workspace.yaml")
BIN = WS.drop_zones["right"]


def rig(cfg):
    kin = Kinematics(cfg.urdf, WS.base, cfg.joint_limits_deg)
    d = Recorder(cfg.poses["home"])
    ctl = MotionController(d, kin, Safety(cfg.joint_limits_deg, WS, kin, z_floor_mm=cfg.tip_floor_mm, table_z_mm=cfg.table_z_mm), cfg.control_hz)
    return d, ctl, Skills(ctl, kin, cfg, WS, speed_pct=100)


def parallel_cfg(tmp: Path, extra: str = ""):
    (tmp / "local.yaml").write_text("gripper: parallel\n" + extra, encoding="utf-8")
    return load_robot("config/robot.yaml", local_path=tmp / "local.yaml")


def test_new_table_values_and_bin_zone():
    assert WS.table_mm == {"x": 425, "y": 375} and (WS.base.x_mm, WS.base.y_mm, WS.base.yaw_deg) == (215.2, 21.4, 90)
    assert load_robot("config/robot.yaml").table_z_mm == -2.9
    assert BIN == {"type": "bin", "x_mm": 355, "y_mm": 80, "w_mm": 110, "h_mm": 110, "wall_mm": 25}
    assert WS.zone_slots("right") is None  # 통은 칸을 고르지 않는다
    assert math.hypot(BIN["x_mm"] - WS.base.x_mm, BIN["y_mm"] - WS.base.y_mm) == pytest.approx(151.6, abs=0.1)
    (x, y, r), = WS.ignore_zones()  # 레이더 무시 원: 반 대각선 + 벽 + safety_margin_mm
    assert (x, y) == (355, 80) and r == pytest.approx(math.hypot(55, 55) + 25 + WS.safety_margin_mm)


def test_slot_rectangle_drops_cells_off_the_table():
    ws = replace(WS, drop_zones={"r": {"x_mm": 400, "y_mm": 150, "w_mm": 100, "h_mm": 80, "slot_mm": 50}})
    slots = ws.zone_slots("r")
    assert slots and all(0 <= x <= 425 and 0 <= y <= 375 for x, y in slots) and len(slots) < 3 * 2  # x 450 칸은 판 밖


def test_bin_walls_are_collision_parts_of_the_service_scene():
    walls = {p.name: p for p in load_scene(ws=WS).parts if p.name.startswith("bin_right_")}
    assert set(walls) == {"bin_right_s", "bin_right_n", "bin_right_w", "bin_right_e"} and all(p.collide and p.attach == "table" for p in walls.values())
    assert walls["bin_right_e"].T[:3, 3].tolist() == pytest.approx([355 + (110 + BIN_WALL_T_MM) / 2, 80, 0])
    assert walls["bin_right_s"].mesh.extents.tolist() == pytest.approx([110 + 2 * BIN_WALL_T_MM, BIN_WALL_T_MM, 25])
    assert not [p for p in load_scene().parts if p.name.startswith("bin_")]  # ws 없이 읽으면 더하지 않는다(그 쪽 세션이 ws를 넘긴다)


@pytest.mark.slow
def test_pick_and_drop_into_the_bin_opens_above_the_wall_and_leaves_no_placed_object():
    cfg = load_robot("config/robot.yaml")
    d, ctl, sk = rig(cfg)
    try:
        obj = DetectedObject("a", 215.0, 160.0, 0.0, 30.0, 100.0, 1.0, "manual", 40.0)  # config/manual_objects.yaml 첫 물체
        d.grip_block = sk.gripper_for_width(30.0)
        assert sk.pick(obj, sk.place_targets(zone="right")).status == "ok"
        t, n = sk.grip["tip_h"], len(d.log)
        r = sk.place("right")
        assert r.status == "ok", r
        opening = [q for q in d.log[n:] if q.gripper > d.log[n].gripper + 1.0]  # 여는 동안의 자세
        f, m = sk.kin.jaw_tips_at(opening[0])
        low = min(f[2], m[2]) - cfg.table_z_mm
        assert low == pytest.approx(BIN["wall_mm"] + cfg.bin_drop_clear_mm + t, abs=2.0)  # 쥔 물체 바닥이 벽 위 10mm
        mid = (f[:2] + m[:2]) / 2
        assert math.hypot(mid[0] - BIN["x_mm"], mid[1] - BIN["y_mm"]) < 3.0
        assert sk.placed == [] and sk.grip is None and sk.last_place_xy == (355.0, 80.0)
        top = sk.kin.jaw_tips_at(d.log[-1])
        assert min(top[0][2], top[1][2]) - cfg.table_z_mm > low + cfg.approach_height_mm - 15.0  # 연 뒤 수직 상승
    finally:
        ctl.close()


def test_bin_unreachable_is_ik_fail_and_keeps_the_grip():
    _d, ctl, sk = rig(load_robot("config/robot.yaml"))
    try:
        sk.grip = {"tilted": False, "tip_h": 20.0, "obj_h": 40.0, "obj_w": 30.0, "obj_l": 100.0, "held_gap": 30.0, "yaw": 90.0, "angle": 0.0}
        sk._bin_plan = lambda *a: None
        r = sk.place("right")
        assert r.status == "ik_fail" and r.reason.startswith("bin_unreachable") and sk.grip is not None
    finally:
        ctl.close()


# 확인 ③(6.6.1): 통에 β 0·20·35, φ 0·90°로 여는 자세까지 하강이 풀린다(2026-11-18: 모두 풀림, 수직 호버 25mm·기운 자세 40mm)
@pytest.mark.slow
def test_tilted_grips_reach_the_bin(tmp_path):
    cfg = parallel_cfg(tmp_path, "grasp_approach: auto\n")
    d, ctl, sk = rig(cfg)
    try:
        v = sk.gripper_for_width(58.0)
        d._q = replace(d._q, gripper=v)
        for beta in (0.0, 20.0, 35.0):
            for phi in (0.0, 90.0):
                g = {"obj_h": 39.0, "tip_h": 20.0, "tilted": False}
                if beta:
                    g["approach"] = {"beta": beta, "phi": phi, "gamma": 0.0, "face": "width", "pick_az_deg": 90.0}
                plan = sk._bin_plan(BIN["x_mm"], BIN["y_mm"], sk._bin_tip_h(BIN["wall_mm"], g), g, v)
                assert plan is not None, (beta, phi)
    finally:
        ctl.close()
