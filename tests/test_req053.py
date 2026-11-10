"""REQ-053 (arm-software-spec 11.3.3~11.3.4): 놓을 때도 두 턱이 수직에서 같은 각도로 벌어지는 대칭 V자. --sim 부분."""
import itertools
import math
from types import SimpleNamespace

import numpy as np
import pytest

from so101_studio.arm.kinematics import VALUE_TO_Q
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)
ZONE = (WS.drop_zones["right"]["x_mm"], WS.drop_zones["right"]["y_mm"])
POINT = (300.0, 200.0)


def run(tmp_path, width, target, angle=0.0, height=40.0, known=(), forget=False):
    """폭 width·높이 height 물체를 (300, 150)에서 집어 target에 놓는다. angle 0은 턱 축이 방사 방향(기울여 집는다), 60은 기울이지 못해 수직으로 집는다.
    forget이면 집기 기억을 지우고 놓는다. 돌려주는 값: status, path(ARM-OPENING-PATH), fallback, legacy, release(내려간 자세), grip(그때 집기 기억), v(그때 그리퍼 값), trajs, route, sk."""
    block = float(np.interp(width, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev, seen, sk = [], {}, s.skills
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    sk.known_objects = list(known)
    descent, scan = sk._descent, sk._scan_opening

    def spy_descent(release, *args):
        seen.update(release=release, grip=dict(sk.grip) if sk.grip else None, v=sk._q().gripper)
        return descent(release, *args)

    def spy_scan(trajs, route, env):
        out = scan(trajs, route, env)
        if out[0] is None:
            seen.update(trajs=trajs, route=route)
        return out

    sk._descent, sk._scan_opening = spy_descent, spy_scan
    try:
        obj = DetectedObject("o", 300.0, 150.0, angle, width, width, 1.0, "manual", height)
        assert sk.pick(obj).status == "ok"
        if forget:
            sk.grip = None
        r = sk.place_at(*target) if target != ZONE else sk.place("right")
        after = (sk.grip, sk._place_ctx)
    finally:
        s.disconnect()
    of = lambda code: [d for c, d in ev if c == code]
    return SimpleNamespace(status=r.status, path=of("ARM-OPENING-PATH"), fallback=of("ARM-OPENING-PATH-FALLBACK"), legacy=of("ARM-PLACE-LEGACY"), sk=sk, after=after, **seen)


def sym_error(sk, pt) -> float:
    """경로 한 점의 대칭 오차 |2τ - θ| (11.3.2)."""
    return abs(2.0 * (-90.0 - sk.kin.fk(JointState(*pt)).pitch_deg) - VALUE_TO_Q[1] * pt[-1])


def check_symmetric_open(d):
    assert d["sym_err_max_deg"] <= 3.0 and d["actual_sym_err_end_deg"] <= 3.0
    assert d["jaw_angle_fixed_deg"][1] >= 15.0 and d["jaw_angle_moving_deg"][1] >= 15.0  # 끝에서 두 턱이 각각 수직에서 15° 이상
    assert d["actual_mid_move_mm"] <= 3.0  # 두 턱 가운데(물체 자리)는 수평으로 제자리


@pytest.mark.slow
@pytest.mark.parametrize("width", [30, 45, 58])
def test_tilted_grip_opens_as_a_symmetric_v_at_a_point(tmp_path, width):
    r = run(tmp_path, width, POINT)
    assert r.status == "ok" and len(r.path) == 1 and not r.fallback and not r.legacy
    d = r.path[0]
    assert d["route"] == "tilted" and d["skipped"] == [] and d["yaw_deg"] == pytest.approx(90.0, abs=0.5)  # 놓는 점에서 팔이 뻗은 방향
    check_symmetric_open(d)
    assert d["gap_end_mm"] >= d["gap_start_mm"] + 2 * 10 - 1.0 and 10.0 <= d["clearance_mm"] <= CFG.place_clearance_mm
    f, m = r.sk.kin.jaw_tip_offsets(r.release.pitch_deg, r.release.yaw_deg, r.v)  # 물체 바닥 높이 = 내려간 낮은 턱 끝 - 작업 면 - 집을 때 높이 t_g: 0~5mm
    bottom = r.release.z_mm + float(min(f[2], m[2])) - CFG.table_z_mm - r.grip["tip_h"]
    assert 0.0 <= bottom <= 5.0 and bottom == pytest.approx(CFG.place_drop_mm, abs=0.5)
    assert max(sym_error(r.sk, pt) for pt in r.trajs[0].pts) <= 3.0  # 벌리는 동안 모든 단계가 대칭


@pytest.mark.slow
def test_tilted_grip_opens_symmetrically_at_the_drop_zone_for_a_narrow_object(tmp_path):
    r = run(tmp_path, 30, ZONE)
    assert r.status == "ok" and len(r.path) == 1 and not r.fallback and not r.legacy
    assert r.path[0]["route"] == "tilted" and r.path[0]["yaw_deg"] == pytest.approx(26.6, abs=0.5)
    check_symmetric_open(r.path[0])
    assert r.path[0]["hover_mm"] <= CFG.approach_height_mm  # 호버는 approach_height_mm 이하(REQ-054로 손목 한계를 넓힌 뒤 배출 구역도 전부 올라간다)


@pytest.mark.slow
@pytest.mark.parametrize("width", [45, 58])
def test_wide_objects_at_the_drop_zone_fall_back_to_fixed_tilt_because_the_wrist_cannot_reach(tmp_path, width):
    # 배출 구역(반경 224mm)에서 기울인 채 65mm 넘게 벌리는 자세는 wrist_flex 한계(±101°)에 막힌다: 대칭 경로가 안 풀리면 ③으로 간다
    r = run(tmp_path, width, ZONE)
    assert r.status == "ok" and len(r.path) == 1 and not r.fallback
    assert r.path[0]["route"] == "fixed_tilt" and r.path[0]["skipped"][0]["route"] == "tilted"  # 45mm는 경로가 안 풀리고(ik_failed_along_opening_path), 58mm는 내려가는 자세부터 안 풀려 옛 방식(legacy_pose)


@pytest.mark.slow
def test_vertical_grip_releases_then_tilts_with_shrinking_symmetry_error(tmp_path):
    # 기울이지 못하는 방향으로 집은(수직) 높이를 아는 낮은 물체: A로 수직 놓기 6mm, B로 벌리며 기울여 끝에서 대칭
    r = run(tmp_path, 30, POINT, angle=60.0, height=25.0)
    assert r.status == "ok" and len(r.path) == 1 and not r.fallback and not r.legacy
    d = r.path[0]
    assert d["route"] == "released_then_tilted" and d["skipped"] == [{"route": "tilted", "reason": "not_tilted_grip"}]
    assert d["actual_sym_err_end_deg"] <= 3.0 and d["jaw_angle_fixed_deg"][1] >= 15.0 and d["jaw_angle_moving_deg"][1] >= 15.0 and d["actual_mid_move_mm"] <= 3.0
    a, b = r.trajs
    errs = [sym_error(r.sk, pt) for pt in b.pts]
    assert all(x >= y - 0.3 for x, y in itertools.pairwise(errs)) and errs[-1] <= 3.0  # B 구간에서 대칭 오차가 줄어들기만 한다
    assert r.sk._gap_mm(a.pts[-1][-1]) == pytest.approx(r.sk._gap_mm(a.pts[0][-1]) + CFG.place_release_mm, abs=0.7)


@pytest.mark.slow
def test_tall_vertical_grip_is_not_tilted_when_the_tilt_would_hit_the_object(tmp_path):
    # 높이 40mm 물체는 위로 좁아지는 가동 턱이 B 첫 단계에서 상자에 닿는다(실측 기하): ② 대신 ③으로 가고 이유가 남는다
    r = run(tmp_path, 45, POINT, angle=60.0, height=40.0)
    assert r.status == "ok" and r.path[0]["route"] == "fixed_tilt"
    assert {"route": "released_then_tilted", "reason": "tilt_hits_object"} in r.path[0]["skipped"]


@pytest.mark.slow
def test_without_a_grip_memory_it_places_the_old_way_and_says_why(tmp_path):
    r = run(tmp_path, 30, POINT, forget=True)
    assert r.status == "ok" and r.legacy == [{"reason": "no_grip_memory", "grip_tilted": False}]
    assert r.path[0]["route"] == "fixed_tilt" and {"route": "tilted", "reason": "legacy_pose"} in r.path[0]["skipped"]


@pytest.mark.slow
def test_two_known_neighbours_70mm_along_the_jaw_axis_do_not_block_the_third_placement(tmp_path):
    ux, uy = math.cos(math.radians(26.565)), math.sin(math.radians(26.565))  # 배출 구역에서 팔이 뻗은 방향 = 턱 축
    neighbours = [(ZONE[0] + s * 70.0 * ux, ZONE[1] + s * 70.0 * uy, 29.0) for s in (-1, 1)]  # 폭 58mm
    r = run(tmp_path, 30, ZONE, known=neighbours)
    assert r.status == "ok" and len(r.path) == 1 and not r.fallback
    assert r.path[0]["route"] == "tilted" and r.path[0]["clearance_mm"] >= 10.0


@pytest.mark.slow
def test_grip_memory_is_forgotten_after_the_place(tmp_path):
    r = run(tmp_path, 30, POINT)
    assert r.status == "ok" and r.after == (None, None)


@pytest.mark.slow
def test_pick_and_place_through_the_service_keeps_the_grip_memory(tmp_path):
    block = float(np.interp(30, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((code, data))
    try:
        assert s.pick_and_place(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0), place_at=POINT).status == "ok"
    finally:
        s.disconnect()
    assert not [d for c, d in ev if c == "ARM-PLACE-LEGACY"] and [d["route"] for c, d in ev if c == "ARM-OPENING-PATH"] == ["tilted"]


@pytest.mark.slow
def test_a_place_that_fails_before_the_opening_keeps_the_grip_memory(tmp_path):
    block = float(np.interp(30, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    sk = s.skills
    try:
        assert sk.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        run_ = sk._run
        sk._run = lambda trajs: SimpleNamespace(status="stopped", duration_s=0.0, final_tool=None)  # 하강 중 정지
        assert sk.place_at(*POINT).status == "stopped"
        sk._run = run_
        assert sk.grip is not None  # 물체를 아직 쥐고 있으니 다시 놓을 때 새 방식으로 간다
        assert sk.place_at(*POINT).status == "ok" and sk.grip is None
    finally:
        s.disconnect()
