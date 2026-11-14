"""REQ-023: follow_path(경로 따라가기와 센서 위치 샘플), 팔 링크 점, 도달 불가 거부."""
import itertools
import math
import threading
import time
from dataclasses import replace

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
Z = 55.0
OFF = {"x": 0.0, "y": 30.0, "z": 55.0}  # 호출하는 쪽(초음파 설정)이 넘기는 값


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    ok, msg = s.connect()
    assert ok, msg
    s.set_sensor_offset(OFF)
    yield s
    s.disconnect()


def _goto(svc, x, y, yaw=90.0):
    """경로 시작점에 먼저 가 둔다(follow_path의 샘플이 접근을 빼고 경로만 담게)."""
    assert svc.move_tool(svc.point_pose(x, y, Z, yaw)).status == "ok"


def _tip(svc, q):
    ref = svc.kin.fk(q)
    f = svc.kin.jaw_tip_offsets(ref.pitch_deg, ref.yaw_deg, q.gripper)[0]
    return np.array([ref.x_mm + f[0], ref.y_mm + f[1], ref.z_mm + f[2] - CFG.table_z_mm])


def test_follow_straight_line_samples_monotonic_about_50hz(svc):
    pts = [(260.0, 200.0), (340.0, 200.0)]
    _goto(svc, *pts[0])
    samples = svc.follow_path(pts, Z, 60.0)
    assert svc.last_path.status == "ok" and len(samples) > 20
    t = np.array([s.t_ns for s in samples])
    assert (np.diff(t) > 0).all()
    assert 0.015 < np.median(np.diff(t)) / 1e9 < 0.03  # 50Hz 근처
    xy = np.array([s.sensor_mm[:2] for s in samples])
    # 센서는 손끝에서 (오프셋 y 30) 옆: 손끝 선에서 일정 거리만 벗어나고, 첫·끝 샘플은 선 양 끝 부근
    d = np.hypot(*(xy - np.array(pts[0])).T)
    assert d.min() < 50 and np.hypot(*(xy[-1] - np.array(pts[1]))) < 50
    assert all(s.sensor_mm[2] == pytest.approx(Z + OFF["z"], abs=3.0) for s in samples)


def test_sensor_is_offset_from_tip_by_config_value(svc):
    svc.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 60.0)
    q = svc.driver.read_joints()
    tip = _tip(svc, q)
    x, y, z = svc.sensor_position(q)
    assert math.hypot(x - tip[0], y - tip[1]) == pytest.approx(30.0, abs=0.5)
    assert z == pytest.approx(tip[2] + 55.0, abs=0.5)
    svc.set_sensor_offset({"x": 0, "y": 0, "z": 10})
    assert svc.sensor_position(q)[2] == pytest.approx(tip[2] + 10.0, abs=0.5)


def test_parallel_sensor_position_does_not_move_with_the_jaw_opening(tmp_path):
    """병렬은 센서 기준이 두 턱 끝 가운데라 그리퍼 값 0과 100에서 같다(예전엔 한쪽 턱 끝 기준이라 벌림/2만큼 어긋났다). 손끝(tool_position)도 가운데."""
    from dataclasses import replace
    from pathlib import Path

    local = Path(tmp_path) / "local.yaml"
    local.write_text("gripper: parallel\n", encoding="utf-8")
    s = ArmService(SimDriver(CFG.poses["home"]), load_robot("config/robot.yaml", local_path=local), WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    s.set_sensor_offset(OFF)
    try:
        q = s.driver.read_joints()
        closed, opened = replace(q, gripper=0.0), replace(q, gripper=100.0)
        assert np.allclose(s.sensor_position(closed), s.sensor_position(opened), atol=0.5)
        assert np.allclose(s.tool_position(closed), s.tool_position(opened), atol=0.5)  # 손끝도 가운데 기준(CHECK 경로 기준점 REQ)
        # follow_path·좌표 이동의 점도 같은 기준: 벌린 채 보낸 점에 tool_position이 온다
        pose = s.point_pose(300.0, 200.0, 55.0)
        assert s.tool_position(replace(s.skills._ik(pose), gripper=q.gripper))[:2] == pytest.approx((300.0, 200.0), abs=0.5)
        # REQ-064: 벌린 채 follow_path로 준 선 위에 PoseSample.tool_mm(두 턱 가운데)이 온다
        assert s.set_gripper(100.0).status == "ok"
        samples = s.follow_path([(300.0, 180.0), (300.0, 240.0)], Z, 60.0)  # 병렬 손은 Z 55에서 (260,200)→(320,200)이 안 닿는다
        assert s.last_path.status == "ok" and samples
        on_line = [p for p in samples if 185.0 < p.tool_mm[1] < 235.0 and abs(p.tool_mm[2] - Z) < 3.0]  # 접근 구간 샘플은 뺀다
        assert on_line and all(abs(p.tool_mm[0] - 300.0) < 2.0 for p in on_line)
        assert samples[-1].tool_mm[1] == pytest.approx(240.0, abs=2.0)
        assert samples[-1].tool_mm == pytest.approx(s.tool_position(s.driver.read_joints()), abs=1.0)
    finally:
        s.disconnect()


def test_tip_tracks_polyline_within_tolerance(svc):
    pts = [(260.0, 180.0), (320.0, 180.0), (320.0, 240.0)]
    svc.follow_path(pts, Z, 60.0)
    tip = _tip(svc, svc.driver.read_joints())
    assert np.hypot(tip[0] - 320.0, tip[1] - 240.0) < 3.0 and tip[2] == pytest.approx(Z, abs=3.0)


def _count_executes(svc, start):
    """시작점에 먼저 가 둔 뒤(접근이 길이 0이 되게) execute 호출을 센다."""
    _goto(svc, *start)
    calls = []
    orig = svc.ctl.execute
    svc.ctl.execute = lambda tr: (calls.append(tr), orig(tr))[1]
    return calls


def _ring(c=(300.0, 180.0), r=45.0, n=37):
    return [(c[0] + r * math.cos(math.radians(i * 360 / (n - 1))), c[1] + r * math.sin(math.radians(i * 360 / (n - 1)))) for i in range(n)]


def test_circle_is_one_continuous_run_not_36_stops(svc):
    ring = _ring()
    calls = _count_executes(svc, ring[0])
    samples = svc.follow_path(ring, Z, 60.0)
    assert svc.last_path.status == "ok" and len(calls) == 1 and len(samples) > 20  # 원이 한 번의 동작
    xy = np.array([s.sensor_mm[:2] for s in samples])
    assert np.ptp(xy[:, 0]) > 80  # 원을 한 바퀴 돈 센서 궤적


def test_any_corner_is_one_call_and_one_motion(svc):
    calls = _count_executes(svc, (260.0, 180.0))
    svc.follow_path([(260.0, 180.0), (320.0, 180.0), (320.0, 240.0)], Z, 60.0)  # 90° 꺾임
    assert len(calls) == 1
    svc.follow_path([(320.0, 240.0), (280.0, 240.0), (320.0, 240.0)], Z, 60.0)  # 180° 되돌아감도 한 번의 동작(꺾이는 점에서만 선다)
    assert len(calls) == 2


def _limits(svc, pts, speed=100.0):
    """경로를 계획만 하고(움직이지 않음) 제어 틱(50Hz)마다 궤적을 샘플해 실제 손끝 위치의 속도와 2차 차분 가속(벡터 크기)을 잰다. 한 주기 이동량 한도(check_step)도 확인한다."""
    v = min(speed, CFG.tool_v_max_mm_s * svc.speed / 100)
    _pre, runs, fail, why = svc._plan_path(pts, Z, v, None)
    assert fail is None, (fail, why)
    tr = runs[-1]
    ts = np.arange(0.0, tr.duration_s + 0.02, 0.02)
    qs = [tr.sample(t) for t in ts]
    assert all(svc.safety.check_step(a, b) is None for a, b in itertools.pairwise(qs))
    xyz = np.array([svc.tool_position(q) for q in qs])
    vel = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / 0.02
    acc = np.linalg.norm(np.diff(xyz, 2, axis=0), axis=1) / 0.02**2
    return {"vmax": float(vel.max()), "amax": float(acc.max()), "vel": vel, "acc": acc, "shape": tr.shape, "dur": tr.duration_s}


def test_circle_actual_acceleration_stays_within_tool_a_max(svc):
    _goto(svc, *_ring()[0])
    m = _limits(svc, _ring())
    assert m["vmax"] <= CFG.tool_v_max_mm_s * 1.001
    assert m["amax"] <= CFG.tool_a_max_mm_s2  # 50Hz로 샘플한 실제 손끝 2차 차분(관측 최대 약 116, 허용폭 없이 한도 120 이내)
    assert m["vel"][30:-30].min() > 5.0  # 도중에 서지 않는다(처음과 끝만 선다)


def test_90_degree_corner_slows_but_does_not_stop_and_respects_limits(svc):
    pts = [(240.0, 180.0), (330.0, 180.0), (330.0, 270.0)]
    _goto(svc, *pts[0])
    m = _limits(svc, pts)
    assert m["vmax"] <= CFG.tool_v_max_mm_s * 1.001 and m["amax"] <= CFG.tool_a_max_mm_s2
    (vc,) = m["shape"].corner_speeds
    assert 5.0 < vc < 40.0  # 꺾임에서 줄지만 서지 않는다(모서리를 2.5mm 안으로 깎는 반지름 약 6mm 호: v = sqrt(a/√2·R) ≈ 22mm/s)
    assert m["vel"][len(m["vel"]) // 4: 3 * len(m["vel"]) // 4].min() > 3.0


def test_135_degree_corner_and_l3_l4_shape_respect_limits(svc):
    c = (300.0, 210.0)
    for pts in ([(c[0] - 90, c[1]), (c[0] + 90, c[1]), (c[0], c[1] - 90)],  # 135° 꺾임
                [(c[0] - 90, c[1]), (c[0] + 90, c[1]), (c[0], c[1] - 90), (c[0], c[1] + 90)]):  # L3 + 건너가기 + L4 (한 호출)
        _goto(svc, *pts[0])
        m = _limits(svc, pts)
        assert m["amax"] <= CFG.tool_a_max_mm_s2 and m["vmax"] <= CFG.tool_v_max_mm_s * 1.001
        assert all(v > 0 for v in m["shape"].corner_speeds)  # 어느 꺾임에서도 서지 않는다


def test_reversal_stops_at_the_turn_within_limits(svc):
    pts = [(240.0, 180.0), (320.0, 180.0), (240.0, 180.0)]
    _goto(svc, *pts[0])
    m = _limits(svc, pts)
    assert m["shape"].corner_speeds == [0.0]  # 되돌아가는 꺾임은 서서 돈다
    assert m["amax"] <= CFG.tool_a_max_mm_s2


def test_fillet_geometry_is_a_real_arc_within_the_corner_tolerance():
    from so101_studio.arm.trajectory import fillet_path

    P = [np.array(p, float) for p in ((0, 0, 0), (90, 0, 0), (90, 90, 0))]
    pts, _s, arcs, _vs, cusps = fillet_path(P, CFG.tool_a_max_mm_s2 / math.sqrt(2), 1.0)
    assert cusps == [] and len(arcs) == 1
    assert np.linalg.norm(pts[-1] - P[-1]) < 1e-9 and np.linalg.norm(pts[0] - P[0]) < 1e-9
    near = np.linalg.norm(pts - P[1], axis=1).min()
    assert 1.0 < near <= 2.5 + 1e-6  # 꼭짓점에서 최대 2.5mm 안쪽으로 깎인 호
    assert np.max(np.linalg.norm(np.diff(pts, axis=0), axis=1)) <= 1.0 + 1e-9  # 경유점 간격 1mm 이하


def test_unreachable_first_point_falls_back_to_safe_approach(svc, monkeypatch):
    import so101_studio.service.arm_service as mod

    real = mod.smooth_l
    n = {"c": 0}

    def first_fails(*a, **k):  # 지금 위치에서 첫 점까지 직선으로 못 가는 경우를 흉내: 합친 궤적 계획만 실패
        n["c"] += 1
        return None if n["c"] == 1 else real(*a, **k)

    monkeypatch.setattr(mod, "smooth_l", first_fails)
    pre, runs, fail, _why = svc._plan_path([(260.0, 180.0), (320.0, 180.0)], Z, 30.0, None)
    assert fail is None and len(pre) >= 1 and len(runs) == 1  # plan_goto의 안전 경로가 접근으로 남는다


@pytest.mark.slow
def test_samples_use_actual_joint_reads_not_commands(svc):
    class Lagging(SimDriver):  # 읽는 값이 명령보다 늘 뒤처지는 장치
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.hist = []

        def write_joints(self, q):
            self.hist.append(q)
            super().write_joints(q)

        def read_joints(self):
            return self.hist[-6] if len(self.hist) > 6 else super().read_joints()

    s = ArmService(Lagging(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    s.connect()
    s.ctl.blocked_cfg = None  # 일부러 뒤처진 읽기: 막힘 감지(결정 0059)는 끈다
    s.set_sensor_offset(OFF)
    pts = [(240.0, 200.0), (340.0, 200.0)]
    smp = s.follow_path(pts, Z, 60.0)
    cmd = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    cmd.connect()
    cmd.set_sensor_offset(OFF)
    ref = cmd.follow_path(pts, Z, 60.0)
    assert smp and ref
    # 뒤처진 읽기라 같은 시각 번호의 센서 x는 명령 기준보다 뒤(작아야 한다)
    k = len(smp) // 2
    assert smp[k].sensor_mm[0] < ref[min(k, len(ref) - 1)].sensor_mm[0] - 1.0
    s.disconnect()
    cmd.disconnect()


def test_unreachable_path_is_rejected_before_moving(svc):
    before = svc.driver.read_joints()
    assert svc.follow_path([(260.0, 200.0), (300.0, 900.0)], Z, 60.0) == []
    assert svc.last_path.status == "ik_fail"
    assert svc.driver.read_joints() == before  # 접근도 하지 않았다


def test_out_of_workspace_reports_reason(svc):
    svc.follow_path([(260.0, 200.0), (700.0, 200.0)], Z, 60.0)
    assert svc.last_path.status == "ik_fail"  # 팔이 닿지 않는 점은 도달 불가로 거부(작업대 검사까지 가지 않는다)


@pytest.mark.slow
def test_speed_is_capped_by_tool_v_max_and_speed_pct(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    s.connect()
    s.set_sensor_offset(OFF)
    t0 = time.perf_counter()
    samples = s.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 500.0)
    cap = CFG.tool_v_max_mm_s * 0.3
    assert samples[-1].t_ns - samples[0].t_ns >= 1e9 * 40.0 / cap * 0.5 and time.perf_counter() - t0 > 0
    s.disconnect()


def test_stop_during_path_returns_samples_so_far(svc):
    out = {}
    pts = [(240.0, 200.0), (360.0, 200.0)]
    th = threading.Thread(target=lambda: out.update(s=svc.follow_path(pts, Z, 40.0)))
    th.start()
    deadline = time.time() + 10
    while svc.ctl.tap is None and time.time() < deadline:
        time.sleep(0.01)
    time.sleep(0.3)
    svc.stop()
    th.join(10)
    assert not th.is_alive() and svc.last_path.status != "ok"
    assert 0 < len(out["s"]) and svc.ctl.tap is None


def test_estop_during_path_ends_it_and_keeps_stopped(svc):
    out = {}
    th = threading.Thread(target=lambda: out.update(s=svc.follow_path([(240.0, 200.0), (360.0, 200.0)], Z, 40.0)))
    th.start()
    deadline = time.time() + 10
    while svc.ctl.tap is None and time.time() < deadline:
        time.sleep(0.01)
    time.sleep(0.2)
    svc.estop()
    th.join(10)
    assert not th.is_alive() and svc.last_path.status != "ok" and svc.ctl.state == "stopped"
    assert svc.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 40.0) == [] and svc.last_path.status == "stopped"


def test_too_few_points_or_bad_speed_raise(svc):
    with pytest.raises(ValueError):
        svc.follow_path([(260.0, 200.0)], Z, 40.0)
    with pytest.raises(ValueError):
        svc.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 0.0)


def test_arm_links_are_shoulder_elbow_wrist_tip_in_table_xy(svc):
    q = svc.driver.read_joints()
    pts = svc.arm_links(q)
    assert len(pts) == 4
    base = (WS.base.x_mm, WS.base.y_mm)
    d = [math.hypot(x - base[0], y - base[1]) for x, y in pts]
    assert d[0] < 40  # 어깨는 팔 원점 근처
    assert pts[3] == pytest.approx((svc.status().tool.x_mm + (_tip(svc, q)[0] - svc.status().tool.x_mm),
                                    svc.status().tool.y_mm + (_tip(svc, q)[1] - svc.status().tool.y_mm)))
    # 팔을 뻗은 자세에서는 손끝이 어깨에서 가장 멀다
    svc.follow_path([(300.0, 250.0), (300.0, 270.0)], Z, 60.0)
    p2 = svc.arm_links()
    d2 = [math.hypot(x - base[0], y - base[1]) for x, y in p2]
    assert d2[3] > 150 and d2[2] > d2[1] > d2[0]  # 손끝과 손목이 멀리 나가고 어깨→팔꿈치→손목 순으로 멀어진다


def test_samples_carry_tool_and_sensor_and_yaw_is_held(svc):
    _goto(svc, 260.0, 200.0, yaw=0.0)
    smp = svc.follow_path([(260.0, 200.0), (340.0, 200.0)], Z, 60.0, yaw_deg=0.0)
    assert svc.last_path.status == "ok" and svc.status().tool.yaw_deg == pytest.approx(0.0, abs=1.0)
    for s in smp[::10]:
        assert s.tool_mm[2] == pytest.approx(Z, abs=3.0)
        assert math.hypot(s.sensor_mm[0] - s.tool_mm[0], s.sensor_mm[1] - s.tool_mm[1]) == pytest.approx(30.0, abs=0.5)
    first = smp[0]  # yaw 0°면 센서 방향이 yaw 90°일 때와 90° 다르다
    assert (first.sensor_mm[0] - first.tool_mm[0], first.sensor_mm[1] - first.tool_mm[1]) != pytest.approx((-30.0, 0.0), abs=1.0)


def test_follow_path_requires_sensor_offset(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    s.connect()
    with pytest.raises(ValueError):
        s.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 60.0)
    s.disconnect()


class ReadFails(SimDriver):
    """write는 되는데 read만 실패하는 장치(서보 하나가 빠지거나 버스 잡음 때 sync_read만 응답이 없다)."""
    fail = False

    def read_joints(self):
        if self.fail:
            raise ConnectionError("read timeout")
        return super().read_joints()


def _flaky_service(tmp_path):
    s = ArmService(ReadFails(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    assert s.connect()[0]
    s.set_sensor_offset(OFF)
    return s


def _run_async(fn):
    out = {}
    th = threading.Thread(target=lambda: out.update(r=fn()), daemon=True)
    th.start()
    return th, out


def _wait(cond, limit=10.0):
    end = time.time() + limit
    while not cond() and time.time() < end:
        time.sleep(0.01)
    return cond()


def test_read_failure_during_path_ends_with_comm_lost_fault(tmp_path):
    s = _flaky_service(tmp_path)
    th, _out = _run_async(lambda: s.follow_path([(240.0, 200.0), (420.0, 200.0)], Z, 40.0))
    assert _wait(lambda: s.ctl.tap is not None)
    time.sleep(0.2)
    s.driver.fail = True
    th.join(8)
    assert not th.is_alive(), "읽기가 실패해도 follow_path는 돌아와야 한다"
    assert s.last_path.status == "fault" and s.ctl.state == "fault" and s.ctl.fault == "comm_lost"
    s.driver.fail = False
    s.disconnect()


def test_read_failure_during_approach_does_not_hang(tmp_path):
    s = _flaky_service(tmp_path)
    s.set_speed(30)  # 접근이 길어져(약 10초) 전체 시험 부하로 읽기를 끊는 시점이 늦어도 접근이 먼저 끝나지 않는다
    th, out = _run_async(lambda: s.follow_path([(240.0, 200.0), (420.0, 200.0)], Z, 40.0))
    assert _wait(lambda: s.ctl.state == "moving", 30.0)
    s.driver.fail = True  # 접근 이동 중(마지막 틱에서 결과를 읽을 때)에 읽기가 끊긴다
    th.join(30)  # comm_lost는 0.5초 안에 나지만 부하로 스레드가 늦게 돌아도 기다린다
    assert not th.is_alive() and out["r"] == [] and s.last_path.status == "fault"
    s.driver.fail = False
    s.disconnect()


def test_read_failure_at_the_last_tick_still_finishes_the_handle(tmp_path):
    s = _flaky_service(tmp_path)
    th, out = _run_async(lambda: s.move_joints(replace(s.driver.read_joints(), shoulder_pan=20.0), 100))
    assert _wait(lambda: s.ctl.state == "moving")
    s.driver.fail = True
    th.join(8)
    assert not th.is_alive() and out["r"].status == "fault"
    s.driver.fail = False
    s.disconnect()


@pytest.mark.slow
def test_follow_path_reads_the_bus_a_constant_number_of_times_for_planning(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    s.connect()
    s.set_sensor_offset(OFF)
    n = {"c": 0}
    orig = s.driver.read_joints
    s.driver.read_joints = lambda: (n.__setitem__("c", n["c"] + 1), orig())[1]
    c = (300.0, 180.0)
    ring = [(c[0] + 45 * math.cos(math.radians(i * 10)), c[1] + 45 * math.sin(math.radians(i * 10))) for i in range(37)]
    s.ctl.tap = None
    s.follow_path(ring, Z, 100.0)
    per_tick = len(s.ctl.ring)  # 틱마다 한 번(tap)은 정상이므로 계획 읽기만 센다: 점마다 읽으면 37번이 더 붙는다
    assert n["c"] < per_tick + 30
    s.disconnect()


def test_locked_arm_gets_no_torque_and_returns_locked(svc):
    svc.ctl.locked_reason = "캘리브레이션 불일치(시험)"
    svc.driver._torque = False
    assert svc.follow_path([(260.0, 200.0), (320.0, 200.0)], Z, 60.0) == []
    assert svc.last_path.status == "locked"
    assert svc.driver.torque_enabled() is False  # 잠긴 팔에 토크를 켜지 않았다


def test_identical_points_are_an_argument_error(svc):
    with pytest.raises(ValueError):
        svc.follow_path([(260.0, 200.0), (260.0, 200.0), (260.0, 200.0)], Z, 60.0)


@pytest.mark.slow  # 약 20초
def test_follow_path_starts_from_radar_clear_pose(svc):
    """결정 0039의 대기 자세(손목을 접어 세운 자세)에서 출발해도 첫 점으로 가서 경로를 따라간다."""
    assert svc.skills.radar_clear().status == "ok"
    samples = svc.follow_path([(300.0, 200.0), (380.0, 200.0)], Z, 100.0, 90.0)
    assert svc.last_path.status == "ok" and len(samples) > 20


def test_plan_goto_tries_several_ik_seeds_from_radar_clear(svc):
    """현재 관절(손목을 접은 radar_clear)만 씨앗으로 쓰면 호버 IK가 실패하는 곳도 대표 자세 씨앗으로 푼다 (REQ-030)."""
    from dataclasses import replace

    from so101_studio.service.arm_service import plan_goto

    assert svc.skills.radar_clear().status == "ok"
    q0 = svc.driver.read_joints()
    goal = svc.point_pose(161.0, 58.0, Z, 90.0)
    hover = replace(goal, z_mm=goal.z_mm + CFG.approach_height_mm)
    assert svc.kin.ik(hover, q0) is None  # 현재 관절만으로는 못 푼다(이 시험의 전제)
    assert plan_goto(svc.kin, CFG, q0, goal, 100) is not None
