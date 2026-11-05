"""REQ-023: follow_path(경로 따라가기와 센서 위치 샘플), 팔 링크 점, 도달 불가 거부."""
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


def _tip(svc, q):
    ref = svc.kin.fk(q)
    f = svc.kin.jaw_tip_offsets(ref.pitch_deg, ref.yaw_deg, q.gripper)[0]
    return np.array([ref.x_mm + f[0], ref.y_mm + f[1], ref.z_mm + f[2] - CFG.table_z_mm])


def test_follow_straight_line_samples_monotonic_about_50hz(svc):
    pts = [(260.0, 200.0), (340.0, 200.0)]
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


def test_tip_tracks_polyline_within_tolerance(svc):
    pts = [(260.0, 180.0), (320.0, 180.0), (320.0, 240.0)]
    svc.follow_path(pts, Z, 60.0)
    tip = _tip(svc, svc.driver.read_joints())
    assert np.hypot(tip[0] - 320.0, tip[1] - 240.0) < 3.0 and tip[2] == pytest.approx(Z, abs=3.0)


def _count_executes(svc, start):
    """시작점에 먼저 가 둔 뒤(접근이 한 번의 직선이 되게) execute 호출을 센다."""
    assert svc.move_tool(svc.point_pose(*start, Z)).status == "ok"
    calls = []
    orig = svc.ctl.execute
    svc.ctl.execute = lambda tr: (calls.append(tr), orig(tr))[1]
    return calls


def test_circle_is_one_continuous_run_not_36_stops(svc):
    c = (300.0, 180.0)
    ring = [(c[0] + 45 * math.cos(math.radians(i * 10)), c[1] + 45 * math.sin(math.radians(i * 10))) for i in range(37)]
    calls = _count_executes(svc, ring[0])
    samples = svc.follow_path(ring, Z, 60.0)
    assert svc.last_path.status == "ok" and len(calls) == 2 and len(samples) > 20  # 접근 + 원 한 번
    xy = np.array([s.sensor_mm[:2] for s in samples])
    assert np.ptp(xy[:, 0]) > 80  # 원을 한 바퀴 돈 센서 궤적


def test_only_sharp_corners_stop(svc):
    calls = _count_executes(svc, (260.0, 180.0))
    svc.follow_path([(260.0, 180.0), (320.0, 180.0), (320.0, 240.0)], Z, 60.0)  # 90° 꺾임 하나
    assert len(calls) == 3  # 접근 + 두 구간
    svc.move_tool(svc.point_pose(260.0, 180.0, Z))
    calls.clear()
    svc.follow_path([(260.0, 180.0), (300.0, 180.0), (340.0, 190.0)], Z, 60.0)  # 약 14° 꺾임
    assert len(calls) == 2


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
    assert svc.last_path.status in ("ik_fail", "out_of_bounds")
    assert svc.driver.read_joints() == before  # 접근도 하지 않았다


def test_out_of_workspace_reports_reason(svc):
    svc.follow_path([(260.0, 200.0), (700.0, 200.0)], Z, 60.0)
    assert svc.last_path.status in ("ik_fail", "out_of_bounds")
    if svc.last_path.status == "out_of_bounds":
        assert svc.last_path.reason


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
    th, out = _run_async(lambda: s.follow_path([(240.0, 200.0), (420.0, 200.0)], Z, 40.0))
    assert _wait(lambda: s.ctl.state == "moving")
    s.driver.fail = True  # 접근 이동 중(마지막 틱에서 결과를 읽을 때)에 읽기가 끊긴다
    th.join(8)
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
