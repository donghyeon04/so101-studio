"""REQ-023: follow_path(경로 따라가기와 센서 위치 샘플), 팔 링크 점, 도달 불가 거부."""
import math
import threading
import time

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
    yield s
    s.disconnect()


def _tip(svc, q):
    ref = svc.kin.fk(q)
    f = svc.kin.jaw_tip_offsets(ref.pitch_deg, ref.yaw_deg, q.gripper)[0]
    return np.array([ref.x_mm + f[0], ref.y_mm + f[1], ref.z_mm + f[2] - CFG.table_z_mm])


def test_follow_straight_line_samples_monotonic_about_50hz(svc):
    pts = [(260.0, 200.0), (340.0, 200.0)]
    samples = svc.follow_path(pts, Z, 60.0, OFF)
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
    svc.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 60.0, OFF)
    q = svc.driver.read_joints()
    tip = _tip(svc, q)
    x, y, z = svc.sensor_position(q, OFF)
    assert math.hypot(x - tip[0], y - tip[1]) == pytest.approx(30.0, abs=0.5)
    assert z == pytest.approx(tip[2] + 55.0, abs=0.5)
    assert svc.sensor_position(q, {"x": 0, "y": 0, "z": 10})[2] == pytest.approx(tip[2] + 10.0, abs=0.5)


def test_tip_tracks_polyline_within_tolerance(svc):
    pts = [(260.0, 180.0), (320.0, 180.0), (320.0, 240.0)]
    svc.follow_path(pts, Z, 60.0, OFF)
    tip = _tip(svc, svc.driver.read_joints())
    assert np.hypot(tip[0] - 320.0, tip[1] - 240.0) < 3.0 and tip[2] == pytest.approx(Z, abs=3.0)


def test_unreachable_path_is_rejected_before_moving(svc):
    before = svc.driver.read_joints()
    assert svc.follow_path([(260.0, 200.0), (300.0, 900.0)], Z, 60.0, OFF) == []
    assert svc.last_path.status in ("ik_fail", "out_of_bounds")
    assert svc.driver.read_joints() == before  # 접근도 하지 않았다


def test_out_of_workspace_reports_reason(svc):
    svc.follow_path([(260.0, 200.0), (700.0, 200.0)], Z, 60.0, OFF)
    assert svc.last_path.status in ("ik_fail", "out_of_bounds")
    if svc.last_path.status == "out_of_bounds":
        assert svc.last_path.reason


def test_speed_is_capped_by_tool_v_max_and_speed_pct(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    s.connect()
    t0 = time.perf_counter()
    samples = s.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 500.0, OFF)
    cap = CFG.tool_v_max_mm_s * 0.3
    assert samples[-1].t_ns - samples[0].t_ns >= 1e9 * 40.0 / cap * 0.5 and time.perf_counter() - t0 > 0
    s.disconnect()


def test_stop_during_path_returns_samples_so_far(svc):
    out = {}
    pts = [(240.0, 200.0), (360.0, 200.0)]
    th = threading.Thread(target=lambda: out.update(s=svc.follow_path(pts, Z, 40.0, OFF)))
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
    th = threading.Thread(target=lambda: out.update(s=svc.follow_path([(240.0, 200.0), (360.0, 200.0)], Z, 40.0, OFF)))
    th.start()
    deadline = time.time() + 10
    while svc.ctl.tap is None and time.time() < deadline:
        time.sleep(0.01)
    time.sleep(0.2)
    svc.estop()
    th.join(10)
    assert not th.is_alive() and svc.last_path.status != "ok" and svc.ctl.state == "stopped"
    assert svc.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 40.0, OFF) == [] and svc.last_path.status == "stopped"


def test_too_few_points_or_bad_speed_raise(svc):
    with pytest.raises(ValueError):
        svc.follow_path([(260.0, 200.0)], Z, 40.0, OFF)
    with pytest.raises(ValueError):
        svc.follow_path([(260.0, 200.0), (300.0, 200.0)], Z, 0.0, OFF)


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
    svc.follow_path([(300.0, 250.0), (300.0, 270.0)], Z, 60.0, OFF)
    p2 = svc.arm_links()
    d2 = [math.hypot(x - base[0], y - base[1]) for x, y in p2]
    assert d2[3] > 150 and d2[2] > d2[1] > d2[0]  # 손끝과 손목이 멀리 나가고 어깨→팔꿈치→손목 순으로 멀어진다
