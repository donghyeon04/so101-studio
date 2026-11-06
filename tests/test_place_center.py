"""REQ-030: place는 놓는 순간(쥔 그리퍼 값)의 두 턱 끝 중간점이 배출 구역 점에 오도록 손 xy를 보정한다 (결정 0033, pick과 같은 계산)."""
import math
import threading

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    yield s
    s.disconnect()


def test_place_puts_jaw_midpoint_on_drop_zone(svc):
    zone = next(iter(WS.drop_zones))
    z = WS.drop_zones[zone]
    assert svc.skills._run([svc.skills._gripper(15.0)]).status == "ok"  # 물체를 쥔 값(열린 값과 다르게)
    seen = {}
    orig = svc.skills._closing_path

    def spy(q_start, center, *a, **k):  # _opening_path가 넘기는 center = 열기 시작 때의 두 턱 끝 중간점
        seen["center"] = (center.x_mm, center.y_mm)
        return orig(q_start, center, *a, **k)

    svc.skills._closing_path = spy
    assert svc.place(zone).status == "ok"
    assert math.dist(seen["center"], (z["x_mm"], z["y_mm"])) < 1.0
    dx, dy = svc.skills.last_place_correction_mm
    assert math.hypot(dx, dy) > 5.0  # 보정이 실제로 있었다(손끝을 그대로 내리면 중간점은 그만큼 빗나간다)


def test_place_event_logs_correction(svc, tmp_path):
    zone = next(iter(WS.drop_zones))
    svc.skills._run([svc.skills._gripper(15.0)])
    logged = []
    orig = svc._log
    svc._log = lambda kind, data=None, *a, **k: (logged.append((kind, data)), orig(kind, data, *a, **k))[1]
    svc.place(zone)
    place = [d for k, d in logged if k == "place"]
    assert place and place[0]["correction_mm"] is not None and len(place[0]["correction_mm"]) == 2


def test_powertest_sample_thread_survives_read_failure(tmp_path):
    from so101_studio.cli.powertest import run_powertest

    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        def boom():
            raise ConnectionError("read timeout")

        s.driver.read_power = boom
        unhandled, lines = [], []
        old = threading.excepthook
        threading.excepthook = lambda a: unhandled.append(a)
        try:
            run_powertest(s, repeats=1, speed_pct=100, out=lines.append, stop=lambda: True)
        finally:
            threading.excepthook = old
        assert not unhandled
        assert any("부하 표본" in ln for ln in lines)
    finally:
        s.disconnect()


def test_place_at_arbitrary_point_puts_jaw_midpoint_on_it(svc):
    """REQ-031: 배출 구역이 아닌 작업대 점에도 같은 보정을 적용한다."""
    svc.skills._run([svc.skills._gripper(15.0)])
    seen = {}
    orig = svc.skills._closing_path

    def spy(q_start, center, *a, **k):
        seen["center"] = (center.x_mm, center.y_mm)
        return orig(q_start, center, *a, **k)

    svc.skills._closing_path = spy
    target = (360.0, 170.0)
    assert svc.place(at=target).status == "ok"
    assert math.dist(seen["center"], target) < 1.0


def test_place_at_out_of_reach_is_rejected_without_moving(svc):
    before = svc.driver.read_joints()
    r = svc.place(at=(300.0, 560.0))  # 팔 원점 (300, 50)에서 510mm
    assert r.status == "out_of_bounds" and "place_out_of_reach" in r.reason
    assert svc.driver.read_joints() == before


def test_place_needs_exactly_one_of_zone_and_point(svc):
    with pytest.raises(ValueError):
        svc.place()
    with pytest.raises(ValueError):
        svc.place("right", at=(300.0, 200.0))


def test_pick_and_place_rejects_out_of_reach_points_without_moving(svc):
    from so101_studio.core.types import DetectedObject

    before = svc.driver.read_joints()
    ok_obj = DetectedObject("o", 300.0, 200.0, 0.0, 30.0, 100.0, 1.0, "manual")
    far_obj = DetectedObject("f", 300.0, 560.0, 0.0, 30.0, 100.0, 1.0, "manual")
    r = svc.pick_and_place(far_obj, place_at=(350.0, 200.0))
    assert r.status == "out_of_bounds" and r.reason.startswith("pick_out_of_reach")
    r = svc.pick_and_place(ok_obj, place_at=(300.0, 560.0))
    assert r.status == "out_of_bounds" and r.reason.startswith("place_out_of_reach")
    assert svc.driver.read_joints() == before
    with pytest.raises(ValueError):
        svc.pick_and_place(ok_obj, zone="right", place_at=(350.0, 200.0))


def test_place_lifts_straight_up_after_opening(svc):
    """열면서 손이 옆으로 옮겨진 뒤에는 원래 하강 경로를 거꾸로 타지 않고 지금 자리에서 수직으로 올린다 (REQ-033). 연 채로 올린다."""
    zone = next(iter(WS.drop_zones))
    svc.skills._run([svc.skills._gripper(15.0)])
    calls: list[list] = []
    orig_run = svc.skills._run

    def run(trajs):
        calls.append([])
        return orig_run(trajs)

    svc.skills._run = run
    svc.ctl.trace = lambda t, q: calls[-1].append(q) if calls else None
    assert svc.place(zone).status == "ok"
    svc.ctl.trace = None
    opened, lift = calls[-2], calls[-1]  # 마지막 두 번의 실행: 열기, 올리기
    x0, y0, z0 = svc.tool_position(opened[-1])
    for q in lift:
        x, y, _ = svc.tool_position(q)
        assert math.hypot(x - x0, y - y0) < 1.0  # 올리는 동안 손 xy는 열기가 끝난 자리
        assert q.gripper == pytest.approx(opened[-1].gripper, abs=0.5)  # 올라오는 동안 연 채
    assert svc.tool_position(lift[-1])[2] - z0 > CFG.approach_height_mm - 2.0  # 호버 높이까지 올라왔다
