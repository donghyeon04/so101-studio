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
