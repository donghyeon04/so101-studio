"""E-STOP 감시 프로세스와 정지 수신 스레드 (REQ-011, decisions/0026)."""
import dataclasses
import json
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.service.arm_service import ArmService
from so101_studio.service.stop_listener import StopListener
from so101_studio.watchdog.core import Watchdog
from so101_studio.watchdog.hotkey import vk
from so101_studio.watchdog.server import make_server

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def recorded(wd, acked=True):
    """실제 UDP·프로세스 종료 대신 호출만 기록한다."""
    calls = []
    wd._send_estop = lambda port, reason: calls.append(("estop", port, reason)) or acked
    wd._kill = lambda pid: calls.append(("kill", pid)) or True
    return calls


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ---- 단위: 가짜 시계로 작동 판정
def test_trips_only_after_one_second_without_heartbeat_when_armed():
    clk = Clock()
    wd = Watchdog(clock=clk)
    calls = recorded(wd)
    clk.t += 5
    assert not wd.tick() and not calls  # armed가 아니면 오래 끊겨도 작동하지 않는다
    wd.arm(True, pid=4242, stop_port=8091)
    clk.t += 0.9
    assert not wd.tick()
    wd.heartbeat("browser")
    wd.heartbeat("server")
    clk.t += 0.9
    assert not wd.tick()  # 둘 다 0.9초 전에 왔다
    clk.t += 0.2  # 1.1초 경과
    assert wd.tick() and calls == [("estop", 8091, "heartbeat 끊김: browser,server")]  # ack가 오면 프로세스는 끝내지 않는다
    assert wd.status()["tripped"] and not wd.tick()  # 한 번만 작동하고 상태를 유지한다


@pytest.mark.parametrize("alive", ["browser", "server"])
def test_either_missing_heartbeat_trips(alive):
    clk = Clock()
    wd = Watchdog(clock=clk)
    calls = recorded(wd)
    wd.arm(True, pid=4242, stop_port=8091)
    for _ in range(3):
        clk.t += 0.5
        wd.heartbeat(alive)
        wd.tick()
    other = "server" if alive == "browser" else "browser"
    assert calls and other in calls[0][2] and alive not in calls[0][2]


def test_no_ack_kills_the_process_and_rearm_resets():
    clk = Clock()
    wd = Watchdog(clock=clk)
    calls = recorded(wd, acked=False)
    wd.arm(True, pid=4242, stop_port=8091)
    clk.t += 1.1
    assert wd.tick()
    assert [c[0] for c in calls] == ["estop", "kill"] and calls[1] == ("kill", 4242)
    wd.arm(True, pid=4242, stop_port=8091)  # 다시 등록하면 작동 상태가 풀린다
    assert not wd.status()["tripped"] and not wd.tick()


def test_sim_and_disarmed_only_log():
    clk, logs = Clock(), []
    wd = Watchdog(sim=True, clock=clk, log=logs.append)
    calls = recorded(wd)
    wd.arm(True, pid=4242, stop_port=8091)
    clk.t += 3
    assert not wd.tick() and not wd.trip("단축키 F9") and not calls and not wd.status()["tripped"]
    assert any(r["event"] == "trip_ignored" for r in logs)
    wd2 = Watchdog(clock=clk)
    calls2 = recorded(wd2)
    assert not wd2.trip("단축키 F9") and not calls2  # armed가 아니면 단축키도 기록만


def test_arm_validation_and_hotkey_names():
    wd = Watchdog()
    with pytest.raises(ValueError):
        wd.arm(True)
    with pytest.raises(ValueError):
        wd.heartbeat("nobody")
    assert vk("F9") == 0x78 and vk("f1") == 0x70 and vk("q") == ord("Q")
    with pytest.raises(ValueError):
        vk("F13")


# ---- HTTP 입구
def test_http_api_and_cors():
    wd = Watchdog(sim=True)
    srv = make_server(wd, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def call(path, method="GET", body=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(base + path, data=data, method=method, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=2) as r:
                return r.status, r.headers, json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            return e.code, e.headers, json.loads(e.read())

    try:
        st, h, _ = call("/hb", "OPTIONS")
        assert st == 204 and h["Access-Control-Allow-Origin"] == "*"
        assert call("/hb?src=browser")[0::2] == (200, {"ok": True}) and call("/hb?src=server", "POST")[0] == 200
        assert call("/hb?src=x")[0] == 400
        assert call("/arm", "POST", {"armed": True, "pid": 4242})[0] == 400  # stop_port 없음
        assert call("/arm", "POST", {"armed": True, "pid": 4242, "stop_port": 8091})[0] == 200
        st, h, s = call("/status")
        assert s["alive"] and s["armed"] and not s["tripped"] and set(s["age_s"]) == {"browser", "server"} and h["Access-Control-Allow-Origin"] == "*"
        assert call("/estop", "POST")[2]["tripped"] is False  # 시뮬레이션: 기록만
        assert call("/nope")[0] == 404
    finally:
        srv.shutdown()


# ---- 통합: SimDriver ArmService + StopListener + 감시
@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), dataclasses.replace(CFG), WS, speed_pct=50, log_root=str(tmp_path))
    ok, msg = s.connect()
    assert ok, msg
    yield s
    s.disconnect()


def test_stop_listener_estops_and_acks(svc):
    svc.driver.set_torque(True)
    lis = StopListener(svc, port=0).start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(1)
            s.sendto(b"not json", ("127.0.0.1", lis.port))  # 이상한 입력은 무시
            s.sendto(json.dumps({"cmd": "estop"}).encode(), ("127.0.0.1", lis.port))
            assert json.loads(s.recvfrom(256)[0]) == {"ack": "estop"}
        assert svc.ctl.state == "stopped" and svc.driver.torque_enabled()  # 토크는 유지
    finally:
        lis.close()


def test_heartbeat_loss_stops_the_arm_within_1_3_seconds(svc):
    lis = StopListener(svc, port=0).start()
    dummy = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    wd, stop = Watchdog(), threading.Event()
    threading.Thread(target=wd.run, args=(stop,), daemon=True).start()
    try:
        wd.arm(True, pid=dummy.pid, stop_port=lis.port)
        t_end = time.monotonic() + 0.5
        while time.monotonic() < t_end:  # 0.5초 동안은 heartbeat가 정상으로 온다
            wd.heartbeat("browser")
            wd.heartbeat("server")
            time.sleep(0.1)
        assert svc.ctl.state == "idle"
        t_last = time.monotonic()
        while not wd.status()["tripped"] and time.monotonic() - t_last < 3:
            time.sleep(0.01)
        assert wd.status()["tripped"] and time.monotonic() - t_last < 1.3
        assert svc.ctl.state == "stopped" and dummy.poll() is None  # 응답이 왔으니 프로세스는 끝내지 않는다
    finally:
        stop.set()
        lis.close()
        dummy.kill()
        dummy.wait()


def test_unresponsive_target_process_is_terminated():
    dummy = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    wd, stop = Watchdog(), threading.Event()
    threading.Thread(target=wd.run, args=(stop,), daemon=True).start()
    try:
        wd.arm(True, pid=dummy.pid, stop_port=free_udp_port())  # 아무도 받지 않는 포트
        assert dummy.wait(timeout=3) is not None  # 1.0초 감시 + 0.3초 응답 대기 뒤 끝난다
        assert wd.status()["tripped"]
    finally:
        stop.set()
        dummy.kill()


def test_second_watchdog_cannot_bind_the_same_port():
    srv = make_server(Watchdog(sim=True), 0)
    try:
        with pytest.raises(OSError):
            make_server(Watchdog(sim=True), srv.server_address[1])
    finally:
        srv.server_close()
