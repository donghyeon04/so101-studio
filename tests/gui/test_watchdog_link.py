"""감시 프로세스 연결 (gui-spec G2b). 감시 프로세스 대신 HTTP 서버를 띄워 규약만 시험한다."""

import asyncio
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import httpx
import pytest

pytest.importorskip("nicegui")

from so101_studio.gui.watchdog_link import WatchdogLink


@pytest.fixture
def fake_watchdog():
    server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    server.calls, server.status = [], {"alive": True, "tripped": False}
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield server
    server.shutdown()


class _Handler(BaseHTTPRequestHandler):
    def _reply(self, body):
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.server.calls.append(self.path)
        self._reply(self.server.status if self.path == "/status" else {"ok": True})

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        self.server.calls.append((self.path, body))
        self._reply({"ok": True})

    def log_message(self, *a):
        pass


def _link(server) -> WatchdogLink:
    return WatchdogLink(port=server.server_address[1])


def _run_for(link, seconds):
    async def go():
        with pytest.raises(TimeoutError):
            await asyncio.wait_for(link.run(), seconds)

    asyncio.run(go())


def test_server_heartbeat_and_status(fake_watchdog):
    link = _link(fake_watchdog)
    _run_for(link, 1.0)  # 0.2초 주기. 전체 시험 중 부하에서 0.7초는 3번에 못 미친 적이 있다
    assert fake_watchdog.calls.count("/hb?src=server") >= 3
    assert "/status" in fake_watchdog.calls
    assert link.alive and link.text() == "정상"


def test_registration_carries_the_current_run_folder(fake_watchdog):
    link = _link(fake_watchdog)
    link.want_armed = True
    _run_for(link, 0.1)
    assert "run_dir" not in fake_watchdog.calls[1][1]
    link.run_dir = "runs/새 실행"
    link.rearm()
    _run_for(link, 0.1)
    bodies = [c[1] for c in fake_watchdog.calls if isinstance(c, tuple)]
    assert bodies[-1]["run_dir"] == os.path.abspath("runs/새 실행")


def test_refused_run_folder_turns_the_watchdog_off(fake_watchdog, monkeypatch):
    link = _link(fake_watchdog)
    link.want_armed, link.run_dir = True, "runs/밖"
    link.status = {"alive": True}

    async def refuse(self, *_a, **_kw):
        return httpx.Response(400, request=httpx.Request("POST", "http://test"))

    monkeypatch.setattr(httpx.AsyncClient, "post", refuse)
    _run_for(link, 0.1)
    assert not link.alive and link.text() == "꺼짐"


def test_tripped_status_shows_as_activated(fake_watchdog):
    fake_watchdog.status = {"alive": True, "tripped": True}
    link = _link(fake_watchdog)
    _run_for(link, 0.1)
    assert link.text() == "작동함"


def test_silent_watchdog_is_off_and_blocks_real_arm():
    link = WatchdogLink(port=1)  # 아무도 듣지 않는다
    link.status = {"alive": True}
    _run_for(link, 0.3)
    assert link.text() == "꺼짐"
    assert not link.alive


def test_leftover_sim_watchdog_does_not_count_for_real_arm():
    link = WatchdogLink(sim=False)
    link.status = {"alive": True, "sim": True, "tripped": False}
    assert not link.alive
    assert link.text() == "꺼짐"


def test_arms_only_after_a_browser_owns_control(fake_watchdog, backend):
    link = _link(fake_watchdog)
    backend.link = link
    try:
        _run_for(link, 0.1)
        assert not [c for c in fake_watchdog.calls if isinstance(c, tuple)]
        backend.claim("A")
        _run_for(link, 0.1)
    finally:
        backend.link = None
    posts = [c for c in fake_watchdog.calls if isinstance(c, tuple)]
    assert len(posts) == 1
    path, body = posts[0]
    assert path == "/arm" and body["armed"] is True and body["pid"] > 0 and body["stop_port"] == link.stop_port


def test_resume_registers_again(fake_watchdog, backend):
    link = _link(fake_watchdog)
    link.want_armed = True
    backend.link = link
    try:
        _run_for(link, 0.1)
        backend.arm.estop()
        backend.resume()
        assert backend.arm.ctl.state == "idle"
        _run_for(link, 0.1)
    finally:
        backend.link = None
    assert len([c for c in fake_watchdog.calls if isinstance(c, tuple)]) == 2


def test_real_arm_blocked_without_a_live_watchdog(fake_watchdog, backend):
    assert not backend.real_allowed()  # 연결 자체가 없음
    link = _link(fake_watchdog)
    backend.link = link
    try:
        assert not backend.real_allowed()  # 아직 응답을 못 받음
        _run_for(link, 0.1)
        assert backend.real_allowed()
    finally:
        backend.link = None


def test_start_reuses_a_running_watchdog(fake_watchdog, monkeypatch):
    spawned = []
    monkeypatch.setattr("subprocess.Popen", lambda *a, **k: spawned.append(a))
    _link(fake_watchdog).start()
    assert spawned == []


def test_start_spawns_when_nothing_answers(monkeypatch):
    spawned = []
    monkeypatch.setattr("subprocess.Popen", lambda cmd, **k: spawned.append(cmd))
    WatchdogLink(sim=True, port=1).start()
    assert spawned and spawned[0][1:4] == ["-m", "so101_studio.watchdog", "--port"] and "--sim" in spawned[0]
    spawned.clear()
    WatchdogLink(sim=False, port=1).start()
    assert "--sim" not in spawned[0]


def test_alive_needs_a_recent_tick_when_the_watchdog_reports_it():
    """감사 2026-11-07: 감시 스레드가 죽어도 HTTP는 답한다. last_tick_age_s가 오래되면 살아 있지 않다고 본다."""
    link = WatchdogLink(sim=True, port=1)
    link.status = {"sim": True, "last_tick_age_s": 0.2}
    assert link.alive
    link.status = {"sim": True, "last_tick_age_s": 5.0}
    assert not link.alive
    link.status = {"sim": True}  # 옛 감시 프로세스(필드 없음)는 응답만으로 판정
    assert link.alive
