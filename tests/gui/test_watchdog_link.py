"""감시 프로세스 연결 (gui-spec G2b). 감시 프로세스 대신 HTTP 서버를 띄워 규약만 시험한다."""

import asyncio
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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
    _run_for(link, 0.7)
    assert fake_watchdog.calls.count("/hb?src=server") >= 3
    assert "/status" in fake_watchdog.calls
    assert link.alive and link.text() == "정상"


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


def test_missing_module_reads_as_none(monkeypatch):
    # REQ-011 임시: 모듈이 main에 들어오면 이 분기와 함께 지운다
    monkeypatch.setattr("importlib.util.find_spec", lambda name: None)
    link = WatchdogLink()
    link.start()
    assert link.text() == "없음"
    assert not link.alive
    asyncio.run(asyncio.wait_for(link.run(), 0.1))
