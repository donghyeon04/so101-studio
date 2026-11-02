"""감시 프로세스의 HTTP 입구 (127.0.0.1). 브라우저가 부르므로 CORS를 허용한다."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


def make_server(wd, port: int = 8090) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code: int, body: dict | None = None) -> None:
            data = json.dumps(body).encode() if body is not None else b""
            self.send_response(code)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _json(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n) or b"{}")

        def do_OPTIONS(self):
            self._reply(204)

        def _route(self):
            u = urlparse(self.path)
            try:
                if u.path == "/hb":
                    wd.heartbeat(parse_qs(u.query).get("src", [""])[0])
                    return self._reply(200, {"ok": True})
                if u.path == "/status" and self.command == "GET":
                    return self._reply(200, wd.status())
                if u.path == "/arm" and self.command == "POST":
                    b = self._json()
                    wd.arm(b.get("armed") is True, b.get("pid"), b.get("stop_port"))
                    return self._reply(200, {"ok": True})
                if u.path == "/estop" and self.command == "POST":
                    return self._reply(200, {"ok": True, "tripped": wd.trip("수동 E-STOP 요청")})
            except (ValueError, TypeError) as e:
                return self._reply(400, {"ok": False, "error": str(e)})
            self._reply(404, {"ok": False})

        do_GET = do_POST = _route

        def log_message(self, *a):  # 20Hz heartbeat 기록이 화면을 덮지 않게
            pass

    class Server(ThreadingHTTPServer):
        allow_reuse_address = False  # Windows에서는 켜 두면 같은 포트에 감시 프로세스가 둘 뜰 수 있다

    return Server(("127.0.0.1", port), Handler)
