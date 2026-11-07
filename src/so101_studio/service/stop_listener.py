"""정지 수신 스레드 (REQ-011, decisions/0026): 감시 프로세스가 UDP로 보내는 {"cmd":"estop"}을 받아 arm_service.estop()(토크 유지)을 부르고 {"ack":"estop"}로 답한다."""
import json
import socket
import threading


class StopListener:
    def __init__(self, arm_service, port: int = 8091):
        self.arm, self.port = arm_service, port
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> "StopListener":
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", self.port))
        self.port = self._sock.getsockname()[1]  # port=0이면 정해진 번호
        self._sock.settimeout(0.2)
        self._thread = threading.Thread(target=self._loop, args=(self._sock,), daemon=True)
        self._thread.start()
        return self

    def _loop(self, sock: socket.socket) -> None:
        while self._sock is sock:
            try:
                data, addr = sock.recvfrom(1024)
            except TimeoutError:
                continue
            except OSError:  # close()로 닫힘
                return
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            if isinstance(msg, dict) and msg.get("cmd") == "estop":
                err = self.arm.estop()  # 정지가 먼저: 기록·사고 묶음이 느려도 정지를 늦추지 않는다. 예외면 응답하지 않아 감시 프로세스가 프로세스를 끝낸다
                try:
                    getattr(self.arm, "on_watchdog_stop", lambda r: None)(msg.get("reason"))  # 기록과 사고 묶음
                except Exception:  # noqa: BLE001, S110 기록 실패가 ack를 막지 않는다
                    pass
                ack = {"ack": "estop", "ok": err is None}  # ok false = 서보에 위치 고정을 쓰지 못했다(통신 오류)
                if err:
                    ack["comm_error"] = err
                sock.sendto(json.dumps(ack).encode(), addr)

    def close(self) -> None:
        s, self._sock = self._sock, None
        if s:
            s.close()
        if self._thread:
            self._thread.join(timeout=1.0)
