"""감시 판정과 정지 순서. 시계와 기록을 바깥에서 받아 단위 시험이 쉽다."""
import json
import os
import signal
import socket
import threading
import time

HB_TIMEOUT_S = 1.0  # armed인 동안 heartbeat가 이만큼 끊기면 작동
TRIP_RETRY_S = 1.0  # trip이 예외로 실패하면 이만큼 뒤에 다시 시도한다
ACK_TIMEOUT_S = 0.3  # 정지 명령에 이 시간 안에 응답이 없으면 팔 프로세스를 끝낸다
SOURCES = ("browser", "server")


class Watchdog:
    def __init__(self, sim: bool = False, clock=time.monotonic, log=lambda rec: None, hb_timeout=HB_TIMEOUT_S, ack_timeout=ACK_TIMEOUT_S):
        self.sim, self.clock, self.log = sim, clock, log
        self.on_run_dir = None  # on_run_dir(경로): 등록 때 받은 새 실행 폴더로 기록 위치를 바꾼다. 허용하지 않으면 ValueError (watchdog/runlog.py)
        self.hb_timeout, self.ack_timeout = hb_timeout, ack_timeout
        self.armed, self.pid, self.stop_port = False, None, None
        self.tripped, self.reason = False, None
        self._busy = False  # 정지 순서 진행 중. tripped는 순서가 끝난 뒤에 켠다(ack 전에 작동으로 보이지 않게)
        self._last = dict.fromkeys(SOURCES, clock())
        self._last_tick: float | None = None  # 마지막 tick 시각. 판정 스레드가 죽거나 멈추면 status의 last_tick_age_s가 커진다
        self._retry_at = 0.0  # trip이 예외로 실패했을 때 다시 시도할 가장 이른 시각(50ms마다 되풀이해 기록이 쏟아지지 않게)
        self._noted = False  # 시뮬레이션에서 같은 끊김을 한 번만 기록
        self._lock = threading.Lock()

    def heartbeat(self, src: str) -> None:
        if src not in SOURCES:
            raise ValueError(f"src는 {SOURCES} 중 하나")
        with self._lock:
            self._last[src] = self.clock()

    def arm(self, armed: bool, pid: int | None = None, stop_port: int | None = None, run_dir: str | None = None) -> None:
        """감시 대상을 등록·해제한다. 등록하면 heartbeat 시계를 지금으로 맞추고, 이전의 작동 상태를 푼다.
        run_dir를 함께 주면 기록을 그 실행 폴더로 옮긴다(팔을 다시 연결할 때마다 실행 폴더가 새로 생긴다)."""
        if armed and not (isinstance(pid, int) and pid > 0 and isinstance(stop_port, int) and 0 < stop_port < 65536):
            raise ValueError("armed=true에는 pid와 stop_port가 필요하다")
        if run_dir is not None and self.on_run_dir:
            self.on_run_dir(run_dir)  # 검증이 먼저: 거절되면 등록도 하지 않는다
            self.log({"event": "run_dir", "run_dir": run_dir})
        with self._lock:
            self.armed, self.pid, self.stop_port = armed, (pid if armed else None), (stop_port if armed else None)
            self.tripped, self.reason, self._noted, self._busy, self._retry_at = False, None, False, False, 0.0
            self._last = dict.fromkeys(SOURCES, self.clock())
        self.log({"event": "arm", "armed": armed, "pid": pid, "stop_port": stop_port, "sim": self.sim})

    def status(self) -> dict:
        with self._lock:
            now = self.clock()
            return {"alive": True, "armed": self.armed, "sim": self.sim, "tripped": self.tripped, "reason": self.reason,
                    "age_s": {s: round(now - t, 3) for s, t in self._last.items()},
                    "last_tick_age_s": None if self._last_tick is None else round(now - self._last_tick, 3)}

    def tick(self) -> bool:
        """20Hz로 부른다. 작동했으면 True."""
        with self._lock:
            self._last_tick = self.clock()
            if not self.armed or self.tripped or self._busy:
                return False
            now = self.clock()
            if now < self._retry_at:
                return False
            stale = [s for s in SOURCES if now - self._last[s] > self.hb_timeout]
        if not stale:
            self._noted = False
            return False
        return self.trip(f"heartbeat 끊김: {','.join(stale)}")

    def trip(self, reason: str) -> bool:
        """정지 순서: ① 팔 프로세스의 정지 수신 스레드에 UDP 정지 명령 ② 응답이 없으면 그 프로세스를 끝낸다. 시뮬레이션이거나 armed가 아니면 기록만 한다."""
        with self._lock:
            armed, pid, port = self.armed, self.pid, self.stop_port
            live = armed and not self.sim and not self._busy and not self.tripped
            if live:
                self._busy = True
        if not live:
            if not self._noted:
                self._noted = True
                self.log({"event": "trip_ignored", "reason": reason, "armed": armed, "sim": self.sim})
            return False
        try:
            acked = self._send_estop(port, reason)
            killed = False if acked else self._kill(pid)
        except Exception as e:  # noqa: BLE001 어떤 예외든 _busy가 남아 감시가 영영 멈추지 않게 한다: 기록하고 다음 tick에 다시 시도
            with self._lock:
                self._busy = False
                self._retry_at = self.clock() + TRIP_RETRY_S
            self.log({"event": "trip_error", "reason": reason, "error": repr(e), "pid": pid, "retry_in_s": TRIP_RETRY_S})
            return False
        with self._lock:
            self.tripped, self.reason, self._busy = True, reason, False
        self.log({"event": "trip", "reason": reason, "acked": acked, "killed": killed, "pid": pid})
        return True

    def _send_estop(self, port: int, reason: str) -> bool:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(self.ack_timeout)
            try:
                s.sendto(json.dumps({"cmd": "estop", "reason": reason}).encode(), ("127.0.0.1", port))
                return json.loads(s.recvfrom(256)[0]).get("ack") == "estop"
            except (OSError, ValueError):  # 시간 초과, 포트 닫힘(Windows는 ICMP로 ConnectionReset), 이상한 응답
                return False

    @staticmethod
    def _kill(pid: int) -> bool:
        if pid == os.getpid():
            return False
        try:
            os.kill(pid, signal.SIGTERM)  # Windows에서는 TerminateProcess
            return True
        except OSError:
            return False

    def run(self, stop: threading.Event, hz: float = 20.0) -> None:
        while not stop.wait(1.0 / hz):
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001 한 번의 예외로 감시 스레드가 죽으면 /status는 alive인데 아무도 안 본다: 기록하고 계속 돈다
                try:
                    self.log({"event": "tick_error", "error": repr(e)})
                except Exception:  # noqa: BLE001, S110 기록 실패도 감시를 멈추지 않는다
                    pass
