"""ESP32 시리얼 연결(7절 규약): 한 줄 JSON, 시계 맞추기, 끊김 감지."""
import json
import queue
import threading

from .model import RealClock


class UltrasonicLinkError(Exception):
    pass


class SerialTransport:
    """pyserial 위의 전송 계층. 읽기 스레드가 줄을 큐에 쌓는다. 가상 ESP32도 같은 write/readlines를 가진다."""

    def __init__(self, port: str, baud: int = 921600):
        import serial

        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._q: queue.Queue = queue.Queue()
        self._err: Exception | None = None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        try:
            while self._ser.is_open:
                line = self._ser.readline()
                if line:
                    self._q.put(line.decode(errors="replace"))
        except Exception as e:  # noqa: BLE001  케이블이 뽑히면 여기로 온다
            self._err = e
            return
        self._err = EOFError("포트가 닫혔다")  # 오류 없이 루프가 끝난 경우도 끊김으로 알린다

    def write(self, line: str) -> None:
        self._ser.write((line + "\n").encode())

    def readlines(self) -> list[str]:
        err = self._err  # 오류 전에 큐에 들어간 줄을 먼저 돌려준 뒤 다음 호출에서 던진다
        out = []
        while not self._q.empty():
            out.append(self._q.get())
        if err and not out:
            raise UltrasonicLinkError(f"시리얼 오류: {err}")
        return out

    def close(self) -> None:
        self._ser.close()


class Link:
    LOST_S = 1.0  # 데이터를 기다리는데 이만큼 조용하면 끊김(결정 0031과 같은 처리)

    def __init__(self, transport, clock=None, sync_every_s: float = 10):
        self.tx = transport
        self.clock = clock or RealClock()
        self.sync_every_ns = int(sync_every_s * 1e9)
        self.offset_ns = 0  # pc_ns = esp_us * 1000 + offset_ns
        self.sync_rtt_ns: int | None = None
        self._us_last = 0
        self._epoch = 0  # micros()는 32비트라 71분마다 돈다
        self._sync_t0: int | None = None
        self._last_sync = self.clock.now_ns()
        self._expect = False
        self._last_rx = self.clock.now_ns()
        self.inbox: list[dict] = []

    def send(self, **msg) -> None:
        self.tx.write(json.dumps(msg))

    def expect(self, on: bool) -> None:
        """데이터가 계속 와야 하는 동안 켠다. 1초 넘게 조용하면 poll()이 오류를 낸다."""
        self._expect = on
        self._last_rx = self.clock.now_ns()

    def to_ns(self, us: int) -> int:
        if us < self._us_last - 2**31:
            self._epoch += 2**32
        self._us_last = us
        return (self._epoch + us) * 1000 + self.offset_ns

    def _on_sync(self, m):
        if self._sync_t0 is None:
            return
        t1 = self.clock.now_ns()
        rtt = t1 - self._sync_t0
        if self.sync_rtt_ns is None or rtt <= self.sync_rtt_ns or rtt < 20_000_000:
            self.offset_ns = (self._sync_t0 + t1) // 2 - (self._epoch + m["us"]) * 1000
            self.sync_rtt_ns = rtt
        self._sync_t0 = None
        self._last_sync = t1

    def poll(self) -> list[dict]:
        """받은 메시지를 모두 돌려준다. 측정(r, a, end)에는 PC 시계 t_ns가 붙는다."""
        for line in self.tx.readlines():
            try:
                m = json.loads(line)
            except json.JSONDecodeError:
                continue  # 부팅 로그 등
            if not isinstance(m, dict):
                continue  # 숫자·리스트 같은 부팅 잡음
            self._last_rx = self.clock.now_ns()
            if m.get("t") in ("sync", "r", "a", "end") and not isinstance(m.get("us"), int):
                continue  # us가 없거나 깨진 줄
            if m.get("t") == "sync":
                self._on_sync(m)
                continue
            if m.get("t") in ("r", "a", "end"):
                m["t_ns"] = self.to_ns(m["us"])
            self.inbox.append(m)
        now = self.clock.now_ns()
        if self._sync_t0 is not None and now - self._sync_t0 > 1_000_000_000:
            self._sync_t0 = None  # 응답이 안 왔다: 다음 poll에서 다시 보낸다
        if self._expect and now - self._last_rx > self.LOST_S * 1e9:
            raise UltrasonicLinkError("US-LOST: ESP32 응답이 1초 넘게 없다")
        if self._sync_t0 is None and now - self._last_sync >= self.sync_every_ns:
            self._sync_t0 = now
            self.send(cmd="sync", pc=now)
        out, self.inbox = self.inbox, []
        return out

    def sync(self, n: int = 5) -> None:
        """연결할 때 왕복 시간이 가장 짧았던 시도로 시계 차이를 구한다."""
        self.sync_rtt_ns = None
        for _ in range(n):
            self._sync_t0 = self.clock.now_ns()
            self.send(cmd="sync", pc=self._sync_t0)
            for _ in range(500):
                kept = self.poll()
                self.inbox = kept + self.inbox  # 끼어든 다른 메시지는 보존
                if self._sync_t0 is None:
                    break
                self.clock.sleep(0.001)
            else:
                raise UltrasonicLinkError("sync 응답 없음")

    def wait_for(self, t: str, timeout_s: float = 2.0) -> dict:
        end = self.clock.now_ns() + int(timeout_s * 1e9)
        while self.clock.now_ns() < end:
            msgs = self.poll()
            hit = next((m for m in msgs if m.get("t") == t), None)
            if hit:
                self.inbox = [m for m in msgs if m is not hit] + self.inbox  # 다른 메시지는 다음 poll로
                return hit
            self.inbox = msgs + self.inbox
            self.clock.sleep(0.005)
        raise UltrasonicLinkError(f"'{t}' 응답 없음")
