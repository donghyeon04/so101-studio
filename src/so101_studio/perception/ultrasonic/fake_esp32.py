"""가상 ESP32: 7절 규약 그대로. 가상 시계(SimClock)를 따라 지연 계산하므로 스레드·실제 대기가 없다.
transport 인터페이스(write, readlines)가 SerialTransport와 같아 Link가 구분하지 못한다."""
import json
import random
from collections import deque

from .model import Box, arm_echo, radar_echo


class SimClock:
    def __init__(self):
        self.t_ns = 0

    def now_ns(self) -> int:
        return self.t_ns

    def sleep(self, s: float) -> None:
        self.t_ns += int(s * 1e9)


class FakeESP32:
    SERVO_MS = 100  # 회전 시작 때 서보가 시작 각도까지 가는 시간
    DISC_R = 20  # 팔 링크 원 반경(mm)

    def __init__(self, clock, boxes: list[Box], cfg, *, arm_sensor_fn=None, arm_links_fn=None,
                 seed: int = 0, noise_mm: float = 2.0, noise_pct: float = 0.003, dropout: float = 0.03,
                 boot_offset_us: int = 5_000_000, wrist_beam_half_deg: float | None = None,
                 max_incidence_deg: float | None = None, servo_lag_deg: float = 0.0, corner=None,
                 ignore_first_writes: int = 0, servo: str = "pwm", ang_noise_deg: float = 0.1):
        self.servo, self.ang_noise_deg = servo, ang_noise_deg  # "dxl"이면 샘플 r에 읽은 실제 각도 ang를 싣는다(7절)
        self._ignore_writes = ignore_first_writes  # 부팅 중이라 처음 몇 줄은 못 받는다(포트를 열면 ESP32가 리셋되는 경우)
        # 센서 모델(8.5절): 거리 잡음 = noise_mm + noise_pct x 거리, 응답 없음 dropout, 손목 센서 빔 반각, 레이더 면 반사 허용각,
        # 서보 지연(갈 때 -lag°, 올 때 +lag°), 모서리 반사 corner = (반각 배율, 거리 mm, 확률)
        self.noise_pct, self.wrist_beam_half_deg = noise_pct, wrist_beam_half_deg
        self.max_incidence_deg, self.servo_lag_deg, self.corner = max_incidence_deg, servo_lag_deg, corner
        self.clock, self.boxes, self.cfg = clock, boxes, cfg
        self.arm_sensor_fn, self.arm_links_fn = arm_sensor_fn, arm_links_fn  # t_ns -> (x,y,z) / [(x,y)]
        self.rng = random.Random(seed)
        self.noise_mm, self.dropout = noise_mm, dropout
        self.boot_offset_us = boot_offset_us
        self.gap_ns = int(cfg.ping_gap_ms * 1e6)
        self._out: deque[str] = deque()
        self._next_ns = 0  # 다음 측정이 가능한 가장 이른 시각
        self._once: deque[tuple[str, int]] = deque()  # (센서, max_mm)
        self._sweep: dict | None = None
        self._stream: dict | None = None
        self._dead = False
        self._servo_deg = 90  # sweep 밖(once 등)에서 레이더가 보는 각도

    # --- transport ---
    def write(self, line: str) -> None:
        if self._dead:
            return
        if self._ignore_writes > 0:
            self._ignore_writes -= 1
            return
        self._advance()  # 명령 이전 시각까지 먼저 처리
        m = json.loads(line)
        now = self.clock.now_ns()
        if self._idle():
            self._next_ns = max(self._next_ns, now)
        getattr(self, "_cmd_" + m["cmd"], self._cmd_unknown)(m, now)

    def readlines(self) -> list[str]:
        if self._dead:
            return []
        self._advance()
        out = list(self._out)
        self._out.clear()
        return out

    def disconnect(self) -> None:
        """케이블이 뽑힌 것처럼 더 이상 아무것도 보내지 않는다."""
        self._dead = True

    # --- 명령 ---
    def _us(self, t_ns: int) -> int:
        return (t_ns // 1000 + self.boot_offset_us) & 0xFFFFFFFF

    def _emit(self, **m) -> None:
        self._out.append(json.dumps(m, separators=(",", ":")))

    def _idle(self) -> bool:
        return not (self._once or self._sweep or self._stream)

    def _cmd_hello(self, m, now):
        self._emit(t="hello", fw="0.1", servo=self.servo, k=["r", "a"])

    def _cmd_sync(self, m, now):
        self._emit(t="sync", pc=m["pc"], us=self._us(now))

    def _cmd_once(self, m, now):
        for _ in range(m.get("n", 5)):
            for k in m["k"]:
                self._once.append((k, m.get("max_mm", 600)))

    def _cmd_gap(self, m, now):
        self.gap_ns = int(min(max(m.get("ms", 33), 5), 1000) * 1e6)
        self._emit(t="gap", ms=self.gap_ns // 1_000_000)

    def _cmd_sweep(self, m, now):
        angles = list(range(m["from"], m["to"] + 1, m["step"]))
        self._stream = None
        self._sweep = {"angles": angles, "i": 0, "dir": 1, "bounce": m.get("bounce", True), "left": m.get("sweeps", 0),
                           "max_mm": m.get("max_mm", 600)}
        self._next_ns = max(self._next_ns, now + self.SERVO_MS * 1_000_000)

    def _cmd_stream(self, m, now):
        self._sweep = None  # 팔 센서 우선: 레이더는 멈춘다
        period = max(int(1e9 / m.get("hz", 30)), self.gap_ns)
        self._stream = {"period": period, "max_mm": m.get("max_mm", 300)}
        self._next_ns = max(self._next_ns, now)

    def _cmd_stop(self, m, now):
        self._once.clear()
        self._sweep = self._stream = None

    def _cmd_servo(self, m, now):
        self._servo_deg = m.get("deg", 90)

    def _cmd_unknown(self, m, now):
        self._emit(t="err", msg="unknown cmd")

    # --- 시간 진행 ---
    def _advance(self) -> None:
        now = self.clock.now_ns()
        while self._next_ns <= now:
            t = self._next_ns
            if self._once:
                k, mx = self._once.popleft()
                self._measure(k, t, mx, None)
                self._next_ns = t + self.gap_ns
                if not self._once:
                    self._emit(t="done")  # once 끝(7절)
            elif self._stream:
                self._measure("a", t, self._stream["max_mm"], None)
                self._next_ns = t + self._stream["period"]
            elif self._sweep:
                self._step_sweep(t)
                self._next_ns = t + self.gap_ns
            else:
                break

    def _step_sweep(self, t: int) -> None:
        sw = self._sweep
        a = sw["angles"][sw["i"]]
        self._measure("r", t, sw["max_mm"], a, sw["dir"])
        sw["i"] += sw["dir"]
        if 0 <= sw["i"] < len(sw["angles"]):
            return
        self._emit(t="end", dir=sw["dir"], us=self._us(t))
        if sw["bounce"]:
            sw["dir"] = -sw["dir"]
            sw["i"] = len(sw["angles"]) - 1 if sw["dir"] < 0 else 0
        else:
            sw["i"] = 0
        if sw["left"]:  # sweeps = 한 방향 회전 횟수, 0이면 stop까지
            sw["left"] -= 1
            if sw["left"] == 0:
                self._sweep = None

    def _measure(self, k: str, t: int, max_mm: float, angle, direction=1) -> None:
        d = self._dist(k, t, angle, direction)
        if d is not None:
            d += self.rng.gauss(0, self.noise_mm + self.noise_pct * d)
        if d is None or d > max_mm or self.rng.random() < self.dropout:
            d = None
        else:
            d = round(d)
        if k == "r":
            extra = {}
            if self.servo == "dxl":
                extra["ang"] = round(angle - direction * self.servo_lag_deg + self.rng.gauss(0, self.ang_noise_deg), 2)
            self._emit(t="r", a=angle, d=d, us=self._us(t), **extra)
        else:
            self._emit(t="a", d=d, us=self._us(t))

    def _dist(self, k: str, t: int, angle, direction=1) -> float | None:
        c, half = self.cfg, self.cfg.beam_half_deg
        if k == "r":
            r = c.radar
            discs = [(x, y, self.DISC_R) for x, y in (self.arm_links_fn(t) if self.arm_links_fn else [])]
            angle = self._servo_deg if angle is None else angle
            az = r.heading_deg + (angle - 90 - direction * self.servo_lag_deg)
            return radar_echo(r.station_mm.x, r.station_mm.y, az, self.boxes, discs, half, 1e9, self.max_incidence_deg,
                              self.corner, self.rng)
        pose = self.arm_sensor_fn(t) if self.arm_sensor_fn else None
        return None if pose is None else arm_echo(*pose, self.boxes, self.wrist_beam_half_deg or half)
