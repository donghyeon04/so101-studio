"""서보 건강 대시보드 (결정 0043-5): 정지 기준 한 단계 앞에서 '주의'를 판정하고 최근 추이를 모은다. 상태 스레드가 update를 부른다."""

import threading
from collections import deque

# 측정값: (단위, 주의, 오류, 높을수록 나쁨). 오류 값은 팔이 멈추는 기준(arm/safety.py, 결정 0028·0043)
LIMITS = {
    "temperature": ("°C", 50.0, 60.0, True),
    "load": ("%", 70.0, 90.0, True),
    "current": ("A", 3.0, 4.0, True),
    "voltage": ("V", 11.5, 11.0, False),
}
LABELS = {"temperature": "온도", "load": "부하", "current": "전류 합", "voltage": "전압"}
HOLD_S = 3.0  # 이만큼 이어져야 판정한다(한 번 튀는 값은 무시)
HISTORY_S = 300  # 추이 그래프 길이(1초 간격)
LEVEL_TEXT = {"ok": "정상", "warn": "주의", "error": "오류"}


def level_of(metric: str, value: float) -> str:
    _, warn, error, high_bad = LIMITS[metric]
    if high_bad:
        return "error" if value >= error else "warn" if value >= warn else "ok"
    return "error" if value < error else "warn" if value < warn else "ok"


def readings(status) -> dict[str, float]:
    """판정할 값: {'temperature:<관절>', 'load:<관절>', 'current', 'voltage'}."""
    out = {f"temperature:{j}": t for j, t in status.temperature_c.items()}
    out.update({f"load:{j}": v for j, v in status.load_pct.items()})
    out["current"] = status.current_ma / 1000
    if status.voltage_v > 0:  # 0은 아직 못 읽은 값
        out["voltage"] = status.voltage_v
    return out


class ServoHealth:
    def __init__(self, hold_s: float = HOLD_S):
        self.hold_s = hold_s
        self.history: deque = deque(maxlen=HISTORY_S)  # (t, 온도 dict, 부하 dict, 전류 A, 전압 V)
        self.peak: dict[str, float] = {}
        self.levels: dict[str, str] = {}  # 3초 이어진 판정
        self._raw: dict[str, tuple[str, float]] = {}  # 지금 판정, 그 판정이 시작된 시각
        self.changes: deque = deque(maxlen=50)  # (t, 키, 판정, 값): 판정이 바뀐 순간. 화면이 알림으로 읽는다
        self._lock = threading.Lock()

    def update(self, status) -> None:
        t = status.t
        with self._lock:
            for key, value in readings(status).items():
                self.peak[key] = max(self.peak.get(key, value), value) if key != "voltage" else min(self.peak.get(key, value), value)
                raw = level_of(key.split(":")[0], value)
                if self._raw.get(key, (None,))[0] != raw:
                    self._raw[key] = (raw, t)
                if t - self._raw[key][1] >= self.hold_s and self.levels.get(key, "ok") != raw:
                    self.levels[key] = raw
                    self.changes.append((t, key, raw, value))
            if not self.history or t - self.history[-1][0] >= 1.0:
                self.history.append((t, dict(status.temperature_c), dict(status.load_pct), status.current_ma / 1000, status.voltage_v))

    def level(self, key: str) -> str:
        return self.levels.get(key, "ok")

    def worst(self) -> tuple[str, list[str]]:
        """(가장 나쁜 판정, 주의·오류인 항목 글)."""
        with self._lock:
            bad = {k: v for k, v in self.levels.items() if v != "ok"}
        texts = [describe(k, bad[k]) for k in sorted(bad)]
        return ("error" if "error" in bad.values() else "warn" if bad else "ok"), texts

    def series(self) -> dict:
        """그래프용: {'t': 초(지금 기준 음수), 'temperature': {관절: [...]}, 'load': {...}, 'current': [...], 'voltage': [...]}."""
        with self._lock:
            rows = list(self.history)
        if not rows:
            return {"t": [], "temperature": {}, "load": {}, "current": [], "voltage": []}
        now = rows[-1][0]
        joints = list(rows[-1][1])
        return {
            "t": [round(r[0] - now, 1) for r in rows],
            "temperature": {j: [r[1].get(j) for r in rows] for j in joints},
            "load": {j: [r[2].get(j) for r in rows] for j in joints},
            "current": [round(r[3], 2) for r in rows],
            "voltage": [round(r[4], 2) for r in rows],
        }


def describe(key: str, level: str = "warn") -> str:
    metric, _, joint = key.partition(":")
    unit, warn, error, high_bad = LIMITS[metric]
    sign = "≥" if high_bad else "<"
    return f"{joint + ' ' if joint else ''}{LABELS[metric]} {sign} {error if level == 'error' else warn:g}{unit}"


EVENT_KINDS = {"comm_lost": "통신 끊김", "torque_lost": "토크 풀림", "fault": "오류 정지"}


def count_events(events: list[dict]) -> dict[str, tuple[int, float | None, str]]:
    """{종류: (횟수, 마지막 시각 s, 마지막 원인)}. 통신 끊김은 fault로도 한 번 더 남으므로 fault에서 뺀다."""
    out = {kind: [0, None, ""] for kind in EVENT_KINDS}
    for e in events:
        kind, data = e.get("kind"), e.get("data") or {}
        if kind not in out or (kind == "fault" and data.get("reason") == "comm_lost"):
            continue
        row = out[kind]
        row[0] += 1
        row[1] = e["t"] / 1e9
        row[2] = str(data.get("reason") or data.get("status_bits") or "")
    return {k: tuple(v) for k, v in out.items()}
