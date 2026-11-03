"""동작 녹화·재생 (arm-software-spec.md 6.8). 리더 암 없이 토크를 끄고 손으로 움직여 가르친다."""
import json
import threading
import time
from dataclasses import asdict, astuple
from pathlib import Path

import numpy as np

from ..core.types import JointState, MotionResult, ToolPose
from .driver import JOINTS
from .trajectory import SequenceTrajectory, cfg_move_j

MAX_SPEED_DEG_S = 220.0  # 구간별 속도 배율을 적용한 뒤에도 이 속도를 넘으면 거부한다(한 주기 이동량 한계 5° = 250°/s 안)
LABELS = ("멈춤", "느림", "빠름")
SMOOTH = 5  # 재생 전에 이동평균 창(표본 수)
CLAMP_TOL_DEG = 3.0  # 손으로 움직인 자세가 관절 한계를 이만큼까지 넘으면 한계로 잘라 재생한다(받침 자세 근처 등). 더 넘으면 그대로 두어 안전 검사가 거부한다


def analyze(q: np.ndarray, t: np.ndarray, pause: float = 3.0, fast: float = 40.0, min_s: float = 0.5) -> dict:
    """기록의 길이·속도 요약과 속도에 따른 구간(멈춤/느림/빠름). 속도는 가장 빠르게 움직이는 관절의 °/s다."""
    n = len(q)
    if n < 3:
        return {"points": n, "duration_s": float(t[-1] - t[0]) if n > 1 else 0.0, "hz_effective": 0.0, "mean_deg_s": 0.0, "max_deg_s": 0.0, "segments": []}
    dt = np.maximum(np.diff(t), 1e-6)
    v = (np.abs(np.diff(q, axis=0)) / dt[:, None]).max(axis=1)  # 구간(표본 사이)마다 가장 빠른 관절의 속도
    k = max(1, round(0.3 / float(np.median(dt))))
    vs = np.convolve(np.pad(v, (k // 2, k - 1 - k // 2), mode="edge"), np.ones(k) / k, mode="valid")
    cls = np.where(vs < pause, 0, np.where(vs < fast, 1, 2))
    runs, start = [], 0
    for i in range(1, len(cls) + 1):
        if i == len(cls) or cls[i] != cls[start]:
            runs.append([start, i - 1])
            start = i
    merged: list[list[int]] = []
    for a, b in runs:  # 너무 짧은 구간은 앞 구간에 합친다
        if merged and t[b + 1] - t[a] < min_s:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    segs: list[list[int]] = []
    for a, b in merged:  # 합친 구간의 평균 속도로 이름을 다시 정하고, 이웃과 이름이 같으면 한 구간으로 본다
        lab = 0 if v[a:b + 1].mean() < pause else (1 if v[a:b + 1].mean() < fast else 2)
        if segs and segs[-1][2] == lab:
            segs[-1][1] = b
        else:
            segs.append([a, b, lab])
    out = [{"index": i, "start_s": float(t[a] - t[0]), "end_s": float(t[b + 1] - t[0]), "label": LABELS[lab],
            "mean_deg_s": float(v[a:b + 1].mean()), "max_deg_s": float(v[a:b + 1].max())} for i, (a, b, lab) in enumerate(segs)]
    moving = v[v >= pause]
    return {"points": n, "duration_s": float(t[-1] - t[0]), "hz_effective": float((n - 1) / (t[-1] - t[0])),
            "mean_deg_s": float(moving.mean()) if len(moving) else 0.0, "max_deg_s": float(v.max()),
            "max_per_joint_deg_s": [float(x) for x in (np.abs(np.diff(q, axis=0)) / dt[:, None]).max(axis=0)], "segments": out}


def retime(t: np.ndarray, segments: list[dict], factors: dict[int, float], speed: float = 1.0) -> np.ndarray:
    """구간별 속도 배율(2 = 두 배 빠르게, 0.5 = 절반 속도)과 전체 배율을 반영한 새 시각. 한 구간만 느리게/빠르게 할 수 있다."""
    dt = np.diff(t)
    scale = np.full(len(dt), float(speed))
    rel = t[:-1] - t[0]
    for seg in segments:
        f = factors.get(seg["index"], 1.0)
        if f != 1.0:
            scale[(rel >= seg["start_s"] - 1e-9) & (rel < seg["end_s"] - 1e-9)] *= f
    return np.concatenate([[0.0], np.cumsum(dt / scale)])


def smooth(q: np.ndarray, window: int = SMOOTH) -> np.ndarray:
    """이동평균. 처음과 끝 자세는 그대로 둔다."""
    if window < 2 or len(q) < window:
        return q
    pad = window // 2
    ext = np.pad(q, ((pad, pad), (0, 0)), mode="edge")
    out = np.stack([np.convolve(ext[:, j], np.ones(window) / window, mode="valid") for j in range(q.shape[1])], axis=1)
    out[0], out[-1] = q[0], q[-1]
    return out


class Teach:
    def __init__(self, ctl, cfg, seq_dir: str | Path = "config/sequences"):
        self.ctl, self.cfg, self.dir = ctl, cfg, Path(seq_dir)
        self._rows: list[list[float]] = []
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._t: list[float] = []  # 표본마다 실제 시각(초). 손으로 움직인 속도를 그대로 재현하려고 함께 저장한다
        self.truncated = False
        self.hz, self.max_s = cfg.record_hz, cfg.record_max_s
        self.last_cut = None  # until_limit 재생이 잘린 위치 (재생한 점 수, 전체 점 수, 이유)

    @property
    def recording(self) -> bool:
        return self._thread is not None

    def record_start(self, hz: int | None = None, max_s: float | None = None) -> None:
        """토크를 끄고 관절값을 20Hz로 기록한다. 시작 전에 팔을 손으로 받쳐야 한다(토크가 꺼지면 처진다).
        표본마다 실제 시각을 함께 기록해 손으로 움직인 속도가 재생에 그대로 반영된다. 기록 주기(hz)와 최대 시간(max_s)은 설정값을 이번만 바꿀 수 있다."""
        if self.recording:
            return
        if self.ctl.locked_reason:
            raise RuntimeError(self.ctl.locked_reason)  # 토크를 끄는 것도 서보에 쓰는 일이다
        self.ctl.stop()
        self.ctl.driver.set_torque(False)
        self.hz, self.max_s = int(hz or self.cfg.record_hz), float(max_s or self.cfg.record_max_s)
        self._rows, self._t, self.truncated = [], [], False
        self._stop.clear()

        def loop():
            t0 = time.perf_counter()
            while not self._stop.is_set():
                now = time.perf_counter() - t0
                if now > self.max_s:
                    self.truncated = True
                    return
                self._rows.append(list(astuple(self.ctl.driver.read_joints())))
                self._t.append(now)
                time.sleep(1.0 / self.hz)

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def record_stop(self, name: str) -> Path:
        """기록을 config/sequences/<name>.json으로 저장하고 토크를 켠다(현재 자세를 목표로 쓴 뒤 켜므로 튀지 않는다)."""
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        self.ctl.driver.set_torque(True)
        self.ctl.sync()
        self.dir.mkdir(parents=True, exist_ok=True)
        path = self.dir / f"{name}.json"
        meta = self._analyze(np.array(self._rows, float), np.array(self._t, float))
        meta["recorded_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        path.write_text(json.dumps({"hz": self.hz, "t": self._t, "joints": self._rows, "meta": meta}, ensure_ascii=False), encoding="utf-8")
        return path

    def _analyze(self, q: np.ndarray, t: np.ndarray) -> dict:
        return analyze(q, t, self.cfg.seg_pause_deg_s, self.cfg.seg_fast_deg_s, self.cfg.seg_min_s)

    def info(self, name: str) -> dict:
        """기록의 길이·속도 요약과 구간 목록. 요약이 없는 옛 파일은 지금 계산한다."""
        data = json.loads((self.dir / f"{name}.json").read_text(encoding="utf-8"))
        q, t = self.load(name)
        return {**self._analyze(q, t), **({"recorded_at": data["meta"]["recorded_at"]} if "meta" in data and "recorded_at" in data["meta"] else {})}

    def load(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        """(관절 자세 목록, 표본 시각[초]). 시각이 없는 옛 파일은 hz로 일정하다고 본다."""
        data = json.loads((self.dir / f"{name}.json").read_text(encoding="utf-8"))
        q = np.array(data["joints"], float)
        t = np.array(data["t"], float) if len(data.get("t", [])) == len(q) else np.arange(len(q)) / data["hz"]
        return q, t

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        q = q.copy()
        for i, (name, (lo, hi)) in enumerate((n, self.ctl.safety.limits[n]) for n in JOINTS if n in self.ctl.safety.limits):
            over = np.maximum(lo - q[:, i], q[:, i] - hi)
            if 0 < over.max() <= CLAMP_TOL_DEG:
                q[:, i] = np.clip(q[:, i], lo, hi)
        return q

    def replay(self, name: str, speed: float = 1.0, until_limit: bool = False, segment_speeds: dict[int, float] | None = None) -> MotionResult:
        """기록을 다듬어 재생한다. 시작 전에 첫 자세로 MoveJ(저속)로 간 뒤 시간에 맞춰 따라간다.
        한계·작업 영역 밖이면 움직이지 않고 거부한다(execute의 안전 검사).
        segment_speeds={구간 번호: 배율}이면 그 구간만 빠르게/느리게 한다(구간은 info로 본다). 배율은 0.1~5, 적용 뒤 속도가 220°/s를 넘으면 ValueError."""
        q, t = self.load(name)
        factors = segment_speeds or {}
        segs = self._analyze(q, t)["segments"]
        if any(not 0.1 <= f <= 5.0 for f in factors.values()) or any(i >= len(segs) or i < 0 for i in factors):
            raise ValueError(f"구간 번호는 0~{len(segs) - 1}, 배율은 0.1~5 사이여야 합니다")
        t_new = retime(t, segs, factors, speed)
        step = np.abs(np.diff(q, axis=0)) if len(q) > 1 else np.zeros((1, 6))
        peak = float((step / np.maximum(np.diff(t_new), 1e-6)[:, None]).max()) if len(q) > 1 else 0.0
        orig = float((step / np.maximum(np.diff(t), 1e-6)[:, None]).max()) if len(q) > 1 else 0.0
        if peak > MAX_SPEED_DEG_S and peak > orig * 1.001:  # 배율 때문에 한계를 넘는 경우만 거부한다(원래부터 빠른 기록은 안전 검사가 판단)
            raise ValueError(f"이 배율이면 최대 속도가 {peak:.0f}°/s로 한계 {MAX_SPEED_DEG_S:.0f}°/s를 넘습니다")
        t = t_new
        q = self._clamp(smooth(q))
        first = JointState(*q[0].tolist())
        self.last_cut = None
        if until_limit:  # 한계·바닥을 처음 넘는 자세 앞까지만 재생한다
            total = len(q)
            for i, row in enumerate(q):
                why = self.ctl.safety.check_goal(JointState(*row.tolist()), first)
                if why:
                    q, t, self.last_cut = q[:i], t[:i], (i, total, why)
                    break
            if len(q) < 10:
                return MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0))
        r = self.ctl.execute(cfg_move_j(self.cfg, self.ctl.driver.read_joints(), first, 20)).wait()
        if r.status != "ok":
            return r
        return self.ctl.execute(SequenceTrajectory(t - t[0], q)).wait()

    def run_sequence(self, name: str) -> MotionResult:
        return self.replay(name, 1.0)

    def names(self) -> list[str]:
        return sorted(p.stem for p in self.dir.glob("*.json"))

    def current(self) -> dict:
        return asdict(self.ctl.driver.read_joints())
