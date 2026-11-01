"""떨림 추적: 키보드 조종 중 명령(목표)과 실제 관절 위치를 최대한 빠르게 기록하고, 움직이는 동안 관절별 떨림 크기와 주파수를 요약한다.
어느 관절이 몇 Hz로 떠는지 알아내는 용도다 (10Hz status.csv로는 보이지 않는다)."""
import csv
import threading
import time
from dataclasses import astuple

import numpy as np

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def analyze(t: np.ndarray, meas: np.ndarray, goal: np.ndarray, smooth_s: float = 0.1) -> list[dict]:
    """움직이는 구간(목표가 변하는 구간)에서 실제 위치의 떨림(이동 평균을 뺀 나머지)의 RMS와 지배 주파수, 추종 오차."""
    hz = (len(t) - 1) / (t[-1] - t[0])
    k = max(3, int(smooth_s * hz) | 1)
    kernel = np.ones(k) / k
    gv = np.abs(np.gradient(goal, axis=0))
    out = []
    for j, name in enumerate(NAMES):
        moving = np.convolve(gv[:, j] > 1e-4, np.ones(k), "same") > 0
        pad = k // 2
        hp = meas[:, j] - np.convolve(np.pad(meas[:, j], pad, mode="edge"), kernel, "valid")
        m = moving[:len(hp)] & (np.arange(len(hp)) > pad) & (np.arange(len(hp)) < len(hp) - pad)
        if m.sum() < 16:
            out.append({"joint": name, "samples": int(m.sum())})
            continue
        x = hp[m] - hp[m].mean()
        spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        f = np.fft.rfftfreq(len(x), 1 / hz)
        band = f > 2.0
        out.append({"joint": name, "samples": int(m.sum()), "tremor_rms_deg": float(np.sqrt(np.mean(x**2))),
                    "dominant_hz": float(f[band][np.argmax(spec[band])]), "track_err_max_deg": float(np.abs(meas[m, j] - goal[m, j]).max())})
    return out


class Tracer:
    def __init__(self, svc):
        self.svc, self.rows, self._stop = svc, [], threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            q = astuple(self.svc.driver.read_joints())
            self.rows.append((time.perf_counter(), *q, *astuple(self.svc.ctl._last)))
            time.sleep(0.002)  # 제어 스레드의 쓰기에 통신 선을 양보한다

    def start(self) -> None:
        self._th.start()

    def result(self) -> list[dict] | None:
        """멈추고 관절별 떨림 요약을 돌려준다(기록이 너무 짧으면 None)."""
        self._stop.set()
        self._th.join(timeout=2)
        if len(self.rows) < 50:
            return None
        a = np.array(self.rows)
        return analyze(a[:, 0], a[:, 1:7], a[:, 7:13])

    def stop(self, out=print) -> None:
        self._stop.set()
        self._th.join(timeout=2)
        if len(self.rows) < 50:
            out("추적 기록이 너무 짧습니다.")
            return
        a = np.array(self.rows)
        path = self.svc.logger.dir / "trace.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", *[f"meas_{n}" for n in NAMES], *[f"goal_{n}" for n in NAMES]])
            w.writerows([[round(r[0] - a[0, 0], 4), *r[1:]] for r in a])
        out(f"추적: {len(a)}개, 초당 {(len(a) - 1) / (a[-1, 0] - a[0, 0]):.0f}회, 기록 {path}")
        for r in analyze(a[:, 0], a[:, 1:7], a[:, 7:13]):
            if "tremor_rms_deg" in r:
                out(f"  {r['joint']:<14} 떨림 RMS {r['tremor_rms_deg']:.3f}°  지배 주파수 {r['dominant_hz']:.0f}Hz  추종 오차 최대 {r['track_err_max_deg']:.2f}°  ({r['samples']}개)")
