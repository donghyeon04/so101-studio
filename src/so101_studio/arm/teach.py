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

RECORD_HZ = 20
SMOOTH = 5  # 재생 전에 이동평균 창(표본 수)
CLAMP_TOL_DEG = 3.0  # 손으로 움직인 자세가 관절 한계를 이만큼까지 넘으면 한계로 잘라 재생한다(받침 자세 근처 등). 더 넘으면 그대로 두어 안전 검사가 거부한다


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
        self._saved_torque = None
        self.last_cut = None  # until_limit 재생이 잘린 위치 (재생한 점 수, 전체 점 수, 이유)

    @property
    def recording(self) -> bool:
        return self._thread is not None

    def record_start(self, follow: bool = False, stiffness: float = 1.0) -> None:
        """관절값을 20Hz로 기록한다. 기본은 토크를 끈다(시작 전에 팔을 손으로 받쳐야 한다).
        follow=True면 토크를 켠 채 서보 토크 제한을 낮추고(cfg.teach_torque x stiffness) 손으로 밀어 움직이는 대로 목표를 따라가게 한다."""
        if self.recording:
            return
        self.ctl.stop()
        self._saved_torque = None
        if follow:
            drv = self.ctl.driver
            self._saved_torque = (drv.get_torque_limits(), drv.get_p(), drv.get_d())
            drv.set_torque(True)
            drv.set_torque_limits({n: int(min(self._saved_torque[0].get(n, 1000), max(50, v * stiffness))) for n, v in self.cfg.teach_torque.items()})
            p = {**self.cfg.teach_p, **(self.cfg.teach_p_gravity if self.ctl.gravity is not None else {})}
            drv.set_p({n: max(1, round(v * stiffness)) for n, v in p.items()})
            drv.set_d({n: max(0, round(v * stiffness)) for n, v in self.cfg.teach_d.items()})
            self.ctl.follow_dead_deg = self.cfg.teach_dead_deg
            if not self.ctl.follow(True):
                self._restore_servo(drv)
                raise RuntimeError("지금은 추종 교시를 시작할 수 없습니다 (동작 중이거나 정지·fault 상태)")
        else:
            self.ctl.driver.set_torque(False)
        self._rows, _ = [], self._stop.clear()

        def loop():
            while not self._stop.is_set():
                self._rows.append(list(astuple(self.ctl.driver.read_joints())))
                time.sleep(1.0 / RECORD_HZ)

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def _restore_servo(self, drv) -> None:
        limits, p, d = self._saved_torque
        drv.set_torque_limits(limits)
        drv.set_p(p)
        drv.set_d(d)
        self._saved_torque = None

    def record_stop(self, name: str) -> Path:
        """기록을 config/sequences/<name>.json으로 저장하고 토크를 켠다(현재 자세를 목표로 쓴 뒤 켜므로 튀지 않는다)."""
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        if self._saved_torque is not None:  # 추종 교시: 추종을 끄고 토크 제한을 되돌린다
            self.ctl.follow(False)
            self._restore_servo(self.ctl.driver)
        self.ctl.driver.set_torque(True)
        self.ctl.sync()
        self.dir.mkdir(parents=True, exist_ok=True)
        path = self.dir / f"{name}.json"
        path.write_text(json.dumps({"hz": RECORD_HZ, "joints": self._rows}, ensure_ascii=False), encoding="utf-8")
        return path

    def load(self, name: str) -> np.ndarray:
        data = json.loads((self.dir / f"{name}.json").read_text(encoding="utf-8"))
        return np.array(data["joints"], float), data["hz"]

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        q = q.copy()
        for i, (name, (lo, hi)) in enumerate((n, self.ctl.safety.limits[n]) for n in JOINTS if n in self.ctl.safety.limits):
            over = np.maximum(lo - q[:, i], q[:, i] - hi)
            if 0 < over.max() <= CLAMP_TOL_DEG:
                q[:, i] = np.clip(q[:, i], lo, hi)
        return q

    def replay(self, name: str, speed: float = 1.0, until_limit: bool = False) -> MotionResult:
        """기록을 다듬어 재생한다. 시작 전에 첫 자세로 MoveJ(저속)로 간 뒤 시간에 맞춰 따라간다.
        한계·작업 영역 밖이면 움직이지 않고 거부한다(execute의 안전 검사)."""
        q, hz = self.load(name)
        q = self._clamp(smooth(q))
        first = JointState(*q[0].tolist())
        self.last_cut = None
        if until_limit:  # 한계·바닥을 처음 넘는 자세 앞까지만 재생한다
            total = len(q)
            for i, row in enumerate(q):
                why = self.ctl.safety.check_goal(JointState(*row.tolist()), first)
                if why:
                    q, self.last_cut = q[:i], (i, total, why)
                    break
            if len(q) < 10:
                return MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0))
        r = self.ctl.execute(cfg_move_j(self.cfg, self.ctl.driver.read_joints(), first, 20)).wait()
        if r.status != "ok":
            return r
        return self.ctl.execute(SequenceTrajectory(np.arange(len(q)) / hz / speed, q)).wait()

    def run_sequence(self, name: str) -> MotionResult:
        return self.replay(name, 1.0)

    def names(self) -> list[str]:
        return sorted(p.stem for p in self.dir.glob("*.json"))

    def current(self) -> dict:
        return asdict(self.ctl.driver.read_joints())
