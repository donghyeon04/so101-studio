"""동작 실행: 50Hz 스레드가 궤적을 샘플링해 장치에 쓴다 (arm-software-spec.md 6.4). 장치는 이 스레드만 쓴다."""
import threading
import time
from dataclasses import astuple, fields

import numpy as np

from ..core.types import JointState, MotionResult, ToolPose

JOINTS = [f.name for f in fields(JointState)]
MAX_STEP_DEG = 5.0  # 한 주기 최대 이동량 (spec 6.5)


class MotionHandle:
    def __init__(self):
        self._done = threading.Event()
        self.result: MotionResult | None = None

    def _finish(self, result: MotionResult) -> None:
        self.result = result
        self._done.set()

    def wait(self, timeout: float | None = None) -> MotionResult | None:
        self._done.wait(timeout)
        return self.result


class MotionController:
    def __init__(self, driver, kin=None, limits: dict[str, list[float]] | None = None, hz: int = 50):
        self.driver, self.kin, self.limits, self.dt = driver, kin, limits or {}, 1.0 / hz
        self.state = "idle"  # "idle" | "moving" | "stopped" | "fault"
        self._job = None  # (traj, handle, t0, kind)
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._last = driver.read_joints()  # 마지막으로 명령한 위치
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _final_tool(self) -> tuple[ToolPose, float | None]:
        if self.kin is None:
            return ToolPose(0, 0, 0, 0), None
        got = self.kin.fk(self.driver.read_joints())
        want = self.kin.fk(self._last)
        return got, float(np.linalg.norm([got.x_mm - want.x_mm, got.y_mm - want.y_mm, got.z_mm - want.z_mm]))

    def _result(self, status: str, t0: float) -> MotionResult:
        tool, err = self._final_tool()
        return MotionResult(status, time.perf_counter() - t0, tool, err)

    def _out_of_bounds(self, traj) -> bool:  # 시작점은 검사하지 않는다(받침에 기댄 자세가 한계 밖일 수 있고, 5차 궤적은 단조라 목표만 보면 된다)
        end = traj.sample(traj.duration_s)
        return any(not lo <= getattr(end, name) <= hi for name, (lo, hi) in self.limits.items())

    def execute(self, traj) -> MotionHandle:
        handle, now = MotionHandle(), time.perf_counter()
        with self._lock:
            if self.state == "stopped" or self.state == "fault":
                handle._finish(MotionResult(self.state, 0.0, ToolPose(0, 0, 0, 0)))
            elif self._job is not None:
                raise RuntimeError("이미 동작 중")
            elif self._out_of_bounds(traj):
                handle._finish(self._result("out_of_bounds", now))
            else:
                self._job, self.state = (traj, handle, now, "move"), "moving"
        return handle

    def stop(self) -> None:
        """0.3초 안에 감속 정지 후 현재 위치 유지."""
        from .trajectory import StopTrajectory

        with self._lock:
            if self._job is None:
                return
            traj, handle, t0, kind = self._job
            if kind == "stop":
                return
            now = time.perf_counter()
            v = traj.velocity(now - t0)
            handle._finish(self._result("stopped", t0))
            self._job = (StopTrajectory(np.array(astuple(self._last)), v), MotionHandle(), now, "stop")

    def estop(self) -> None:
        """즉시 현재 위치 고정. 토크는 끄지 않는다(끄면 팔이 떨어짐)."""
        with self._lock:
            if self._job is not None:
                self._job[1]._finish(self._result("stopped", self._job[2]))
            self._job, self.state = None, "stopped"
            self._last = self.driver.read_joints()
            self.driver.write_joints(self._last)

    def resume(self) -> None:
        with self._lock:
            if self.state == "stopped":
                self.state = "idle"

    def _loop(self) -> None:
        nxt = time.perf_counter()
        while not self._quit.is_set():
            nxt += self.dt
            with self._lock:
                if self._job is not None:
                    self._tick()
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _tick(self) -> None:
        traj, handle, t0, kind = self._job
        t = time.perf_counter() - t0
        q = traj.sample(t)
        if np.abs(np.array(astuple(q)) - np.array(astuple(self._last))).max() > MAX_STEP_DEG:
            self._job, self.state = None, "fault"
            handle._finish(self._result("fault", t0))
            return
        self.driver.write_joints(q)
        self._last = q
        if t >= traj.duration_s:
            self._job = None
            if self.state == "moving":
                self.state = "idle"
            handle._finish(self._result("ok" if kind == "move" else "stopped", t0))

    def close(self) -> None:
        self._quit.set()
        self._thread.join(timeout=1.0)
