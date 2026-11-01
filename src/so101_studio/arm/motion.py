"""동작 실행: 50Hz 스레드가 궤적을 샘플링해 장치에 쓴다 (arm-software-spec.md 6.4). 장치는 이 스레드만 쓴다."""
import contextlib
import math
import threading
import time
from dataclasses import astuple, fields

import numpy as np

from ..core.types import JointState, MotionResult, ToolPose
from .safety import HealthMonitor, Safety

JOINTS = [f.name for f in fields(JointState)]
HEALTH_PERIOD_S = 0.5  # 온도·부하는 2Hz로만 읽는다 (spec 6.1)
DEADMAN_S = 0.3  # 원격 조종 입력이 이만큼 끊기면 정지 (spec 6.5)


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
    def __init__(self, driver, kin=None, safety: Safety | None = None, hz: int = 50):
        self.driver, self.kin, self.safety, self.dt = driver, kin, safety or Safety({}), 1.0 / hz
        self.health = HealthMonitor()
        self._next_health = 0.0
        self.state = "idle"  # "idle" | "moving" | "teleop" | "stopped" | "fault"
        self.jog_joint_max, self.jog_tool_max = 40.0, 60.0  # 원격 조종 최대 속도 (deg/s, mm/s)
        self._jog = None  # (mode, 속도 벡터, 마지막 입력 시각)
        self.fault: str | None = None
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

    def execute(self, traj) -> MotionHandle:
        handle, now = MotionHandle(), time.perf_counter()
        with self._lock:
            if self.state == "stopped" or self.state == "fault":
                handle._finish(MotionResult(self.state, 0.0, ToolPose(0, 0, 0, 0)))
            elif self._job is not None:
                raise RuntimeError("이미 동작 중")
            elif self.safety.check_path(traj):
                handle._finish(self._result("out_of_bounds", now))
            else:
                self._jog = None
                self._last = traj.sample(0)  # 새 궤적은 실제 위치에서 시작한다. 서보가 뒤처져 있어도 'step' fault로 오인하지 않게 기준을 옮긴다
                self._job, self.state = (traj, handle, now, "move"), "moving"
        return handle

    def sync(self) -> None:
        """손으로 움직이는 등 명령 없이 팔이 움직인 뒤, 마지막 명령 위치를 실제 위치로 맞춘다."""
        with self._lock:
            self._last = self.driver.read_joints()

    def jog(self, mode: str, velocity) -> bool:
        """원격 조종 한 걸음 (spec 6.7). mode: "joint"(관절 6개 deg/s) 또는 "cartesian"(작업대 x, y, z mm/s, yaw deg/s).
        입력이 DEADMAN_S 넘게 끊기면 멈춘다. 동작 중이거나 정지·fault 상태면 무시한다."""
        with self._lock:
            if self.state in ("stopped", "fault") or self._job is not None:
                return False
            self._jog, self.state = (mode, np.array(velocity, float), time.perf_counter()), "teleop"
            return True

    def retarget(self, goal: JointState, v_max: float, a_max: float, j_max: float) -> MotionHandle:
        """진행 중인 동작을 지금 위치·속도에서 새 목표로 매끄럽게 교체한다 (T11). 이전 동작의 결과는 preempted."""
        from .trajectory import RuckigTrajectory

        handle, now = MotionHandle(), time.perf_counter()
        with self._lock:
            if self.state in ("stopped", "fault"):
                handle._finish(MotionResult(self.state, 0.0, ToolPose(0, 0, 0, 0)))
                return handle
            v0, old = np.zeros(len(astuple(goal))), None
            if self._job is not None:
                old = self._job
                v0 = np.array(old[0].velocity(now - old[2]), float)
            new = RuckigTrajectory(np.array(astuple(self._last), float), np.array(astuple(goal), float), v_max, a_max, j_max, v0)
            if self.safety.check_path(new):
                handle._finish(self._result("out_of_bounds", now))
                return handle
            if old is not None:
                old[1]._finish(self._result("preempted", old[2]))
            self._jog = None
            self._job, self.state = (new, handle, now, "move"), "moving"
        return handle

    def stop(self) -> None:
        """0.3초 안에 감속 정지 후 현재 위치 유지."""
        from .trajectory import StopTrajectory

        with self._lock:
            self._jog = None
            if self.state == "teleop":
                self.state = "idle"
            if self._job is None:
                return
            traj, handle, t0, kind = self._job
            if kind == "stop":
                return
            now = time.perf_counter()
            v = traj.velocity(now - t0)
            handle._finish(self._result("stopped", t0))
            self._job = (StopTrajectory(np.array(astuple(self._last)), v), MotionHandle(), now, "stop")

    def wait_idle(self, timeout: float = 2.0) -> None:
        """감속 정지가 끝날 때까지 기다린다(stop() 직후에는 감속 궤적이 아직 실행 중이다)."""
        end = time.perf_counter() + timeout
        while self._job is not None and time.perf_counter() < end:
            time.sleep(0.01)

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
            if self.state in ("stopped", "fault"):
                self.state, self.fault = "idle", None
                self.health = HealthMonitor()

    def _loop(self) -> None:
        nxt = time.perf_counter()
        while not self._quit.is_set():
            nxt += self.dt
            with self._lock:
                try:
                    self._check_health()
                    if self._job is not None:
                        self._tick()
                    elif self._jog is not None:
                        self._tick_jog()
                except Exception:  # noqa: BLE001 통신 오류 등 어떤 실패든 0.5초 넘게 이어지면 fault
                    self._fault(self.health.comm_failed(time.perf_counter()))
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _fault(self, reason: str | None) -> None:
        if reason is None or self.state == "fault":
            return
        self.state = "fault"
        job, self._job = self._job, None
        if job is not None:
            job[1]._finish(MotionResult("fault", time.perf_counter() - job[2], ToolPose(0, 0, 0, 0)))
        self.fault = reason
        with contextlib.suppress(Exception):  # 통신이 끊긴 상태일 수 있다
            self.driver.write_joints(self._last)  # 현재 위치 유지 (토크는 끄지 않는다)

    def _check_health(self) -> None:
        now = time.perf_counter()
        if now < self._next_health or self.state == "fault":
            return
        self._next_health = now + HEALTH_PERIOD_S
        self._fault(self.health.update_health(self.driver.read_health(), now))
        if self._job is None:  # 움직이는 중에는 쓰기 성공만 통신 정상으로 본다
            self.health.comm_ok()

    def _tick(self) -> None:
        traj, handle, t0, kind = self._job
        t = time.perf_counter() - t0
        q = traj.sample(t)
        if self.safety.check_step(self._last, q):
            self._fault("step")
            return
        self._last = q  # 쓰기 실패도 시도한 위치로 센다(통신 끊김이 'step' fault로 오인되지 않게)
        self.driver.write_joints(q)
        self.health.comm_ok()
        if t >= traj.duration_s:
            self._job = None
            if self.state == "moving":
                self.state = "idle"
            handle._finish(self._result("ok" if kind == "move" else "stopped", t0))

    def _tick_jog(self) -> None:
        mode, v, t_in = self._jog
        if time.perf_counter() - t_in > DEADMAN_S:  # 데드맨: 입력이 끊기면 그 자리에서 멈춘다
            self._jog, self.state = None, "idle"
            return
        last = self._last
        if mode == "joint":
            nxt = np.array(astuple(last), float) + np.clip(v, -self.jog_joint_max, self.jog_joint_max) * self.dt
            for i, name in enumerate(JOINTS):
                if name in self.safety.limits:
                    lo, hi = self.safety.limits[name]
                    nxt[i] = min(max(nxt[i], lo), hi)  # 한계에서 멈춘다
            q = JointState(*nxt.tolist())
        else:
            vx, vy, vz, vyaw = (list(v) + [0.0] * 4)[:4]
            speed = math.hypot(vx, vy, vz)
            k = min(1.0, self.jog_tool_max / speed) if speed else 1.0
            q = self.kin.step(last, vx * k * self.dt, vy * k * self.dt, vz * k * self.dt, vyaw * self.dt)
            if q is None:
                return
        if self.safety.check_goal(q) or q == last:  # 작업 영역·한계 밖이면 움직이지 않는다
            return
        if self.safety.check_step(last, q):
            self._fault("step")
            return
        self._last = q
        self.driver.write_joints(q)
        self.health.comm_ok()

    def close(self) -> None:
        self._quit.set()
        self._thread.join(timeout=1.0)
