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
DEADMAN_S = 0.3
JOG_ACCEL_JOINT = 400.0  # °/s². 조그 속도를 이 가속도로 올리고 내려 시작·정지 충격(속도 계단)을 없앤다
JOG_ACCEL_TOOL = 600.0  # mm/s², 회전은 같은 값을 °/s²로  # 원격 조종 입력이 이만큼 끊기면 정지 (spec 6.5)


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
        self.state = "idle"  # "idle" | "moving" | "teleop" | "follow" | "stopped" | "fault"
        self.jog_joint_max, self.jog_tool_max = 40.0, 60.0  # 원격 조종 최대 속도 (deg/s, mm/s)
        self._jog = None  # (mode, 속도 벡터, 마지막 입력 시각)
        self._jog_v = np.zeros(6)  # 가속도 제한을 거친 지금 조그 속도
        self._jog_t = 0.0  # 직전 조그 틱 시각
        self.assist_deg = np.zeros(6)  # 추종 교시 방향 보조 크기(°, 관절별). 0이면 끔
        self.assist_gain = 1.0
        self.assist_v_on, self.assist_v_max = 3.0, 40.0  # °/s. 이 속도부터 보조, 이 속도에서는 보조 0(저절로 달아나지 않게)
        self._prev_present, self._prev_t, self._vel = None, 0.0, np.zeros(6)
        self.gravity = None  # 중력 보상 모델(arm/gravity.py). 있으면 추종 교시에서 목표에 중력분을 미리 더한다
        self.gravity_gain = 1.0
        self.follow_dead_deg = 1.0  # 추종 교시: 목표와 실제 위치가 이만큼 넘게 어긋나면 손으로 민 것으로 보고 목표를 따라 옮긴다
        self._follow = False
        self.follow_block = None  # 추종이 안전 검사에 막힌 이유
        self.jog_block = None  # 조그가 막힌 이유(풀리지 않음 'ik_fail' 또는 안전 검사 이유). 움직이면 None
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

    def follow(self, on: bool) -> bool:
        """추종 교시: 토크를 켠 채 손으로 밀어 위치가 어긋나면 목표를 그 위치로 따라 옮긴다. 한계·바닥 검사는 그대로 적용된다.
        동작 중이거나 정지·fault 상태에서는 시작하지 않는다."""
        with self._lock:
            if on and (self.state in ("stopped", "fault") or self._job is not None):
                return False
            self._jog = None
            self._follow, self.follow_block = on, None
            self._prev_present, self._vel = None, np.zeros(6)
            self.state = "follow" if on else ("idle" if self.state == "follow" else self.state)
            if on:
                self._last = self.driver.read_joints()
                self.driver.write_joints(self._last)
            return True

    def jog(self, mode: str, velocity) -> bool:
        """원격 조종 한 걸음 (spec 6.7). mode: "joint"(관절 6개 deg/s) 또는 "cartesian"(작업대 x, y, z mm/s, yaw deg/s).
        입력이 DEADMAN_S 넘게 끊기면 멈춘다. 동작 중이거나 정지·fault 상태면 무시한다."""
        with self._lock:
            if self.state in ("stopped", "fault", "follow") or self._job is not None:
                return False
            if self._jog is None:
                self._jog_v = np.zeros(6)
                self._jog_t = time.perf_counter() - self.dt
            self._jog, self.state = (mode, np.array(velocity, float), time.perf_counter()), "teleop"
            return True

    def jog_stop(self) -> None:
        """키를 뗐을 때: 조그 속도를 가속도 한계로 0까지 내려 멈추고 그 자리를 유지한다."""
        with self._lock:
            if self._jog is not None:  # 속도를 0으로 내려 부드럽게 멈춘다(바로 끊지 않는다)
                self._jog = (self._jog[0], np.zeros_like(self._jog[1]), time.perf_counter())

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
            if self.state in ("teleop", "follow"):
                self._follow, self.state = False, "idle"
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
            self._job, self.state, self._follow = None, "stopped", False
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
                    elif self._follow:
                        self._tick_follow()
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

    def _assist(self, present: tuple) -> np.ndarray:
        """손으로 움직이는 방향(측정 속도의 부호)으로 목표를 앞에 두는 각도. 속도가 빠를수록 줄어 assist_v_max에서 0이 된다."""
        now = time.perf_counter()
        p = np.array(present, float)
        if self._prev_present is not None and now > self._prev_t:
            raw = (p - self._prev_present) / (now - self._prev_t)
            self._vel = 0.5 * self._vel + 0.5 * raw
        else:
            self._vel = np.zeros(6)
        self._prev_present, self._prev_t = p, now
        speed = np.abs(self._vel)
        scale = np.clip(1.0 - speed / self.assist_v_max, 0.0, 1.0) * (speed > self.assist_v_on)
        return np.sign(self._vel) * self.assist_deg * self.assist_gain * scale

    def _tick_follow(self) -> None:
        present = astuple(self.driver.read_joints())
        goal = list(astuple(self._last))
        for i, p in enumerate(present):
            if abs(p - goal[i]) > self.follow_dead_deg:
                goal[i] = p  # 손으로 밀린 만큼 목표를 옮긴다
        q = JointState(*goal)
        if q != self._last:
            why = self.safety.check_goal(q, self._last) or self.safety.check_step(self._last, q)
            self.follow_block = why
            if not why:
                self._last = q
        cmd = np.array(astuple(self._last), float)
        if self.gravity is not None:  # 팔 무게를 버티는 만큼 목표를 미리 앞에 둔다(P를 낮춰도 처지지 않게)
            cmd = cmd + self.gravity.ff_deg(JointState(*present), self.gravity_gain)
        lead = self._assist(present)
        if lead.any():
            with_assist = JointState(*(cmd + lead).tolist())
            if not self.safety.check_goal(with_assist, self._last):  # 한계·바닥을 넘는 보조는 하지 않는다
                cmd = cmd + lead
        self.driver.write_joints(JointState(*cmd.tolist()))
        self.health.comm_ok()

    def _tick_jog(self) -> None:
        mode, v, t_in = self._jog
        target = v if time.perf_counter() - t_in <= DEADMAN_S else np.zeros_like(v)  # 데드맨: 입력이 끊기면 속도를 0으로 내린다
        now = time.perf_counter()  # 틱 간격이 들쭉날쭉해도(이 기록에서 표준편차 8ms) 속도가 일정하도록 실제 경과 시간으로 적분한다
        dt = min(max(now - self._jog_t, 0.0005), 3 * self.dt)
        self._jog_t = now
        step = (JOG_ACCEL_JOINT if mode == "joint" else JOG_ACCEL_TOOL) * dt
        cur = self._jog_v[: len(v)]
        dv = target - cur
        n = float(np.linalg.norm(dv))
        cur = target.copy() if n <= step else cur + dv * (step / n)
        self._jog_v[: len(v)] = cur
        if not cur.any() and not target.any():
            self._jog, self.state = None, "idle"
            return
        v = cur
        last = self._last
        if mode == "joint":
            nxt = np.array(astuple(last), float) + np.clip(v, -self.jog_joint_max, self.jog_joint_max) * dt
            limits = self.safety.limits
            for i, name in enumerate(JOINTS):
                if name in limits:
                    lo, hi = limits[name]
                    lo, hi = min(lo, getattr(last, name)), max(hi, getattr(last, name))  # 한계 밖에서 출발해도 안쪽으로는 움직일 수 있다
                    nxt[i] = min(max(nxt[i], lo), hi)  # 한계에서 멈춘다
            q = JointState(*nxt.tolist())
        else:
            vx, vy, vz, vyaw = (list(v) + [0.0] * 4)[:4]
            speed = math.hypot(vx, vy, vz)
            k = min(1.0, self.jog_tool_max / speed) if speed else 1.0
            q = self.kin.step(last, vx * k * dt, vy * k * dt, vz * k * dt, vyaw * dt)
            if q is None:
                self.jog_block = "ik_fail"
                return
        why = self.safety.check_goal(q, last)
        if why or q == last:  # 작업 영역·한계 밖이면 움직이지 않는다
            self.jog_block = why or self.jog_block
            return
        if self.safety.check_step(last, q):
            self._fault("step")
            return
        self.jog_block = None
        self._last = q
        self.driver.write_joints(q)
        self.health.comm_ok()

    def close(self) -> None:
        self._quit.set()
        self._thread.join(timeout=1.0)
