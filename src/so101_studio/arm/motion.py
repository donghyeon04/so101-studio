"""동작 실행: 50Hz 스레드가 궤적을 샘플링해 장치에 쓴다 (arm-software-spec.md 6.4). 장치는 이 스레드만 쓴다."""
import contextlib
import math
import threading
import time
from dataclasses import astuple, fields, replace

import numpy as np

from ..core.blackbox import RingBuffer
from ..core.types import JointState, MotionResult, ToolPose
from .safety import HealthMonitor, Safety

JOINTS = [f.name for f in fields(JointState)]
HEALTH_PERIOD_S = 0.5  # 온도·부하는 2Hz로만 읽는다 (spec 6.1)
DEADMAN_S = 0.3
MAX_TICK_ADVANCE = 2.0  # 한 틱에 궤적 시간이 최대 이만큼(틱 주기의 배수) 진행. 틱이 크게 늦어도 위치가 한 번에 건너뛰어 'step' fault가 나지 않고, 늦은 시간만큼 동작이 늦어진다
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
        self.state = "idle"  # "idle" | "moving" | "teleop" | "stopped" | "fault"
        self.jog_joint_max, self.jog_tool_max = 40.0, 60.0  # 원격 조종 최대 속도 (deg/s, mm/s)
        self._jog = None  # (mode, 속도 벡터, 마지막 입력 시각)
        self._jog_v = np.zeros(6)  # 가속도 제한을 거친 지금 조그 속도
        self._jog_t = 0.0  # 직전 조그 틱 시각
        self.jog_block = None  # 조그가 막힌 이유(풀리지 않음 'ik_fail' 또는 안전 검사 이유). 움직이면 None
        self.fault: str | None = None
        self.grip_hold: float | None = None  # 들고 있는 동안 그리퍼 목표(실제보다 예압만큼 닫힌 값). 궤적이 무엇을 주든 매 틱 이 값으로 덮는다 (결정 0033)
        self.grip_warn_pct: float | None = None  # 그리퍼 부하 감시(Skills가 설정에서 채운다). None이면 끔
        self.grip_load_s, self.grip_open = 1.0, 60.0
        self._grip_over_since: float | None = None
        self._grip_warned_at: float | None = None
        self._job = None  # (traj, handle, t0, kind)
        self.locked_reason: str | None = None  # 이 이유가 있으면 움직이는 명령을 모두 거부한다(status "locked"). ArmService.connect가 정한다 (decisions/0027)
        self.ring = RingBuffer()  # 50Hz 틱의 최근 10초(명령, 실제, 틱 간격, 버스 오류). 사고 묶음이 쓴다 (decisions/0030)
        self.on_event = None  # on_event(kind, level, code, data): fault·E-STOP 같은 이벤트를 기록하는 쪽(ArmService)이 연결한다
        self.actual_hint = None  # 가장 최근에 읽은 실제 관절값(10Hz 상태 루프가 채운다). 틱 안에서는 버스를 읽지 않는다
        self._tick_ref = None  # (handle, 직전 틱의 궤적 시간)
        self.time_lost_s = 0.0  # 틱 지연 때문에 늦춘 동작 시간의 누적(진단용: 크면 제어 주기가 밀리고 있다)
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

    def _result(self, status: str, t0: float, needs_read: bool = False) -> MotionResult:
        """동작의 결과. 버스를 못 읽어도 결과는 반드시 돌려준다: job을 비운 뒤 여기서 예외가 나면 handle이 끝나지 못해 wait()가 영원히 안 돌아온다(REQ-026).
        읽기 실패는 끊김의 시작으로 건강 감시에 알리고(0.5초 뒤 comm_lost), needs_read(정상 끝)이면 status를 fault로 바꿔 돌려준다. 손끝 위치는 모른다(0)."""
        try:
            tool, err = self._final_tool()
        except Exception as e:  # noqa: BLE001 포트 빠짐·응답 없음 등 어떤 읽기 실패든
            self.health.comm_failed(time.perf_counter())
            self._emit("final_read_failed", "WARN", "ARM-FINAL-READ-FAILED", {"status": status, "error": repr(e)[:120]})
            tool, err = ToolPose(0, 0, 0, 0), None
            if needs_read:
                status = "fault"
        return MotionResult(status, time.perf_counter() - t0, tool, err)

    def execute(self, traj) -> MotionHandle:
        handle, now = MotionHandle(), time.perf_counter()
        with self._lock:
            if self.locked_reason:
                handle._finish(MotionResult("locked", 0.0, ToolPose(0, 0, 0, 0)))
            elif self.state == "stopped" or self.state == "fault":
                handle._finish(MotionResult(self.state, 0.0, ToolPose(0, 0, 0, 0)))
            elif self._job is not None:
                raise RuntimeError("이미 동작 중")
            elif (why := self.safety.check_path(traj)):
                handle._finish(replace(self._result("out_of_bounds", now), reason=why))
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
            if self.locked_reason or self.state in ("stopped", "fault") or self._job is not None:
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
            if self.locked_reason:
                handle._finish(MotionResult("locked", 0.0, ToolPose(0, 0, 0, 0)))
                return handle
            if self.state in ("stopped", "fault"):
                handle._finish(MotionResult(self.state, 0.0, ToolPose(0, 0, 0, 0)))
                return handle
            v0, old = np.zeros(len(astuple(goal))), None
            if self._job is not None:
                old = self._job
                v0 = np.array(old[0].velocity(now - old[2]), float)
            new = RuckigTrajectory(np.array(astuple(self._last), float), np.array(astuple(goal), float), v_max, a_max, j_max, v0)
            if (why := self.safety.check_path(new)):
                handle._finish(replace(self._result("out_of_bounds", now), reason=why))
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
        err = None
        with self._lock:
            if self._job is not None:
                try:
                    res = self._result("stopped", self._job[2])
                except Exception:  # noqa: BLE001 버스를 못 읽어도(포트 빠짐) 동작은 끝낸다
                    res = MotionResult("stopped", time.perf_counter() - self._job[2], ToolPose(0, 0, 0, 0))
                self._job[1]._finish(res)
            self._job = None
            if not (self.state == "fault" and self.fault == "comm_lost"):  # 통신 끊김 fault는 stopped로 덮지 않는다: 다시 연결해야만 풀린다 (결정 0031)
                self.state = "stopped"
            try:
                last = self.driver.read_joints()
                if self.grip_hold is not None:  # 예압 고정과 조임 한계는 유지한다(정지가 물체를 놓치게 하지 않는다)
                    last = replace(last, gripper=self.grip_hold)
                self.driver.write_joints(last)
                self._last = last
            except Exception as e:  # noqa: BLE001 통신이 끊긴 상태: 예외를 밖으로 내지 않고 기록만 한다. 끊김은 상태 루프와 틱이 fault로 본다
                err = repr(e)[:120]
        self._emit("estop", "WARN", "ARM-ESTOP", {"comm_error": err} if err else {})

    def resume(self) -> bool:
        """stop·estop·fault 뒤 idle로 되돌린다. 통신 끊김 fault는 다시 연결해야 풀리므로 False (decisions/0031)."""
        with self._lock:
            if self.state == "fault" and self.fault == "comm_lost":
                return False
            if self.state in ("stopped", "fault"):
                self.state, self.fault = "idle", None
                self.health = HealthMonitor()
            return True

    def note_comm(self, ok: bool) -> None:
        """제어 틱 밖(10Hz 상태 루프)에서 본 통신 성공·실패를 건강 감시에 알린다. 정지 중에도 끊김을 빨리 본다."""
        with self._lock:
            if ok:
                if self._job is None:  # 움직이는 중에는 쓰기 성공만 통신 정상으로 본다
                    self.health.comm_ok(time.perf_counter())
            elif self.state != "fault":
                self._fault(self.health.comm_failed(time.perf_counter()))

    def _emit(self, kind: str, level: str, code: str, data: dict) -> None:
        cb = self.on_event
        if cb:
            try:
                cb(kind, level, code, data)
            except Exception:  # noqa: BLE001, S110 기록이 실패해도 제어는 계속한다
                pass

    def tick_dt_max_1s(self) -> float:
        """직전 1초 틱 간격의 최대값(초). 제어 주기가 밀리는지 본다."""
        return self.ring.max_dt(1.0)

    def _loop(self) -> None:
        nxt = prev = time.perf_counter()
        while not self._quit.is_set():
            nxt += self.dt
            now = time.perf_counter()
            tick_dt, prev, err = now - prev, now, None
            with self._lock:
                try:
                    self._check_health()
                    if self._job is not None:
                        self._tick()
                    elif self._jog is not None:
                        self._tick_jog()
                except Exception as e:  # noqa: BLE001 통신 오류 등 어떤 실패든 0.5초 넘게 이어지면 fault
                    err = repr(e)[:120]
                    self._fault(self.health.comm_failed(time.perf_counter()))
            self.ring.add(time.time_ns(), self._last, tick_dt, err, self.state, self.actual_hint)
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _fault(self, reason: str | None) -> None:
        if reason is None or self.state == "fault":
            return
        self.state = "fault"
        job, self._job = self._job, None
        if job is not None:
            job[1]._finish(MotionResult("fault", time.perf_counter() - job[2], ToolPose(0, 0, 0, 0)))
        self.fault = reason
        self._jog = None  # 조그 중이던 입력이 fault 뒤에도 명령을 보내지 않게
        self._emit("fault", "ERROR", "ARM-FAULT", {"reason": reason, "job": job is not None})
        if reason == "comm_lost":  # 통신이 끊겼다: 다시 연결하기 전까지 아무 명령도 보내지 않는다 (decisions/0031)
            self._emit("comm_lost", "ERROR", "ARM-COMM-LOST", {"job": job is not None})
            return
        with contextlib.suppress(Exception):
            self.driver.write_joints(self._last)  # 현재 위치 유지 (토크는 끄지 않는다)

    def _check_health(self) -> None:
        now = time.perf_counter()
        if self.state == "fault":
            return
        if self.health._comm_down_since is not None:  # 상태 루프가 끊김을 알렸다: 0.5초가 차는 틱에서 바로 fault
            self._fault(self.health.comm_failed(now))
        if now < self._next_health or self.state == "fault":
            return
        self._next_health = now + HEALTH_PERIOD_S
        h = self.driver.read_health()
        self._fault(self.health.update_health(h, now))
        self._check_grip(h, now)
        if self._job is None:  # 움직이는 중에는 쓰기 성공만 통신 정상으로 본다
            self.health.comm_ok(time.perf_counter())

    def hold_gripper(self, value: float | None) -> None:
        """그리퍼 목표를 value로 고정한다(None이면 풀어 궤적을 따른다). 지금 바로 한 번 쓴다."""
        with self._lock:
            self.grip_hold = value
            if value is not None and self.state not in ("fault", "stopped"):
                self._last = replace(self._last, gripper=value)
                self.driver.write_joints(self._last)

    def _check_grip(self, health: dict, now: float) -> None:
        """그리퍼 부하 두 단계 (결정 0033): 기준을 gripper_load_s 넘게 넘으면 목표를 지금 위치로 맞춰 조임을 멈추고 WARN,
        그래도 1초 더 이어지면 그리퍼를 열고 fault."""
        if self.grip_warn_pct is None or self.state == "fault":
            return
        load = health.get("load_pct", {}).get("gripper", 0.0)
        if load < self.grip_warn_pct:
            self._grip_over_since = self._grip_warned_at = None
            return
        self._grip_over_since = now if self._grip_over_since is None else self._grip_over_since
        if self._grip_warned_at is None:
            if now - self._grip_over_since >= self.grip_load_s:
                self._grip_warned_at = now
                with contextlib.suppress(Exception):
                    self.grip_hold = self.driver.read_joints().gripper  # 조임을 멈춘다: 목표 = 지금 위치
                    self._last = replace(self._last, gripper=self.grip_hold)
                    self.driver.write_joints(self._last)
                self._emit("grip_overload_warn", "WARN", "ARM-GRIP-OVERLOAD-WARN", {"load_pct": load, "limit_pct": self.grip_warn_pct})
        elif now - self._grip_warned_at >= 1.0:
            self.grip_hold = None
            self._last = replace(self._last, gripper=self.grip_open)  # fault가 이 값을 써서 연다
            self._emit("grip_overload", "ERROR", "ARM-GRIP-OVERLOAD", {"load_pct": load, "limit_pct": self.grip_warn_pct})
            self._fault("grip_overload")

    def _tick(self) -> None:
        traj, handle, t0, kind = self._job
        t = time.perf_counter() - t0
        ref = self._tick_ref
        prev = ref[1] if ref is not None and ref[0] is handle else 0.0
        cap = MAX_TICK_ADVANCE * self.dt
        if t - prev > cap:  # 틱이 늦었다: 궤적 시간을 cap만큼만 진행하고 나머지는 동작을 늦춘다
            lost = t - prev - cap
            t0 += lost
            self.time_lost_s += lost
            self._job = (traj, handle, t0, kind)
            t = prev + cap
        self._tick_ref = (handle, t)
        q = traj.sample(t)
        if self.grip_hold is not None:
            q = replace(q, gripper=self.grip_hold)
        if self.safety.check_step(self._last, q):
            self._fault("step")
            return
        self._last = q  # 쓰기 실패도 시도한 위치로 센다(통신 끊김이 'step' fault로 오인되지 않게)
        self.driver.write_joints(q)
        self.health.comm_ok(time.perf_counter())
        if t >= traj.duration_s:
            self._job = None
            if self.state == "moving":
                self.state = "idle"
            handle._finish(self._result("ok" if kind == "move" else "stopped", t0, needs_read=True))

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
        self.health.comm_ok(time.perf_counter())

    def close(self) -> None:
        self._quit.set()
        self._thread.join(timeout=1.0)
        with self._lock:  # 진행 중인 동작을 끝내 wait() 중인 스레드가 영원히 남지 않게 한다 (REQ-014)
            if self._job is not None:
                _, handle, t0, _ = self._job
                self._job = None
                handle._finish(MotionResult("stopped", time.perf_counter() - t0, ToolPose(0, 0, 0, 0)))
