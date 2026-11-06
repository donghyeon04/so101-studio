"""동작 실행: 50Hz 스레드가 궤적을 샘플링해 장치에 쓴다 (arm-software-spec.md 6.4). 장치는 이 스레드만 쓴다."""
import contextlib
import math
import threading
import time
from dataclasses import astuple, fields, replace

import numpy as np

from ..core.blackbox import RingBuffer
from ..core.types import JointState, MotionResult, ToolPose
from .fault_text import describe
from .safety import HealthMonitor, Safety

JOINTS = [f.name for f in fields(JointState)]
HEALTH_PERIOD_S = 0.5  # 온도·부하는 2Hz로만 읽는다 (spec 6.1)
DEADMAN_S = 0.3
STEP_MARGIN = 0.95  # 따라잡기로 진행할 때 한 주기 이동량 한도의 이 비율을 넘지 않는다
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
        self.fault_text: str | None = None  # 마지막 fault의 사람이 읽을 문장(원인과 해결법)
        self.fault_detail: dict = {}  # 마지막 fault의 상세(관절, 값, 한도)
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
        self.trace = None  # 틱마다 trace(perf_counter_ns, 보낸 관절값)을 받는 함수. 버스를 읽지 않는다(작업 실행이 팔 링크 위치 기록을 남길 때)
        self.tap = None  # 틱마다 tap(읽은 시각 perf_counter_ns, 실제 관절값, 읽는 데 걸린 ns)를 받는 함수(경로 따라가기가 센서 궤적을 모을 때). 설정했을 때만 틱에서 버스를 읽는다
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
                handle._finish(MotionResult("locked", 0.0, ToolPose(0, 0, 0, 0), reason=describe("locked", {"locked_reason": self.locked_reason})))
            elif self.state == "stopped" or self.state == "fault":
                handle._finish(MotionResult(self.state, 0.0, ToolPose(0, 0, 0, 0), reason=self.fault_text if self.state == "fault" else None))
            elif self._job is not None:
                raise RuntimeError("이미 동작 중")
            elif (why := self.safety.check_path(traj)):
                self._emit("out_of_bounds", "WARN", "ARM-OUT-OF-BOUNDS", {"reason": why, "text": describe(why)})  # 사람이 읽을 문장도 같이 남긴다
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
                self.state, self.fault, self.fault_text, self.fault_detail = "idle", None, None, {}
                self.health = HealthMonitor()
            return True

    @property
    def busy(self) -> bool:
        """동작이나 조그가 진행 중인가(작업대 설정을 다시 읽어도 되는지 볼 때)."""
        return self._job is not None or self._jog is not None

    def note_comm(self, ok: bool) -> None:
        """제어 틱 밖(10Hz 상태 루프)에서 본 읽기 성공·실패를 건강 감시에 알린다. 정지 중에도, 움직이는 중에도 읽기 시계를 따로 센다."""
        with self._lock:
            if ok:
                self.health.comm_ok(time.perf_counter(), "read")
            elif self.state != "fault":
                self._fault(self.health.comm_failed(time.perf_counter(), "read"))

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
                kind = "read"
                try:
                    self._check_health()
                    kind = "write"
                    if self._job is not None:
                        self._tick()
                    elif self._jog is not None:
                        self._tick_jog()
                except Exception as e:  # noqa: BLE001 통신 오류 등 어떤 실패든 0.5초 넘게 이어지면 fault
                    err = repr(e)[:120]
                    self._fault(self.health.comm_failed(time.perf_counter(), kind))
            self.ring.add(time.time_ns(), self._last, tick_dt, err, self.state, self.actual_hint)
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _fault(self, reason: str | None, detail: dict | None = None) -> None:
        """fault로 들어간다. detail(관절·값·한도 등)과 사람이 읽을 문장(text, 해결법 포함)을 이벤트·진행 중이던 동작의 MotionResult.reason·fault_text에 담는다 (REQ-034)."""
        if reason is None or self.state == "fault":
            return
        self.state = "fault"
        text = describe(reason, detail)
        job, self._job = self._job, None
        if job is not None:
            job[1]._finish(MotionResult("fault", time.perf_counter() - job[2], ToolPose(0, 0, 0, 0), reason=text))
        self.fault, self.fault_text, self.fault_detail = reason, text, detail or {}
        self._jog = None  # 조그 중이던 입력이 fault 뒤에도 명령을 보내지 않게
        self._emit("fault", "ERROR", "ARM-FAULT", {"reason": reason, "job": job is not None, "detail": detail or {}, "text": text})
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
            self._fault(self.health.comm_lost(now))
        if now < self._next_health or self.state == "fault":
            return
        self._next_health = now + HEALTH_PERIOD_S
        h = self.driver.read_health()
        why = self.health.update_health(h, now)
        self._fault(why, self._health_detail(why, h))
        self._check_grip(h, now)
        self.health.comm_ok(time.perf_counter(), "read")  # 건강 읽기가 됐다(읽기 시계만 지운다)

    def _health_detail(self, reason: str | None, h: dict) -> dict | None:
        """과열·과부하·과전류 fault의 관절, 값, 한도 (사람이 읽을 문장에 쓰인다)."""
        if reason is None:
            return None
        name, _, joint = reason.partition(":")
        m = self.health
        if name == "overheat":
            return {"joint": joint, "temp_c": h["temperature_c"].get(joint), "limit_c": m.temp_limit_c}
        if name == "overload":
            return {"joint": joint, "load_pct": h["load_pct"].get(joint), "limit_pct": m.load_limit_pct, "seconds": m.load_s}
        if name == "overcurrent":
            return {"current_ma": h.get("current_ma"), "limit_ma": m.current_limit_ma}
        return None

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
            self._fault("grip_overload", {"load_pct": load, "limit_pct": self.grip_warn_pct})

    def _step_over(self, q: JointState) -> float:
        """마지막 명령에서 q까지 관절 하나가 움직이는 최대 변화(°)."""
        return max(abs(a - b) for a, b in zip(astuple(q), astuple(self._last), strict=True))

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
        q = traj.sample(t)
        if self.grip_hold is not None:
            q = replace(q, gripper=self.grip_hold)
        adv = t - prev
        if adv > self.dt * 1.001 and self._step_over(q) > STEP_MARGIN * self.safety.max_step_deg:
            # 따라잡는 몫(한 주기 넘는 진행)이 한 주기 이동량 한도의 95%를 넘기게 한다면 그만큼만 진행한다: 루프가 늦어도 step fault 대신 동작이 늦어진다.
            # 평소 한 주기(dt) 진행으로도 넘는 궤적은 그대로 두어 step 검사가 잡는다(안전망은 유지).
            lo, hi = self.dt, adv
            for _ in range(8):  # 이분 탐색: 한도 안인 가장 큰 진행
                mid = (lo + hi) / 2
                qm = traj.sample(prev + mid)
                if self.grip_hold is not None:
                    qm = replace(qm, gripper=self.grip_hold)
                lo, hi = (mid, hi) if self._step_over(qm) <= STEP_MARGIN * self.safety.max_step_deg else (lo, mid)
            lost = adv - lo
            t0 += lost
            self.time_lost_s += lost
            self._job = (traj, handle, t0, kind)
            t = prev + lo
            q = traj.sample(t)
            if self.grip_hold is not None:
                q = replace(q, gripper=self.grip_hold)
        self._tick_ref = (handle, t)
        if self.safety.check_step(self._last, q):
            self._fault("step", self.safety.step_detail(self._last, q))
            return
        self._last = q  # 쓰기 실패도 시도한 위치로 센다(통신 끊김이 'step' fault로 오인되지 않게)
        self.driver.write_joints(q)
        if self.trace is not None:
            self.trace(time.perf_counter_ns(), q)
        if self.tap is not None:  # 실제 관절값을 읽어 넘긴다(읽는 데 걸린 시간도 함께, 틱 주기를 지키는지 보려고)
            r0 = time.perf_counter_ns()
            actual = self.driver.read_joints()  # 실패하면 예외: 통신 정상 표시(아래)를 건너뛰어 끊김 시계가 이어지고 0.5초 안에 comm_lost가 된다
            r1 = time.perf_counter_ns()
            self.tap((r0 + r1) // 2, actual, r1 - r0)
            self.health.comm_ok(time.perf_counter(), "read")
        self.health.comm_ok(time.perf_counter(), "write")
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
            self._fault("step", self.safety.step_detail(last, q))
            return
        self.jog_block = None
        self._last = q
        self.driver.write_joints(q)
        self.health.comm_ok(time.perf_counter(), "write")

    def close(self) -> None:
        self._quit.set()
        self._thread.join(timeout=1.0)
        with self._lock:  # 진행 중인 동작을 끝내 wait() 중인 스레드가 영원히 남지 않게 한다 (REQ-014)
            if self._job is not None:
                _, handle, t0, _ = self._job
                self._job = None
                handle._finish(MotionResult("stopped", time.perf_counter() - t0, ToolPose(0, 0, 0, 0)))
