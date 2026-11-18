"""시뮬레이션 시계 StepClock (mujoco-spec 13.10.2, 결정 0070): `MuJoCoDriver(realtime=False)`의 시간을 틱 스레드가 `advance`로 흘린다.

- `now()`는 이 시계가 실제로 `advance`한 스텝 수 × 2ms다(0에서 시작, `driver.data.time`은 읽지 않는다).
- 번갈아 돌기(lockstep): 참여 스레드(만든 스레드 + `with clock.participate():`)가 모두 `sleep`·`wait` 안에 있을 때만 틱 스레드가 시간을 흘린다.
  계산하는 동안(IK, 접촉 판단)에는 시간이 멈춰 있어 같은 씨앗이면 결과가 같다.
- `every(period_s, fn)`: 시뮬레이션 시간이 period_s의 배수를 지날 때마다 틱 스레드가 fn()을 부른다(표본기·기록기·정책 한 걸음·상태 한 바퀴).
- 멈춤 감지: 기다리는 동안 `driver.failed_reason`이면 바로 RuntimeError. 참여 스레드가 모두 기다리는데 틱 스레드가 벽시계 stall_s 동안
  `tick_sleep`에 들어오지 않으면 RuntimeError(틱 스레드가 죽었거나 막힘). 참여 스레드가 계산하는 시간은 세지 않는다.
규약 이름(now, now_ns, sleep, wait, tick_sleep, participate, drives_status)은 core/clock.py의 Clock(팔 영역, REQ-082)과 같다."""
import contextlib
import math
import threading
import time

STEP_S = 0.002  # mujoco_model.load_model의 timestep(2ms)
STALL_S = 5.0
POLL_S = 0.002  # 이벤트 기다림을 다시 보는 벽시계 간격(Event.set은 이 시계의 조건 변수를 깨우지 않는다)


class StepClock:
    drives_status = True

    def __init__(self, driver, stall_s: float = STALL_S):
        if getattr(driver, "realtime", True):
            raise ValueError("StepClock은 MuJoCoDriver(realtime=False)와만 쓴다")
        if getattr(driver, "viewer", False):
            raise ValueError("StepClock은 수동 뷰어(--viewer)와 함께 쓸 수 없다(--viewer grid를 쓴다)")
        self.driver, self.stall_s = driver, stall_s
        self._steps, self._carry = 0, 0.0
        self._cv = threading.Condition()
        self._parts: dict[int, list] = {threading.get_ident(): [1, []]}  # 스레드 → [참여 깊이, 기다림 조건 쌓기 [(깨어날 시각|None, 이벤트|None)]]. 만든 스레드는 저절로 참여
        self._tick_ident: int | None = None
        self._tick_wall = time.monotonic()  # 틱 스레드가 마지막으로 tick_sleep에 들어온 벽시계
        self._idle_since: float | None = None  # 참여 스레드가 모두 기다리기 시작한 벽시계
        self._every: list[list] = []  # [주기, fn, 다음 시각]
        self._closed = False

    # -- 시각 --
    def now(self) -> float:
        return self._steps * STEP_S

    def now_ns(self) -> int:
        return int(self._steps * STEP_S * 1e9)

    # -- 참여 --
    @contextlib.contextmanager
    def participate(self):
        me = threading.get_ident()
        with self._cv:
            self._parts.setdefault(me, [0, []])[0] += 1
            self._update_idle()
        try:
            yield self
        finally:
            with self._cv:
                p = self._parts[me]
                p[0] -= 1
                if p[0] == 0:
                    del self._parts[me]
                self._update_idle()
                self._cv.notify_all()

    def _ready(self, cond) -> bool:
        """기다림 조건이 찼는가(시각이 됐거나 이벤트가 섰다): 그 스레드는 곧 깨어나므로 틱은 기다린다."""
        deadline, event = cond
        return (deadline is not None and self.now() >= deadline - 1e-12) or (event is not None and event.is_set())

    def _all_waiting(self) -> bool:
        """참여 스레드가 모두 기다리고, 그 누구의 깨어날 조건도 아직 차지 않았다(그래야 틱이 깨어날 시각을 지나쳐 달리지 않는다)."""
        return bool(self._parts) and all(conds and not self._ready(conds[-1]) for _, conds in self._parts.values())

    def _update_idle(self) -> None:
        if self._all_waiting():
            if self._idle_since is None:
                self._idle_since = time.monotonic()
        else:
            self._idle_since = None

    # -- 기다림(참여 스레드) --
    @contextlib.contextmanager
    def _waiting(self, deadline: float | None, event: threading.Event | None):
        me = threading.get_ident()
        with self._cv:
            p = self._parts.get(me)
            if p is not None:
                p[1].append((deadline, event))
                self._update_idle()
                self._cv.notify_all()
        try:
            yield
        finally:
            with self._cv:
                if p is not None:
                    p[1].pop()
                    self._update_idle()

    def _check(self) -> None:
        """기다리는 동안 부른다(_cv를 쥔 채). 물리 멈춤·틱 멈춤이면 RuntimeError."""
        why = getattr(self.driver, "failed_reason", None)
        if why:
            raise RuntimeError(f"물리 시뮬레이션이 멈췄습니다: {why}")
        if self._idle_since is not None and not self._closed:
            since = max(self._idle_since, self._tick_wall)
            if time.monotonic() - since > self.stall_s:
                raise RuntimeError(f"시뮬레이션 시계가 멈췄습니다: 참여 스레드가 모두 기다리는데 틱 스레드가 {self.stall_s:g}초 동안 돌지 않았다")

    def sleep(self, seconds: float) -> None:
        if seconds <= 0:
            return
        target = self.now() + seconds
        with self._waiting(target, None), self._cv:
            while self.now() < target - 1e-12 and not self._closed:
                self._check()
                self._cv.wait(0.05)

    def wait(self, event: threading.Event, timeout: float | None = None) -> bool:
        if event.is_set():
            return True
        target = None if timeout is None else self.now() + timeout
        with self._waiting(target, event), self._cv:
            while not event.is_set():
                if self._closed or (target is not None and self.now() >= target - 1e-12):
                    return event.is_set()
                self._check()
                self._cv.wait(POLL_S)
        return True

    # -- 틱 스레드 --
    def tick_sleep(self, seconds: float) -> None:
        """틱 스레드만: 참여 스레드가 모두 기다릴 때까지 기다린 뒤 seconds만큼(2ms 정수배, 남은 몫은 쌓아 둠) advance하고 every를 부른다."""
        me = threading.get_ident()
        with self._cv:
            if self._tick_ident is None:
                self._tick_ident = me
            elif self._tick_ident != me:
                raise RuntimeError("tick_sleep은 틱 스레드만 부른다")
            self._tick_wall = time.monotonic()
            self._carry += seconds
            n = int(self._carry / STEP_S + 1e-9)
            self._carry -= n * STEP_S
            while not self._closed and not self._all_waiting():
                self._cv.wait(0.05)
                self._tick_wall = time.monotonic()
            if self._closed:
                return
            if n:
                self.driver.advance(n * STEP_S)
                self._steps += n
            due = []
            for e in self._every:
                while e[2] <= self.now() + 1e-12:
                    due.append(e[1])
                    e[2] += e[0]
            self._tick_wall = time.monotonic()
            self._cv.notify_all()
        for fn in due:  # 틱 락 밖에서(fn이 sleep하지 않는다는 전제: 상태 한 바퀴·기록 한 줄·정책 한 걸음)
            try:
                fn()
            except Exception as e:  # noqa: BLE001 콜백 하나가 깨져도 시계는 계속 돈다
                emit = getattr(self.driver, "_emit", None)
                if emit is not None:
                    emit("sim", "WARN", "SIM-CLOCK-CALLBACK-ERROR", {"fn": getattr(fn, "__qualname__", repr(fn)), "error": repr(e)[:200]})

    def every(self, period_s: float, fn) -> None:
        """시뮬레이션 시간이 period_s의 배수를 지날 때마다 틱 스레드가 fn()을 부른다(등록 순서대로)."""
        if period_s <= 0:
            raise ValueError("period_s는 0보다 커야 한다")
        with self._cv:
            self._every.append([period_s, fn, (math.floor(self.now() / period_s + 1e-9) + 1) * period_s])

    def close(self) -> None:
        """기다리는 쪽과 틱 스레드를 모두 풀어 준다(틱 스레드는 advance하지 않고 돌아가 자기 _quit을 본다)."""
        with self._cv:
            self._closed = True
            self._cv.notify_all()
