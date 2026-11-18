"""제어 시간의 시계 규약 (mujoco-spec 13.10.2, 결정 0070). 팔 제어(MotionController·Skills·ArmService)는 time 대신 받은 시계를 쓴다.
기본 WallClock은 지금까지의 time 호출과 같다. 시뮬레이션 시계 StepClock은 arm/mujoco_clock.py(MuJoCo 영역)다.
기록 시각(로그·데이터 줄의 time.time())은 시계가 아니라 벽시계 그대로다."""
import contextlib
import threading
import time
from typing import Protocol


class Clock(Protocol):
    drives_status: bool  # True면 상태 한 바퀴를 시계가 every로 돌린다(ArmService가 상태 스레드를 띄우지 않는다)

    def now(self) -> float: ...  # 단조 초
    def now_ns(self) -> int: ...
    def sleep(self, seconds: float) -> None: ...
    def wait(self, event: threading.Event, timeout: float | None = None) -> bool: ...
    def tick_sleep(self, seconds: float) -> None: ...  # 틱 스레드(MotionController._loop)만 부른다
    def participate(self): ...  # 문맥 관리자: 이 스레드를 번갈아 돌기에 참여시킨다


class WallClock:
    """time.perf_counter·time.sleep·Event.wait를 그대로 부른다(초음파 RealClock과 같은 축)."""
    drives_status = False

    def now(self) -> float:
        return time.perf_counter()

    def now_ns(self) -> int:
        return time.perf_counter_ns()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def wait(self, event: threading.Event, timeout: float | None = None) -> bool:
        return event.wait(timeout)

    def tick_sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def participate(self):
        return contextlib.nullcontext()
