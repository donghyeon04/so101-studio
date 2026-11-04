"""블랙박스의 메모리 쪽 (decisions/0030): 50Hz 틱의 링 버퍼와 처리 안 된 예외 감시."""
import dataclasses
import sys
import threading
import time
from collections import deque

RING_SECONDS = 10.0
RING_HZ = 50


class RingBuffer:
    """최근 RING_SECONDS초의 제어 틱(명령, 실제, 틱 간격, 버스 오류). 제어 스레드가 add만 하고(락 없음, deque.append는 원자적), 읽는 쪽은 복사해서 쓴다."""

    def __init__(self, seconds: float = RING_SECONDS, hz: int = RING_HZ):
        self.seconds = seconds
        self._d: deque = deque(maxlen=int(seconds * hz * 1.5) + 10)

    def add(self, t_ns: int, cmd, dt_s: float, err: str | None, state: str, actual=None) -> None:
        self._d.append((t_ns, cmd, dt_s, err, state, actual))

    def max_dt(self, seconds: float = 1.0) -> float:
        """직전 seconds초의 틱 간격 최대값(초)."""
        end = time.time_ns() - int(seconds * 1e9)
        out = 0.0
        for t, _c, dt, *_ in reversed(self._d):
            if t < end:
                break
            out = max(out, dt)
        return out

    def dump(self, now_ns: int | None = None) -> list[dict]:
        """최근 seconds초의 틱을 오래된 순서의 dict 목록으로."""
        now_ns = time.time_ns() if now_ns is None else now_ns
        start = now_ns - int(self.seconds * 1e9)
        conv = lambda j: dataclasses.asdict(j) if dataclasses.is_dataclass(j) else j
        return [{"t": t, "dt_ms": round(dt * 1000, 3), "cmd": conv(c), "actual": conv(a), "err": e, "state": s}
                for t, c, dt, e, s, a in list(self._d) if t >= start]

    def __len__(self) -> int:
        return len(self._d)


def install_excepthooks(on_exception) -> callable:
    """처리 안 된 예외(메인·다른 스레드)를 on_exception(exc_type, exc, tb, where)에 알리고 원래 훅도 그대로 부른다. 되돌리는 함수를 돌려준다."""
    old_sys, old_thread = sys.excepthook, threading.excepthook

    def sys_hook(t, v, tb):
        try:
            on_exception(t, v, tb, "main")
        finally:
            old_sys(t, v, tb)

    def thread_hook(args):
        try:
            on_exception(args.exc_type, args.exc_value, args.exc_traceback, getattr(args.thread, "name", "thread"))
        finally:
            old_thread(args)

    sys.excepthook, threading.excepthook = sys_hook, thread_hook

    def restore() -> None:
        if sys.excepthook is sys_hook:
            sys.excepthook = old_sys
        if threading.excepthook is thread_hook:
            threading.excepthook = old_thread

    return restore
