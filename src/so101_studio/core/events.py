"""내부 이벤트 버스: 단순 발행·구독. 구독자는 발행한 스레드에서 호출된다."""
import threading
from collections import defaultdict
from collections.abc import Callable


class EventBus:
    def __init__(self):
        self._subs: dict[str, list[Callable]] = defaultdict(list)
        self._lock = threading.Lock()

    def subscribe(self, topic: str, callback: Callable) -> None:
        with self._lock:
            self._subs[topic].append(callback)

    def publish(self, topic: str, payload) -> None:
        with self._lock:
            subs = list(self._subs[topic])  # 콜백 안에서 구독해도 교착되지 않게 복사
        for cb in subs:
            cb(payload)
