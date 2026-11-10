"""팔 자세 기록: 50Hz 제어 틱마다 보낸 관절값을 시각과 함께 남기고, 시각별 팔 링크·센서 위치를 돌려준다.
레이더가 팔 자리를 지울 때 쓰는 arm_links_fn(t_ns -> [(x, y)])과 가상 ESP32의 팔 센서 위치가 이것을 쓴다 (REQ-025)."""
import threading
from bisect import bisect_left


class PoseTrail:
    def __init__(self, arm, maxlen: int = 15000):  # 50Hz로 약 5분
        self.arm, self.maxlen = arm, maxlen
        self._t: list[int] = []
        self._q: list = []
        self._lock = threading.Lock()

    def attach(self) -> None:
        self.arm.ctl.trace = self.add

    def detach(self) -> None:
        self.arm.ctl.trace = None

    def add(self, t_ns: int, q) -> None:
        with self._lock:
            self._t.append(t_ns)
            self._q.append(q)
            if len(self._t) > 2 * self.maxlen:
                del self._t[: -self.maxlen], self._q[: -self.maxlen]

    def _nearest(self, t_ns: int):
        with self._lock:
            if not self._t:
                return None, None
            i = bisect_left(self._t, t_ns)
            j = min((k for k in (i - 1, i) if 0 <= k < len(self._t)), key=lambda k: abs(self._t[k] - t_ns))
            return self._t[j], self._q[j]

    def links_at(self, t_ns: int) -> list[tuple[float, float]]:
        """t_ns에 가장 가까운 틱의 팔 링크 점. 기록이 없으면 지금 자세."""
        _, q = self._nearest(t_ns)
        return self.arm.arm_links(q)

    def sensor_at(self, t_ns: int, max_gap_s: float = 0.1):
        """t_ns의 손목 센서 위치. 가까운 틱이 없으면(팔이 안 움직이는 동안) None. 가상 ESP32 전용이다."""
        t, q = self._nearest(t_ns)
        return None if q is None or abs(t - t_ns) > max_gap_s * 1e9 else self.arm.sensor_position(q)
