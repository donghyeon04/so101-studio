"""엔진이 쓰는 버스를 감싸 전압·온도를 확인한다. 엔진 코드는 고치지 않는다.
엔진은 매 틱 Present_Position을 읽으므로 그때 간격을 두고 전압·온도를 함께 읽는다. 기준을 넘으면 AutoCalibrationError를 던지고,
엔진의 finally가 레지스터를 되돌린다. 한 번 작동한 뒤에는 검사하지 않는다(되돌리는 읽기가 다시 던지지 않게)."""
import time

from .engine import AutoCalibrationError

MIN_VOLT = 11.0
MAX_TEMP = 60.0
PERIOD_S = 0.25
CONFIRM = 2  # 한 번 잘못 읽힌 값(온도 80°C, 실제 34~39°C)으로 중단하지 않는다: 연속 2회(약 0.5초) 넘을 때만


def read_vitals(bus) -> tuple[float, float]:
    """(가장 낮은 전압 V, 가장 높은 온도 °C). Present_Voltage는 0.1V 단위다."""
    volts = bus.sync_read("Present_Voltage", normalize=False, num_retry=3)
    temps = bus.sync_read("Present_Temperature", normalize=False, num_retry=3)
    return min(volts.values()) / 10, float(max(temps.values()))


def vitals_problem(bus, min_volt: float = MIN_VOLT, max_temp: float = MAX_TEMP) -> str | None:
    volt, temp = read_vitals(bus)
    if volt < min_volt:
        return f"전압 {volt:.1f}V가 {min_volt:.1f}V보다 낮다: 전원 어댑터와 케이블을 확인한다"
    if temp > max_temp:
        return f"온도 {temp:.0f}°C가 {max_temp:.0f}°C보다 높다: 식을 때까지 쉰다"
    return None


class GuardedBus:
    def __init__(self, bus, min_volt: float = MIN_VOLT, max_temp: float = MAX_TEMP, period_s: float = PERIOD_S, clock=time.monotonic, confirm: int = CONFIRM):
        self._bus, self.min_volt, self.max_temp, self.period_s, self._clock, self.confirm = bus, min_volt, max_temp, period_s, clock, confirm
        self._last = clock()
        self._bad = 0  # 연속으로 기준을 넘은 횟수
        self.tripped: str | None = None

    def __getattr__(self, name):  # motors, write, read, sync_write, disable_torque 등은 그대로 넘긴다
        return getattr(self._bus, name)

    def sync_read(self, data_name, motors=None, **kw):
        if data_name == "Present_Position":
            self._check()
        return self._bus.sync_read(data_name, motors, **kw)

    def _check(self) -> None:
        now = self._clock()
        if self.tripped or now - self._last < self.period_s:
            return
        self._last = now
        problem = vitals_problem(self._bus, self.min_volt, self.max_temp)
        self._bad = self._bad + 1 if problem else 0
        if problem and self._bad >= self.confirm:
            self.tripped = problem
            raise AutoCalibrationError(f"{problem} (자동 캘리브레이션 중단, 레지스터를 되돌린다)")
