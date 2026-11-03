"""엔진이 쓰는 버스를 감싸 전압·온도를 확인한다. 엔진 코드는 고치지 않는다.
엔진은 매 틱 Present_Position을 읽으므로 그때 간격을 두고 전압·온도를 함께 읽는다. 기준을 넘으면 AutoCalibrationError를 던지고,
엔진의 finally가 레지스터를 되돌린다. 한 번 작동한 뒤에는 검사하지 않는다(되돌리는 읽기가 다시 던지지 않게)."""
import time

from .engine import AutoCalibrationError

MIN_VOLT = 11.0
MAX_TEMP = 60.0
PERIOD_S = 0.25
# 한 번 읽은 값이 기준을 넘어도 바로 멈추지 않고 이만큼 더 읽어 모두 넘을 때만 멈춘다. 첫 실물 시험에서 실제로는 37°C인 서보를 잘못 읽은 68°C 한 번에
# 시작하자마자 중단됐다(2026-11-03). 진짜 과열이면 몇십 ms 안에 다시 읽어도 그대로라 안전은 그대로다.
CONFIRM_READS = 2
CONFIRM_GAP_S = 0.05


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
    def __init__(self, bus, min_volt: float = MIN_VOLT, max_temp: float = MAX_TEMP, period_s: float = PERIOD_S, clock=time.monotonic,
                 confirm_reads: int = CONFIRM_READS, confirm_gap_s: float = CONFIRM_GAP_S, sleep=time.sleep):
        self._bus, self.min_volt, self.max_temp, self.period_s, self._clock = bus, min_volt, max_temp, period_s, clock
        self.confirm_reads, self.confirm_gap_s, self._sleep = confirm_reads, confirm_gap_s, sleep
        self._last = clock()
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
        for _ in range(self.confirm_reads if problem else 0):
            self._sleep(self.confirm_gap_s)
            problem = vitals_problem(self._bus, self.min_volt, self.max_temp)  # 다시 읽어도 넘어야 진짜다
            if not problem:
                break
        if problem:
            self.tripped = problem
            raise AutoCalibrationError(f"{problem} (자동 캘리브레이션 중단, 레지스터를 되돌린다)")
