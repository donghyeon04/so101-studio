"""REQ-083 StepClock(mujoco-spec 13.10.2 작업 나누기 ②): 가짜 드라이버로 번갈아 돌기·every·멈춤 감지·닫기를 본다."""
import threading
import time

import pytest

from so101_studio.arm.mujoco_clock import STEP_S, StepClock


class FakeDriver:
    realtime, viewer = False, False

    def __init__(self):
        self.t, self.failed_reason, self.events = 0.0, None, []

    def advance(self, seconds):
        self.t += seconds

    def _emit(self, kind, level, code, data):
        self.events.append(code)


def start_tick(clock, period=0.02):
    """MotionController._loop 대신: _quit이 설 때까지 tick_sleep을 돈다. 끝나면 done을 세운다."""
    quit_, done = threading.Event(), threading.Event()

    def loop():
        while not quit_.is_set():
            clock.tick_sleep(period)
        done.set()

    threading.Thread(target=loop, daemon=True).start()
    return quit_, done


def test_rejects_realtime_driver_and_manual_viewer():
    d = FakeDriver()
    d.realtime = True
    with pytest.raises(ValueError):
        StepClock(d)
    d.realtime, d.viewer = False, True
    with pytest.raises(ValueError):
        StepClock(d)


def test_tick_waits_while_a_participant_computes_and_sleep_wakes_after_advance():
    d = FakeDriver()
    clock = StepClock(d)
    quit_, done = start_tick(clock)
    time.sleep(0.2)  # 만든 스레드(참여)가 깨어 계산 중: 시간이 흐르지 않는다
    assert clock.now() == 0 and d.t == 0
    clock.sleep(0.1)
    assert clock.now() == pytest.approx(0.1, abs=1e-9) and d.t == pytest.approx(clock.now())  # 깨어날 시각을 지나쳐 달리지 않는다
    quit_.set()
    assert clock.wait(done, 1.0)


def test_nested_participation_from_another_thread_does_not_hang():
    clock = StepClock(FakeDriver())
    quit_, done = start_tick(clock)
    out = []

    def worker():
        with clock.participate(), clock.participate():
            clock.sleep(0.04)
            out.append(clock.now())

    t = threading.Thread(target=worker)
    t.start()
    clock.sleep(0.1)  # 두 참여 스레드가 모두 기다려야 시간이 흐른다
    t.join(2.0)
    assert not t.is_alive() and out and out[0] >= 0.04
    quit_.set()
    clock.wait(done, 1.0)


def test_close_releases_a_blocked_tick_thread():
    clock = StepClock(FakeDriver())
    quit_, done = start_tick(clock)
    time.sleep(0.1)  # 만든 스레드가 깨어 있어 틱 스레드는 tick_sleep 안에서 기다린다
    quit_.set()
    clock.close()
    assert done.wait(1.0)


def test_failed_reason_raises_right_away():
    d = FakeDriver()
    clock = StepClock(d)
    start_tick(clock)
    d.failed_reason = "관절값이 NaN"
    t0 = time.monotonic()
    with pytest.raises(RuntimeError, match="NaN"):
        clock.sleep(10.0)
    assert time.monotonic() - t0 < 1.0


def test_stall_when_no_tick_thread_runs():
    clock = StepClock(FakeDriver(), stall_s=0.3)
    with pytest.raises(RuntimeError, match="멈췄"):
        clock.sleep(1.0)


def test_tick_sleep_keeps_the_remainder_and_only_the_tick_thread_may_call_it():
    d = FakeDriver()
    clock = StepClock(d)
    quit_, done = start_tick(clock, period=1 / 30)  # 2ms의 정수배가 아니다
    clock.sleep(1.0)
    quit_.set()
    clock.wait(done, 1.0)
    assert abs(clock.now() - round(clock.now() / (1 / 30)) / 30) <= STEP_S  # 30Hz 틱 수 × 1/30초와 한 스텝 안
    with pytest.raises(RuntimeError):
        clock.tick_sleep(0.02)  # 만든 스레드는 틱 스레드가 아니다


def test_every_fires_on_period_multiples_and_survives_errors():
    d = FakeDriver()
    clock = StepClock(d)
    seen = []
    clock.every(0.1, lambda: seen.append(round(clock.now(), 3)))
    clock.every(0.25, lambda: 1 / 0)
    quit_, done = start_tick(clock)
    clock.sleep(0.5)
    quit_.set()
    clock.wait(done, 1.0)
    assert seen[:5] == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert d.events.count("SIM-CLOCK-CALLBACK-ERROR") >= 2


def test_wait_returns_on_event_or_sim_timeout():
    clock = StepClock(FakeDriver())
    quit_, done = start_tick(clock)
    ev = threading.Event()
    assert clock.wait(ev, 0.1) is False and clock.now() == pytest.approx(0.1, abs=1e-9)
    threading.Timer(0.05, ev.set).start()
    assert clock.wait(ev) is True
    quit_.set()
    clock.wait(done, 1.0)


def test_same_script_gives_the_same_sim_times():
    """같은 순서의 계산·기다림이면 OS 스케줄과 상관없이 시뮬레이션 시각이 같다(완료 기준 3의 바탕)."""

    def run():
        clock = StepClock(FakeDriver())
        quit_, done = start_tick(clock)
        seen = []

        def worker():
            with clock.participate():
                for _ in range(5):
                    time.sleep(0.003)  # 계산(벽시계): 이동안 시간은 멈춰 있다
                    clock.sleep(0.03)
                    seen.append(("w", round(clock.now(), 6)))

        t = threading.Thread(target=worker)
        t.start()
        for _ in range(4):
            clock.sleep(0.05)
            seen.append(("m", round(clock.now(), 6)))
        t.join(2.0)
        quit_.set()
        clock.wait(done, 1.0)
        return sorted(seen)

    assert run() == run() == run()
