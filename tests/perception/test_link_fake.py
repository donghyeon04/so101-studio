import pytest

from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
from so101_studio.perception.ultrasonic.link import Link, UltrasonicLinkError
from so101_studio.perception.ultrasonic.model import Box, load_config

SWEEP = {"from": 10, "to": 170, "step": 4, "bounce": True, "max_mm": 600}


def make(boxes=(), **kw):
    clk = SimClock()
    esp = FakeESP32(clk, list(boxes), load_config(), **kw)
    return clk, esp, Link(esp, clk)


def test_sync_offset_and_wrap():
    clk, esp, link = make(boot_offset_us=2**32 - 1000)  # 곧 32비트가 돈다
    clk.sleep(3)
    link.sync()
    assert abs(link.to_ns(esp._us(clk.t_ns)) - clk.t_ns) < 2_000_000


def test_sweep_bounce_angles_and_timing():
    clk, _esp, link = make([Box(300, 190, 0)], dropout=0)
    link.sync()
    link.send(cmd="sweep", sweeps=2, **SWEEP)
    clk.sleep(3.5)
    msgs = link.poll()
    r = [m for m in msgs if m["t"] == "r"]
    assert [e["dir"] for e in msgs if m_end(e)] == [1, -1]
    assert [m["a"] for m in r[:41]] == list(range(10, 171, 4))
    assert [m["a"] for m in r[41:]] == list(range(170, 9, -4))
    assert 33 <= (r[1]["t_ns"] - r[0]["t_ns"]) / 1e6 <= 34  # ping_gap_ms
    front = next(m["d"] for m in r if m["a"] == 90)  # 정면 가까이: 물체 앞면 약 195mm
    assert abs(front - 195) < 10


def m_end(m):
    return m["t"] == "end"


def test_stream_pauses_radar_and_stop():
    clk, _esp, link = make(arm_sensor_fn=lambda t: (300, 190, 110))
    link.send(cmd="sweep", sweeps=0, **SWEEP)
    clk.sleep(0.5)
    link.poll()
    link.send(cmd="stream", hz=30, max_mm=300)
    clk.sleep(0.5)
    assert {m["t"] for m in link.poll()} == {"a"}  # 레이더는 멈췄다
    link.send(cmd="stop")
    clk.sleep(0.5)
    assert link.poll() == []


def test_sync_timeout_resends():
    """sync 응답을 못 받으면 1초 뒤 비우고 다시 보낸다(안 그러면 다시는 sync하지 않는다)."""
    clk, esp, link = make()
    link.sync()
    link.sync_every_ns = 0
    esp.disconnect()  # 응답이 없다
    link.poll()
    assert link._sync_t0 is not None
    clk.sleep(1.5)
    link.poll()  # 1초 넘어 비우고 새로 보냄
    assert link._sync_t0 == clk.t_ns
    esp._dead = False
    link.sync_every_ns = 10**12
    clk.sleep(1.1)
    link.poll()  # 다시 비우고 보냄(이번엔 ESP32가 받는다)
    link.poll()
    assert link._sync_t0 is None


def test_link_lost():
    clk, esp, link = make()
    link.sync()
    link.expect(True)
    esp.disconnect()
    clk.sleep(0.9)
    link.poll()
    clk.sleep(0.2)
    with pytest.raises(UltrasonicLinkError):
        link.poll()


def test_real_clock_is_perf_counter_ns():
    """팔 PoseSample.t_ns(perf_counter_ns)와 같은 시계다: 두 값의 차이가 작고, 해상도가 15.6ms가 아니다."""
    import time

    from so101_studio.perception.ultrasonic.model import RealClock

    a, b, c = time.perf_counter_ns(), RealClock.now_ns(), time.perf_counter_ns()
    assert a <= b <= c and c - a < 5_000_000
    ticks = {RealClock.now_ns() for _ in range(2000)}
    assert len(ticks) > 20  # 15.6ms 단위로 계단 지는 시계면 몇 개 안 된다
