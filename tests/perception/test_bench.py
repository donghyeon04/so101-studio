import pytest

from so101_studio.perception.ultrasonic.bench import outlier_ratio


def test_outlier_ratio():
    rows = [("r", 300, 0)] * 8 + [("r", None, 0), ("r", 500, 0)] + [("a", 100, 0)] * 4
    out = outlier_ratio(rows, 20)
    assert out["r"] == (300, 0.2)
    assert out["a"] == (100, 0.0)


def test_benchmark_summary_has_accuracy_metrics():
    """벤치마크 요약: 레이더 후보 오차, 확인 위치 오차, 방향 정답률(1절 윗 3줄용)이 같은 표본에서 나온다."""
    from so101_studio.perception.ultrasonic.benchmark import summarize

    rows = [{"radius": None, "n": 3, "ok": 2, "recog_s": 50.0, "left": 0, "radar_errs": [10, 20, 100], "check_errs": [2.0, 4.0],
             "angles": [True, False], "h_errs": [1.0, 3.0]},
            {"radius": None, "n": 3, "ok": 3, "recog_s": 60.0, "left": 0, "radar_errs": [30], "check_errs": [6.0], "angles": [True], "h_errs": [2.0]}]
    s = summarize(rows)
    assert s["radar_n"] == 4 and s["radar_within90_pct"] == 75 and s["radar_err_median_mm"] == 25
    assert s["check_n"] == 3 and s["check_err_median_mm"] == 4.0 and s["angle_pct"] == pytest.approx(200 / 3)
    assert s["h_err_median_mm"] == 2.0 and s["h_err_p90_mm"] == 3.0  # REQ-035 높이 오차


def test_bench_run_once_against_fake_esp32():
    """bench.py가 7절 규약(gap 응답, once의 done)을 가상 ESP32와 맞게 주고받는다."""
    from so101_studio.perception.ultrasonic.bench import run_once
    from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
    from so101_studio.perception.ultrasonic.model import Box, load_config

    clk = SimClock()
    esp = FakeESP32(clk, [Box(300, 330, 0)], load_config(), arm_sensor_fn=lambda t: (300, 330, 110), noise_mm=0, dropout=0)

    class Ser:  # pyserial 흉내: 한 줄씩 읽고, 줄이 없으면 가상 시계를 조금 진행
        def __init__(self):
            self.q = []

        def write(self, b):
            esp.write(b.decode().strip())

        def readline(self):
            for _ in range(2000):
                self.q += esp.readlines()
                if self.q:
                    return (self.q.pop(0) + "\n").encode()
                clk.sleep(0.01)
            return b""

    rows = run_once(Ser(), "ra", 6, 40, 600)
    assert len(rows) == 12 and {r[0] for r in rows} == {"r", "a"}
    assert esp.gap_ns == 40_000_000
