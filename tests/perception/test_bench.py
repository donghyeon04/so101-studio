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
             "angles": [True, False]},
            {"radius": None, "n": 3, "ok": 3, "recog_s": 60.0, "left": 0, "radar_errs": [30], "check_errs": [6.0], "angles": [True]}]
    s = summarize(rows)
    assert s["radar_n"] == 4 and s["radar_within90_pct"] == 75 and s["radar_err_median_mm"] == 25
    assert s["check_n"] == 3 and s["check_err_median_mm"] == 4.0 and s["angle_pct"] == pytest.approx(200 / 3)
