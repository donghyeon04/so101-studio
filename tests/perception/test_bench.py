from so101_studio.perception.ultrasonic.bench import outlier_ratio


def test_outlier_ratio():
    rows = [("r", 300, 0)] * 8 + [("r", None, 0), ("r", 500, 0)] + [("a", 100, 0)] * 4
    out = outlier_ratio(rows, 20)
    assert out["r"] == (300, 0.2)
    assert out["a"] == (100, 0.0)
