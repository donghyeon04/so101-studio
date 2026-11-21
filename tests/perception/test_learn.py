import random


def test_ridge_residual_learns_a_bias_that_the_rule_misses():
    """8.8 길: 규칙 중심이 늘 (+8, −6)mm 치우치면 릿지 보정이 leave-one-run-out에서 그 치우침을 지운다."""
    from so101_studio.perception.ultrasonic.learn import evaluate

    rng = random.Random(0)
    rows = []
    for run in ("a", "b", "c"):
        for _ in range(40):
            tx, ty, ta = rng.uniform(100, 300), rng.uniform(100, 250), rng.choice((0, 45, 90, 135))
            rx, ry = tx + 8 + rng.gauss(0, 1), ty - 6 + rng.gauss(0, 1)
            x = [rx - tx, ry - ty] + [rng.gauss(0, 1) for _ in range(29)]
            rows.append({"run": run, "x": x, "cand": (tx, ty), "rule": (rx, ry, ta), "truth": (tx, ty, ta)})
    rule, learned = evaluate.loro(rows, alpha=1.0)
    assert rule["median"] > 8 and learned["median"] < 3 and learned["n"] == rule["n"] == 120
