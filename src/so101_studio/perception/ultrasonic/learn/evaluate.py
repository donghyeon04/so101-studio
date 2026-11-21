"""규칙과 릿지 보정을 실행 폴더 단위 leave-one-run-out으로 나란히 평가(8.8 평가). 
python -m so101_studio.perception.ultrasonic.learn.evaluate runs/ultrasonic-truth/<실행> ... [--alpha 10]"""
import argparse
import math
import statistics

from ..benchmark import angle_ok
from ..truth_report import p90
from . import dataset, fit


def summary(pred, rows) -> dict:
    errs = [math.dist(p[:2], r["truth"][:2]) for p, r in zip(pred, rows, strict=True)]
    ang = [angle_ok(p[2], r["truth"][2]) for p, r in zip(pred, rows, strict=True) if p[2] is not None and r["truth"][2] is not None]
    return {"n": len(rows), "median": statistics.median(errs), "p90": p90(errs), "within10": 100 * sum(e <= 10 for e in errs) / len(errs),
            "angle_pct": 100 * sum(ang) / len(ang) if ang else float("nan")}


def loro(rows, alpha=10.0):
    """실행 하나씩 빼고 나머지로 학습해 뺀 실행을 예측한다. 돌려주는 값 = (규칙 요약, 보정 요약)."""
    runs = sorted({r["run"] for r in rows})
    test, pred = [], []
    for run in runs:
        train = [r for r in rows if r["run"] != run]
        held = [r for r in rows if r["run"] == run]
        if not train or not held:
            continue
        pred += fit.predict(fit.fit(train, alpha), held)
        test += held
    return summary([r["rule"] for r in test], test), summary(pred, test)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--alpha", type=float, default=10.0)
    a = ap.parse_args(argv)
    rows = [r for d in a.runs for r in dataset.rows(d)]
    rule, learned = loro(rows, a.alpha)
    print(f"표본 {len(rows)}개, 실행 {len({r['run'] for r in rows})}개 (leave-one-run-out, 가상 데이터라 성능 숫자로 쓰지 않는다)")
    print("| 추정 | n | 위치 중앙(mm) | 90%(mm) | 10mm 안 % | 방향 % |\n|---|---|---|---|---|---|")
    for name, s in (("규칙", rule), (f"릿지 보정 α={a.alpha:g}", learned)):
        print(f"| {name} | {s['n']} | {s['median']:.1f} | {s['p90']:.1f} | {s['within10']:.0f} | {s['angle_pct']:.0f} |")


if __name__ == "__main__":
    main()
