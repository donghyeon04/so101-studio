"""전원 시험: 큰 동작을 왕복하는 동안 전압과 서보별 전류를 최대한 빠르게 읽어 순간 전압 강하와 전류 최댓값을 본다.
건강 읽기(2Hz)로는 출발·정지 순간의 강하를 놓치기 때문에 따로 만들었다. 어댑터 교체 전에 근거를 얻는 용도다."""
import csv
import threading
import time
from dataclasses import replace

SETTLE_S = 1.0


def run_powertest(svc, repeats: int = 3, out=print) -> dict:
    svc.driver.set_torque(True)
    start = svc.driver.read_joints()
    far = replace(start, shoulder_lift=start.shoulder_lift + 50, elbow_flex=start.elbow_flex - 40, wrist_flex=start.wrist_flex + 30)
    rows, stop, mark = [], threading.Event(), {}

    def sample() -> None:
        while not stop.is_set():
            v, cur = svc.driver.read_power()
            rows.append((time.perf_counter(), v, sum(cur.values()), max(cur, key=cur.get), max(cur.values())))

    th = threading.Thread(target=sample, daemon=True)
    th.start()
    time.sleep(SETTLE_S)
    mark["t0"] = time.perf_counter()
    results = []
    for i in range(repeats):
        results.append(svc.move_joints(far).status)
        time.sleep(0.3)
        results.append(svc.move_joints(start).status)
        time.sleep(0.3)
    mark["t1"] = time.perf_counter()
    time.sleep(0.5)
    stop.set()
    th.join(timeout=2)

    base = [r for r in rows if r[0] < mark["t0"]]
    move = [r for r in rows if mark["t0"] <= r[0] <= mark["t1"]]
    path = svc.logger.dir / "powertest.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "voltage_v", "total_current_ma", "max_motor", "max_motor_current_ma"])
        w.writerows((round(r[0] - mark["t0"], 4), *r[1:]) for r in rows)
    if not base or not move:
        out("측정값이 부족합니다 (읽기에 실패했거나 동작이 너무 짧음).")
        return {}
    v0 = sum(r[1] for r in base) / len(base)
    low = min(move, key=lambda r: r[1])
    peak = max(move, key=lambda r: r[2])
    hz = len(rows) / (rows[-1][0] - rows[0][0])
    res = {"baseline_v": v0, "min_v": low[1], "drop_v": v0 - low[1], "peak_total_ma": peak[2], "peak_motor": peak[3],
           "peak_motor_ma": max(r[4] for r in move), "sample_hz": hz, "moves": results, "csv": str(path)}
    out(f"동작 결과 {results}")
    out(f"전압: 정지 때 {v0:.1f}V, 동작 중 최저 {low[1]:.1f}V (강하 {v0 - low[1]:.1f}V, 출발 후 {low[0] - mark['t0']:.2f}s 시점)")
    out(f"전류(추정): 6개 합 최대 {peak[2] / 1000:.2f}A ({peak[3]} 최대 {res['peak_motor_ma'] / 1000:.2f}A), 어댑터 5A 대비 {peak[2] / 50:.0f}%")
    out(f"초당 {hz:.0f}회 읽음. 기록: {path}")
    out("참고: 전류는 레지스터 값 x 6.5mA 추정이고 제조사 값 확인 전이다. 강하가 크거나 합계가 5A에 가까우면 전원 의심.")
    return res
