"""실측용 스크립트(U0): ESP32 실측 펌웨어에서 측정을 받아 CSV로 남긴다.

python -m so101_studio.perception.ultrasonic.bench --port COM5 --k ra --n 200 --gap 33 --label B3
센서 2개 번갈아 간격별 실측(B3)은 --gaps 20,25,29,33,40 으로 한 번에 돈다.
"""
import argparse
import csv
import json
import statistics
import time
from pathlib import Path

import serial


def read_json(ser, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                pass  # 부팅 로그 등 깨진 줄은 건너뜀
    return None


def send(ser, **msg):
    ser.write((json.dumps(msg) + "\n").encode())


def run_once(ser, k, n, gap_ms, max_mm):
    send(ser, cmd="gap", ms=gap_ms)
    read_json(ser, 2)
    send(ser, cmd="once", k=k, n=n, max_mm=max_mm)
    rows = []
    while (m := read_json(ser, 5)) is not None and m.get("t") != "done":
        if m.get("t") in ("r", "a"):
            rows.append((m["t"], m["d"], m["us"]))
    return rows


def outlier_ratio(rows, tol_mm):
    """센서별 중앙값에서 tol_mm 넘게 벗어난(또는 응답 없음) 비율. 고정 물체 앞에서 B3에 쓴다."""
    out = {}
    for k in sorted({r[0] for r in rows}):
        d = [r[1] for r in rows if r[0] == k]
        vals = [x for x in d if x is not None]
        med = statistics.median(vals) if vals else None
        bad = sum(1 for x in d if x is None or abs(x - med) > tol_mm)
        out[k] = (med, bad / len(d))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--k", default="ra", help="r, a, ra(번갈아)")
    p.add_argument("--n", type=int, default=200)
    p.add_argument("--gap", type=int, default=33)
    p.add_argument("--gaps", help="쉼표로 여러 간격(ms), 지정하면 --gap 무시 (B3)")
    p.add_argument("--max-mm", type=int, default=600)
    p.add_argument("--tol-mm", type=float, default=20, help="엉뚱한 값 판정 기준")
    p.add_argument("--label", default="bench")
    p.add_argument("--out", default="runs/bench")
    a = p.parse_args(argv)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d-%H%M%S')}-{a.label}.csv"
    gaps = [int(g) for g in a.gaps.split(",")] if a.gaps else [a.gap]
    with serial.Serial(a.port, a.baud, timeout=0.5) as ser, open(path, "w", newline="") as f:
        time.sleep(2)  # ESP32 리셋 대기
        ser.reset_input_buffer()
        send(ser, cmd="hello")
        print("hello:", read_json(ser, 3))
        w = csv.writer(f)
        w.writerow(["gap_ms", "sensor", "d_mm", "us"])
        for g in gaps:
            rows = run_once(ser, a.k, a.n, g, a.max_mm)
            w.writerows((g, *r) for r in rows)
            for k, (med, ratio) in outlier_ratio(rows, a.tol_mm).items():
                print(f"gap {g}ms sensor {k}: n={sum(1 for r in rows if r[0] == k)} "
                      f"median={med} mm, outlier/none {ratio:.1%}")
    print("saved", path)


if __name__ == "__main__":
    main()
