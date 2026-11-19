"""레이더 실물 점검 도구(B4·R1): ESP32에 연결해 서보 가운데 맞추기, 빈 작업대 기준값, 후보 찾기, 서보 지연 재기.

python -m so101_studio.perception.ultrasonic.radar_check --port COM5 center      # driver를 설정대로 켜고 서보를 90°(정면, XL330 2048)로
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 calibrate   # 빈 작업대 기준값 -> config/ultrasonic_baseline.json
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 scan        # 후보 위치 출력 (R1)
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 lag         # 서보 지연 d/2 (B4-1, 정면 약 300mm에 물체 하나)
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 r1 --obj 40x80x40   # R1: 3x3 자리 x 3회, 자리마다 Enter. 정답 쌍 기록(truth.py)
python -m so101_studio.perception.ultrasonic.radar_check --sim r1                   # 가상 ESP32로 같은 흐름(사람 입력 없음)
"""
import argparse
import math
import statistics

from . import truth
from .link import Link, SerialTransport
from .model import RealClock, load_config
from .source import UltrasonicSource

R1_X, R1_Y = (150, 215, 280), (150, 190, 230)  # ultrasonic-spec 10절 R1, radar-bringup 5단계(작업대 좌표 mm)


def center(src) -> None:
    """서보를 90°로 보낸다. 브래킷을 끼우기 전에 한다(XL330 눈금 2048)."""
    src.link.send(cmd="servo", deg=90)


def lag_deg(src, n: int = 20) -> dict:
    """같은 물체에서 가장 가까운 호의 중심 각도를 갈 때(dir=+1)·올 때(-1)로 구해 차이/2 = 서보 지연(°). 합격은 |지연| 3° 이하."""
    src.begin_cycle()
    src._run_passes(n)
    centers = {1: [], -1: []}
    for p in src._passes:
        hit = [(a, d) for _, a, d in p.meas if d is not None]
        if not hit:
            continue
        near = min(d for _, d in hit)
        arc = [a for a, d in hit if d <= near + 30]
        centers[p.dir].append(statistics.mean(arc))
    if not centers[1] or not centers[-1]:
        raise RuntimeError("물체가 안 보인다: 정면 300mm 앞에 물체를 둔다")
    f, b = statistics.mean(centers[1]), statistics.mean(centers[-1])
    return {"갈 때 중심": round(f, 2), "올 때 중심": round(b, 2), "지연": round((f - b) / 2, 2), "바퀴": (len(centers[1]), len(centers[-1]))}


def r1(src, run: truth.TruthRun, place, obj=(40, 80, 40), angle=0.0, reps=3, spots=None) -> list[dict]:
    """R1: 자리마다 place(x, y, rep)(사람이 놓고 Enter, False면 끝) -> scan -> 정답과 짝지어 기록. 자리별 가장 가까운 후보 오차(mm, 없으면 None)."""
    w, length, h = obj
    out = []
    for rep in range(1, reps + 1):
        for x, y in spots or [(x, y) for y in R1_Y for x in R1_X]:
            if place(x, y, rep) is False:
                return out
            before = len(src.saved)
            cands = src.scan().objects
            run.add([truth.obj(x, y, angle, w, length, h)], src.saved[before:], rep)
            err = min((math.dist((c.x_mm, c.y_mm), (x, y)) for c in cands), default=None)
            out.append({"x": x, "y": y, "rep": rep, "err_mm": err, "n": len(cands)})
            print(f"  {rep}회 ({x},{y}): 후보 {len(cands)}개, 가장 가까운 오차 " + ("없음" if err is None else f"{err:.0f}mm"))
    return out


def r1_summary(res: list[dict], limit=90.0) -> str:
    errs = [r["err_mm"] for r in res]
    ok = sum(e is not None and e <= limit for e in errs)
    found = sorted(e for e in errs if e is not None)
    med = f"{statistics.median(found):.0f}mm" if found else "-"
    return f"R1: {ok}/{len(errs)}이 {limit:.0f}mm 안(합격 95%), 오차 중앙 {med}, 후보 없음 {errs.count(None)}"


def _ask(x, y, rep):
    a = input(f"{rep}회: 물체 중심을 x={x} y={y}에 놓고 손을 빼고 Enter (q = 끝) ")
    return a.strip().lower() != "q"


def _sim_source(cfg_box=None):
    """--sim: 벤치마크의 가상 ESP32·팔(낙관 모델), 빈 판 기준값까지. place가 물체를 옮긴다."""
    from .benchmark import build, make_source
    from .model import Box
    cfg, clk, esp, arm = build([], seed=0)
    src = make_source(cfg, clk, esp, arm)
    src.ignore_zones = []  # 실제 radar_check처럼 배출 구역을 빼지 않는다(R1 자리 (280,150)이 통 무시 구역 안이다)

    def place(x, y, rep):
        w, length, h = cfg_box
        esp.boxes = [Box(x, y, 0, w, length, h)]
    return src, place


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--sim", action="store_true", help="가상 ESP32 (r1만)")
    ap.add_argument("cmd", choices=["center", "calibrate", "scan", "lag", "r1"])
    ap.add_argument("--n", type=int, default=20, help="lag의 바퀴 수")
    ap.add_argument("--obj", default="40x80x40", help="r1 물체 폭x길이x높이 mm")
    ap.add_argument("--angle", type=float, default=0.0, help="r1 물체 긴 축 각도(작업대 x축 기준 °)")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--note", default="", help="r1 기록 메모(물체 재질 등)")
    a = ap.parse_args(argv)
    if a.cmd == "r1":
        box = tuple(float(v) for v in a.obj.lower().split("x"))
        if a.sim:
            src, place = _sim_source(box)
        else:
            if not a.port:
                ap.error("--port가 필요하다")
            cfg = load_config()
            src = UltrasonicSource(Link(SerialTransport(a.port, cfg.baud), RealClock()), cfg)
            src.connect()  # 기준값은 먼저 calibrate로 만든다(없으면 scan이 알린다)
            place = _ask
        run = truth.TruthRun("R1", src.cfg, sim=a.sim, note=a.note)
        src.record_dir = run.dir
        try:
            res = r1(src, run, place, box, a.angle, a.reps)
        finally:
            run.close()
        print(r1_summary(res))
        print(f"기록: {run.dir}")
        return
    if a.sim or not a.port:
        ap.error("--sim은 r1만, 나머지는 --port가 필요하다")
    cfg = load_config()
    src = UltrasonicSource(Link(SerialTransport(a.port, cfg.baud), RealClock()), cfg)
    src.connect()  # servo.driver(ultrasonic.local.yaml)를 펌웨어에 알리고 시계를 맞춘다
    print(f"연결됨: 서보 구동 {cfg.servo.driver}")
    if a.cmd == "center":
        center(src)
        print("서보를 90°로 보냈다. 브래킷을 −y(팔 쪽)로 향해 끼운다")
    elif a.cmd == "calibrate":
        src.calibrate()
        print(f"기준값 저장: {src.baseline_path} (바퀴 사이 불안정한 각도 {len(src.baseline.unstable)}개)")
    elif a.cmd == "scan":
        for o in src.scan().objects:
            print(f"후보 x={o.x_mm:.0f} y={o.y_mm:.0f} (mm, 작업대 좌표)")
    else:
        print(lag_deg(src, a.n))


if __name__ == "__main__":
    main()
