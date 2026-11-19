"""레이더 실물 점검 도구(B4·R1): ESP32에 연결해 서보 가운데 맞추기, 빈 작업대 기준값, 후보 찾기, 서보 지연 재기.

python -m so101_studio.perception.ultrasonic.radar_check --port COM5 center      # driver를 설정대로 켜고 서보를 90°(정면, XL330 2048)로
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 calibrate   # 빈 작업대 기준값 -> config/ultrasonic_baseline.json
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 scan        # 후보 위치 출력 (R1)
python -m so101_studio.perception.ultrasonic.radar_check --port COM5 lag         # 서보 지연 d/2 (B4-1, 정면 약 300mm에 물체 하나)
"""
import argparse
import statistics

from .link import Link, SerialTransport
from .model import RealClock, load_config
from .source import UltrasonicSource


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


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("cmd", choices=["center", "calibrate", "scan", "lag"])
    ap.add_argument("--n", type=int, default=20, help="lag의 바퀴 수")
    a = ap.parse_args(argv)
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
