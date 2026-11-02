"""전원·부하 시험: 팔을 크게, 최대 속도로 휘두르는 동안 전압, 서보별 전류, 서보별 부하를 최대한 빠르게 읽는다.
건강 읽기(2Hz)로는 출발·정지 순간의 강하를 놓치기 때문에 따로 만들었다. 어댑터 교체 전에 근거를 얻는 용도다.
안전: 목표와 경로는 관절 한계와 바닥(턱 끝 높이) 검사를 통과한 것만 실행하고, 통과하지 못한 자세는 건너뛴다.
팔을 끝까지 펴면 작업대 가장자리 밖으로 나가므로 이 시험 동안만 작업대 가장자리 검사(check_xy)를 끈다(주변 공간을 비워야 한다).
시험 동안만 서보 가속 한계를 최대(254)로, 한 주기 이동량 한계를 8°로 올리고 끝나면 설정값으로 되돌린다."""
import csv
import threading
import time

from ..arm.trajectory import move_j
from ..core.types import JointState

SETTLE_S = 1.0
SAMPLE_GAP_S = 0.006  # 읽기 사이 간격. 첫 거친 시험(0.0005s)에서 이동량 한계 경고와 함께 멈췄고, 읽기가 버스를 점유해 쓰기가 밀린 것으로 추정
V_MAX_DEG_S = 200.0  # 100% 기준 속도. 한 주기 이동량 한계(5°/주기 = 250°/s) 안에서 가장 빠르게
A_MAX_DEG_S2 = 1500.0
# 큰 스윙 자세: pan, lift, elbow, wrist_flex, wrist_roll, gripper (시작 자세에서 출발해 차례로 갔다가 돌아온다)
SWINGS = [
    [60, -40, 30, 50, 120, 60],
    [-60, -20, 10, 20, -120, 3],
    [100, -70, 70, -40, 160, 90],
    [-100, -30, 50, 80, -160, 3],
    [0, 85, -75, -10, 0, 3],  # 팔을 끝까지 편 자세(도달 약 440mm, 턱 끝이 작업대에서 약 95mm 위)
    [80, 85, -75, -10, 0, 3],  # 편 채로 오른쪽으로 크게 휘두름 (작업대 밖으로 나간다)
    [-80, 85, -75, -10, 0, 3],  # 편 채로 왼쪽으로 크게 휘두름
    [0, 70, -65, -10, 0, 3],  # 편 채로 더 높게(턱 끝 약 150mm)
]


class _Aborted(Exception):
    pass


def run_powertest(svc, repeats: int = 3, speed_pct: float = 100.0, out=print, stop=lambda: False) -> dict:
    svc.driver.set_torque(True)
    # 200°/s는 주기당 4°라 평소 한계(5°)에 여유가 없다. 이 시험 동안만 8°로 올리고, 작업대 가장자리 검사는 끈다
    with svc.safety.relaxed(check_xy=False, max_step_deg=8.0):
        return _powertest(svc, repeats, speed_pct, out, stop)


def _powertest(svc, repeats: int, speed_pct: float, out, stop_req) -> dict:
    start = svc.driver.read_joints()
    rest_acc = svc.driver.acceleration
    targets, skipped = [], []
    here = start
    for i, s in enumerate(SWINGS):
        q = JointState(*s)
        why = svc.safety.check_path(move_j(here, q, V_MAX_DEG_S * speed_pct / 100, A_MAX_DEG_S2))
        (skipped.append((i + 1, why)) if why else targets.append(q))
        here = q if not why else here
    if not targets:
        out(f"안전 검사를 통과한 자세가 없어 시험하지 않았습니다: {skipped}")
        return {}
    rows, stop, mark = [], threading.Event(), {}

    def sample() -> None:
        while not stop.is_set():
            v, cur, load = svc.driver.read_power()
            m = max(cur, key=cur.get)
            lm = max(load, key=load.get)
            rows.append((time.perf_counter(), v, sum(cur.values()), m, cur[m], lm, load[lm]))
            time.sleep(SAMPLE_GAP_S)  # 통신 버스를 제어 스레드(50Hz 쓰기)에게 양보한다. 읽기가 버스를 계속 쥐면 쓰기가 밀려 한 주기 이동량이 커질 수 있다

    th = threading.Thread(target=sample, daemon=True)
    th.start()
    time.sleep(SETTLE_S)
    mark["t0"] = time.perf_counter()
    results = []
    try:
        svc.driver.set_acceleration(254)
        for _ in range(repeats):
            for q in [*targets, start]:
                if stop_req():
                    out("중단 요청: 시작 자세로 돌아간다")
                    results.append(svc._run([move_j(svc.driver.read_joints(), start, V_MAX_DEG_S * speed_pct / 100, A_MAX_DEG_S2)]).status)
                    raise _Aborted
                traj = move_j(svc.driver.read_joints(), q, V_MAX_DEG_S * speed_pct / 100, A_MAX_DEG_S2)
                results.append(svc._run([traj]).status)
                time.sleep(0.2)
    except _Aborted:
        pass
    finally:
        svc.driver.set_acceleration(rest_acc)
        mark["t1"] = time.perf_counter()
        time.sleep(0.5)
        stop.set()
        th.join(timeout=2)

    base = [r for r in rows if r[0] < mark["t0"]]
    move = [r for r in rows if mark["t0"] <= r[0] <= mark["t1"]]
    path = svc.logger.dir / "powertest.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "voltage_v", "total_current_ma", "max_current_motor", "max_current_ma", "max_load_motor", "max_load_pct"])
        w.writerows((round(r[0] - mark["t0"], 4), *r[1:]) for r in rows)
    if not base or not move:
        out("측정값이 부족합니다 (읽기에 실패했거나 동작이 너무 짧음).")
        return {}
    v0 = sum(r[1] for r in base) / len(base)
    low = min(move, key=lambda r: r[1])
    peak = max(move, key=lambda r: r[2])
    hot = max(move, key=lambda r: r[6])
    hz = len(rows) / (rows[-1][0] - rows[0][0])
    res = {"baseline_v": v0, "min_v": low[1], "drop_v": v0 - low[1], "peak_total_ma": peak[2], "peak_motor": peak[3],
           "peak_motor_ma": max(r[4] for r in move), "peak_load_pct": hot[6], "peak_load_motor": hot[5], "sample_hz": hz,
           "moves": results, "skipped": skipped, "csv": str(path)}
    if "fault" in results:
        out(f"중간에 멈춤(fault): {svc.ctl.fault}")
    out(f"실행한 자세 {len(targets)}개 x {repeats}회, 건너뜀 {skipped or '없음'}, 결과 {sorted(set(results))}")
    out(f"전압: 정지 때 {v0:.1f}V, 동작 중 최저 {low[1]:.1f}V (강하 {v0 - low[1]:.1f}V, 출발 후 {low[0] - mark['t0']:.2f}s 시점)")
    out(f"전류(추정): 6개 합 최대 {peak[2] / 1000:.2f}A ({peak[3]} 최대 {res['peak_motor_ma'] / 1000:.2f}A), 어댑터 5A 대비 {peak[2] / 50:.0f}%")
    out(f"부하: 최대 {hot[6]:.0f}% ({hot[5]})")
    out(f"초당 {hz:.0f}회 읽음. 기록: {path}")
    out("참고: 전류는 레지스터 값 x 6.5mA 추정이다. 부하(%)와 같이 오르내리는지 보고, 부하가 큰데 전류가 거의 0이면 전류 값은 믿지 않는다.")
    return res
