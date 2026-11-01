"""떨림 비교 시험: 같은 동작을 서보 가속 한계(25 / 254)와 속도(20 / 60 / 200°/s)를 바꿔가며 여러 번 왕복하고,
이동마다 6Hz 이상 흔들림(RMS°)을 따로 잰다. 진행 상황을 번호로 계속 출력하고, 떨림을 느끼면 m 키를 눌러 그 순간을 표시할 수 있다.
끝에 조건별 평균과 가장 많이 떤 이동, 표시한 순간에 진행 중이던 이동과 그때의 측정값을 보여준다.
코드 경로: 궤적(quintic) -> MotionController 50Hz 쓰기 -> 서보(가속 한계 설정값). 키보드 조그는 이 경로를 거치지 않는다(목표를 매 주기 바꿔 보냄)."""
import csv
import time
from dataclasses import replace

import numpy as np

from ..arm.trajectory import move_j
from .trace import Tracer, analyze

ACCELS = (25, 254)
SPEEDS = (20.0, 60.0, 200.0)  # °/s (가장 많이 움직이는 관절 기준 최대 속도)
A_MAX_DEG_S2 = 1500.0


def run_shaketest(svc, reps: int = 10, out=print, poll_mark=None) -> list[dict]:
    """poll_mark(): 호출할 때마다 떨림 표시 키가 눌렸는지(True/False)를 돌려주는 함수. 없으면 표시 기능을 쓰지 않는다."""
    svc.driver.set_torque(True)
    start = svc.driver.read_joints()
    goal = replace(start, shoulder_pan=start.shoulder_pan + 50, shoulder_lift=start.shoulder_lift + 40,
                   elbow_flex=start.elbow_flex - 40, wrist_flex=start.wrist_flex + 30)
    why = svc.safety.check_path(move_j(start, goal, SPEEDS[0], A_MAX_DEG_S2))
    if why:
        out(f"이 자세에서는 시험할 수 없습니다: {why}. 홈 자세에서 시작하세요.")
        return []
    rest_acc, rest_step = svc.driver.acceleration, svc.safety.max_step_deg
    svc.safety.max_step_deg = 8.0
    tr, moves, marks, total = Tracer(svc), [], [], len(ACCELS) * len(SPEEDS) * reps * 2
    tr.start()
    t_begin = time.perf_counter()
    try:
        for acc in ACCELS:
            svc.driver.set_acceleration(acc)
            for v in SPEEDS:
                out(f"\a=== 가속 한계 {acc}, 속도 {v:.0f}°/s ({reps}회 왕복) ===")
                for rep in range(reps):
                    for name, q in (("갈 때", goal), ("올 때", start)):
                        label = f"가속{acc}/{v:.0f}°/s/{rep + 1}번째 {name}"
                        t0 = time.perf_counter()
                        out(f"[{len(moves) + 1}/{total}] {label}  (시작 후 {t0 - t_begin:.0f}s)")
                        st = svc._run([move_j(svc.driver.read_joints(), q, v, A_MAX_DEG_S2)]).status
                        t1 = time.perf_counter()
                        moves.append({"label": label, "acc": acc, "speed": v, "t0": t0, "t1": t1, "status": st})
                        if poll_mark and poll_mark():
                            marks.append(time.perf_counter())
                            out(f"  >> 떨림 표시 ({label} 직후)")
                        if st != "ok":
                            out(f"  중간에 멈춤: {st} {svc.ctl.fault}")
                            raise RuntimeError(st)
    except RuntimeError:
        pass
    finally:
        svc.driver.set_acceleration(rest_acc)
        svc.safety.max_step_deg = rest_step
    tr.result()
    for m in moves:  # 이동마다 6Hz 이상 흔들림
        seg = tr.slice(m["t0"], m["t1"])
        res = analyze(*seg) if seg else []
        worst = max((r for r in res if "tremor_rms_deg" in r), key=lambda r: r["tremor_rms_deg"], default=None)
        m["rms"] = worst["tremor_rms_deg"] if worst else float("nan")
        m["joint"] = worst["joint"] if worst else "-"
        m["hz"] = worst["dominant_hz"] if worst else float("nan")
    out("\n=== 조건별 평균(이동마다 가장 많이 떤 관절의 6Hz 이상 흔들림 RMS°) ===")
    for acc in ACCELS:
        for v in SPEEDS:
            r = [m["rms"] for m in moves if m["acc"] == acc and m["speed"] == v and m["rms"] == m["rms"]]
            if r:
                out(f"가속 {acc:>3} 속도 {v:>4.0f}°/s  평균 {np.mean(r):.3f}°  최대 {np.max(r):.3f}°  ({len(r)}번)")
    out("\n=== 가장 많이 떤 이동 5개 ===")
    for m in sorted((m for m in moves if m["rms"] == m["rms"]), key=lambda m: -m["rms"])[:5]:
        out(f"{m['label']}  {m['joint']} {m['rms']:.3f}°/{m['hz']:.0f}Hz  (시작 후 {m['t0'] - t_begin:.0f}s)")
    for k in marks:
        near = [m for m in moves if m["t0"] <= k <= m["t1"] + 1.0]
        for m in near[-1:]:
            out(f"표시한 순간: {m['label']}  {m['joint']} {m['rms']:.3f}°/{m['hz']:.0f}Hz")
    path = svc.logger.dir / "shaketest_moves.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["label", "accel", "speed_deg_s", "start_s", "status", "worst_joint", "rms_deg", "hz"])
        w.writerows([m["label"], m["acc"], m["speed"], round(m["t0"] - t_begin, 2), m["status"], m["joint"], round(m["rms"], 4), round(m["hz"], 1)] for m in moves)
    out(f"기록: {path}")
    return moves
