"""떨림 비교 시험: 같은 동작을 서보 가속 한계(25 / 254)와 속도(느림 / 중간 / 빠름)를 바꿔가며 왕복하고,
움직이는 동안 관절별 떨림 크기(RMS°)를 표로 비교한다. '느릴 때 떨리는가', '가속 한계가 영향을 주는가'를 수치로 답하는 용도다."""
from dataclasses import replace

from ..arm.trajectory import move_j
from .trace import Tracer

ACCELS = (25, 254)
SPEEDS = (20.0, 60.0, 200.0)  # °/s (가장 많이 움직이는 관절 기준 최대 속도)
A_MAX_DEG_S2 = 1500.0


def run_shaketest(svc, out=print) -> list[dict]:
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
    table = []
    try:
        for acc in ACCELS:
            svc.driver.set_acceleration(acc)
            for v in SPEEDS:
                tr = Tracer(svc)
                tr.start()
                ok = [svc._run([move_j(svc.driver.read_joints(), q, v, A_MAX_DEG_S2)]).status for q in (goal, start)]
                res = tr.result() or []
                row = {"accel": acc, "speed": v, "status": ok, **{r["joint"]: r.get("tremor_rms_deg") for r in res}}
                table.append(row)
                out(f"가속 {acc:>3}  속도 {v:>5.0f}°/s  {ok}  " + "  ".join(
                    f"{r['joint'][:8]} {r['tremor_rms_deg']:.3f}°/{r['dominant_hz']:.0f}Hz" for r in res if "tremor_rms_deg" in r))
    finally:
        svc.driver.set_acceleration(rest_acc)
        svc.safety.max_step_deg = rest_step
    out("각 칸은 떨림 RMS°/지배 주파수. 값이 클수록 많이 떨린 것이다.")
    return table
