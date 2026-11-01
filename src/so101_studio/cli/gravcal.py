"""중력 보상 보정: 여러 자세에서 서보가 팔 무게를 버티는 부하를 재서 URDF 중력 모델과 맞춘다.
1) 정지 자세 18곳에서 부하(Present_Load, 부호 있음)를 평균한다. 2) 교시용 P 계수에서 위치 오차 1°당 부하(s)를 잰다(작은 계단 입력).
3) 관절별로 부하 = k x 중력토크 + b 를 구해 config/gravity.json에 저장한다. 팔이 작업대 위 안전 범위 안에서 천천히(30%) 움직인다."""
import time
from dataclasses import astuple, replace

import numpy as np

from ..arm import gravity
from ..arm.kinematics import ARM_JOINTS
from ..arm.trajectory import move_j
from ..core.types import JointState

SETTLE_S = 1.2
# (어깨 들기, 팔꿈치, 손목 굽힘) — 어깨 회전 0, 손목 회전 0. 안전 검사를 통과하지 못하는 자세는 건너뛴다
POSES = [(-85, 90, 0), (-70, 70, 20), (-55, 50, -20), (-40, 30, 0), (-25, 10, 30), (-10, -10, -30), (5, -30, 0), (20, -45, 20),
         (35, -60, -20), (50, -70, 0), (65, -75, 15), (-70, 40, -30), (-45, 70, 30), (-20, 50, 0), (0, 20, -25), (15, 0, 25),
         (30, -20, -10), (-60, 10, 0)]
SLOPE_POSES = (3, 7, 13)  # POSES 번호. 이 자세들에서 계단 입력으로 s를 잰다
STEP_DEG = 2.0


def mean_loads(svc, n: int = 12, gap: float = 0.05) -> dict:
    rows = []
    for _ in range(n):
        rows.append(svc.driver.read_loads_signed())
        time.sleep(gap)
    return {j: float(np.mean([r[j] for r in rows])) for j in rows[0]}


def step_slope(svc, joint: str) -> float:
    """위치 오차 1°당 부하(단위/°). 목표를 ±2° 한 번 옮기고 바로 부하의 최대 변화를 본다(팔이 움직이기 전의 힘)."""
    drv = svc.driver
    q0 = drv.read_joints()
    base = mean_loads(svc, 5, 0.02)[joint]
    peaks = []
    for sign in (1, -1):
        drv.write_joints(replace(q0, **{joint: getattr(q0, joint) + sign * STEP_DEG}))
        end, dev = time.perf_counter() + 0.15, 0.0
        while time.perf_counter() < end:
            dev = max(dev, abs(drv.read_loads_signed()[joint] - base))
        peaks.append(dev)
        drv.write_joints(q0)
        time.sleep(0.7)
    return float(np.mean(peaks) / STEP_DEG)


def run_gravcal(svc, out=print) -> dict | None:
    with svc.safety.relaxed(check_xy=False):  # 팔을 뻗는 자세가 작업대 가장자리 밖으로 나가므로 가장자리 검사만 끈다(바닥·관절 한계는 유지)
        return _gravcal(svc, out)


def _gravcal(svc, out) -> dict | None:
    drv, cfg = svc.driver, svc.cfg
    g = gravity.Gravity(cfg.urdf)
    drv.set_torque(True)
    p_before = drv.get_p()
    p_teach = {**cfg.teach_p, **cfg.teach_p_gravity}
    taus, loads, slopes = [], [], {j: [] for j in gravity.GRAVITY_JOINTS}
    try:
        for i, (lift, elbow, wflex) in enumerate(POSES):
            goal = JointState(0, lift, elbow, wflex, 0, svc.driver.read_joints().gripper)
            why = svc.safety.check_path(move_j(drv.read_joints(), goal, 30, 120))
            if why:
                out(f"[{i + 1}/{len(POSES)}] 건너뜀 {POSES[i]}: {why}")
                continue
            r = svc.move_joints(goal, speed_pct=30)
            if r.status != "ok":
                out(f"[{i + 1}/{len(POSES)}] 이동 실패 {r.status}")
                continue
            time.sleep(SETTLE_S)
            if not drv.torque_enabled():  # 전원이 순간 끊기는 등으로 토크가 풀리면 더 진행하지 않는다
                out(f"[{i + 1}/{len(POSES)}] 토크가 꺼졌습니다. 중단합니다. 팔을 받치고 전원·케이블을 확인하세요.")
                break
            q, ld = drv.read_joints(), mean_loads(svc)
            taus.append(g.torques(q))
            loads.append(ld)
            with (svc.logger.dir / "gravcal.csv").open("a", encoding="utf-8") as f:  # 도중에 끊겨도 잰 값은 남긴다
                f.write(",".join(f"{v:.3f}" for v in [*astuple(q), *taus[-1], *[ld[j] for j in gravity.GRAVITY_JOINTS]]) + chr(10))
            out(f"[{i + 1}/{len(POSES)}] 자세 {POSES[i]}  부하 " + " ".join(f"{j[:5]} {ld[j]:+.0f}" for j in gravity.GRAVITY_JOINTS))
            if i in SLOPE_POSES:
                drv.set_p({j: p_teach[j] for j in gravity.GRAVITY_JOINTS})
                for j in gravity.GRAVITY_JOINTS:
                    slopes[j].append(step_slope(svc, j))
                drv.set_p(p_before)
    finally:
        drv.set_p(p_before)
    if len(taus) < 8:
        out("측정한 자세가 너무 적어 보정하지 않았습니다.")
        return None
    taus = np.array(taus)
    params = {}
    for j in gravity.GRAVITY_JOINTS:
        f = gravity.fit(taus[:, ARM_JOINTS.index(j)], np.array([ld[j] for ld in loads]))
        f["s"] = float(np.median(slopes[j])) if slopes[j] else 0.0
        params[j] = f
        flag = "" if f["r2"] >= 0.6 and f["s"] > 0 else "  <- 신뢰 낮음"
        out(f"{j}: 부하 = {f['k']:+.0f} x 중력토크 {f['b']:+.0f}  (r2 {f['r2']:.2f}, 잔차 {f['resid_std']:.0f}), 1°당 부하 s={f['s']:.1f}{flag}")
    good = {j: f for j, f in params.items() if f["r2"] >= 0.6 and f["s"] > 0 and f["k"] > 0}  # 부호가 반대거나 설명력이 낮은 관절은 쓰지 않는다
    if not good:
        out("믿을 만한 보정이 없어 저장하지 않았습니다 (정지 부하가 중력보다 마찰·접근 방향의 영향을 더 받음).")
        return params
    gravity.save(good)
    out(f"저장: {gravity.DEFAULT_PATH} ({', '.join(good)})")
    return params
