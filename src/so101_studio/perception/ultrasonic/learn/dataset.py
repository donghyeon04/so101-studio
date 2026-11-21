"""truth.pairs(run_dir) → 확인 하나당 표본 하나(8.8 입력). 좌표는 후보 중심 기준이라 자리가 달라도 같은 꼴이다."""
import math
from pathlib import Path

import numpy as np

from .. import truth

STAGES = ("L1", "L2", "R", "L3", "L4")
MATCH_MM = 30.0  # 정답 물체가 이 안에 없으면 뺀다(8.8, 벤치마크 판정과 같은 값)


def _stage_feats(seg, cx, cy, hit_mm):
    """한 단계의 점 (x, y, z, d)에서 높이 h = z − d가 hit_mm를 넘는 점의 무게중심(후보 기준)과 개수·평균 높이."""
    hits = [(x - cx, y - cy, z - d) for x, y, z, d in seg.get("points") or [] if z - d > hit_mm]
    if not hits:
        return [0.0, 0.0, 0.0, 0.0]
    a = np.array(hits)
    return [a[:, 0].mean(), a[:, 1].mean(), math.log1p(len(hits)), a[:, 2].mean()]


def features(check: dict, hit_mm: float = 15.0) -> list[float]:
    """규칙 결과(중심 차이, 2φ의 cos·sin, 폭·길이, R, 원 반지름, 높이) + 단계별 메아리 무게중심."""
    cand, res, info = check["candidate"], check["result"], check.get("info") or {}
    cx, cy = cand["x_mm"], cand["y_mm"]
    ang = res.get("angle_deg")
    f = [res["x_mm"] - cx, res["y_mm"] - cy,
         math.cos(math.radians(2 * ang)) if ang is not None else 0.0, math.sin(math.radians(2 * ang)) if ang is not None else 0.0,
         float(ang is None), res.get("width_mm") or 0.0, res.get("length_mm") or 0.0, float(info.get("R") or 0.0),
         float(info.get("ring_r") or 0.0), float(info.get("h_mm") or 0.0), float(info.get("stages") or 0)]
    by = {s["stage"]: s for s in check.get("segments") or []}
    for st in STAGES:
        f += _stage_feats(by.get(st, {}), cx, cy, hit_mm)
    return f


def rows(run_dir) -> list[dict]:
    """표본 = {run, x(특징), rule(규칙 중심·각도), truth(정답 중심·각도·폭·길이), cand}. ok가 아니거나 30mm 안에 정답이 없으면 뺀다."""
    out = []
    for p, _, checks in truth.pairs(run_dir):
        for c in checks:
            res = c.get("result")
            if c.get("status") != "ok" or not res or not p["objects"]:
                continue
            o = min(p["objects"], key=lambda o: math.dist((o["x_mm"], o["y_mm"]), (res["x_mm"], res["y_mm"])))
            if math.dist((o["x_mm"], o["y_mm"]), (res["x_mm"], res["y_mm"])) > MATCH_MM:
                continue
            out.append({"run": Path(run_dir).name, "x": features(c), "cand": (c["candidate"]["x_mm"], c["candidate"]["y_mm"]),
                        "rule": (res["x_mm"], res["y_mm"], res.get("angle_deg")), "truth": (o["x_mm"], o["y_mm"], o["angle_deg"])})
    return out
