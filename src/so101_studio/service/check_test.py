"""팔 확인 실측 A1(`checktest a1`)과 I1 정답 기록(`run --source ultrasonic --truth-layout K`), ultrasonic-spec 8.6.
A1은 레이더를 쓰지 않고 정답 가까운 후보에서 CHECK 한 번씩만 돌려 확인 정확도를 잰다. 기록은 truth.py(TruthRun) 형식."""
import json
import math
import random
import re
import statistics
import time
from pathlib import Path

from ..core.config import load_workspace
from ..core.types import DetectedObject
from ..perception.ultrasonic import truth
from ..perception.ultrasonic.benchmark import (
    SCENARIOS,
    angle_ok,
    edge_gap,
    on_plate,
    random_layout,
)
from ..perception.ultrasonic.check import pick_yaw
from ..perception.ultrasonic.model import Box

A1_SPOTS = (("S1", 189, 169), ("S2", 125, 177), ("S3", 270, 224))
A1_ANGLES = (0, 45, 90, 135)
A1_N = len(A1_SPOTS) * len(A1_ANGLES)  # 한 묶음 12회
A0_SPOT = ("S1", 189, 169)  # 센서 오프셋 보정(8.7): 막대 하나, 같은 후보로 5회
A0_N = 5


def a1_plan(seed: int = 0, offset_mm: float = 30.0, obj=(40, 80, 40), reps: int = 1) -> list[dict]:
    """[{n, rep, spot, truth, cand}]: 자리마다 네 방향, 후보 = 자리 + 씨앗으로 고정한 오프셋(방위 0~360°, 거리 0~offset_mm)."""
    w, length, h = obj
    out = []
    for rep in range(1, reps + 1):
        for i in range(A1_N):
            n = i + 1
            spot, x, y = A1_SPOTS[i // len(A1_ANGLES)]
            rng = random.Random(f"a1-{seed}-{rep}-{n}")
            th, d = math.radians(rng.uniform(0, 360)), rng.uniform(0, offset_mm)
            cand = DetectedObject(f"a1-{rep}-{n}", x + d * math.cos(th), y + d * math.sin(th), None, 0.0, 0.0, 0.3, "ultrasonic")
            ang = None if w == length else A1_ANGLES[i % len(A1_ANGLES)]
            out.append({"n": n, "rep": rep, "spot": spot, "truth": truth.obj(x, y, ang, w, length, h, id="P1"), "cand": cand})
    return out


def run_a1(svc, runner, src, run, place, plan, per_rep: int = A1_N) -> list[dict]:
    """회마다 radar_clear → place(item)(정답 dict, None이면 끝) → check_candidate → run.add. aborted·radar_clear 실패면 멈춘다.
    per_rep: 한 묶음 회 수(표시 n/per_rep, a0는 A0_N). 행에 결과 x_mm·y_mm·height_mm(ok 아니면 None)·truth_xy·truth_height_mm·cand_xy(8.7)."""
    res = []
    try:
        for item in plan:
            c = item["cand"]
            row = {"n": item["n"], "rep": item["rep"], "spot": item["spot"], "err_mm": math.inf, "angle_ok": None,
                   "square": item["truth"]["angle_deg"] is None, "check_s": None, "x_mm": None, "y_mm": None, "height_mm": None,
                   "cand_xy": (c.x_mm, c.y_mm)}
            if runner._wait_pose(True).status != "ok":
                res.append({**row, "status": "radar_clear"})
                print(f"  {item['n']}/{per_rep}: radar_clear 이동 실패, 멈춘다")
                break
            t = place(item)
            if t is None:
                break
            row.update(square=t["angle_deg"] is None, truth_xy=(t["x_mm"], t["y_mm"]), truth_height_mm=t["height_mm"])
            before, t0 = len(src.saved), time.time()
            obj, status = runner.check_candidate(item["cand"])
            row.update(status=status, check_s=round(time.time() - t0, 2))
            run.add([t], src.saved[before:], item["rep"])
            if status == "ok" and obj is not None:
                row.update(x_mm=obj.x_mm, y_mm=obj.y_mm, height_mm=obj.height_mm)
                row["err_mm"] = math.dist((obj.x_mm, obj.y_mm), (t["x_mm"], t["y_mm"]))
                row["angle_ok"] = None if row["square"] else angle_ok(obj.angle_deg, t["angle_deg"])
            res.append(row)
            err = "-" if math.isinf(row["err_mm"]) else f"{row['err_mm']:.1f}mm"
            ang = "-" if row["angle_ok"] is None else ("맞음" if row["angle_ok"] else "틀림")
            print(f"  {item['n']}/{per_rep} {item['spot']}: {status}, 위치 오차 {err}, 방향 {ang}, 확인 {row['check_s']:.1f}초")
            if status == "aborted":
                break
    finally:
        if runner._trail is not None:
            runner._trail.detach()
        run.close()
    return res


def a1_summary(res: list[dict]) -> str:
    """묶음(rep)마다: 분모 12, ok 아닌 회는 오차 +∞. 합격 = 중앙값 ≤ 5mm, 10mm 안 ≥ 11, 방향 맞음 ≥ 10(정사각형이면 뺀다)."""
    lines = []
    for rep in sorted({r["rep"] for r in res}) or [1]:
        rows = [r for r in res if r["rep"] == rep]
        errs = [r["err_mm"] for r in rows]
        med = statistics.median(errs + [math.inf] * (A1_N - len(errs))) if errs else math.inf
        near = sum(e <= 10 for e in errs)
        square = bool(rows) and all(r["square"] for r in rows)
        ang = sum(r["angle_ok"] is True for r in rows)
        text = f"중앙 {'-' if math.isinf(med) else f'{med:.1f}mm'}, 10mm 안 {near}/{A1_N}" + ("" if square else f", 방향 맞음 {ang}/{A1_N}")
        if len(rows) < A1_N or any(r["status"] in ("aborted", "radar_clear") for r in rows):
            verdict = f"미완료({len(rows)}/{A1_N}회)"
        else:
            verdict = "합격" if med <= 5 and near >= 11 and (square or ang >= 10) else "불합격"
        lines.append(f"A1 {rep}묶음: {verdict}  {text}")
    return "\n".join(lines)


def a0_plan(obj=(10, 10, 40)) -> list[dict]:
    """a1_plan과 같은 모양 A0_N개(rep 1): 정답 = S1의 막대(정사각형, 각도 None), 후보 = 막대 자리 그대로(오프셋 0)."""
    w, length, h = obj
    spot, x, y = A0_SPOT
    return [{"n": n, "rep": 1, "spot": spot, "truth": truth.obj(x, y, None, w, length, h, id="P1"),
             "cand": DetectedObject(f"a0-{n}", x, y, None, 0.0, 0.0, 0.3, "ultrasonic")} for n in range(1, A0_N + 1)]


def a0_delta(e, psi_deg: float) -> tuple[float, float]:
    """막대 자리 - 확인 중심 e를 손 좌표 오프셋 차이 (δx, δy)로(8.7): 수직 아래 손은 거울이라 δx = -(e·u), δy = e·v, u = (cosψ, sinψ), v = (-sinψ, cosψ)."""
    c, s = math.cos(math.radians(psi_deg)), math.sin(math.radians(psi_deg))
    return -(e[0] * c + e[1] * s), -e[0] * s + e[1] * c


def a0_offsets(res: list[dict], cfg, origin) -> list[tuple[float, float, float]]:
    """ok인 회마다 δ = (δx, δy, δz). ψ = CHECK가 쓴 손목 각도 pick_yaw(cfg, 후보 자리, origin). δz = 막대 높이 - 확인 높이."""
    out = []
    for r in res:
        if r.get("status") != "ok" or r.get("x_mm") is None:
            continue
        e = (r["truth_xy"][0] - r["x_mm"], r["truth_xy"][1] - r["y_mm"])
        dx, dy = a0_delta(e, pick_yaw(cfg, r["cand_xy"], origin))
        out.append((dx, dy, r["truth_height_mm"] - (r["height_mm"] or 0.0)))
    return out


def a0_summary(res: list[dict], cfg, origin) -> str:
    """5회 모두 ok여야 계산. 성분별 중앙값·퍼짐(최대-최소). 퍼짐 x·y > 4 또는 z > 6이면 불안정(값 없음).
    |중앙 x|·|중앙 y| <= 2, |중앙 z| <= 3이면 합격, 아니면 고칠 값(설정 + 중앙값, 0.1mm) 한 줄. 설정 파일은 쓰지 않는다."""
    ok = sum(r.get("status") == "ok" for r in res)
    if ok < A0_N:
        return f"A0: 미완료(ok {ok}/{A0_N}회)"
    ds = a0_offsets(res, cfg, origin)
    med = [statistics.median(d[i] for d in ds) for i in range(3)]
    spread = [max(d[i] for d in ds) - min(d[i] for d in ds) for i in range(3)]
    text = "δ 중앙 ({:+.1f}, {:+.1f}, {:+.1f})mm, 퍼짐 ({:.1f}, {:.1f}, {:.1f})mm".format(*med, *spread)
    if spread[0] > 4 or spread[1] > 4 or spread[2] > 6:
        return f"A0: 불안정(막대·브래킷 흔들림, 메아리 끊김)  {text}. 값을 내지 않는다"
    if abs(med[0]) <= 2 and abs(med[1]) <= 2 and abs(med[2]) <= 3:
        return f"A0: 합격: 지금 값 그대로  {text}"
    off = cfg.arm_sensor_offset_mm
    new = [round(v + m, 1) for v, m in zip((off.x, off.y, off.z), med, strict=True)]
    return f"A0: 고칠 값  {text}\narm_sensor_offset_mm: {{x: {new[0]:g}, y: {new[1]:g}, z: {new[2]:g}}}"


def i1_layout(k: int) -> list[Box]:
    """I1 배치 K: show80 배치(반경 150~220)를 좌표 5mm·각도 5°로 반올림한 물체 3개.
    반올림 뒤 판 안·모서리 80mm·반경 150~220(149.x가 생길 수 있다)을 다시 보고, 어기면 같은 rng로 다음 배치를 뽑는다. 20번 안에 안 되면 RuntimeError."""
    r5 = lambda v: round(v / 5) * 5
    rng, ws = random.Random(f"i1-{k}"), load_workspace()
    lo, hi = SCENARIOS["show80"][1]
    bx, by = ws.robot_base_in_table["x_mm"], ws.robot_base_in_table["y_mm"]
    for _ in range(20):
        boxes = [Box(r5(b.x), r5(b.y), r5(b.angle_deg), b.width, b.length, b.height) for b in random_layout(rng, "show80")]
        if (all(on_plate(b, ws) and lo <= math.dist((b.x, b.y), (bx, by)) <= hi for b in boxes)
                and all(edge_gap(a, b) >= 80 for i, a in enumerate(boxes) for b in boxes[i + 1:])):
            return boxes
    raise RuntimeError(f"I1 배치 {k}: 반올림 뒤 시연 배치 규칙을 지키는 배치를 못 만들었다")


def i1_truth(boxes: list[Box]) -> list[dict]:
    return [truth.obj(b.x, b.y, None if b.width == b.length else b.angle_deg % 180, b.width, b.length, b.height, id=f"P{i}")
            for i, b in enumerate(boxes, 1)]


def _read(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


def i1_summary(truth_dirs: list[str]) -> str:
    """실행마다 확인 ok 결과를 가장 가까운 정답(30mm 안)과 짝짓고 위치 15mm + 방향(정사각형은 위치만)이면 성공. 정답 하나에 첫 결과만.
    인식 시간 = note의 log= 폴더 summary.json에서 물체별 radar_s + check_s의 합. 합격: 평균 성공 2개 이상, 인식 평균 100초 이하."""
    goods, secs = [], []
    for d in map(Path, truth_dirs):
        data = _read(d / "truth.json")
        objs = [o for p in data["placements"] for o in p["objects"]]
        matched, good = set(), 0
        for f in (f for p in data["placements"] for f in p["checks"]):
            c = _read(d / f)
            if c.get("status") != "ok" or not c.get("result"):
                continue
            r = c["result"]
            o = min(objs, key=lambda o: math.dist((o["x_mm"], o["y_mm"]), (r["x_mm"], r["y_mm"])), default=None)
            if o is None or o["id"] in matched or (err := math.dist((o["x_mm"], o["y_mm"]), (r["x_mm"], r["y_mm"]))) > 30:
                continue
            matched.add(o["id"])
            good += err <= 15 and (o["angle_deg"] is None or angle_ok(r.get("angle_deg"), o["angle_deg"]))
        goods.append(good)
        m = re.search(r"log=(\S+)", data.get("note", ""))
        if m and (s := Path(m.group(1)) / "summary.json").exists():
            secs.append(sum(row.get("radar_s", 0.0) + row.get("check_s", 0.0) for row in _read(s).get("per_object", [])))
    if not goods:
        return "I1: 기록 없음"
    g = statistics.mean(goods)
    t = statistics.mean(secs) if secs else None
    verdict = "합격" if g >= 2 and t is not None and t <= 100 else "불합격"
    return (f"I1 {len(goods)}회: {verdict}  성공 평균 {g:.1f}개(합격 2 이상), 인식 평균 "
            + ("-" if t is None else f"{t:.0f}초") + f"(합격 100 이하, summary {len(secs)}/{len(goods)}회)")
