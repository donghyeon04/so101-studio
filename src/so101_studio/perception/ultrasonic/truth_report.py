"""정답 쌍 분석 (ultrasonic-spec 8.4·10절, 결정 0072): runs/ultrasonic-truth/의 실행을 읽어 레이더(R1)·팔 확인(A1) 오차를 표로 낸다.

python -m so101_studio.perception.ultrasonic.truth_report <실행 폴더|all>
python -m so101_studio.perception.ultrasonic.truth_report all --set dip_mm=25 --set merge_mm=90   # 레이더 인자를 바꿔 다시 계산

레이더: 자리(물체)마다 그 자리 첫 scan의 가장 가까운 후보 오차. 합격은 95%가 90mm 안(10절 R1).
--set은 기록한 각도별 합친 값(values)과 기준값으로 find_candidates만 다시 돌린다. 그래서 후보 찾기 인자만 바꿀 수 있다(RECOMPUTE_KEYS).
팔 확인: check마다 결과와 가장 가까운 정답의 위치 오차, 방향(±22.5°). 목표는 중앙 5mm·90% 10mm(결정 0072). 확인 경로는 앞 결과에 따라
바뀌므로 기록만으로는 인자를 바꿔 다시 계산할 수 없다.
반경·방위는 팔 원점 기준, 통 표시는 배출 구역 무시 원(workspace ignore_zones) 안이면 "통 안", 둘레 50mm 안이면 "통 둘레"."""
import argparse
import json
import math
import statistics
from pathlib import Path

from ...core.config import load_workspace
from . import radar, truth
from .benchmark import angle_ok
from .model import _wrap

RECOMPUTE_KEYS = {"dip_mm", "min_arc", "split_mm", "merge_mm", "range_offset_mm", "max_mm", "face_offset_mm", "heading_deg"}
NEAR_BIN_MM = 50


def p90(v):
    s = sorted(v)
    return s[math.ceil(0.9 * len(s)) - 1]  # 가장 가까운 순위(벤치마크 summarize와 같다)


def _site(o, ws):
    b = ws.base
    dx, dy = o["x_mm"] - b.x_mm, o["y_mm"] - b.y_mm
    gap = min((math.hypot(o["x_mm"] - x, o["y_mm"] - y) - r for x, y, r in ws.ignore_zones()), default=math.inf)
    return {"r": math.hypot(dx, dy), "bearing": math.degrees(math.atan2(dy, dx)),
            "bin": "통 안" if gap < 0 else "통 둘레" if gap <= NEAR_BIN_MM else ""}


def recompute(scan: dict, cfg) -> list[tuple[float, float]]:
    """기록한 values·baseline으로 후보를 다시 찾는다(cfg = 바꾼 설정). 무시 구역·지운 후보는 적용하지 않는다(radar_check와 같다)."""
    num = lambda d: {int(float(a)): v for a, v in d.items()}
    base = radar.Baseline(num(scan["baseline"]["d"]), set(scan["baseline"]["unstable"]))
    cands, _ = radar.find_candidates(num(scan["values"]), base, cfg, {int(float(a)): v for a, v in (scan["shift"] or {}).items()})
    return [(c.x, c.y) for c in cands]


def radar_rows(run_dir, overrides=None, ws=None) -> list[dict]:
    ws = ws or load_workspace()
    cfg = None
    if overrides:
        t = json.loads((Path(run_dir) / "truth.json").read_text(encoding="utf-8"))
        cfg = _wrap(t["config"])
        cfg.radar.update(overrides)
    rows = []
    for p, scans, _ in truth.pairs(run_dir):
        if not scans:
            continue
        cands = recompute(scans[0], cfg) if cfg else [(c["x_mm"], c["y_mm"]) for c in scans[0]["candidates"]]
        for o in p["objects"]:
            err = min((math.dist(c, (o["x_mm"], o["y_mm"])) for c in cands), default=None)
            rows.append({"n": p["n"], "rep": p["rep"], "x": o["x_mm"], "y": o["y_mm"], "cands": len(cands), "err": err, **_site(o, ws)})
    return rows


def check_rows(run_dir, ws=None) -> list[dict]:
    ws = ws or load_workspace()
    rows = []
    for p, _, checks in truth.pairs(run_dir):
        for c in checks:
            res = c["result"]
            row = {"n": p["n"], "status": c["status"], "err": None, "angle": None}
            if res and p["objects"]:
                o = min(p["objects"], key=lambda o: math.dist((o["x_mm"], o["y_mm"]), (res["x_mm"], res["y_mm"])))
                row.update(x=o["x_mm"], y=o["y_mm"], err=math.dist((o["x_mm"], o["y_mm"]), (res["x_mm"], res["y_mm"])), **_site(o, ws))
                if o["angle_deg"] is not None:
                    row["angle"] = angle_ok(res["angle_deg"], o["angle_deg"])
            rows.append(row)
    return rows


def radar_summary(rows, limit=90.0) -> dict:
    errs = [r["err"] for r in rows]
    found = [e for e in errs if e is not None]
    ok = sum(e <= limit for e in found)
    out = {"n": len(errs), "within": ok, "pct": 100 * ok / len(errs) if errs else 0.0, "pass": bool(errs) and ok >= 0.95 * len(errs),
           "none": errs.count(None)}
    if found:
        out.update(median=statistics.median(found), p90=p90(found), max=max(found))
    return out


def check_summary(rows) -> dict:
    errs = [r["err"] for r in rows if r["err"] is not None]
    ang = [r["angle"] for r in rows if r["angle"] is not None]
    out = {"n": len(rows), "measured": len(errs), "angle_ok": sum(ang), "angle_n": len(ang),
           "status": {s: sum(r["status"] == s for r in rows) for s in sorted({r["status"] for r in rows})}}
    if errs:
        out.update(median=statistics.median(errs), p90=p90(errs), max=max(errs),
                   pass_=statistics.median(errs) <= 5 and p90(errs) <= 10)
    return out


def _mm(v):
    return "-" if v is None else f"{v:.0f}"


def _by_site(rows):
    """자리(x, y)별 반복을 묶는다: 가장 큰 오차와 후보 없음 수. 반경 순."""
    sites: dict = {}
    for r in rows:
        sites.setdefault((r["x"], r["y"]), []).append(r)
    out = []
    for (x, y), rs in sorted(sites.items(), key=lambda kv: (kv[1][0]["r"], kv[1][0]["bearing"])):
        found = [r["err"] for r in rs if r["err"] is not None]
        out.append(f"  ({x:.0f},{y:.0f}) 반경 {rs[0]['r']:.0f} 방위 {rs[0]['bearing']:.0f}°: 최대 {_mm(max(found, default=None))}mm"
                   f" 평균 {_mm(statistics.mean(found) if found else None)}mm, 후보 없음 {len(rs) - len(found)}/{len(rs)} {rs[0]['bin']}")
    return out


def report(run_dirs, overrides=None) -> str:
    ws = load_workspace()
    lines = []
    rr = [dict(r, run=Path(d).name) for d in run_dirs for r in radar_rows(d, overrides, ws)]
    cr = [dict(r, run=Path(d).name) for d in run_dirs for r in check_rows(d, ws)]
    if overrides:
        lines.append("다시 계산: " + ", ".join(f"{k}={v}" for k, v in overrides.items()))
    if rr:
        s = radar_summary(rr)
        lines.append(f"레이더: {s['within']}/{s['n']}이 90mm 안 ({s['pct']:.0f}%, 합격 95% {'통과' if s['pass'] else '불합격'}), "
                     f"중앙 {_mm(s.get('median'))} · 90% {_mm(s.get('p90'))} · 최대 {_mm(s.get('max'))}mm, 후보 없음 {s['none']}")
        lines += _by_site(rr)
    if cr:
        s = check_summary(cr)
        verdict = "" if "median" not in s else f" (목표 중앙 5·90% 10mm {'통과' if s['pass_'] else '불합격'})"
        lines.append(f"팔 확인: {s['measured']}/{s['n']}회 결과, 중앙 {_mm(s.get('median'))} · 90% {_mm(s.get('p90'))} · "
                     f"최대 {_mm(s.get('max'))}mm{verdict}, 방향 {s['angle_ok']}/{s['angle_n']}, 상태 {s['status']}")
        lines += _by_site([r for r in cr if r["err"] is not None])
    if not rr and not cr:
        lines.append("scan·check 기록이 없다")
    return "\n".join(lines)


def _runs(target, root=None) -> list[Path]:
    root = Path(root or truth.ROOT)
    if target != "all":
        return [Path(target)]
    return [root / Path(line["run"]).name for line in truth.index(root)]


def _value(v: str):
    f = float(v)
    return int(f) if f.is_integer() else f


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="실행 폴더 또는 all(index.jsonl의 모든 실행)")
    ap.add_argument("--root", help=f"all의 기록 폴더(기본 {truth.ROOT})")
    ap.add_argument("--set", action="append", default=[], metavar="키=값", help="레이더 인자를 바꿔 다시 계산: " + ", ".join(sorted(RECOMPUTE_KEYS)))
    a = ap.parse_args(argv)
    over = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        if k not in RECOMPUTE_KEYS:
            ap.error(f"{k}: 기록만으로 다시 계산할 수 없다(바퀴 합치기·팔 지우기 인자는 팔 링크 기록이 필요하다). 쓸 수 있는 키: {', '.join(sorted(RECOMPUTE_KEYS))}")
        over[k] = _value(v)
    runs = _runs(a.target, a.root)
    if not runs:
        ap.error("실행이 없다")
    print(f"실행 {len(runs)}개: " + ", ".join(r.name for r in runs))
    print(report(runs, over or None))


if __name__ == "__main__":
    main()
