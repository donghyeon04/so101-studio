"""초음파 성능 벤치마크 (ultrasonic-spec 8.5절, 결정 0037): 가상 ESP32 + 가상 팔 위에서 실제 radar.py·check.py·source.py를 돌린다.
성능 숫자는 이 도구로만 낸다. 새 아이디어(확인선 길이, 모드, 속도 등)는 옵션으로 붙인다.

python -m so101_studio.perception.ultrasonic.benchmark [--scenario spread90|spread120|demo|single|all] [--seeds N] [--mode accurate|fast|both]
                                                       [--model optimistic|pessimistic|both] [--line-mm N] [--speed N] [--verify]
"""
import argparse
import itertools
import json
import math
import random
import statistics
import subprocess
import tempfile
import time
from bisect import bisect_right
from pathlib import Path

from ...core.config import load_workspace
from .fake_esp32 import FakeESP32, SimClock
from .link import Link
from .model import Box, PoseSample, load_config
from .source import UltrasonicSource

DT = 0.02  # 팔 50Hz
TOOL_V_MAX = 100.0  # robot.yaml tool_v_max_mm_s
REACH = (50.0, 265.0)  # 8.5절 손끝 도달 반경(REQ-023 계산)

# 8.5절 센서 모델. 실측(B1·B2·B4)으로 고친다.
MODELS = {
    "optimistic": {"wrist_beam_half_deg": 7.5, "max_incidence_deg": 12.0, "corner": (0.7, 450.0, 0.9), "servo_lag_deg": 3.0},
    "pessimistic": {"wrist_beam_half_deg": 15.0, "max_incidence_deg": 8.0, "corner": (0.7, 300.0, 0.5), "servo_lag_deg": 3.0},
}
OBJECT_SIZES = ((30, 100), (40, 80), (50, 50))  # 폭 x 길이, 높이 40
SCENARIOS = {  # 이름: (물체 수, 반경 mm, 방위 °, 중심 간격 mm 이상)
    "spread90": (3, (100, 240), (25, 155), 90),
    "spread120": (3, (100, 240), (25, 155), 120),
    "demo": (3, (120, 220), (25, 155), 120),
    "single": (1, (100, 240), (25, 155), 0),
}
SINGLE_RADII = (100, 120, 140, 160, 180, 200, 220, 240)


def arm_base() -> tuple[float, float]:
    b = load_workspace().base
    return b.x_mm, b.y_mm


class FakeArm:
    """가상 팔: 손끝 위치 기록, 센서 위치(손끝 + yaw로 돌린 오프셋), follow_path(REQ-023 모양).
    시간 모델(8.5절): 호출마다 정지에서 출발·정지로 끝, 가속·감속 accel, 호출 사이 gap_s. 못 가는 경로는 움직이지 않고 []."""

    def __init__(self, clk, cfg, base, travel_mm_s=400.0, reach=REACH, accel=120.0, gap_s=0.25, motion_scale=1.0):
        self.clk, self.base, self.reach = clk, base, reach
        self.travel, self.accel, self.gap_s, self.motion_scale = travel_mm_s, accel, gap_s, motion_scale
        self.off0 = (cfg.arm_sensor_offset_mm.x, cfg.arm_sensor_offset_mm.y, cfg.arm_sensor_offset_mm.z)
        self.yaw = 90.0  # 평소 손목 자세
        self.tip = base
        self.z = 150.0
        self.hist = [(0, base, self.z)]  # (t_ns, 손끝 xy, 손끝 z)
        self.last_path = None
        self.calls = []  # (경로 길이 mm, 걸린 초)

    def _sensor(self, xy, z):
        a = math.radians(self.yaw)
        x, y, oz = self.off0
        return xy[0] + x * math.cos(a) - y * math.sin(a), xy[1] + x * math.sin(a) + y * math.cos(a), z + oz

    def _tick(self, xy, z):
        self.clk.sleep(DT)
        self.tip, self.z = xy, z
        self.hist.append((self.clk.now_ns(), xy, z))
        return PoseSample(self.clk.now_ns(), (*xy, z), self._sensor(xy, z))

    def reachable(self, xy):
        if self.reach is None:
            return True
        return self.reach[0] <= math.hypot(xy[0] - self.base[0], xy[1] - self.base[1]) <= self.reach[1]

    def _at(self, t):
        i = bisect_right([h[0] for h in self.hist], t) - 1
        return self.hist[min(max(i, 0), len(self.hist) - 1)]

    def sensor_at(self, t):
        _, xy, z = self._at(t)
        return self._sensor(xy, z)

    def links_at(self, t):
        _, xy, _ = self._at(t)
        mid = ((self.base[0] + xy[0]) / 2, (self.base[1] + xy[1]) / 2)
        return [self.base, mid, xy, self._sensor(xy, 0)[:2]]

    def move(self, to, z, speed=None):
        """접근·복귀 이동(일정 속도). follow_path 바깥."""
        L = math.dist(self.tip, to)
        n = max(1, round(L / (speed or self.travel) / DT))
        a = self.tip
        for k in range(1, n + 1):
            self._tick((a[0] + (to[0] - a[0]) * k / n, a[1] + (to[1] - a[1]) * k / n), z)

    def _profile_walk(self, pts, z, vmax):
        seg = [math.dist(a, b) for a, b in itertools.pairwise(pts)]
        L = sum(seg)
        a = self.accel / self.motion_scale**2
        v = min(vmax / self.motion_scale, math.sqrt(a * L)) if L else 0.0
        ta, tc = v / a, (L - v * v / a) / v if v else 0.0
        T = 2 * ta + tc
        out, t = [], 0.0
        while t < T - 1e-9:
            t = min(t + DT, T)
            s = 0.5 * a * t * t if t < ta else (v * v / (2 * a) + v * (t - ta) if t < ta + tc else L - 0.5 * a * (T - t) ** 2)
            i, rem = 0, s
            while i < len(seg) - 1 and rem > seg[i]:
                rem -= seg[i]
                i += 1
            f = rem / seg[i] if seg[i] else 1.0
            p, q = pts[i], pts[i + 1]
            out.append(self._tick((p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f), z))
        return out

    def follow_path(self, pts, z, speed, yaw=None):
        bad = [p for p in pts if not self.reachable(p)]
        if bad:
            self.last_path = {"reason": "unreachable", "points": bad}
            return []
        if yaw is not None:
            self.yaw = yaw
        speed = min(speed, TOOL_V_MAX)
        if math.dist(self.tip, pts[0]) > 0.5 or abs(self.z - z) > 0.5:
            self.move(pts[0], z)
        self.clk.sleep(self.gap_s)
        t0 = self.clk.now_ns()
        first = PoseSample(self.clk.now_ns(), (*pts[0], z), self._sensor(pts[0], z))
        poses = [first, *self._profile_walk(pts, z, speed)]
        self.calls.append((sum(math.dist(a, b) for a, b in itertools.pairwise(pts)), (self.clk.now_ns() - t0) / 1e9))
        return poses


def build(boxes, mode="accurate", seed=0, model="optimistic", reach=REACH, arm_kw=None, esp_kw=None, radar=None, **check):
    """설정 + 가상 시계 + 가상 ESP32 + 가상 팔. model: MODELS 이름 또는 None(이상적 센서)."""
    cfg = load_config(mode=mode, radar=radar, **check)
    clk = SimClock()
    holder = {}
    kw = dict(MODELS[model]) if model else {}
    kw.update(esp_kw or {})
    esp = FakeESP32(clk, boxes, cfg, seed=seed, arm_sensor_fn=lambda t: holder["a"].sensor_at(t),
                    arm_links_fn=lambda t: holder["a"].links_at(t), **kw)
    arm = holder["a"] = FakeArm(clk, cfg, arm_base(), reach=reach, **(arm_kw or {}))
    return cfg, clk, esp, arm


def make_source(cfg, clk, esp, arm, baseline_dir=None, calibrate=True):
    path = Path(baseline_dir or tempfile.mkdtemp()) / "baseline.json"
    src = UltrasonicSource(Link(esp, clk), cfg, arm_origin_mm=arm.base, arm_links_fn=arm.links_at, baseline_path=path)
    src.connect()
    if calibrate:
        boxes, esp.boxes = esp.boxes, []  # 빈 작업대로 기준값
        src.calibrate()
        esp.boxes = boxes
    return src


def do_check(src, arm, cand):
    probe = src.start_check(cand)
    t0 = arm.clk.now_ns()
    while (seg := probe.next_segment()) is not None:
        probe.feed(arm.follow_path(seg.points_mm, seg.tool_z_mm, seg.speed_mm_s, seg.yaw_deg))
    return probe.result(), probe, (arm.clk.now_ns() - t0) / 1e9


def angle_ok(est, true_deg):
    if est is None:
        return False
    d = abs(est - true_deg) % 180
    return min(d, 180 - d) <= 22.5


def random_layout(rng, name) -> list[Box]:
    """8.5절 시나리오: 팔 원점 기준 반경·방위 범위 안, 중심 간격 이상. 물체는 30x100·40x80·50x50, 방향 0/45/90/135° ±5°."""
    n, (r0, r1), (b0, b1), spacing = SCENARIOS[name]
    bx, by = arm_base()
    for _ in range(10000):
        boxes = []
        for _ in range(n):
            r, th = rng.uniform(r0, r1), math.radians(rng.uniform(b0, b1))
            w, length = rng.choice(OBJECT_SIZES)
            boxes.append(Box(bx + r * math.cos(th), by + r * math.sin(th), rng.choice((0, 45, 90, 135)) + rng.uniform(-5, 5), w, length))
        if all(math.dist((a.x, a.y), (b.x, b.y)) >= spacing for a, b in itertools.combinations(boxes, 2)):
            return boxes
    raise RuntimeError("배치를 못 만들었다")


def run_cycle(src, arm, esp, place_s=5.0, drop=(500.0, 150.0), max_iter=12, verify_at_home=False):
    """처음 scan()부터 물체가 다 없어질 때까지(작업 실행 흉내). 인식 시간 = 레이더 + 확인, 집기·놓기는 뺀다.
    판정(8.5절): 확인 결과 자리에 가장 가까운 실제 물체가 30mm 안이면 그 물체를 집은 것. 위치 오차 15mm 이하이고 방향까지 맞으면 성공(정사각형은 위치만).
    verify_at_home: (설계서 2절은 "후보 없음 → DONE"이라 기본은 끔) 켜면 후보가 없을 때 팔을 홈으로 옮겨(배출 구역 근처 물체를 가리던 팔을 치운 뒤) 한 번 더 scan()하고 끝낸다.
    prefetch는 PLACE 시작(집은 뒤 배출 구역으로 옮기기 시작할 때)에 건다(2절·T5)."""
    clk, recog, log, ok, verified = arm.clk, 0.0, [], 0, False
    src.begin_cycle()
    for _ in range(max_iter):
        t0 = clk.now_ns()
        scan = src.scan()
        recog += (clk.now_ns() - t0) / 1e9
        near = [o for o in scan.objects if 90 <= math.dist((o.x_mm, o.y_mm), arm.base) <= 250]
        if not near:
            if verify_at_home and not verified and math.dist(arm.tip, arm.base) > 1:
                verified = True
                arm.move(arm.base, 150)
                continue
            break
        cand = min(near, key=lambda o: math.dist((o.x_mm, o.y_mm), arm.base))
        obj, probe, dt = do_check(src, arm, cand)
        recog += dt
        if obj is None:
            log.append({"result": probe.status, "cand": (cand.x_mm, cand.y_mm)})
            continue
        box = min(esp.boxes, key=lambda b: math.dist((b.x, b.y), (obj.x_mm, obj.y_mm)), default=None)
        err = math.dist((box.x, box.y), (obj.x_mm, obj.y_mm)) if box else 1e9
        picked = err <= 30
        square = picked and box.width == box.length
        good = picked and err <= 15 and (square or angle_ok(obj.angle_deg, box.angle_deg % 180))
        log.append({"result": "ok" if good else "bad" if picked else "miss", "err": err})
        ok += good
        if picked:
            esp.boxes.remove(box)
        src.notify_picked(obj.id)
        arm.move((obj.x_mm, obj.y_mm), 55)
        src.prefetch(arm.links_at)
        arm.move(drop, 55)
        clk.sleep(max(place_s - math.dist((obj.x_mm, obj.y_mm), drop) / arm.travel, 0))
    return ok, recog, log


def run_scenario(name, model, seeds, mode, verify=False, **opts):
    """시나리오 하나: 씨앗값 0..seeds-1마다 배치를 만들어 사이클을 돈다. single은 반경 구간(SINGLE_RADII)마다 seeds번."""
    rows = []
    cells = [(r, s) for r in SINGLE_RADII for s in range(seeds)] if name == "single" else [(None, s) for s in range(seeds)]
    for radius, seed in cells:
        rng = random.Random(f"{name}-{radius}-{seed}")
        if radius is not None:
            bx, by = arm_base()
            th = math.radians(rng.uniform(25, 155))
            w, length = rng.choice(OBJECT_SIZES)
            boxes = [Box(bx + radius * math.cos(th), by + radius * math.sin(th), rng.choice((0, 45, 90, 135)) + rng.uniform(-5, 5), w, length)]
        else:
            boxes = random_layout(rng, name)
        n = len(boxes)
        cfg, clk, esp, arm = build(boxes, mode=mode, seed=seed, model=model, **opts)
        src = make_source(cfg, clk, esp, arm)
        ok, secs, log = run_cycle(src, arm, esp, verify_at_home=verify)
        rows.append({"radius": radius, "seed": seed, "n": n, "ok": ok, "recog_s": secs, "left": len(esp.boxes), "log": log})
    return rows


def summarize(rows) -> dict:
    n = rows[0]["n"]
    secs = sorted(r["recog_s"] for r in rows)
    out = {"runs": len(rows), "objects": n, "success_mean": statistics.mean(r["ok"] for r in rows),
           "success_pct": 100 * statistics.mean(r["ok"] for r in rows) / n, "left_mean": statistics.mean(r["left"] for r in rows),
           "recog_mean_s": statistics.mean(secs), "recog_p90_s": secs[min(int(len(secs) * 0.9), len(secs) - 1)]}
    if rows[0]["radius"] is not None:
        out["by_radius_pct"] = {r: 100 * statistics.mean(x["ok"] for x in rows if x["radius"] == r) for r in SINGLE_RADII}
    return out


def commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], check=False, capture_output=True, text=True, timeout=5).stdout.strip() or "?"
    except Exception:  # noqa: BLE001  git이 없어도 벤치마크는 돈다
        return "?"


def format_table(results: list[dict]) -> str:
    lines = [f"commit {results[0]['commit']}", "| 시나리오 | 모델 | 모드 | 수 | 성공 평균 | 성공 % | 남은 물체 | 인식 평균(초) | 인식 90%(초) |", "|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        s = r["summary"]
        lines.append(f"| {r['scenario']} | {r['model']} | {r['mode']} | {s['runs']} | {s['success_mean']:.2f}/{s['objects']} | {s['success_pct']:.0f}% | "
                     f"{s['left_mean']:.2f} | {s['recog_mean_s']:.0f} | {s['recog_p90_s']:.0f} |")
        if "by_radius_pct" in s:
            lines.append("|  | 반경별 성공 % | " + " ".join(f"{k}:{v:.0f}" for k, v in s["by_radius_pct"].items()) + " | | | | | | |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="all", help="spread90 | spread120 | demo | single | all")
    ap.add_argument("--seeds", type=int, default=20, help="시나리오마다 배치 수(8.5절: 20 이상). single은 반경 구간마다")
    ap.add_argument("--mode", default="both", help="accurate | fast | both")
    ap.add_argument("--model", default="both", help="optimistic | pessimistic | both")
    ap.add_argument("--line-mm", type=int, help="확인 직선 길이(기본 설정값)")
    ap.add_argument("--speed", type=float, help="확인 속도 mm/s(기본 설정값)")
    ap.add_argument("--verify", action="store_true", help="후보가 없으면 팔을 홈으로 옮겨 한 번 더 scan()한다(설계서 2절과 다른 흐름이라 기본 끔)")
    ap.add_argument("--out", default="runs/bench")
    a = ap.parse_args(argv)
    names = list(SCENARIOS) if a.scenario == "all" else [a.scenario]
    models = list(MODELS) if a.model == "both" else [a.model]
    modes = ["accurate", "fast"] if a.mode == "both" else [a.mode]
    opts = {k: v for k, v in (("line_mm", a.line_mm), ("speed_mm_s", a.speed)) if v is not None}
    results, sha = [], commit()
    for name, model, mode in itertools.product(names, models, modes):
        rows = run_scenario(name, model, a.seeds, mode, verify=a.verify, **opts)
        results.append({"commit": sha, "scenario": name, "model": model, "mode": mode, "options": {**opts, "verify": a.verify}, "summary": summarize(rows),
                        "rows": [{k: v for k, v in r.items() if k != "log"} for r in rows]})
        print(format_table(results[-1:]).split("\n", 1)[1] if len(results) > 1 else format_table(results), flush=True)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d-%H%M%S')}-{sha}.json"
    path.write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")
    print("saved", path)


if __name__ == "__main__":
    main()
