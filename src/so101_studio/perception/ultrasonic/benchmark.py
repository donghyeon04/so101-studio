"""초음파 성능 벤치마크 (ultrasonic-spec 8.5절, 결정 0037): 가상 ESP32 + 가상 팔 위에서 실제 radar.py·check.py·source.py를 돌린다.
성능 숫자는 이 도구로만 낸다. 새 아이디어(확인선 길이, 모드, 속도 등)는 옵션으로 붙인다.

python -m so101_studio.perception.ultrasonic.benchmark [--scenario spread90|spread120|demo|single|all] [--seeds N] [--mode accurate|fast|both]
                                                       [--model optimistic|pessimistic|both] [--line-mm N] [--speed N] [--rescan-home on|off] [--max-fail N] [--empty-extra N]
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
    시간 모델(8.5절, #77): 호출마다 정지에서 출발·정지로 끝, 가속·감속 accel, 호출 사이 gap_s, 한 호출 안에서는 서지 않되
    꺾임·원에서 가속 한도를 넘지 않게 속도를 줄인다(원은 v²/r <= accel, 반지름 45mm면 약 73mm/s). 못 가는 경로는 움직이지 않고 []."""

    def __init__(self, clk, cfg, base, travel_mm_s=400.0, reach=REACH, accel=120.0, gap_s=0.25, motion_scale=1.0, corner_radius_mm=10.0):
        self.clk, self.base, self.reach = clk, base, reach
        self.travel, self.accel, self.gap_s, self.motion_scale = travel_mm_s, accel, gap_s, motion_scale
        self.corner_radius_mm = corner_radius_mm  # 모서리를 지나는 가상 곡률 반경(#77 가정, #73 결과로 고친다)
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

    def _vertex_limits(self, pts, vmax):
        """꺾임·원에서 가속 한도(accel)를 넘지 않게 줄인 속도(#77): 완전히 서는 것은 호출의 처음과 끝뿐이다.
        작은 꺾임(30° 미만, 원 같은 곡선을 꺾은선으로 근사한 것)은 곡률 반경 R = 이웃 구간 중 짧은 길이 / (2 sin(꺾임/2)) 에서 v = sqrt(accel x R).
        큰 꺾임(모서리)은 모서리를 corner_radius_mm로 둥글게 지난다고 보고 v = sqrt(accel x corner_radius_mm)."""
        out = []
        for i in range(1, len(pts) - 1):
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            l1, l2 = math.dist(a, b), math.dist(b, c)
            if l1 < 1e-9 or l2 < 1e-9:
                out.append(vmax)
                continue
            cosang = ((b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1])) / (l1 * l2)
            turn = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
            if turn < 1.0:
                out.append(vmax)
            elif turn < 30.0:
                out.append(min(vmax, math.sqrt(self.accel * min(l1, l2) / (2 * math.sin(math.radians(turn) / 2)))))
            else:
                out.append(min(vmax, math.sqrt(self.accel * self.corner_radius_mm)))
        return out

    def _profile_walk(self, pts, z, vmax):
        """폴리라인 전체를 한 번에 간다. 거리 1mm 단위로 속도 한도(꺾임·원 감속)를 잡고 가속·감속 120mm/s²로 앞뒤 훑어
        처음·끝 0에서 출발·정지하는 속도 곡선을 만든 뒤 50Hz로 샘플링한다."""
        seg = [math.dist(a, b) for a, b in itertools.pairwise(pts)]
        L = sum(seg)
        if L < 1e-9:
            return []
        a = self.accel / self.motion_scale**2
        vmax = vmax / self.motion_scale
        n = max(2, math.ceil(L))
        ds = L / n
        limit = [vmax] * (n + 1)
        cum = 0.0
        for k, v in enumerate(self._vertex_limits(pts, vmax)):
            cum += seg[k]
            limit[round(cum / ds)] = min(limit[round(cum / ds)], v)
        v = [0.0] + [0.0] * n
        for i in range(1, n + 1):  # 앞으로: 가속 한도
            v[i] = min(limit[i], math.sqrt(v[i - 1] ** 2 + 2 * a * ds))
        v[n] = 0.0
        for i in range(n - 1, -1, -1):  # 뒤로: 감속 한도
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a * ds))
        out, s_pos, t = [], 0.0, 0.0
        while s_pos < L - 1e-9:
            i = min(int(s_pos / ds), n - 1)
            vi = max((v[i] + v[i + 1]) / 2, 1.0)  # 0에서 출발할 때 멈추지 않게 바닥값
            step = max(min(vi * DT, L - s_pos), 1e-6)
            s_pos += step
            t += step / vi
            out.append(self._at_arclen(pts, seg, s_pos, z))
        return out

    def _at_arclen(self, pts, seg, s, z):
        i, rem = 0, s
        while i < len(seg) - 1 and rem > seg[i]:
            rem -= seg[i]
            i += 1
        f = rem / seg[i] if seg[i] else 1.0
        p, q = pts[i], pts[i + 1]
        return self._tick((p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f), z)

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


def run_cycle(src, arm, esp, place_s=5.0, drop=(500.0, 150.0), max_iter=12, rescan_home=None):
    """처음 scan()부터 물체가 다 없어질 때까지(작업 실행 흉내). 인식 시간 = 레이더 + 확인, 집기·놓기는 뺀다.
    판정(8.5절): 확인 결과 자리에 가장 가까운 실제 물체가 30mm 안이면 그 물체를 집은 것. 위치 오차 15mm 이하이고 방향까지 맞으면 성공(정사각형은 위치만).
    rescan_home(RESCAN, 기획 #77): 후보가 없으면 팔을 대기 자세로 옮겨 src.rescan()(4바퀴)을 한 번 하고, 후보가 있으면 계속, 또 없으면 끝낸다.
    대기 자세(PREPARE 자세)는 아직 정해지지 않아 팔 원점 위(arm.base, 높이 150mm)로 흉내 낸다. None이면 설정 radar.rescan_home을 따른다.
    check.max_fail_per_run: 확인 실패가 그만큼이면 끝낸다(src.give_up).
    prefetch는 PLACE 시작(집은 뒤 배출 구역으로 옮기기 시작할 때)에 건다(2절·T5)."""
    if rescan_home is None:
        rescan_home = src.cfg.radar.rescan_home
    clk, recog, log, ok, rescanned = arm.clk, 0.0, [], 0, False
    src.begin_cycle()
    for _ in range(max_iter):
        if src.give_up:
            break
        t0 = clk.now_ns()
        if rescanned:  # 대기 자세로 옮긴 뒤의 RESCAN 결과
            scan = src.rescan()
        else:
            scan = src.scan()
        recog += (clk.now_ns() - t0) / 1e9
        near = [o for o in scan.objects if 90 <= math.dist((o.x_mm, o.y_mm), arm.base) <= 250]
        if not near:
            if rescan_home and not rescanned:
                rescanned = True
                arm.move(arm.base, 150)  # 레이더 시야 밖 대기 자세(흉내)
                continue
            break
        rescanned = False  # 후보가 나왔으니 다음에 또 없으면 다시 RESCAN할 수 있다(연달아 두 번만 안 함)
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
        if not picked:  # 확인한 자리에 물체가 없어 집기 실패: 작업 실행이 실패를 알린다
            src.note_failure(cand.x_mm, cand.y_mm)
        ok += good
        if picked:
            esp.boxes.remove(box)
        src.notify_picked(obj.id)
        arm.move((obj.x_mm, obj.y_mm), 55)
        src.prefetch(arm.links_at)
        arm.move(drop, 55)
        clk.sleep(max(place_s - math.dist((obj.x_mm, obj.y_mm), drop) / arm.travel, 0))
    return ok, recog, log


def run_scenario(name, model, seeds, mode, rescan_home=None, **opts):
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
        ok, secs, log = run_cycle(src, arm, esp, rescan_home=rescan_home)
        rows.append({"radius": radius, "seed": seed, "n": n, "ok": ok, "recog_s": secs, "left": len(esp.boxes), "log": log})
    return rows


def summarize(rows) -> dict:
    n = rows[0]["n"]
    secs = sorted(r["recog_s"] for r in rows)
    out = {"runs": len(rows), "objects": n, "success_mean": statistics.mean(r["ok"] for r in rows),
           "success_pct": 100 * statistics.mean(r["ok"] for r in rows) / n, "left_mean": statistics.mean(r["left"] for r in rows),
           "recog_mean_s": statistics.mean(secs), "recog_median_s": statistics.median(secs),
           "recog_p90_s": secs[math.ceil(0.9 * len(secs)) - 1],  # 가장 가까운 순위(n=20이면 18번째). 평균과 같은 표본
           "recog_max_s": secs[-1]}
    if rows[0]["radius"] is not None:
        out["by_radius_pct"] = {r: 100 * statistics.mean(x["ok"] for x in rows if x["radius"] == r) for r in SINGLE_RADII}
    return out


def commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], check=False, capture_output=True, text=True, timeout=5).stdout.strip() or "?"
    except Exception:  # noqa: BLE001  git이 없어도 벤치마크는 돈다
        return "?"


def format_table(results: list[dict]) -> str:
    lines = [f"commit {results[0]['commit']}", f"재현: {results[0].get('command', '?')}", "| 시나리오 | 모델 | 모드 | 수 | 성공 평균 | 성공 % | 남은 물체 | 인식 평균(초) | 인식 중앙값(초) | 인식 90%(초) | 인식 최대(초) |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        s = r["summary"]
        lines.append(f"| {r['scenario']} | {r['model']} | {r['mode']} | {s['runs']} | {s['success_mean']:.2f}/{s['objects']} | {s['success_pct']:.0f}% | "
                     f"{s['left_mean']:.2f} | {s['recog_mean_s']:.0f} | {s['recog_median_s']:.0f} | {s['recog_p90_s']:.0f} | {s['recog_max_s']:.0f} |")
        if "by_radius_pct" in s:
            lines.append("|  | 반경별 성공 % | " + " ".join(f"{k}:{v:.0f}" for k, v in s["by_radius_pct"].items()) + " | | | | | | | |")
    lines += compare_modes(results)
    return "\n".join(lines)


def compare_modes(results) -> list[str]:
    """빠른 모드가 정확 모드보다 인식 시간이 10% 이상 짧은지(모드를 합칠지 판단용). 같은 시나리오·모델끼리 평균 시간을 비교한다."""
    by = {(r["scenario"], r["model"], r["mode"]): r["summary"] for r in results}
    out = []
    for (sc, md, mode), s in by.items():
        if mode != "accurate" or (sc, md, "fast") not in by:
            continue
        f = by[(sc, md, "fast")]
        gain = 100 * (s["recog_mean_s"] - f["recog_mean_s"]) / s["recog_mean_s"]
        out.append(f"| 비교 {sc} {md} | 빠른이 정확보다 {gain:+.0f}% 빠름({'10% 이상 ✓' if gain >= 10 else '10% 미만'}), 성공 {f['success_pct'] - s['success_pct']:+.0f}%p |")
    return (["", "| 빠른 vs 정확 | |", "|---|---|", *out]) if out else []


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="all", help="spread90 | spread120 | demo | single | all")
    ap.add_argument("--seeds", type=int, default=20, help="시나리오마다 배치 수(8.5절: 20 이상). single은 반경 구간마다")
    ap.add_argument("--mode", default="both", help="accurate | fast | both")
    ap.add_argument("--model", default="both", help="optimistic | pessimistic | both")
    ap.add_argument("--line-mm", type=int, help="확인 직선 길이(기본 설정값)")
    ap.add_argument("--speed", type=float, help="확인 속도 mm/s(기본 설정값)")
    ap.add_argument("--rescan-home", choices=["on", "off"], help="후보가 없을 때 대기 자세로 옮겨 RESCAN(기본: 설정 radar.rescan_home)")
    ap.add_argument("--max-fail", type=int, help="check.max_fail_per_run(확인 실패가 이만큼이면 끝냄, 기본: 설정값 0)")
    ap.add_argument("--empty-extra", type=int, help="radar.empty_extra_sweeps(기본: 설정값 3)")
    ap.add_argument("--out", default="runs/bench")
    a = ap.parse_args(argv)
    names = list(SCENARIOS) if a.scenario == "all" else [a.scenario]
    models = list(MODELS) if a.model == "both" else [a.model]
    modes = ["accurate", "fast"] if a.mode == "both" else [a.mode]
    opts = {k: v for k, v in (("line_mm", a.line_mm), ("speed_mm_s", a.speed), ("max_fail_per_run", a.max_fail)) if v is not None}
    if a.empty_extra is not None:
        opts["radar"] = {"empty_extra_sweeps": a.empty_extra}
    rescan = None if a.rescan_home is None else a.rescan_home == "on"
    results, sha = [], commit()
    cmd = "python -m so101_studio.perception.ultrasonic.benchmark " + " ".join(argv if argv is not None else __import__("sys").argv[1:])
    for name, model, mode in itertools.product(names, models, modes):
        rows = run_scenario(name, model, a.seeds, mode, rescan_home=rescan, **opts)
        results.append({"commit": sha, "command": cmd, "scenario": name, "model": model, "mode": mode, "options": {**opts, "rescan_home": rescan}, "summary": summarize(rows),
                        "rows": [{k: v for k, v in r.items() if k != "log"} for r in rows]})
        print(format_table(results[-1:]).split("\n", 1)[1] if len(results) > 1 else format_table(results), flush=True)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d-%H%M%S')}-{sha}.json"
    path.write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")
    print("saved", path)


if __name__ == "__main__":
    main()
