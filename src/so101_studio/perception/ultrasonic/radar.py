"""레이더 처리 (ultrasonic-spec 5절): 기준값, 바퀴 합치기, 팔 지우기, 후보. 시리얼·시계를 모르는 순수 함수."""
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Pass:
    dir: int  # 1 = 각도 증가
    meas: list  # (t_ns, 명령 각도, d_mm 또는 None)
    shift: dict | None = None  # 명령 각도 -> (읽은 실제 각도 - 명령 각도). 서보가 각도를 읽어 줄 때만(7절 ang)


@dataclass
class Candidate:
    x: float
    y: float
    servo_deg: float  # 대표 각도
    dist_mm: float  # 대표 거리(앞면까지)
    arc_len: int  # 호를 이룬 각도 수


@dataclass
class Baseline:
    d: dict  # 각도 -> mm, None = 600 안에 메아리 없음
    unstable: set

    def save(self, path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(
            json.dumps({"d": {str(a): v for a, v in self.d.items()}, "unstable": sorted(self.unstable)}), encoding="utf-8")

    @classmethod
    def load(cls, path) -> "Baseline":
        j = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls({int(a): v for a, v in j["d"].items()}, set(j["unstable"]))


def azimuth_deg(cfg, servo_deg: float) -> float:
    return cfg.radar.heading_deg + (servo_deg - 90)


def polar(cfg, servo_deg: float, dist_mm: float) -> tuple[float, float]:
    st, az = cfg.radar.station_mm, math.radians(azimuth_deg(cfg, servo_deg))
    return st.x + dist_mm * math.cos(az), st.y + dist_mm * math.sin(az)


def make_baseline(passes: list[Pass]) -> Baseline:
    """빈 작업대 여러 바퀴의 각도별 중앙값. 바퀴 사이 차이가 20mm를 넘으면 불안정."""
    vals: dict = {}
    for p in passes:
        for _, a, d in p.meas:
            vals.setdefault(a, []).append(d)
    base, unstable = {}, set()
    for a, v in vals.items():
        found = [x for x in v if x is not None]
        base[a] = statistics.median(found) if found else None
        if found and (max(found) - min(found) > 20 or (len(found) < len(v) and len(found) < 2)):
            unstable.add(a)
    return Baseline(base, unstable)


def mask_arm(meas, links_fn, cfg, shift=None) -> tuple[list, set]:
    """그 측정 시각의 팔 링크 점 방위 ±(빔 반각+arm_mask_deg), 거리 차 arm_mask_mm 안이면 버린다."""
    if links_fn is None:
        return list(meas), set()
    st, lim = cfg.radar.station_mm, cfg.beam_half_deg + cfg.radar.arm_mask_deg
    kept, masked = [], set()
    for t, a, d in meas:
        hide = False
        if d is not None:
            az = azimuth_deg(cfg, a + (shift or {}).get(a, 0.0))
            for x, y in links_fn(t):
                bearing = math.degrees(math.atan2(y - st.y, x - st.x))
                diff = (bearing - az + 180) % 360 - 180
                if abs(diff) <= lim and abs(math.hypot(x - st.x, y - st.y) - d) <= cfg.radar.arm_mask_mm:
                    hide = True
                    break
        if hide:
            masked.add(a)
        else:
            kept.append((t, a, d))
    return kept, masked


def combine(passes: list[Pass], links_fn, cfg) -> tuple[dict, dict]:
    """각도별 중앙값(응답 없음·지운 측정은 빼고 센다). 값이 없으면 None. raw = 각도별 바퀴 값, 지운 각도."""
    per: dict = {}
    masked_all: set = set()
    for p in passes:
        kept, masked = mask_arm(p.meas, links_fn, cfg, p.shift)
        masked_all |= masked
        for _, a, d in kept:
            per.setdefault(a, []).append(d)
        for a in masked:
            per.setdefault(a, [])
    out = {}
    for a, v in per.items():
        found = [x for x in v if x is not None]
        out[a] = statistics.median(found) if found else None
    offs: dict = {}
    for p in passes:
        for a, o in (p.shift or {}).items():
            offs.setdefault(a, []).append(o)
    shift = {a: statistics.median(v) for a, v in offs.items()}
    return out, {"per_angle": {a: v for a, v in per.items()}, "masked": sorted(masked_all), "shift": shift}


def find_candidates(vals: dict, base: Baseline, cfg, shift=None) -> tuple[list[Candidate], list]:
    """기준값보다 dip_mm 이상 가까운 이웃 각도 호 -> 붙어 있는 호 합침 -> 후보. 두 번째 값은 호 목록(raw용)."""
    r = cfg.radar
    step = r.sweep.step_deg
    angles = sorted(vals)
    shift = shift or {}

    def is_dip(a):
        d = vals[a]
        if d is None or a in base.unstable:
            return False
        b = base.d.get(a)
        return (r.max_mm if b is None else b) - d >= r.dip_mm

    arcs, run = [], []

    def close():
        if len(run) >= r.min_arc:
            arcs.append({"first": run[0], "last": run[-1],
                             "center": (run[0] + run[-1]) / 2 + statistics.mean(shift.get(a, 0.0) for a in run),
                             "d": statistics.median(vals[a] for a in run)})

    for a in angles:
        if is_dip(a) and run and a - run[-1] == step and abs(vals[a] - vals[run[-1]]) <= r.split_mm:
            run.append(a)
            continue
        close()
        run = [a] if is_dip(a) else []
    close()

    groups: list[list[dict]] = []
    for arc in arcs:
        if groups:
            prev = groups[-1][-1]
            gap_ok = arc["first"] - prev["last"] <= 2 * step  # 응답 없음 한 칸은 이웃으로 본다
            px, py = polar(cfg, prev["center"], prev["d"])
            ax, ay = polar(cfg, arc["center"], arc["d"])
            if gap_ok and math.hypot(px - ax, py - ay) <= r.merge_mm:
                groups[-1].append(arc)
                continue
        groups.append([arc])

    cands = []
    for g in groups:
        ang = statistics.mean(a["center"] for a in g)
        dist = statistics.mean(a["d"] for a in g)
        x, y = polar(cfg, ang, dist + r.range_offset_mm)
        cands.append(Candidate(x, y, ang, dist, sum(a["last"] - a["first"] + step for a in g) // step))
    return cands, arcs
