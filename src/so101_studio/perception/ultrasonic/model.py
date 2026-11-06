"""빔 모델, 설정, 시계. 가상 ESP32와 추정(radar·check)이 같이 쓴다."""
import math
import time
from dataclasses import dataclass
from pathlib import Path

import yaml


class Cfg(dict):
    """yaml 설정을 cfg.radar.dip_mm처럼 읽는다. 없는 키는 AttributeError."""

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e


def _wrap(v):
    return Cfg({k: _wrap(x) for k, x in v.items()}) if isinstance(v, dict) else v


def load_config(path="config/ultrasonic.yaml", radar=None, **check) -> Cfg:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    raw["check"].update(check)
    raw["radar"].update(radar or {})
    return _wrap(raw)


class RealClock:
    # PoseSample.t_ns(팔, #73)와 같은 시계여야 한다. Windows에서 time.monotonic은 해상도 15.6ms라 perf_counter_ns를 쓴다.
    now_ns = staticmethod(time.perf_counter_ns)
    sleep = staticmethod(time.sleep)


@dataclass
class Box:  # 가상 물체. angle_deg = 긴 축의 x축 기준 각도
    x: float
    y: float
    angle_deg: float
    width: float = 30
    length: float = 100
    height: float = 40


def beam_radius(sensor_h: float, obj_h: float, half_deg: float) -> float:
    """아래로 본 센서가 물체 윗면 높이에서 보는 빔 반경."""
    return (sensor_h - obj_h) * math.tan(math.radians(half_deg))


def _to_box(b: Box, x, y):
    a = math.radians(b.angle_deg)
    dx, dy = x - b.x, y - b.y
    return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)


def ray_box(b: Box, ox, oy, az_deg, max_incidence_deg=None) -> float | None:
    """수평 광선이 물체 옆면에 처음 닿는 거리(slab 방식). 안 닿으면 None.
    max_incidence_deg: 면 법선과 광선이 이 각도보다 비스듬하면 반사가 약해 메아리가 없다(8.5절 면 반사 허용각). None = 항상 있다."""
    a = math.radians(b.angle_deg)
    az = math.radians(az_deg)
    px, py = _to_box(b, ox, oy)
    ux = math.cos(az - a)
    uy = math.sin(az - a)
    lo, hi, enter = 0.0, math.inf, None
    for p, u, h in ((px, ux, b.length / 2), (py, uy, b.width / 2)):
        if abs(u) < 1e-9:
            if abs(p) > h:
                return None
            continue
        t1, t2 = sorted(((-h - p) / u, (h - p) / u))
        if t1 > lo:
            enter = u  # 들어가는 면의 법선 방향 성분
        lo, hi = max(lo, t1), min(hi, t2)
    if lo > hi:
        return None
    if max_incidence_deg is not None and enter is not None and math.degrees(math.acos(min(abs(enter), 1.0))) > max_incidence_deg:
        return None
    return lo


def ray_disc(cx, cy, r, ox, oy, az_deg) -> float | None:
    az = math.radians(az_deg)
    ux, uy = math.cos(az), math.sin(az)
    dx, dy = cx - ox, cy - oy
    t = dx * ux + dy * uy
    d2 = dx * dx + dy * dy - t * t
    if t < 0 or d2 > r * r:
        return None
    return max(t - math.sqrt(r * r - d2), 0.0)


def box_corners(b: Box) -> list[tuple[float, float]]:
    a = math.radians(b.angle_deg)
    c, s = math.cos(a), math.sin(a)
    return [(b.x + sx * b.length / 2 * c - sy * b.width / 2 * s, b.y + sx * b.length / 2 * s + sy * b.width / 2 * c)
            for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]


def corner_echo(ox, oy, az_deg, boxes, half_deg, factor, range_mm, prob, rng) -> float | None:
    """모서리 반사(8.5절): 빔 중심에서 반각 x factor 안, range_mm 안의 보이는 모서리가 확률 prob로 메아리를 낸다."""
    best = None
    for b in boxes:
        for cx, cy in box_corners(b):
            d = math.hypot(cx - ox, cy - oy)
            if d > range_mm or (best is not None and d >= best):
                continue
            diff = abs((math.degrees(math.atan2(cy - oy, cx - ox)) - az_deg + 180) % 360 - 180)
            if diff > factor * half_deg:
                continue
            first = min((x for x in (ray_box(o, ox, oy, math.degrees(math.atan2(cy - oy, cx - ox))) for o in boxes) if x is not None),
                        default=d)
            if first < d - 3 or rng.random() >= prob:  # 다른 면에 가려졌거나 이번엔 안 울렸다
                continue
            best = d
    return best


def radar_echo(ox, oy, az_deg, boxes, discs, half_deg, max_mm, max_incidence_deg=None,
               corner=None, rng=None) -> float | None:
    """수평 빔의 첫 메아리 거리. 빔 안 7개 광선 중 가장 가까운 면(discs = 팔 링크 원).
    corner = (반각 배율, 거리 mm, 확률)이면 모서리 반사도 낸다."""
    best = None
    for k in range(-3, 4):
        az = az_deg + half_deg * k / 3
        for d in [ray_box(b, ox, oy, az, max_incidence_deg) for b in boxes] + [ray_disc(cx, cy, r, ox, oy, az) for cx, cy, r in discs]:
            if d is not None and (best is None or d < best):
                best = d
    if corner is not None:
        c = corner_echo(ox, oy, az_deg, boxes, half_deg, *corner, rng)
        if c is not None and (best is None or c < best):
            best = c
    return best if best is not None and best <= max_mm else None


def arm_echo(sx, sy, sz, boxes, half_deg) -> float:
    """아래로 본 센서의 거리. 물체 윗면(빔 반경만큼 넓힌 윗면 안이면) 아니면 작업대 면."""
    best = sz
    for b in boxes:
        px, py = _to_box(b, sx, sy)
        out = math.hypot(max(abs(px) - b.length / 2, 0), max(abs(py) - b.width / 2, 0))
        if out <= beam_radius(sz, b.height, half_deg):
            best = min(best, sz - b.height)
    return best
