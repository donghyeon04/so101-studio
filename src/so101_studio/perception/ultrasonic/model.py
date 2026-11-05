"""빔 모델, 설정, 시계. 가상 ESP32와 추정(radar·check)이 같이 쓴다."""
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import yaml

from ...core.types import DetectedObject
from ..base import PerceptionSource


# --- REQ-023 임시: 8.2절 형식. 팔 세션의 정의가 main에 합쳐지면 지우고 거기서 가져온다 ---
@dataclass
class PathSegment:  # 작업대 좌표, 손끝은 수직 아래
    points_mm: list[tuple[float, float]]  # 손끝 xy (센서 xy 아님)
    tool_z_mm: float
    speed_mm_s: float  # 실제 속도 = min(이 값, tool_v_max_mm_s x 속도%)
    yaw_deg: float | None = None  # 경로 동안 고정할 손목 각도. None = 지금 값 유지


@dataclass
class PoseSample:  # 팔 50Hz 루프의 실제 관절값으로 계산
    t_ns: int  # PC 단조 시계
    tool_mm: tuple[float, float, float]  # 손끝 위치 (작업대 좌표)
    sensor_mm: tuple[float, float, float]  # 팔 센서 위치 = 손끝 + yaw로 돌린 arm_sensor_offset_mm


class CheckProbe(Protocol):
    def next_segment(self) -> PathSegment | None: ...

    def feed(self, poses: list[PoseSample]) -> None: ...

    def result(self) -> DetectedObject | None: ...  # None = 물체 없음


class CheckableSource(PerceptionSource, Protocol):
    def start_check(self, candidate: DetectedObject) -> CheckProbe: ...

    def prefetch(self, arm_links_fn) -> None: ...
# --- REQ-023 임시 끝 ---


class Cfg(dict):
    """yaml 설정을 cfg.radar.dip_mm처럼 읽는다. 없는 키는 AttributeError."""

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e


def _wrap(v):
    return Cfg({k: _wrap(x) for k, x in v.items()}) if isinstance(v, dict) else v


def load_config(path="config/ultrasonic.yaml", mode=None, **check) -> Cfg:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    raw["mode"] = mode or raw["mode"]
    if raw["mode"] == "fast":
        # T9(early_stop_cells)는 팔 세션 측정(조각마다 정지·가속·0.25초)으로 약 74초라 끈다(기획 답 전 임시, 총괄 지시).
        # line_mm 130은 U9 조정값(시간은 줄지만 성공이 떨어진다: 180이면 3.0/3, 130이면 2.3/3)
        raw["check"].update(approach_line=True, early_stop_cells=0, line_mm=130)
    raw["check"].update(check)
    return _wrap(raw)


class RealClock:
    now_ns = staticmethod(time.monotonic_ns)
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


def ray_box(b: Box, ox, oy, az_deg) -> float | None:
    """수평 광선이 물체 옆면에 처음 닿는 거리(slab 방식). 안 닿으면 None."""
    a = math.radians(b.angle_deg)
    az = math.radians(az_deg)
    px, py = _to_box(b, ox, oy)
    ux = math.cos(az - a)
    uy = math.sin(az - a)
    lo, hi = 0.0, math.inf
    for p, u, h in ((px, ux, b.length / 2), (py, uy, b.width / 2)):
        if abs(u) < 1e-9:
            if abs(p) > h:
                return None
            continue
        t1, t2 = sorted(((-h - p) / u, (h - p) / u))
        lo, hi = max(lo, t1), min(hi, t2)
    return lo if lo <= hi else None


def ray_disc(cx, cy, r, ox, oy, az_deg) -> float | None:
    az = math.radians(az_deg)
    ux, uy = math.cos(az), math.sin(az)
    dx, dy = cx - ox, cy - oy
    t = dx * ux + dy * uy
    d2 = dx * dx + dy * dy - t * t
    if t < 0 or d2 > r * r:
        return None
    return max(t - math.sqrt(r * r - d2), 0.0)


def radar_echo(ox, oy, az_deg, boxes, discs, half_deg, max_mm) -> float | None:
    """수평 빔의 첫 메아리 거리. 빔 안 7개 광선 중 가장 가까운 면(discs = 팔 링크 원)."""
    best = None
    for k in range(-3, 4):
        az = az_deg + half_deg * k / 3
        for d in [ray_box(b, ox, oy, az) for b in boxes] + [ray_disc(cx, cy, r, ox, oy, az) for cx, cy, r in discs]:
            if d is not None and (best is None or d < best):
                best = d
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
