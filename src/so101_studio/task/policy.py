"""물체 선택 정책 (system-architecture.md 5절 DECIDE): 도달 범위 밖 제외 -> 팔에서 가까운 순."""
import math

from ..core import frames
from ..core.config import WorkspaceConfig
from ..core.types import DetectedObject


def radial_mm(obj: DetectedObject, ws: WorkspaceConfig) -> float:
    x, y = frames.table_to_robot(obj.x_mm, obj.y_mm, ws.base)
    return math.hypot(x, y)


def reachable(obj: DetectedObject, ws: WorkspaceConfig) -> bool:
    """수직 하강으로 닿는 반경(measurements.md T6 사전 조사: 약 80~260mm)을 여유를 두고 쓴다."""
    return ws.reach_mm["min"] <= radial_mm(obj, ws) <= ws.reach_mm["max"]


def choose(objects: list[DetectedObject], ws: WorkspaceConfig) -> DetectedObject | None:
    cands = [o for o in objects if reachable(o, ws)]
    return min(cands, key=lambda o: radial_mm(o, ws)) if cands else None
