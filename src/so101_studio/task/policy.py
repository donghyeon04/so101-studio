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
    return ws.pick_reach_mm["min"] <= radial_mm(obj, ws) <= ws.pick_reach_mm["max"]


def choose(objects: list[DetectedObject], ws: WorkspaceConfig) -> DetectedObject | None:
    cands = [o for o in objects if reachable(o, ws)]
    return min(cands, key=lambda o: radial_mm(o, ws)) if cands else None


def explain(objects: list[DetectedObject], ws: WorkspaceConfig, chosen: DetectedObject | None) -> dict:
    """왜 이 물체를 골랐는지(또는 못 골랐는지)를 GUI 표·이벤트 기록용 dict로 (REQ-007)."""
    rows = [{"id": o.id, "x_mm": o.x_mm, "y_mm": o.y_mm, "angle_deg": o.angle_deg, "width_mm": o.width_mm,
             "radial_mm": round(radial_mm(o, ws), 1), "reachable": reachable(o, ws)} for o in objects]
    if chosen is None:
        reason = "도달 가능한 물체가 없음" if objects else "스캔된 물체가 없음"
    else:
        reason = f"도달 가능한 {sum(r['reachable'] for r in rows)}개 중 팔에서 가장 가까움 ({radial_mm(chosen, ws):.0f}mm)"
    return {"objects": rows, "chosen": chosen.id if chosen else None, "reason": reason}
