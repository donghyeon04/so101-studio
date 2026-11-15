"""물체 선택 정책 (system-architecture.md 5절 DECIDE): 도달 범위 밖 제외 -> 팔에서 가까운 순."""
import math

from ..core import frames
from ..core.config import WorkspaceConfig
from ..core.types import DetectedObject


def radial_mm(obj: DetectedObject, ws: WorkspaceConfig) -> float:
    x, y = frames.table_to_robot(obj.x_mm, obj.y_mm, ws.base)
    return math.hypot(x, y)


def reachable(obj: DetectedObject, ws: WorkspaceConfig, margin_mm: float = 0.0) -> bool:
    """수직 하강으로 닿는 반경(measurements.md T6 사전 조사: 약 80~260mm)을 여유를 두고 쓴다.
    margin_mm: 확인(CHECK) 전 후보에만 최대 반경에 더하는 여유(REQ-065, workspace.yaml candidate_reach_margin_mm). 집는 범위 판정은 0으로 부른다."""
    return ws.pick_reach_mm["min"] <= radial_mm(obj, ws) <= ws.pick_reach_mm["max"] + margin_mm


def choose(objects: list[DetectedObject], ws: WorkspaceConfig, margin_mm: float = 0.0) -> DetectedObject | None:
    cands = [o for o in objects if reachable(o, ws, margin_mm)]
    return min(cands, key=lambda o: radial_mm(o, ws)) if cands else None


def explain(objects: list[DetectedObject], ws: WorkspaceConfig, chosen: DetectedObject | None, margin_mm: float = 0.0) -> dict:
    """왜 이 물체를 골랐는지(또는 못 골랐는지)를 GUI 표·이벤트 기록용 dict로 (REQ-007)."""
    rows = [{"id": o.id, "x_mm": o.x_mm, "y_mm": o.y_mm, "angle_deg": o.angle_deg, "width_mm": o.width_mm,
             "radial_mm": round(radial_mm(o, ws), 1), "reachable": reachable(o, ws, margin_mm)} for o in objects]
    if chosen is None:
        reason = "도달 가능한 물체가 없음" if objects else "스캔된 물체가 없음"
    else:
        reason = f"도달 가능한 {sum(r['reachable'] for r in rows)}개 중 팔에서 가장 가까움 ({radial_mm(chosen, ws):.0f}mm)"
    return {"objects": rows, "chosen": chosen.id if chosen else None, "reason": reason}
