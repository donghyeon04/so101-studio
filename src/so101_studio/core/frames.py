"""작업대 좌표계 <-> 로봇 좌표계 변환 (arm-software-spec.md 4절). 외부 단위 mm·도, 내부 계산은 m·rad로 바꿔 쓴다."""
import math
from dataclasses import dataclass, replace

from .types import ToolPose


@dataclass
class Pose2D:  # 작업대 좌표계에서 본 로봇 원점 (workspace.yaml의 robot_base_in_table)
    x_mm: float
    y_mm: float
    yaw_deg: float


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def table_to_robot(x_mm: float, y_mm: float, base: Pose2D) -> tuple[float, float]:
    dx, dy = x_mm - base.x_mm, y_mm - base.y_mm
    c, s = math.cos(math.radians(base.yaw_deg)), math.sin(math.radians(base.yaw_deg))
    return c * dx + s * dy, -s * dx + c * dy


def robot_to_table(x_mm: float, y_mm: float, base: Pose2D) -> tuple[float, float]:
    c, s = math.cos(math.radians(base.yaw_deg)), math.sin(math.radians(base.yaw_deg))
    return base.x_mm + c * x_mm - s * y_mm, base.y_mm + s * x_mm + c * y_mm


def tool_table_to_robot(p: ToolPose, base: Pose2D) -> ToolPose:
    x, y = table_to_robot(p.x_mm, p.y_mm, base)
    return replace(p, x_mm=x, y_mm=y, yaw_deg=_wrap_deg(p.yaw_deg - base.yaw_deg))


def tool_robot_to_table(p: ToolPose, base: Pose2D) -> ToolPose:
    x, y = robot_to_table(p.x_mm, p.y_mm, base)
    return replace(p, x_mm=x, y_mm=y, yaw_deg=_wrap_deg(p.yaw_deg + base.yaw_deg))
