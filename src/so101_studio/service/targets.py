"""끝 위치 목표를 만드는 순수 논리. 키보드 좌표 입력과 나중의 GUI(시뮬레이션에서 점 찍기)가 같이 쓴다.

좌표계 (산업용 로봇의 월드/베이스/툴 관례를 따른다):
- table(= world): 작업대 좌표계(mm). 원점 = 작업대 왼쪽 아래 모서리, x = 긴 변, y = 짧은 변. 고정된 셀 기준이라 월드로 쓴다.
- robot(= base): 로봇 기준(mm). 원점 = 어깨 회전축이 작업대와 만나는 점, 앞(forward) / 왼쪽(left).
- tool: 도구 프레임. 지금 끝점이 원점이라 상대 이동만 받는다(xArm의 set_tool_position과 같다).
  z = 도구 앞(턱 끝 방향, 수직 자세에서는 아래), x = 턱이 벌어지는 축(가동 턱은 -x), y = 나머지.
높이 z는 어느 좌표계에서나 "작업대 면 위 높이(mm)"다(작업대 면 = robot.yaml의 table_z_mm).
끝점(tcp): ref = 고정 턱 안쪽 면(URDF gripper_frame), tip = 고정 턱 끝. 로봇에 보내는 ToolPose는 항상 ref 기준이다."""
from dataclasses import replace

import numpy as np

from ..core import frames
from ..core.types import ToolPose

FRAMES = {"table": "table", "world": "table", "robot": "robot", "base": "robot", "tool": "tool"}
REASONS = {
    "ik_fail": "팔이 닿지 않는 점입니다 (도달 범위 밖이거나 이 자세로는 풀리지 않음)",
    "workspace": "작업대 밖입니다 (경계 안 여유 10mm 안쪽만 가능)",
    "z_floor": "너무 낮습니다 (낮은 턱 끝이 작업대에 닿음)",
    "step": "한 번에 너무 멀리 움직입니다",
}


def reason_text(reason: str) -> str:
    if reason.startswith("scene:"):
        return f"장면 부품 {reason.split(':', 1)[1]}에 너무 가깝습니다 (config/scene.yaml의 margin_mm 안)"
    return "관절 한계를 넘습니다 (" + reason.split(":", 1)[1] + ")" if reason.startswith("joint_limit") else REASONS.get(reason, reason)


def to_table(x: float, y: float, frame: str, base: frames.Pose2D) -> tuple[float, float]:
    """입력 좌표(frame 기준)를 작업대 좌표로."""
    return (x, y) if frame == "table" else frames.robot_to_table(x, y, base)


def from_table(x: float, y: float, frame: str, base: frames.Pose2D) -> tuple[float, float]:
    return (x, y) if frame == "table" else frames.table_to_robot(x, y, base)


def tip_vector(kin, pitch_deg: float, yaw_deg: float, gripper: float, tcp: str) -> np.ndarray:
    """ref 점에서 끝점까지의 벡터(작업대 좌표계 mm). tcp가 ref면 0."""
    return np.zeros(3) if tcp == "ref" else np.asarray(kin.jaw_tip_offsets(pitch_deg, yaw_deg, gripper)[0], float)


def point_to_pose(x: float, y: float, h: float, yaw: float, frame: str, base: frames.Pose2D, table_z: float = 0.0,
                  tip: np.ndarray | None = None, pitch_deg: float = -90.0) -> ToolPose:
    """(frame 기준 x, y, 작업대 면 위 높이 h)인 끝점을 pitch 자세(기본 -90 = 수직 아래, 0 = 수평)로 가져가는 ref 기준 ToolPose. GUI가 점을 찍을 때도 이것을 쓴다.
    tip은 그 pitch·yaw에서 ref에서 끝점까지의 벡터(tip_vector)여야 한다."""
    tip = np.zeros(3) if tip is None else tip
    tx, ty = to_table(x, y, FRAMES[frame], base)
    return ToolPose(tx - tip[0], ty - tip[1], h + table_z - tip[2], yaw, pitch_deg)


def parse_target(line: str, frame: str, base: frames.Pose2D, current: ToolPose, table_z: float = 0.0,
                 tip: np.ndarray | None = None, axes: np.ndarray | None = None, default_yaw: float = 90.0, pitch_deg: float = -90.0) -> ToolPose:
    """'x y h [yaw]'는 절대 좌표(h = 작업대 면 위 높이), 'd dx dy dz [dyaw]'는 지금 끝 위치에서의 상대 이동(해당 좌표계의 축 방향).
    yaw는 작업대 기준 도(생략하면 default_yaw, 상대 이동이면 현재 yaw). tool 좌표계는 상대 이동만 가능하며 axes(도구 축 3x3)가 필요하다."""
    tok = line.replace(",", " ").split()
    relative = bool(tok) and tok[0].lower() in ("d", "dx", "rel")
    nums = tok[1:] if relative else tok
    try:
        v = [float(t) for t in nums]
    except ValueError:
        raise ValueError("숫자만 입력하세요. 예: 300 200 40  /  d 10 0 0") from None
    if len(v) not in ((3, 4) if not relative else (1, 2, 3, 4)):
        raise ValueError("절대: x y 높이 [yaw] (3~4개), 상대: d dx dy dz [dyaw]")
    if not relative:
        if frame == "tool":
            raise ValueError("tool 좌표계는 상대 이동만 됩니다: d dx dy dz")
        return point_to_pose(v[0], v[1], v[2], v[3] if len(v) == 4 else default_yaw, frame, base, table_z, tip, pitch_deg)
    v = (v + [0.0] * 4)[:4]
    if frame == "tool":
        dx, dy, dz = axes @ np.array(v[:3])
        return replace(current, x_mm=current.x_mm + dx, y_mm=current.y_mm + dy, z_mm=current.z_mm + dz, yaw_deg=current.yaw_deg + v[3])
    o = frames.robot_to_table(0, 0, base) if frame == "robot" else (0, 0)
    dx, dy = (a - b for a, b in zip(to_table(v[0], v[1], frame, base), o, strict=True))
    return replace(current, x_mm=current.x_mm + dx, y_mm=current.y_mm + dy, z_mm=current.z_mm + v[2],
                   yaw_deg=current.yaw_deg + v[3], pitch_deg=pitch_deg)
