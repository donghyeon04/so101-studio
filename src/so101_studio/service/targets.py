"""끝 위치 목표를 만드는 순수 논리. 키보드 좌표 입력과 나중의 GUI(시뮬레이션에서 점 찍기)가 같이 쓴다.

좌표계 두 가지:
- table: 작업대 좌표계(mm). 원점 = 작업대 왼쪽 아래, x = 긴 변, y = 짧은 변.
- robot: 로봇 기준(mm). 원점 = 어깨 회전축이 작업대와 만나는 점, 앞(forward) / 왼쪽(left) / 높이(z).
z는 두 좌표계에서 같은 모델 높이다. 사용자 입력은 기준점(고정 턱 면)의 위치다."""
from dataclasses import replace

from ..core import frames
from ..core.types import ToolPose

REASONS = {
    "ik_fail": "팔이 닿지 않는 점입니다 (도달 범위 밖이거나 이 자세로는 풀리지 않음)",
    "workspace": "작업대 밖입니다 (경계 안 여유 10mm 안쪽만 가능)",
    "z_floor": "너무 낮습니다 (낮은 턱 끝이 작업대에 닿음)",
    "step": "한 번에 너무 멀리 움직입니다",
}


def reason_text(reason: str) -> str:
    return "관절 한계를 넘습니다 (" + reason.split(":", 1)[1] + ")" if reason.startswith("joint_limit") else REASONS.get(reason, reason)


def to_table(x: float, y: float, frame: str, base: frames.Pose2D) -> tuple[float, float]:
    """입력 좌표(frame 기준)를 작업대 좌표로."""
    return (x, y) if frame == "table" else frames.robot_to_table(x, y, base)


def from_table(x: float, y: float, frame: str, base: frames.Pose2D) -> tuple[float, float]:
    return (x, y) if frame == "table" else frames.table_to_robot(x, y, base)


def parse_target(line: str, frame: str, base: frames.Pose2D, current: ToolPose, default_yaw: float = 90.0) -> ToolPose:
    """'x y z [yaw]'는 절대 좌표, 'd dx dy dz [dyaw]'는 지금 끝 위치에서의 상대 이동(해당 좌표계의 축 방향).
    yaw는 작업대 기준 도(생략하면 default_yaw, 상대 이동이면 현재 yaw). 자세는 수직 하강(그리퍼가 아래)이다."""
    tok = line.replace(",", " ").split()
    relative = bool(tok) and tok[0].lower() in ("d", "dx", "rel")
    nums = tok[1:] if relative else tok
    try:
        v = [float(t) for t in nums]
    except ValueError:
        raise ValueError("숫자만 입력하세요. 예: 300 200 40  /  d 10 0 0") from None
    if len(v) not in ((3, 4) if not relative else (1, 2, 3, 4)):
        raise ValueError("절대: x y z [yaw] (3~4개), 상대: d dx dy dz [dyaw]")
    if relative:
        v = (v + [0.0] * 4)[:4]
        cx, cy = from_table(current.x_mm, current.y_mm, frame, base)
        x, y = to_table(cx + v[0], cy + v[1], frame, base)
        return replace(current, x_mm=x, y_mm=y, z_mm=current.z_mm + v[2], yaw_deg=current.yaw_deg + v[3], pitch_deg=-90.0)
    x, y = to_table(v[0], v[1], frame, base)
    return ToolPose(x, y, v[2], v[3] if len(v) == 4 else default_yaw, -90.0)
