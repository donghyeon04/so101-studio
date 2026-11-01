"""config/*.yaml 읽기. 알 수 없는 키는 오류로 막는다(오타가 조용히 무시되지 않게)."""
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .frames import Pose2D


def _load(path: str | Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


@dataclass
class RobotConfig:
    port: str
    calibration_id: str
    urdf: str
    control_hz: int = 50
    pid: dict = field(default_factory=lambda: {"p": 16, "i": 0, "d": 32})
    speed_pct_default: int = 30
    servo_acceleration: int = 25
    joint_v_max_deg_s: float = 90
    tool_v_max_mm_s: float = 100
    approach_height_mm: float = 80
    grasp_height_mm: float = 15
    gripper_margin_mm: float = 20
    place_height_mm: float = 30
    grasp_center_offset_mm: float = 0.0
    moving_jaw_sign: int = 1
    gripper_open: float = 60
    gripper_width_mm: list[list[float]] = field(default_factory=lambda: [[0, 0], [100, 90]])
    a_max_deg_s2: float = 120
    joint_offsets_deg: dict[str, float] = field(default_factory=dict)
    joint_limits_deg: dict[str, list[float]] = field(default_factory=dict)
    poses: dict[str, list[float]] = field(default_factory=dict)


@dataclass
class WorkspaceConfig:
    table_mm: dict
    robot_base_in_table: dict
    drop_zones: dict = field(default_factory=dict)
    safety_margin_mm: float = 10

    @property
    def base(self) -> Pose2D:
        return Pose2D(**self.robot_base_in_table)


def load_robot(path: str | Path = "config/robot.yaml") -> RobotConfig:
    return RobotConfig(**_load(path))


def load_workspace(path: str | Path = "config/workspace.yaml") -> WorkspaceConfig:
    return WorkspaceConfig(**_load(path))
