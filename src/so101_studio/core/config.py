"""config/*.yaml 읽기 (draccus, LeRobot과 같은 방식)."""
from dataclasses import dataclass, field
from pathlib import Path

import draccus

from .frames import Pose2D


def _load(cls, path: str | Path):
    """LeRobot과 같은 draccus로 읽는다. 알 수 없는 키는 DecodingError."""
    with Path(path).open(encoding="utf-8") as f:
        return draccus.load(cls, f)


@dataclass
class RobotConfig:
    port: str
    calibration_id: str
    urdf: str
    control_hz: int = 50
    pid: dict = field(default_factory=lambda: {"p": 16, "i": 0, "d": 32})
    speed_pct_default: int = 30
    servo_acceleration: int = 25
    physical_limits_deg: dict = field(default_factory=dict)  # 캘리브레이션으로 잰 실제 가동 범위. 안전 한계를 넓혀도 이 범위는 넘지 않는다
    teach_torque: dict = field(default_factory=lambda: {"shoulder_pan": 250, "shoulder_lift": 700, "elbow_flex": 500,
                                                          "wrist_flex": 300, "wrist_roll": 200, "gripper": 200})  # 추종 교시 때 서보별 토크 제한(0~1000)
    gripper_acceleration: int = 25  # 그리퍼는 접촉 감지(명령-실측 지연)를 이 값에서 맞췄으므로 팔 관절과 따로 둔다
    joint_v_max_deg_s: float = 90
    tool_v_max_mm_s: float = 100
    approach_height_mm: float = 80
    grasp_height_mm: float = 15
    table_z_mm: float = 0.0
    tip_floor_mm: float = 4.0
    gripper_margin_mm: float = 20
    place_height_mm: float = 30
    grasp_bias_mm: float = 0.0
    grasp_open_policy: str = "max"
    grasp_tilt_deg: float = 35.0
    grasp_tilt_mode: str = "symmetric"
    grasp_tilt_tol_deg: float = 15.0
    grasp_max_open_mm: float = 100
    contact_lag_units: float = 4.0
    gripper_squeeze_torque: int = 300
    gripper_open: float = 60
    gripper_width_mm: list[list[float]] = field(default_factory=lambda: [[0, 0], [100, 90]])
    a_max_deg_s2: float = 120
    trajectory_profile: str = "quintic"
    joint_jerk_max_deg_s3: float = 300
    tool_a_max_mm_s2: float = 500
    tool_jerk_max_mm_s3: float = 3000
    joint_offsets_deg: dict[str, float] = field(default_factory=dict)
    joint_limits_deg: dict[str, list[float]] = field(default_factory=dict)
    poses: dict[str, list[float]] = field(default_factory=dict)


@dataclass
class WorkspaceConfig:
    table_mm: dict
    robot_base_in_table: dict
    drop_zones: dict = field(default_factory=dict)
    safety_margin_mm: float = 10
    reach_mm: dict = field(default_factory=lambda: {"min": 90.0, "max": 250.0})

    @property
    def base(self) -> Pose2D:
        return Pose2D(**self.robot_base_in_table)


def load_robot(path: str | Path = "config/robot.yaml") -> RobotConfig:
    return _load(RobotConfig, path)


def load_workspace(path: str | Path = "config/workspace.yaml") -> WorkspaceConfig:
    return _load(WorkspaceConfig, path)
