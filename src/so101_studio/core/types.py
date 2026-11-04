"""공통 데이터 타입. 팔 타입은 arm-software-spec.md 5절, 나머지는 system-architecture.md 3절."""
from dataclasses import dataclass, field


@dataclass
class JointState:  # 도 단위, gripper는 0~100
    shoulder_pan: float
    shoulder_lift: float
    elbow_flex: float
    wrist_flex: float
    wrist_roll: float
    gripper: float


@dataclass
class ToolPose:  # 작업대 좌표계
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float
    pitch_deg: float = -90  # -90 = 수직 하강


@dataclass
class ArmStatus:  # 10Hz 상태 스트림
    t: float
    joints: JointState
    tool: ToolPose
    temperature_c: dict[str, float]
    load_pct: dict[str, float]
    voltage_v: float
    torque_enabled: bool
    state: str  # "idle" | "moving" | "teleop" | "teach" | "stopped" | "fault"
    fault: str | None = None
    current_ma: float = 0.0  # 서보 전류 합(mA, 추정)


@dataclass
class MotionResult:
    status: str  # "ok" | "ik_fail" | "out_of_bounds" | "stopped" | "preempted" | "fault" | "timeout"
    duration_s: float
    final_tool: ToolPose
    position_error_mm: float | None = None
    reason: str | None = None  # out_of_bounds일 때 안전 검사가 막은 이유(workspace, z_floor, joint_limit:<관절> 등). 다른 상태는 None


@dataclass
class DetectedObject:  # 모든 인식 방식의 공통 출력 (decisions/0004)
    id: str
    x_mm: float
    y_mm: float
    angle_deg: float | None  # 0~180, None = 판단 불가
    width_mm: float
    length_mm: float
    confidence: float  # 0~1
    source: str  # "manual" | "ultrasonic" | "camera"


@dataclass
class ScanResult:
    objects: list[DetectedObject]
    duration_s: float
    raw: dict = field(default_factory=dict)


@dataclass
class TaskEvent:
    t: float
    state: str
    object_id: str | None = None
    result: str | None = None  # "ok" | "skipped" | "failed" | 오류 이름
    detail: dict = field(default_factory=dict)
