"""config/*.yaml 읽기 (draccus, LeRobot과 같은 방식)."""
from dataclasses import dataclass, field
from pathlib import Path

import draccus
import yaml

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
    watchdog_hotkey: str = "F9"  # E-STOP 감시 프로세스의 전역 단축키 (REQ-011)
    record_hz: int = 20  # 교시 기록 주기
    record_max_s: float = 90.0  # 한 번에 기록하는 최대 시간(레퍼런스 myCobot 권장값에서 따옴)
    seg_pause_deg_s: float = 3.0  # 이 속도(°/s, 가장 빠른 관절 기준) 미만이면 멈춤 구간
    seg_fast_deg_s: float = 40.0  # 이 속도 이상이면 빠른 구간, 그 사이는 느린 구간
    seg_min_s: float = 0.5  # 이보다 짧은 구간은 이웃에 합친다
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
    grasp_min_contact_mm: float = 8  # 물체 높이로 집는 높이를 정할 때 턱이 물체에 닿아야 하는 최소 높이(mm). 이보다 얕게 닿아야 지나가는 물체는 거부한다
    replay_a_max_deg_s2: float = 600  # 변환 재생(mode=fitted)이 맞추는 관절 가속 한도(°/s²). 손으로 움직인 기록은 계획 이동(a_max_deg_s2)보다 훨씬 거칠어 따로 둔다
    gripper_speed_pct: float = 100  # 그리퍼만 움직이는 동작(열기·닫기·놓으며 열기)의 속도 배율 1~100. 바로 적용된다
    contact_lag_units: float = 4.0
    gripper_squeeze_torque: int = 300
    gripper_hold_torque: int = 200  # 들고 옮기는 동안의 조임 한계(RAM Torque_Limit). 그리퍼 Overload_Torque 25% 아래로 둔다 (결정 0033)
    grip_preload_units: float = 2  # 접촉하면 그리퍼 목표를 지금 위치에서 이만큼 더 닫힌 쪽으로 둔다(명령이 실제보다 많이 닫혀 계속 미는 것을 없앤다)
    gripper_load_warn_pct: float = 60  # 그리퍼 부하가 이 값을 gripper_load_s 넘게 넘으면 조임을 멈추고 WARN, 1초 더 이어지면 열고 fault. 실측 전 시작값
    gripper_load_s: float = 1.0
    gripper_open: float = 60
    gripper: str = "single_jaw"  # 그리퍼 종류: single_jaw(기본) | parallel. 세부 값은 config/grippers/<종류>.yaml이 robot.yaml 위에 겹쳐진다 (결정 0043)
    gripper_tip_offset_mm: float = 0.0  # 병렬 그리퍼: TCP(두 턱 중앙)에서 턱 끝까지 도구 축 방향 거리. 실물 몫(자리만)
    gripper_width_mm: list[list[float]] = field(default_factory=lambda: [[0, 0], [100, 90]])
    a_max_deg_s2: float = 120
    trajectory_profile: str = "quintic"
    joint_jerk_max_deg_s3: float = 300
    tool_a_max_mm_s2: float = 500
    tool_jerk_max_mm_s3: float = 3000
    joint_offsets_deg: dict[str, float] = field(default_factory=dict)
    joint_limits_deg: dict[str, list[float]] = field(default_factory=dict)
    poses: dict[str, list[float]] = field(default_factory=dict)
    arm_name: str = ""  # 고른 팔 파일 이름(config/arms/<이름>.yaml). 비어 있으면 팔 파일 없이 robot.yaml만 읽은 것 (decisions/0027)
    board_serial: str = ""  # 이 팔의 USB 시리얼 보드 시리얼 번호
    measured_on: str = ""  # 팔별 값을 잰 날짜


class ArmSelectionError(Exception):
    """팔을 하나로 고르지 못했다. candidates에 후보 팔 이름이 들어 있다."""

    def __init__(self, message: str, candidates: list[str] | None = None):
        super().__init__(message)
        self.candidates = candidates or []


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

    @property
    def pick_reach_mm(self) -> dict:
        """집기 범위: 로봇 원점에서 손끝을 수직 아래로 내려 집을 수 있는 반경 {min, max}. 작업 실행과 물체 판정(task/policy.reachable, pick_and_place의 범위 검사)이 쓴다.
        손끝 방향이 자유로운 직접 조작·조그의 움직임 범위와는 다르다(ArmService.reach_ranges). 파일의 키 이름은 호환 때문에 reach_mm다."""
        return self.reach_mm


def _read_yaml(path: Path) -> dict:
    return (yaml.safe_load(path.read_text(encoding="utf-8")) or {}) if path.exists() else {}


def arm_files(arms_dir: Path) -> dict[str, Path]:
    """config/arms/*.yaml. 이름이 _로 시작하는 파일(_template)은 팔이 아니다."""
    return {p.stem: p for p in sorted(arms_dir.glob("*.yaml")) if not p.stem.startswith("_")} if arms_dir.is_dir() else {}


def resolve_arm(arms_dir: Path, local: dict, arm: str | None = None, serials=()) -> tuple[str | None, str]:
    """팔 고르기 (decisions/0027): 인자 -> local.yaml의 arm -> 연결된 보드 시리얼 -> 팔 파일이 하나뿐이면 그것.
    (이름, 고른 이유)를 돌려준다. 팔 파일이 없으면 (None, ...). 못 고르면 ArmSelectionError."""
    files = arm_files(arms_dir)
    for name, why in ((arm, "--arm 옵션"), (local.get("arm"), "local.yaml의 arm")):
        if name:
            if name not in files:
                raise ArmSelectionError(f"팔 파일이 없다: {name} (있는 팔: {sorted(files) or '없음'})", sorted(files))
            return name, why
    if not files:
        return None, "팔 파일 없음"
    serials = {str(x) for x in serials if x}
    if serials:
        hit = [n for n, p in files.items() if str(_read_yaml(p).get("board_serial", "")) in serials]
        if len(hit) == 1:
            return hit[0], "연결된 보드 시리얼이 같은 팔 파일"
        if len(hit) > 1:
            raise ArmSelectionError(f"보드 시리얼이 같은 팔 파일이 여럿이다: {hit}. --arm으로 고른다", hit)
    if len(files) == 1:
        return next(iter(files)), "팔 파일이 하나뿐"
    raise ArmSelectionError(f"팔을 고르지 못했다. --arm <이름>이나 config/local.yaml의 arm으로 고른다 (있는 팔: {sorted(files)})", sorted(files))


GRIPPER_TYPES = ("single_jaw", "parallel")


def gripper_files(grippers_dir: Path) -> dict[str, Path]:
    """config/grippers/*.yaml (종류 이름 = 파일 이름)."""
    return {p.stem: p for p in sorted(grippers_dir.glob("*.yaml"))} if grippers_dir.is_dir() else {}


def load_robot_ex(path: str | Path = "config/robot.yaml", arm: str | None = None, serials=(), arms_dir: str | Path | None = None,
                  local_path: str | Path | None = None) -> tuple[RobotConfig, dict]:
    """robot.yaml <- config/arms/<팔>.yaml <- config/grippers/<종류>.yaml <- config/local.yaml 순서로 겹쳐 읽는다(뒤가 앞을 덮고, 키 단위로 통째로 바꾼다).
    arms 폴더와 local.yaml은 robot.yaml 옆에서 찾는다. 둘 다 없으면 robot.yaml만 읽는다(예전과 같다).
    (설정, {"arm": 이름|None, "reason": 이유, "files": [읽은 파일]})."""
    path = Path(path)
    arms_dir = Path(arms_dir) if arms_dir else path.parent / "arms"
    local_path = Path(local_path) if local_path else path.parent / "local.yaml"
    local = _read_yaml(local_path)
    name, reason = resolve_arm(arms_dir, local, arm, serials)
    merged = _read_yaml(path)
    files = [str(path)]
    arm_yaml = _read_yaml(arm_files(arms_dir)[name]) if name else {}
    gtype = local.get("gripper") or arm_yaml.get("gripper") or merged.get("gripper") or "single_jaw"
    if gtype not in GRIPPER_TYPES:
        raise ValueError(f"알 수 없는 그리퍼 종류: {gtype} (있는 종류: {list(GRIPPER_TYPES)})")
    gfile = path.parent / "grippers" / f"{gtype}.yaml"
    if name:
        merged |= arm_yaml
        files.append(str(arm_files(arms_dir)[name]))
        merged["arm_name"] = name
    if gfile.exists():  # 순서: robot.yaml <- 팔 <- 그리퍼 <- local. 팔 파일의 폭 표·조임 값은 단동식을 잰 것이라 다른 그리퍼 종류에서는 그리퍼 파일이 이긴다
        merged |= _read_yaml(gfile)
        files.append(str(gfile))
    merged["gripper"] = gtype
    if local_path.exists():
        files.append(str(local_path))
    merged |= {k: v for k, v in local.items() if k != "arm"}
    return draccus.decode(RobotConfig, merged), {"arm": name, "reason": reason, "files": files, "local": local}


def load_robot(path: str | Path = "config/robot.yaml", arm: str | None = None, serials=(), arms_dir: str | Path | None = None,
               local_path: str | Path | None = None) -> RobotConfig:
    return load_robot_ex(path, arm, serials, arms_dir, local_path)[0]


def load_workspace(path: str | Path = "config/workspace.yaml") -> WorkspaceConfig:
    return _load(WorkspaceConfig, path)
