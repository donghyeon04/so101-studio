"""하드웨어 없이 도는 가짜 장치: 명령받은 위치를 그대로 현재 위치로 돌려준다."""
import math
from dataclasses import replace
from pathlib import Path

import yaml

from ..core.types import JointState
from .driver import JOINTS

REPO_ROOT = Path(__file__).resolve().parents[3]  # 다른 폴더에서 실행해도 config를 찾게(mujoco_model.MODEL_XML과 같은 방식)


def _read_objects(path: Path, what: str, auto_names: bool = False) -> list:
    """YAML의 objects 행을 SimObject로(auto_names면 name이 없는 행은 obj-1, obj-2, …). 파일이 없거나 모르는 키·name/x_mm/y_mm 빠짐·이름 중복이면 ValueError(13.5)."""
    from .mujoco_model import SimObject

    if not path.exists():
        raise ValueError(f"{what} 파일이 없다: {path}")
    rows = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("objects") or []
    objs = []
    for i, row in enumerate(rows, 1):
        try:
            objs.append(SimObject(**({"name": f"obj-{i}", **row} if auto_names else row)))
        except (TypeError, AttributeError) as e:  # 모르는 키·빠진 키(TypeError), 행이 mapping이 아님(AttributeError)
            raise ValueError(f"{what} {path.name}의 {i}번째 물체가 잘못됐다: {e}") from e
    if len({o.name for o in objs}) != len(objs):
        raise ValueError(f"{what} {path.name}의 물체 이름이 겹친다")
    return objs


def _rect_corners(o) -> list[tuple[float, float]]:
    c, s = math.cos(math.radians(o.angle_deg)), math.sin(math.radians(o.angle_deg))
    return [(o.x_mm + c * dx - s * dy, o.y_mm + s * dx + c * dy) for dx, dy in ((o.length_mm / 2, o.width_mm / 2), (-o.length_mm / 2, o.width_mm / 2), (-o.length_mm / 2, -o.width_mm / 2), (o.length_mm / 2, -o.width_mm / 2))]


def _overlap(a, b) -> bool:
    """바닥 사각형 둘이 겹치거나 닿는가(분리축 검사)."""
    ca, cb = _rect_corners(a), _rect_corners(b)
    for poly in (ca, cb):
        for i in range(4):
            ax, ay = poly[i][1] - poly[(i + 1) % 4][1], poly[(i + 1) % 4][0] - poly[i][0]  # 변의 법선
            pa = [ax * x + ay * y for x, y in ca]
            pb = [ax * x + ay * y for x, y in cb]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def _check_overlap(objs: list, path: Path) -> None:
    for i, a in enumerate(objs):
        for b in objs[i + 1:]:
            if _overlap(a, b):
                raise ValueError(f"{path.name}의 {a.name}과 {b.name}이 겹친다")


def make_sim_driver(cfg, ws, scene=None, scenario=None, viewer=False):
    """MuJoCoDriver를 만든다(13.1·13.5). scenario는 이름(config/sim_scenarios/<이름>.yaml) 또는 .yaml 경로, 없으면 config/manual_objects.yaml(obj-1부터). viewer는 `--sim-viewer`(mujoco 뷰어). 두 물체의 바닥 사각형이 겹치면 ValueError(13.5). mujoco가 없으면 ImportError."""
    import mujoco  # noqa: F401  없으면 여기서 ImportError (CLI가 안내문으로 바꾼다)

    from .mujoco_driver import MuJoCoDriver
    if scenario:
        path = Path(scenario) if str(scenario).endswith(".yaml") else REPO_ROOT / "config" / "sim_scenarios" / f"{scenario}.yaml"
        objs, name = _read_objects(path, "시나리오"), path.stem
    else:
        path = REPO_ROOT / "config" / "manual_objects.yaml"
        objs, name = (_read_objects(path, "수동 물체", auto_names=True) if path.exists() else []), None
    _check_overlap(objs, path)
    return MuJoCoDriver(cfg, ws, scene=scene, objects=objs, scenario=name, viewer=viewer)


class SimDriver:
    def __init__(self, home: list[float] | None = None, grip_block: float | None = None):
        self._q = JointState(*(home or [0.0] * 6))
        self._torque = False
        self.grip_block = grip_block  # 물체가 있는 것처럼 그리퍼가 이 값 아래로는 닫히지 않는다 (시험용)
        self.connected = False
        self.acceleration = 10
        self.simulated = True
        self.locked_reason: str | None = None
        self.event_sink = None  # ArmDriver와 같은 자리(시뮬레이션은 레지스터가 없어 쓰지 않는다)
        self.expect_torque: bool | None = None
        self.torque_limit = 1000  # 그리퍼 RAM Torque_Limit(시험이 쓰기 순서를 본다)
        self.torque_limit_writes: list[int] = []
        self.grip_load = 0.0  # 그리퍼 부하(%) 시험용

    def connect(self, port: str = "sim", calibration_id: str = "sim") -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False
        self.acceleration = 10

    def read_joints(self) -> JointState:
        return replace(self._q)

    def write_joints(self, q: JointState) -> None:
        if self.grip_block is not None and q.gripper < self.grip_block:
            q = replace(q, gripper=self.grip_block)
        self._q = replace(q)

    def read_health(self) -> dict:
        return {
            "temperature_c": {m: 25.0 for m in JOINTS},
            "load_pct": {m: (self.grip_load if m == "gripper" else 0.0) for m in JOINTS},
            "voltage_v": 12.0,
            "current_ma": 0.0,
            "current_ma_by_joint": {m: 0.0 for m in JOINTS},
            "torque": {m: int(self._torque) for m in JOINTS},
            "status_bits": {m: 0 for m in JOINTS},
        }

    def probe_reset(self) -> dict:
        """가상 서보의 풀림 진단. simulated_reset이 켜져 있으면 서보가 리셋된 것처럼(RAM 기본값 Acceleration 254, ID 1만 응답) 답한다."""
        reset = getattr(self, "simulated_reset", False)
        exp = {m: self.acceleration for m in JOINTS}
        return {"ping": {m: (m == JOINTS[0] if reset == "id1" else True) for m in JOINTS}, "torque": {m: int(self._torque) for m in JOINTS},
                "acceleration": {m: (254 if reset is True else exp[m]) for m in JOINTS}, "torque_limit": {m: 1000 for m in JOINTS},
                "status": {m: getattr(self, "simulated_status", {}).get(m, 0) for m in JOINTS}, "voltage_dv": {m: 120 for m in JOINTS},
                "ids": {m: i + 1 for i, m in enumerate(JOINTS)}, "expected_acceleration": exp}

    def read_power(self) -> tuple[float, dict, dict]:
        return 12.0, {m: 0.0 for m in JOINTS}, {m: 0.0 for m in JOINTS}

    def read_servo_info(self) -> dict:
        return {m: {"Acceleration": self.acceleration} for m in JOINTS}

    def set_acceleration(self, value: int) -> None:
        self.acceleration = value

    def torque_enabled(self) -> bool:
        return self._torque

    def set_torque(self, on: bool) -> None:
        self._torque = on
        self.expect_torque = on

    def set_gripper_torque_limit(self, value: int) -> None:
        self.torque_limit_writes.append(int(value))
        self.torque_limit = int(value)

    def gripper_torque_limit(self) -> int:
        return self.torque_limit
