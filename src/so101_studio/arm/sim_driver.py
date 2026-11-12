"""하드웨어 없이 도는 가짜 장치: 명령받은 위치를 그대로 현재 위치로 돌려준다."""
from dataclasses import replace

from ..core.types import JointState
from .driver import JOINTS


def make_sim_driver(cfg, ws, scene=None, scenario=None, viewer=False):
    """MuJoCoDriver를 만든다(13.1). scenario는 M3, viewer는 M4에서 쓴다. mujoco가 없으면 ImportError."""
    import mujoco  # noqa: F401  없으면 여기서 ImportError (CLI가 안내문으로 바꾼다)

    from .mujoco_driver import MuJoCoDriver

    if cfg.gripper == "parallel":
        raise ValueError("MuJoCo는 단동식만 지원")
    return MuJoCoDriver(cfg, ws, scene=scene)


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
