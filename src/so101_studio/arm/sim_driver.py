"""하드웨어 없이 도는 가짜 장치: 명령받은 위치를 그대로 현재 위치로 돌려준다."""
from dataclasses import replace

from ..core.types import JointState
from .driver import JOINTS


class SimDriver:
    def __init__(self, home: list[float] | None = None, grip_block: float | None = None):
        self._q = JointState(*(home or [0.0] * 6))
        self._torque = False
        self.grip_block = grip_block  # 물체가 있는 것처럼 그리퍼가 이 값 아래로는 닫히지 않는다 (시험용)
        self.connected = False

    def connect(self, port: str = "sim", calibration_id: str = "sim") -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def read_joints(self) -> JointState:
        return replace(self._q)

    def write_joints(self, q: JointState) -> None:
        if self.grip_block is not None and q.gripper < self.grip_block:
            q = replace(q, gripper=self.grip_block)
        self._q = replace(q)

    def read_health(self) -> dict:
        return {
            "temperature_c": {m: 25.0 for m in JOINTS},
            "load_pct": {m: 0.0 for m in JOINTS},
            "voltage_v": 12.0,
        }

    def torque_enabled(self) -> bool:
        return self._torque

    def set_torque(self, on: bool) -> None:
        self._torque = on

    def set_gripper_torque_limit(self, value: int) -> None:
        pass
