"""장치 계층. 모터 통신은 LeRobot SO101Follower의 버스만 거친다 (arm-software-spec.md 6.1)."""
from dataclasses import asdict, fields

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from ..core.types import JointState

JOINTS = [f.name for f in fields(JointState)]


class ArmDriver:
    def __init__(self, pid: dict | None = None):
        self.pid = pid or {"p": 16, "i": 0, "d": 32}
        self._robot: SO101Follower | None = None

    def connect(self, port: str, calibration_id: str) -> None:
        robot = SO101Follower(SO101FollowerConfig(port=port, id=calibration_id))
        robot.bus.connect()
        if not robot.is_calibrated:
            robot.bus.disconnect(disable_torque=False)
            raise RuntimeError(f"캘리브레이션 불일치: {calibration_id} (lerobot-calibrate 다시 실행)")
        self._robot = robot
        bus = robot.bus
        bus.disable_torque()
        for m in bus.motors:
            bus.write("P_Coefficient", m, self.pid["p"])
            bus.write("I_Coefficient", m, self.pid["i"])
            bus.write("D_Coefficient", m, self.pid["d"])
        # 그리퍼 과열 방지 (LeRobot SO101 기본값)
        bus.write("Max_Torque_Limit", "gripper", 500)
        bus.write("Protection_Current", "gripper", 250)
        bus.write("Overload_Torque", "gripper", 25)

    def disconnect(self) -> None:
        if self._robot:
            self._robot.bus.disconnect(disable_torque=False)  # 팔이 떨어지지 않게 토크는 그대로 둔다
            self._robot = None

    @property
    def _bus(self):
        if self._robot is None:
            raise RuntimeError("연결되지 않음")
        return self._robot.bus

    def read_joints(self) -> JointState:
        return JointState(**self._bus.sync_read("Present_Position"))

    def write_joints(self, q: JointState) -> None:
        self._bus.sync_write("Goal_Position", asdict(q))

    def read_health(self) -> dict:
        bus = self._bus
        return {
            "temperature_c": {m: bus.read("Present_Temperature", m) for m in bus.motors},
            "load_pct": {m: abs(bus.read("Present_Load", m)) / 10 for m in bus.motors},
            "voltage_v": min(bus.read("Present_Voltage", m) for m in bus.motors) / 10,
        }

    def torque_enabled(self) -> bool:
        return any(self._bus.read("Torque_Enable", m) for m in self._bus.motors)

    def set_torque(self, on: bool) -> None:
        if on:
            self._bus.sync_write("Goal_Position", self._bus.sync_read("Present_Position"))  # 켜는 순간 튀지 않게
            self._bus.enable_torque()
        else:
            self._bus.disable_torque()
