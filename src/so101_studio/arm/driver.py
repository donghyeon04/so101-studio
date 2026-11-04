"""장치 계층. 모터 통신은 LeRobot SO101Follower의 버스만 거친다 (arm-software-spec.md 6.1)."""
import functools
import threading
import time
from dataclasses import asdict, fields

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from ..core.types import JointState
from .logged_bus import LoggedBus

JOINTS = [f.name for f in fields(JointState)]


def _locked(fn):
    @functools.wraps(fn)
    def wrapper(self, *a, **k):
        with self._lock:
            return fn(self, *a, **k)

    return wrapper


class ArmDriver:
    def __init__(self, pid: dict | None = None, offsets: dict[str, float] | None = None, acceleration: int = 254,
                 gripper_acceleration: int | None = None):
        self.pid = pid or {"p": 16, "i": 0, "d": 32}
        self.acceleration = acceleration  # 서보 자체 가속 한계(0~254, 클수록 빠름). 낮추면 움직일 때 떨림이 준다
        self.gripper_acceleration = acceleration if gripper_acceleration is None else gripper_acceleration
        self.offsets = offsets or {}  # 관절별 영점 보정(도): 읽은 값 = 모터 값 - offset
        self._robot: SO101Follower | None = None
        self.simulated = False
        self.connect_info: dict = {}
        self.event_sink = None  # sink(kind, level, code, data): ArmService가 이벤트 기록에 연결한다. 레지스터 쓰기와 토크 이벤트가 여기로 간다 (decisions/0030)
        self.expect_torque: bool | None = None  # 컨트롤러가 기대하는 토크 상태(켰으면 True, 껐으면 False, 모르면 None). ARM-TORQUE-LOST 판단에 쓴다
        self._logged: LoggedBus | None = None
        self.locked_reason: str | None = None  # 캘리브레이션이 서보와 다르면 연결은 하되 쓰기를 막는다(읽기만) (decisions/0027)
        self._lock = threading.RLock()  # 시리얼 포트는 한 번에 한 스레드만 쓴다 (동작 스레드와 상태 읽기가 겹칠 수 있다)

    def connect(self, port: str, calibration_id: str) -> None:
        robot = SO101Follower(SO101FollowerConfig(port=port, id=calibration_id))
        robot.bus.connect()
        self._robot = robot
        self._logged = None
        bus = self._bus  # 쓰기를 기록하는 버스
        self.locked_reason = None
        if not robot.is_calibrated:  # 읽기만 허용: 서보에 아무것도 쓰지 않고, 움직이는 명령은 MotionController가 막는다
            self.locked_reason = (f"캘리브레이션 불일치: {calibration_id}. 서보의 호밍 오프셋·범위가 캘리브레이션 파일과 다르거나 파일이 이 PC에 없다. "
                                  "config/calibration/<id>.json을 ~/.cache/huggingface/lerobot/calibration/robots/so_follower/로 복사하거나 lerobot-calibrate를 다시 한다")
            return
        # 연결은 토크를 끄는 쓰기를 하지 않는다: 토크가 켜진 채(특히 팔이 뻗은 자세) 다시 연결해도 팔이 떨어지지 않게 한다.
        # 토크가 꺼져 있는지는 두 번 읽어 모두 꺼져 있을 때만 믿는다(한 번 잘못 읽은 0으로 꺼진 것으로 보지 않는다).
        was_on = any(bus.read("Torque_Enable", m, num_retry=3) for m in bus.motors)
        if not was_on:
            time.sleep(0.05)
            was_on = any(bus.read("Torque_Enable", m, num_retry=3) for m in bus.motors)
        writes: list[tuple[str, str, int]] = []

        def put(m: str, register: str, value: int) -> None:  # 이미 같은 값이면 쓰지 않는다(부하를 받는 서보의 설정을 괜히 건드리지 않는다)
            if bus.read(register, m, normalize=False, num_retry=3) != value:
                bus.write(register, m, value, normalize=False, num_retry=3)
                writes.append((m, register, value))

        if not was_on:  # 토크가 이미 꺼져 있다: EEPROM에 쓰려면 Lock만 풀면 된다(disable_torque는 부르지 않는다)
            for m in bus.motors:
                bus.write("Lock", m, 0, normalize=False, num_retry=3)
        for m in bus.motors:
            put(m, "P_Coefficient", self.pid["p"])
            put(m, "I_Coefficient", self.pid["i"])
            put(m, "D_Coefficient", self.pid["d"])
            put(m, "Acceleration", self.gripper_acceleration if m == "gripper" else self.acceleration)
        if not was_on:  # EEPROM 항목은 토크가 꺼져 있을 때만 쓴다. 그리퍼 과열 방지 (LeRobot SO101 기본값)
            put("gripper", "Max_Torque_Limit", 500)
            put("gripper", "Protection_Current", 250)
            put("gripper", "Overload_Torque", 25)
        self.expect_torque = True if was_on else None
        self.connect_info = {"torque_on_at_connect": was_on, "writes": [list(w) for w in writes]}  # 이벤트 기록에 남긴다: 팔이 떨어지면 원인을 찾는 단서
    def disconnect(self) -> None:
        if self._robot:
            try:
                self._robot.bus.disconnect(disable_torque=False)  # 팔이 떨어지지 않게 토크는 그대로 둔다
            except Exception:  # noqa: BLE001, S110 포트가 이미 사라졌어도(USB 빠짐) 다시 연결할 수 있게 끝까지 정리한다
                pass
            self._robot = None

    @property
    def _bus(self):
        if self._robot is None:
            raise RuntimeError("연결되지 않음")
        raw = self._robot.bus
        if self._logged is None or self._logged._bus is not raw:
            self._logged = LoggedBus(raw, lambda: self.event_sink)
        return self._logged

    @_locked
    def read_joints(self) -> JointState:
        raw = self._bus.sync_read("Present_Position")
        return JointState(**{j: v - self.offsets.get(j, 0.0) for j, v in raw.items()})

    def _require_unlocked(self) -> None:
        if self.locked_reason:
            raise RuntimeError(self.locked_reason)

    @_locked
    def write_joints(self, q: JointState) -> None:
        self._require_unlocked()
        self._bus.sync_write("Goal_Position", {j: v + self.offsets.get(j, 0.0) for j, v in asdict(q).items()})

    @_locked
    def read_health(self) -> dict:
        bus = self._bus
        cur_ma = {m: (v & 0x7FFF) * 6.5 for m, v in bus.sync_read("Present_Current", normalize=False).items()}
        return {
            "temperature_c": {m: bus.read("Present_Temperature", m) for m in bus.motors},
            "load_pct": {m: abs(bus.read("Present_Load", m)) / 10 for m in bus.motors},
            "voltage_v": min(bus.read("Present_Voltage", m) for m in bus.motors) / 10,
            # 6개 서보 전류 합(mA). 레지스터 값 x 6.5mA 추정이고 전압 강하와 상관이 -0.84로 쓸 만하다(powertest)
            "current_ma": sum(cur_ma.values()),
            "current_ma_by_joint": cur_ma,
            # 2Hz 묶음에 붙인 두 번의 sync_read: 켜져 있어야 할 토크가 풀렸는지(ARM-TORQUE-LOST)와 서보 Status 비트(과부하 0x20 등)
            "torque": dict(bus.sync_read("Torque_Enable", normalize=False)),
            "status_bits": dict(bus.sync_read("Status", normalize=False)),
        }

    @_locked
    def read_power(self) -> tuple[float, dict, dict]:
        """전원 시험용 빠른 읽기: (전압 V, 서보별 전류 mA, 서보별 부하 %). 전류는 레지스터 값 x 6.5mA 추정이라 부하와 함께 보며 교차 확인한다."""
        bus = self._bus
        cur = bus.sync_read("Present_Current", normalize=False)
        load = bus.sync_read("Present_Load")
        v = bus.read("Present_Voltage", "shoulder_lift", normalize=False) / 10
        return v, {m: (x & 0x7FFF) * 6.5 for m, x in cur.items()}, {m: abs(x) / 10 for m, x in load.items()}

    INFO_REGS = ("P_Coefficient", "I_Coefficient", "D_Coefficient", "Acceleration", "Minimum_Startup_Force", "CW_Dead_Zone",
                 "CCW_Dead_Zone", "Max_Torque_Limit", "Torque_Limit", "Protection_Current", "Maximum_Acceleration", "Acceleration_Multiplier ",
                 "Angular_Resolution", "Phase", "Return_Delay_Time")

    @_locked
    def read_servo_info(self) -> dict:
        """서보 설정 레지스터를 읽기만 한다(떨림 원인 후보 확인용): PID, 가속, 시동 힘, 데드존, 토크 한계 등."""
        bus = self._bus
        return {m: {r.strip(): bus.read(r, m, normalize=False) for r in self.INFO_REGS} for m in bus.motors}

    @_locked
    def set_acceleration(self, value: int) -> None:
        """팔 관절 5개의 서보 가속 한계를 바꾼다(시험용). 그리퍼는 접촉 감지 때문에 그대로 둔다."""
        self.acceleration = value
        for m in self._bus.motors:
            if m != "gripper":
                self._bus.write("Acceleration", m, value)

    @_locked
    def torque_enabled(self) -> bool:
        return any(self._bus.read("Torque_Enable", m) for m in self._bus.motors)

    @_locked
    def set_gripper_torque_limit(self, value: int) -> None:
        """그리퍼 조임 힘 한계를 RAM Torque_Limit(48번)에 쓴다. EEPROM(Max_Torque_Limit)에는 쓰지 않는다: 전원을 다시 켜면 원래 값으로 돌아온다 (결정 0033)."""
        self._require_unlocked()
        self._bus.write("Torque_Limit", "gripper", int(value), normalize=False)

    @_locked
    def gripper_torque_limit(self) -> int:
        return int(self._bus.read("Torque_Limit", "gripper", normalize=False))

    @_locked
    def set_torque(self, on: bool) -> None:
        if on and self.locked_reason:
            return  # 잠긴 동안은 토크를 켜지 않는다(켜는 순간 목표 위치를 쓰기 때문)
        if on:
            self._bus.sync_write("Goal_Position", self._bus.sync_read("Present_Position"))  # 켜는 순간 튀지 않게
            self._bus.enable_torque()
        else:
            self._bus.disable_torque()
        self.expect_torque = on
