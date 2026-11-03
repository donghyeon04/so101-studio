from typing import ClassVar

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli import main
from so101_studio.core.types import JointState


def test_sim_driver_echoes_written_joints():
    d = SimDriver()
    d.connect()
    d.write_joints(JointState(1, 2, 3, 4, 5, 6))
    assert d.read_joints() == JointState(1, 2, 3, 4, 5, 6)
    d.set_torque(True)
    assert d.torque_enabled() and d.read_health()["voltage_v"] == 12.0


def test_cli_status_sim(capsys):
    main(["--sim", "status"])
    out = capsys.readouterr().out
    assert "shoulder_pan" in out and "voltage 12.0 V" in out and "torque off" in out


def test_arm_driver_applies_joint_offsets():
    from types import SimpleNamespace

    from so101_studio.arm.driver import ArmDriver

    class Bus:
        written = None

        def sync_read(self, name):
            return {"shoulder_pan": -1.5, "shoulder_lift": 0, "elbow_flex": 0, "wrist_flex": 0, "wrist_roll": 0, "gripper": 0}

        def sync_write(self, name, values):
            self.written = values

    d = ArmDriver(offsets={"shoulder_pan": -1.5})
    d._robot = SimpleNamespace(bus=Bus())
    assert d.read_joints().shoulder_pan == 0  # 모터 -1.5° 가 보정된 0°
    d.write_joints(JointState(0, 0, 0, 0, 0, 0))
    assert d._robot.bus.written["shoulder_pan"] == -1.5


def test_arm_driver_power_info_and_acceleration_skip_the_gripper():
    from types import SimpleNamespace

    from so101_studio.arm.driver import ArmDriver

    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    class Bus:
        motors = names

        def __init__(self):
            self.writes = []

        def sync_read(self, name, normalize=True):
            return {"Present_Current": {m: 0x8000 | 20 for m in names}, "Present_Load": {m: -250 for m in names}}[name]

        def read(self, name, motor, normalize=True):
            return 121 if name == "Present_Voltage" else 5

        def write(self, name, motor, value):
            self.writes.append((name, motor, value))

    d = ArmDriver(acceleration=10, gripper_acceleration=25)
    d._robot = SimpleNamespace(bus=Bus())
    v, cur, load = d.read_power()
    assert v == 12.1 and cur["gripper"] == 20 * 6.5 and load["gripper"] == 25.0  # 부호 비트를 떼고, 부하는 절댓값 %
    d.set_acceleration(254)
    acc = {m: val for n, m, val in d._robot.bus.writes if n == "Acceleration"}
    assert acc["shoulder_pan"] == 254 and "gripper" not in acc  # 그리퍼는 건드리지 않는다
    assert set(d.read_servo_info()) == set(names)


# ---- 연결은 토크를 끄는 쓰기를 하지 않는다 (팔이 뻗은 채 다시 연결해도 떨어지지 않게)
class ConnectBus:
    """연결 때 쓰는 버스 부분만 흉내 낸다. 쓰기를 모두 기록한다. torque_reads는 Torque_Enable을 읽을 때 차례로 돌려줄 값(남으면 마지막 값)."""

    names: ClassVar[list[str]] = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    def __init__(self, torque_reads, regs=None):
        self.motors = {n: None for n in self.names}
        self.torque_reads, self.writes, self.regs = list(torque_reads), [], dict(regs or {})
        self.is_calibrated = True

    def connect(self):
        pass

    def read(self, register, motor, normalize=True, num_retry=0):
        if register == "Torque_Enable":
            return self.torque_reads.pop(0) if len(self.torque_reads) > 1 else self.torque_reads[0]
        return self.regs.get((register, motor), 0)

    def write(self, register, motor, value, normalize=True, num_retry=0):
        self.writes.append((register, motor, value))
        self.regs[(register, motor)] = value


def connect_with(monkeypatch, bus):
    from types import SimpleNamespace

    from so101_studio.arm import driver as drv

    robot = SimpleNamespace(bus=bus, is_calibrated=True)
    monkeypatch.setattr(drv, "SO101Follower", lambda cfg: robot)
    monkeypatch.setattr(drv, "SO101FollowerConfig", lambda **k: SimpleNamespace(**k))
    monkeypatch.setattr(drv.time, "sleep", lambda _s: None)
    d = drv.ArmDriver({"p": 16, "i": 0, "d": 32}, {}, 10, 25)
    d.connect("COMX", "arm")
    return d


def test_connect_with_torque_on_never_writes_torque_or_lock(monkeypatch):
    bus = ConnectBus([1])  # 토크가 켜져 있다: 설정값이 다르면 PID·가속만 쓴다
    d = connect_with(monkeypatch, bus)
    assert d.connect_info["torque_on_at_connect"] is True
    assert not [w for w in bus.writes if w[0] in ("Torque_Enable", "Lock", "Max_Torque_Limit", "Protection_Current", "Overload_Torque")]
    assert ("P_Coefficient", "shoulder_pan", 16) in bus.writes and ("Acceleration", "gripper", 25) in bus.writes and ("Acceleration", "elbow_flex", 10) in bus.writes


def test_connect_skips_settings_that_already_have_the_value(monkeypatch):
    regs = {}
    for m in ConnectBus.names:
        regs |= {("P_Coefficient", m): 16, ("I_Coefficient", m): 0, ("D_Coefficient", m): 32, ("Acceleration", m): 25 if m == "gripper" else 10}
    bus = ConnectBus([1], regs)
    d = connect_with(monkeypatch, bus)
    assert bus.writes == [] and d.connect_info == {"torque_on_at_connect": True, "writes": []}  # 같은 값이면 하나도 쓰지 않는다


def test_one_zero_reading_is_not_taken_as_torque_off(monkeypatch):
    bus = ConnectBus([0, 0, 0, 0, 0, 0, 1])  # 첫 번째 읽기(6개 모두 0)는 믿지 않고 한 번 더 읽는다: 이번에는 켜져 있다
    d = connect_with(monkeypatch, bus)
    assert d.connect_info["torque_on_at_connect"] is True and not [w for w in bus.writes if w[0] in ("Lock", "Torque_Enable")]


def test_connect_with_torque_off_only_unlocks_and_writes_the_eeprom_items(monkeypatch):
    bus = ConnectBus([0])
    d = connect_with(monkeypatch, bus)
    assert d.connect_info["torque_on_at_connect"] is False
    assert not [w for w in bus.writes if w[0] == "Torque_Enable"]  # 토크를 끄는 쓰기(disable_torque)는 하지 않는다
    assert {m for r, m, v in bus.writes if r == "Lock" and v == 0} == set(ConnectBus.names)
    assert ("Max_Torque_Limit", "gripper", 500) in bus.writes and ("Overload_Torque", "gripper", 25) in bus.writes

