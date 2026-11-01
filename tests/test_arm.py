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
