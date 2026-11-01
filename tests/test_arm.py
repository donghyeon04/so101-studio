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
