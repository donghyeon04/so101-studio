"""REQ-027: 잠긴 팔(CLAUDE.md: 잠김이면 읽기만 된다)에는 어떤 명령도 토크를 켜지 않는다.
팔 파일이 없는 잠금은 ctl.locked_reason에만 있고 드라이버는 모르므로, 드라이버의 set_torque 잠금만으로는 막지 못한다."""
import types

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
REASON = "팔 파일이 없다(시험)"


class TorqueSpy(SimDriver):
    """set_torque(True) 호출을 센다(드라이버가 잠금을 모르는 팔 파일 없음 잠금 흉내)."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.on_calls = 0

    def set_torque(self, on):
        self.on_calls += on
        super().set_torque(on)


@pytest.fixture
def svc():
    s = ArmService(TorqueSpy(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    assert s.connect()[0]
    s.ctl.locked_reason = REASON  # ArmService.connect가 팔 파일 없음일 때 정하는 값과 같다
    s.driver.on_calls = 0
    yield s
    s.disconnect()


def test_no_motion_command_turns_torque_on_while_locked(svc):
    obj = DetectedObject("obj-1", 300.0, 170.0, 0.0, 30.0, 30.0, 1.0, "manual")
    assert svc.move_joints(JointState(10.0, -60.0, 60.0, 20.0, 0.0, 10.0), 100).status == "locked"
    assert svc.move_tool(svc.point_pose(300.0, 200.0, 55.0)).status == "locked"
    assert svc.pick_and_place(obj, "right").status == "locked"
    assert svc.place("right").status == "locked"
    assert svc.set_gripper(20.0).status == "locked"
    assert svc.home().status == "locked" and svc.park().status == "locked"
    assert svc.jog("joint", [1.0, 0, 0, 0, 0, 0]) is False
    svc.set_sensor_offset({"x": 0.0, "y": 30.0, "z": 55.0})
    assert svc.follow_path([(260.0, 200.0), (320.0, 200.0)], 55.0, 60.0) == [] and svc.last_path.status == "locked"
    with pytest.raises(Exception):  # noqa: B017 파일이 없거나 잠겨서 실패해도 되지만 토크는 켜지 않는다
        svc.replay("없는-교시")
    assert svc.driver.on_calls == 0, "잠긴 팔에 토크를 켰다"


def test_cli_diagnostics_refuse_instead_of_turning_torque_on(svc):
    from so101_studio.cli.powertest import run_powertest
    from so101_studio.cli.shaketest import run_shaketest

    with pytest.raises(RuntimeError, match="팔 파일"):
        run_powertest(svc, 1, 20.0, out=lambda *a: None)
    with pytest.raises(RuntimeError, match="팔 파일"):
        run_shaketest(svc, 1, out=lambda *a: None)
    assert svc.driver.on_calls == 0


def test_cli_run_task_does_not_turn_torque_on_while_locked(svc):
    from so101_studio.cli import run_task

    run_task(svc, types.SimpleNamespace(dry_run=True, step=False))
    assert svc.driver.on_calls == 0


def test_unlocked_commands_still_turn_torque_on():
    s = ArmService(TorqueSpy(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    assert s.connect()[0]
    s.driver.on_calls = 0
    assert s.move_joints(JointState(10.0, -60.0, 60.0, 20.0, 0.0, 10.0), 100).status == "ok"
    assert s.driver.on_calls >= 1
    s.disconnect()


def test_driver_set_torque_on_while_locked_raises_instead_of_silently_returning():
    from so101_studio.arm.driver import ArmDriver
    d = ArmDriver()
    d.locked_reason = REASON
    with pytest.raises(RuntimeError, match="시험"):
        d.set_torque(True)


def test_grip_torque_limit_is_restored_even_if_releasing_the_hold_fails():
    from so101_studio.arm.skills import Skills
    sk = Skills.__new__(Skills)
    written = []
    sk._grip_orig = 1000
    sk.ctl = types.SimpleNamespace(hold_gripper=lambda v: (_ for _ in ()).throw(ConnectionError("x")),
                                   driver=types.SimpleNamespace(set_gripper_torque_limit=written.append))
    with pytest.raises(ConnectionError):
        sk._restore_grip_torque()
    assert written == [1000] and sk._grip_orig is None


def test_status_data_carries_log_write_errors_and_dropped(svc):
    d = svc._status_data(svc.status())
    assert d["log_write_errors"] == 0 and d["log_dropped"] == 0
