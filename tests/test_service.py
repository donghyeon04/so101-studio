import csv
import time

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli import main
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState, ToolPose
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    ok, msg = s.connect()
    assert ok, msg
    yield s
    s.disconnect()


def test_connect_failure_is_reported_not_raised():
    class Broken(SimDriver):
        def connect(self, *a, **k):
            raise ConnectionError("COM9 없음")

    ok, msg = ArmService(Broken(), CFG, WS).connect()
    assert not ok and "COM9" in msg


def test_status_has_tool_pose_and_health(svc):
    st = svc.status()
    assert st.state == "idle" and st.torque_enabled is False and st.voltage_v == pytest.approx(12.0)
    assert st.tool.z_mm > 50 and set(st.temperature_c) == set(vars(st.joints))


def test_status_stream_runs_at_about_10hz_and_writes_status_csv(svc):
    got = []
    svc.status_stream(got.append)
    time.sleep(1.1)
    assert 8 <= len(got) <= 14
    svc.logger._status.flush()
    rows = list(csv.reader((svc.logger.dir / "status.csv").open(encoding="utf-8")))
    assert rows[0][0] == "t" and len(rows) >= 9 and len(rows[1]) == 13 and rows[0][-1] == "voltage_v"


def test_commands_go_through_the_service_and_are_logged(svc):
    r1 = svc.move_joints(JointState(10, -90, 95, 0, 0, 3))
    r2 = svc.move_tool(ToolPose(300, 200, 40, 90))
    r3 = svc.set_gripper(54, as_width=True)
    assert (r1.status, r2.status, r3.status) == ("ok", "ok", "ok")
    svc.logger._events.flush()
    kinds = [__import__("json").loads(line)["kind"] for line in (svc.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    assert kinds[:4] == ["connect", "movej", "goto", "gripper"]


def test_unreachable_goto_returns_ik_fail_without_moving(svc):
    q0 = svc.driver.read_joints()
    assert svc.move_tool(ToolPose(300, 600, 40, 90)).status == "ik_fail"
    assert svc.driver.read_joints() == q0


def test_pick_and_place_and_retarget(svc):
    r = svc.pick_and_place(DetectedObject("o", 300, 200, 0, 0, 0, 1, "manual"), "right")
    assert r.status in ("ok", "no_object")  # 시뮬레이션은 물체가 없어 no_object가 나올 수 있다
    h = svc.retarget(JointState(20, -90, 95, 0, 0, 3))
    assert h.wait(10).status == "ok"
    assert svc.driver.read_joints().shoulder_pan == pytest.approx(20, abs=0.05)


def test_estop_blocks_motion_until_resume(svc):
    svc.estop()
    assert svc.move_joints(JointState(10, -90, 95, 0, 0, 3)).status == "stopped"
    svc.ctl.resume()
    assert svc.move_joints(JointState(10, -90, 95, 0, 0, 3)).status == "ok"


def test_cli_goes_through_the_service(capsys):
    main(["--sim", "--speed", "100", "movej", "5", "-90", "95", "0", "0", "3"])
    main(["--sim", "status"])
    out = capsys.readouterr().out
    assert "ok" in out and "shoulder_pan" in out and "torque" in out
