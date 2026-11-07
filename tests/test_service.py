import time

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli import main
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.logread import read_events, read_status
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


def test_status_stream_runs_at_about_10hz_and_writes_status_jsonl(svc):
    got = []
    svc.status_stream(got.append)
    time.sleep(1.1)
    assert 8 <= len(got) <= 14
    svc.logger.flush()
    rows = read_status(svc.logger.dir)
    assert len(rows) >= 9 and rows[0]["proc"] == "cli" and rows[0]["code"] == "STATUS"
    d = rows[-1]["data"]
    assert {"joints", "cmd", "tool", "torque", "status_bits", "voltage_v", "time_lost_s", "tick_dt_max_ms", "current_ma_by_joint"} <= set(d)
    assert [r["seq"] for r in rows] == sorted(r["seq"] for r in rows)


def test_commands_go_through_the_service_and_are_logged(svc):
    r1 = svc.move_joints(JointState(10, -90, 93, 0, 0, 3))
    r2 = svc.move_tool(ToolPose(300, 200, 40, 90))
    r3 = svc.set_gripper(54, as_width=True)
    assert (r1.status, r2.status, r3.status) == ("ok", "ok", "ok")
    svc.logger.flush()
    kinds = [r["kind"] for r in read_events(svc.logger.dir)]
    assert kinds[:4] == ["connect", "movej", "goto", "gripper"]


def test_unreachable_goto_returns_ik_fail_without_moving(svc):
    q0 = svc.driver.read_joints()
    assert svc.move_tool(ToolPose(300, 600, 40, 90)).status == "ik_fail"
    assert svc.driver.read_joints() == q0


def test_pick_and_place_and_retarget(svc):
    r = svc.pick_and_place(DetectedObject("o", 300, 200, 0, 0, 0, 1, "manual"), "right")
    assert r.status == "no_object"  # 가상 팔에는 물체가 없어 끝까지 닫힌다(고정된 결과)
    h = svc.retarget(JointState(20, -90, 93, 0, 0, 3))
    assert h.wait(10).status == "ok"
    assert svc.driver.read_joints().shoulder_pan == pytest.approx(20, abs=0.05)


def test_estop_blocks_motion_until_resume(svc):
    """E-STOP -> 거부 -> resume -> 다시 동작: 이 흐름은 서비스 단의 이 시험 하나로 본다(컨트롤러·GUI 쪽 중복은 지웠다)."""
    from so101_studio.arm.trajectory import move_j
    start = svc.driver.read_joints()
    svc.ctl.execute(move_j(start, JointState(60, -90, 93, 0, 0, 3), 30, 100))
    time.sleep(0.2)
    err = svc.estop()
    assert err is None
    pos = svc.driver.read_joints()
    assert 0 < pos.shoulder_pan < 60  # 이동 중이었다가 그 자리에 섰다
    time.sleep(0.2)
    assert svc.driver.read_joints() == pos  # 정지 뒤 더 움직이지 않는다
    assert svc.move_joints(JointState(10, -90, 93, 0, 0, 3)).status == "stopped"
    assert svc.ctl.resume()
    assert svc.move_joints(JointState(10, -90, 93, 0, 0, 3)).status == "ok"


def test_cli_goes_through_the_service(capsys):
    main(["--sim", "--speed", "100", "movej", "5", "-90", "93", "0", "0", "3"])
    main(["--sim", "status"])
    out = capsys.readouterr().out
    assert "ok" in out and "shoulder_pan" in out and "torque" in out


def test_one_sided_none_box_never_raises(svc):
    """box에 한쪽만 None이어도(제한 없음) check_goal이 예외 없이 끝난다. 예외는 조그 루프에서 통신 실패로 오인되어 comm_lost가 된다."""
    home = JointState(*CFG.poses["home"])
    for box in ({"z": [None, 150]}, {"z": [5, None]}, {"z": [None, None]}, {"x": [None, 400]}, {"x": [50, None]}, {"y": [None, None]}, {"x": [None, 300], "y": [20, None], "z": [None, 200]}):
        s = svc.set_safety(box_mm=box)
        assert s["box_mm_effective"] is not None
        assert svc.safety.check_goal(home) in (None, "workspace", "z_floor", "z_ceiling")  # 어떤 경우든 예외 없이 사유 하나로 끝난다
        assert svc.safety.check_goal(home, home) in (None, "workspace", "z_floor", "z_ceiling")
    svc.set_safety("normal", 0.0, None)


def test_service_safety_settings_roundtrip_and_validation(svc):
    s = svc.set_safety(mode="free", joint_margin_deg=-3, box_mm={"z": [10, 300]})
    assert s["mode"] == "free" and s["effective_limits_deg"]["elbow_flex"][1] == pytest.approx(97.1) and s["box_mm_effective"]["z"] == [10, 300]
    with pytest.raises(ValueError):
        svc.set_safety(mode="nope")
    with pytest.raises(ValueError):
        svc.set_safety(box_mm={"z": [-5, 100]})  # 작업대 면 아래는 안 된다
    assert svc.get_safety()["mode"] == "free"  # 실패한 호출은 설정을 바꾸지 않는다
    assert svc.set_safety("normal", 0.0, None)["box_mm_effective"]["z"][0] == CFG.tip_floor_mm


def test_pick_and_place_ok_when_the_sim_gripper_is_blocked_by_an_object(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=30.0), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        assert s.pick_and_place(DetectedObject("o", 300, 200, 0, 0, 0, 1, "manual"), "right").status == "ok"
    finally:
        s.disconnect()


def test_unsafe_mode_is_off_and_keeps_only_the_per_step_limit(svc):
    from dataclasses import replace
    home = JointState(*CFG.poses["home"])
    svc.set_safety(mode="off")
    assert svc.get_safety()["mode"] == "off"
    assert svc.safety.check_goal(replace(home, shoulder_pan=500.0)) is None  # 관절 한계·작업 영역은 꺼졌다
    assert svc.safety.check_step(home, replace(home, shoulder_pan=home.shoulder_pan + 20)) == "step"  # 한 주기 이동량 한계는 그대로


def test_cli_unsafe_flag_warns_and_runs(capsys):
    main(["--sim", "--unsafe", "status"])
    assert "모두 끕니다" in capsys.readouterr().out
