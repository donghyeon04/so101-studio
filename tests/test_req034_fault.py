"""REQ-034: fault 상세(관절·값·한도)와 사람이 읽을 문장(원인과 해결법)이 이벤트·MotionResult.reason·상태에 담긴다."""

from so101_studio.arm.fault_text import describe
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import move_j
from so101_studio.core.config import load_robot
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml", gripper="single_jaw")  # 홈 wrist_roll 0(병렬 홈은 90, REQ-103)에서 shoulder_pan만 움직이는 시험
HOME = JointState(*CFG.poses["home"])


def rig(**safety_kw):
    d = SimDriver(CFG.poses["home"])
    ctl = MotionController(d, None, Safety(CFG.joint_limits_deg, **safety_kw), 100)
    ctl.events = []
    ctl.on_event = lambda kind, level, code, data: ctl.events.append((code, data))
    return ctl


def test_step_fault_names_the_joint_value_and_limit():
    ctl = rig(max_step_deg=0.2)
    r = ctl.execute(move_j(HOME, JointState(40, -90, 93, 0, 0, 3), 90, 120)).wait()
    ctl.close()
    assert r.status == "fault" and "shoulder_pan" in r.reason and "0.2" in r.reason and "재개" in r.reason
    _code, data = next(e for e in ctl.events if e[0] == "ARM-FAULT")
    assert data["reason"] == "step" and data["detail"]["joint"] == "shoulder_pan" and data["detail"]["limit_deg"] == 0.2 and data["text"] == r.reason
    assert ctl.fault_text == r.reason and ctl.fault_detail["joint"] == "shoulder_pan"
    assert ctl.resume() and ctl.fault_text is None  # 풀면 지워진다


def test_health_faults_carry_value_and_limit():
    ctl = rig()
    d = ctl._health_detail("overload:shoulder_lift", {"load_pct": {"shoulder_lift": 97.0}, "temperature_c": {}})
    assert d["joint"] == "shoulder_lift" and d["load_pct"] == 97.0 and d["limit_pct"] == ctl.health.load_limit_pct
    t = describe("overload:shoulder_lift", d)
    assert "shoulder_lift" in t and "97%" in t and "재개" in t
    d = ctl._health_detail("overheat:gripper", {"temperature_c": {"gripper": 61.0}, "load_pct": {}})
    assert "61°C" in describe("overheat:gripper", d)
    d = ctl._health_detail("overcurrent", {"current_ma": 4300.0})
    assert "4.3A" in describe("overcurrent", d)
    ctl.close()


def test_comm_lost_workspace_and_locked_texts():
    assert "USB" in describe("comm_lost") and "다시 연결" in describe("comm_lost")
    assert "작업대 밖" in describe("workspace") and "낮" in describe("z_floor") and "elbow_flex" in describe("joint_limit:elbow_flex")
    ctl = rig()
    ctl.locked_reason = "캘리브레이션 불일치: x"
    r = ctl.execute(move_j(HOME, JointState(10, -90, 93, 0, 0, 3), 90, 120)).wait()
    ctl.close()
    assert r.status == "locked" and "캘리브레이션 불일치" in r.reason and "doctor" in r.reason


def test_out_of_bounds_event_has_text():
    ctl = rig()
    r = ctl.execute(move_j(HOME, JointState(0, -90, 93, 150, 0, 3), 90, 120)).wait()  # wrist_flex 한계 밖
    ctl.close()
    assert r.status == "out_of_bounds" and r.reason.startswith("joint_limit")
    _code, data = next(e for e in ctl.events if e[0] == "ARM-OUT-OF-BOUNDS")
    assert "관절 한계" in data["text"]


def test_fault_text_in_arm_status(tmp_path):
    from so101_studio.core.config import load_workspace
    from so101_studio.service.arm_service import ArmService

    s = ArmService(SimDriver(CFG.poses["home"]), CFG, load_workspace("tests/workspace_600x400.yaml"), speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        s.ctl._fault("comm_lost")
        st = s.status()
        assert st.fault == "comm_lost" and "통신" in st.fault_text
        r = s.home()  # 끊김 상태에서 동작을 부르면 같은 문장을 돌려준다
        assert r.status == "fault" and r.reason == st.fault_text
    finally:
        s.disconnect()
