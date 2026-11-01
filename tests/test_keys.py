import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli.keys import KeysSession
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import ToolPose
from so101_studio.service.arm_service import ArmService
from so101_studio.service.targets import from_table, parse_target

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
CUR = ToolPose(300, 200, 50, 90, -90)


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    yield s
    s.disconnect()


def test_parse_absolute_and_relative_frames():
    p = parse_target("300 200 40", "table", WS.base, CUR)
    assert (p.x_mm, p.y_mm, p.z_mm, p.yaw_deg) == (300, 200, 40, 90)
    fx, fy = from_table(CUR.x_mm, CUR.y_mm, "robot", WS.base)
    r = parse_target("d 10 0 5", "robot", WS.base, CUR)  # 로봇 앞쪽 10mm
    assert from_table(r.x_mm, r.y_mm, "robot", WS.base) == pytest.approx((fx + 10, fy), abs=1e-6)
    assert r.z_mm == 55
    with pytest.raises(ValueError):
        parse_target("1 2", "table", WS.base, CUR)
    with pytest.raises(ValueError):
        parse_target("a b c", "table", WS.base, CUR)


def test_preview_does_not_move_and_flags_unreachable(svc):
    q0 = svc.status().joints
    ok_reason, q = svc.preview_tool(ToolPose(300, 150, 60, 90, -90))
    bad_reason, _ = svc.preview_tool(ToolPose(300, 390, 60, 90, -90))
    assert ok_reason is None and q is not None and bad_reason is not None
    assert svc.status().joints == q0


def test_coordinate_mode_moves_and_refuses(svc):
    out = []
    s = KeysSession(svc, 20, lambda k: False, lambda p: "", out=out.append)
    assert s.coord_line("300 150 60") is None
    t = svc.status().tool
    assert (t.x_mm, t.y_mm) == pytest.approx((300, 150), abs=4)
    q = svc.status().joints
    s.coord_line("300 5000 60")
    assert svc.status().joints == q and "이동하지 않음" in out[-1]
    assert s.coord_line("frame robot") is None and s.frame == "robot"
    assert s.coord_line("j") == "joint" and s.coord_line("x") == "quit"


def test_joint_mode_key_hold_then_tab_to_coord(svc):
    seq = iter([{"q"}] * 15 + [set(), {"tab"}])
    cur = {"v": set()}
    def key_down(k):
        if k == "x":  # 한 틱의 시작: 다음 입력 상태로 넘어간다
            cur["v"] = next(seq)
        return k in cur["v"] and k != "x"
    p0 = svc.status().joints.shoulder_pan
    s = KeysSession(svc, 20, key_down, lambda p: "", out=lambda *_: None, tick_s=0.02)
    assert s.joint_mode() == "coord"
    assert svc.status().joints.shoulder_pan > p0 + 1


def test_speed_commands(svc):
    s = KeysSession(svc, 20, lambda k: False, lambda p: "", out=lambda *_: None)
    s.coord_line("speed 40")
    s.coord_line("jog 35")
    assert svc.speed == 40 and s.speed == 35
