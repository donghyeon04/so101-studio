"""low_lift로 멈춘 뒤 home이 방금 놓은 상자를 쓸지 않게 (실물 2-2 11-13)."""
from types import SimpleNamespace

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", arm="so101_follower_01")
WS = load_workspace("config/workspace.yaml")


def make(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    return s


@pytest.mark.slow
def test_home_after_low_lift_refuses_a_path_that_sweeps_the_placed_box(tmp_path):
    """실물 2-2: low_lift로 멈춘 뒤 home이 상자와 0.1mm까지 붙는 경로((360,200) 케이스)는 거부, 6mm 떨어진 경로((442.5,112.5))는 간다."""
    s = make(tmp_path)
    try:
        sk = s.skills
        cases = (((20.89, 17.41, 7.87, 102.68, 0.40, 55.81), (360.0, 200.0), "stopped"), ((65.7, 11.0, 10.3, 102.7, -1.0, 55.8), (442.5, 112.5), "ok"))
        for q, c, want in cases:
            assert sk._run([sk._joint_move(JointState(*q))]).status == "ok"
            sk._box_after_low_lift = SimpleNamespace(center=SimpleNamespace(x_mm=c[0], y_mm=c[1]), box_w=58.0, obj_h=39.0)
            r = sk.home()
            assert r.status == want, (c, r)
            if want == "stopped":
                assert r.reason.startswith("home_blocked") and sk._box_after_low_lift is not None
            else:
                assert sk._box_after_low_lift is None  # 홈에 갔으면 기억을 지운다
    finally:
        s.disconnect()
