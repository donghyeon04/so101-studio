import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.core.types import JointState, ToolPose

URDF = "assets/so101_new_calib.urdf"
SEED = JointState(0, -60, 60, 30, 0, 0)


@pytest.fixture(scope="module")
def kin():
    return Kinematics(URDF)


def test_fk_zero_pose(kin):
    p = kin.fk(JointState(0, 0, 0, 0, 0, 0))
    assert (p.x_mm, p.y_mm, p.z_mm) == pytest.approx((352.5, 0, 226.5), abs=0.5)
    assert p.pitch_deg == pytest.approx(0, abs=0.1)


@pytest.mark.parametrize("target", [(200, 0, 50, 0), (180, 60, 30, 45), (250, -80, 20, 90), (120, 0, 15, 0)])
def test_ik_vertical_down_roundtrip(kin, target):
    x, y, z, yaw = target
    q = kin.ik(ToolPose(x, y, z, yaw), SEED)
    assert q is not None
    got = kin.fk(q)
    assert (got.x_mm, got.y_mm, got.z_mm) == pytest.approx((x, y, z), abs=1.0)
    assert got.pitch_deg == pytest.approx(-90, abs=1.0)
    assert got.yaw_deg == pytest.approx(yaw, abs=1.0)


def test_ik_unreachable_returns_none(kin):
    assert kin.ik(ToolPose(350, 0, 20, 0), SEED) is None


def test_ik_does_not_depend_on_what_other_calls_left_in_the_solver():
    """placo 풀이기는 마지막 fk·턱 높이 계산이 남긴 상태에 따라 같은 입력에 다른 해(또는 None)를 냈다: 상태 스레드의 fk가 끼어드는 부하에서 follow_path가 가끔 ik_fail (REQ 11-13)."""
    from so101_studio.core.config import load_robot, load_workspace

    cfg = load_robot("config/robot.yaml")
    kin = Kinematics(cfg.urdf, load_workspace("config/workspace.yaml").base, cfg.joint_limits_deg)  # 앱과 같은 설정(관절 한계·받침)
    seed, pose = JointState(0.0, -16.43, 29.72, 76.7, 0.0, 3.0), ToolPose(301.0, 200.0, 47.3, 90.0, -90.0)
    first = kin.ik(pose, seed, max_iter=100)
    assert first is not None
    for _ in range(3):
        kin.fk(JointState(0, -90, 85, -90, 0, 3))  # 다른 자세의 fk가 사이에 끼어든다
        kin.jaw_lowest_z(JointState(10, -50, 60, 0, 0, 40))
        assert kin.ik(pose, seed, max_iter=100) == first
