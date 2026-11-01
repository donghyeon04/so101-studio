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
    assert (p.x_mm, p.y_mm, p.z_mm) == pytest.approx((391.4, 0, 226.5), abs=0.5)
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
