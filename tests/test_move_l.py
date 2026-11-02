import numpy as np
import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import MotionController
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import move_l
from so101_studio.core.config import load_robot
from so101_studio.core.types import JointState, ToolPose

CFG = load_robot("config/robot.yaml")
SEED = JointState(0, -60, 60, 30, 0, 0)


@pytest.fixture(scope="module")
def kin():
    return Kinematics(CFG.urdf)


def start_pose(kin):
    return kin.fk(kin.ik(ToolPose(200, 0, 50, 0), SEED))


def test_line_stays_on_the_straight_path_and_reaches_goal(kin):
    a, b = ToolPose(200, 0, 50, 0), ToolPose(200, 80, 50, 0)
    q0 = kin.ik(a, SEED)
    traj = move_l(kin, kin.fk(q0), b, q0, 50)
    assert traj is not None
    for i in range(0, 101, 5):
        p = kin.fk(traj.sample(traj.duration_s * i / 100))
        assert (p.x_mm, p.z_mm) == pytest.approx((200, 50), abs=2.5)  # 직선에서 벗어나지 않는다
    end = kin.fk(traj.sample(traj.duration_s))
    assert (end.x_mm, end.y_mm, end.z_mm) == pytest.approx((200, 80, 50), abs=2)
    assert np.allclose(traj.velocity(0), 0) and np.allclose(traj.velocity(traj.duration_s), 0)


def test_unreachable_point_rejects_the_whole_move(kin):
    q0 = kin.ik(ToolPose(200, 0, 50, 0), SEED)
    assert move_l(kin, kin.fk(q0), ToolPose(380, 0, 50, 0), q0, 50) is None


def test_controller_executes_line_in_sim(kin):
    q0 = kin.ik(ToolPose(200, 0, 50, 0), SEED)
    traj = move_l(kin, kin.fk(q0), ToolPose(200, 60, 40, 0), q0, 200)
    d = SimDriver()
    d.write_joints(q0)
    c = MotionController(d, kin)
    r = c.execute(traj).wait(10)
    c.close()
    end = kin.fk(d.read_joints())
    assert r.status == "ok" and (end.y_mm, end.z_mm) == pytest.approx((60, 40), abs=2)


def test_plan_goto_falls_back_to_hover_then_descend(kin):
    from so101_studio.cli import plan_goto

    home = JointState(*CFG.poses["home"])
    trajs = plan_goto(kin, CFG, home, ToolPose(180, 40, 20, 0), 30)
    assert trajs is not None and 1 <= len(trajs) <= 2
    assert plan_goto(kin, CFG, home, ToolPose(380, 0, 20, 0), 30) is None  # 닿지 않는 점은 움직이지 않는다
