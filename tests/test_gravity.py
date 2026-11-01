import time

import numpy as np
import pytest

from so101_studio.arm import gravity
from so101_studio.arm.kinematics import ARM_JOINTS
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli import gravcal
from so101_studio.core.config import load_robot
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml")


@pytest.fixture(scope="module")
def grav():
    return gravity.Gravity(CFG.urdf, {j: {"k": 300.0, "b": 10.0, "s": 20.0} for j in gravity.GRAVITY_JOINTS})


def test_gravity_torque_grows_when_the_arm_reaches_out(grav):
    folded = grav.torques(JointState(0, -90, 93, 0, 0, 3))
    reach = grav.torques(JointState(0, 85, -75, -10, 0, 3))
    i = ARM_JOINTS.index("shoulder_lift")
    assert abs(reach[i]) > 5 * abs(folded[i])  # 접은 자세는 어깨가 거의 힘을 안 쓴다
    assert abs(grav.torques(JointState(0, 30, 0, 0, 0, 3))[ARM_JOINTS.index("shoulder_pan")]) < 1e-3  # 수직축은 중력 없음


def test_feedforward_angle_is_load_over_slope_and_clipped(grav):
    q = JointState(0, 40, -50, 0, 0, 3)
    ff = grav.ff_deg(q)
    load = grav.load_at(q)
    for j, v in load.items():
        assert ff[ARM_JOINTS.index(j)] == pytest.approx(np.clip(v / 20.0, -gravity.MAX_FF_DEG, gravity.MAX_FF_DEG))
    assert ff[ARM_JOINTS.index("shoulder_pan")] == 0 and abs(grav.ff_deg(q, gain=100)).max() <= gravity.MAX_FF_DEG + 1e-9


def test_fit_recovers_gain_and_offset_from_noisy_samples():
    rng = np.random.default_rng(1)
    tau = rng.uniform(-0.9, 0.9, 30)
    f = gravity.fit(tau, 320 * tau + 25 + rng.normal(0, 8, 30))
    assert f["k"] == pytest.approx(320, rel=0.05) and f["b"] == pytest.approx(25, abs=10) and f["r2"] > 0.95


def test_follow_writes_the_goal_with_the_gravity_feedforward_added(grav):
    d = SimDriver([0, 30, -40, 0, 0, 3])
    written = []
    d.write_joints = lambda q: written.append(q)  # 실제 팔처럼 명령이 위치를 바로 바꾸지 않게 기록만 한다
    ctl = MotionController(d, None, Safety(CFG.joint_limits_deg), CFG.control_hz)
    ctl.gravity = grav
    ctl.follow(True)
    time.sleep(0.2)
    hold = ctl._last
    ctl.follow(False)
    ctl.close()
    ff = grav.ff_deg(d.read_joints())
    assert ff.any() and written[-1].shoulder_lift == pytest.approx(hold.shoulder_lift + ff[1], abs=1e-6)  # 서보에 보낸 값 = 목표 + 미리 더한 각도


def test_gravcal_runs_in_sim_and_saves_params(tmp_path, monkeypatch):
    from so101_studio.core.config import load_workspace
    from so101_studio.service.arm_service import ArmService

    g = gravity.Gravity(CFG.urdf)
    d = SimDriver(CFG.poses["home"])
    svc = ArmService(d, CFG, load_workspace("config/workspace.yaml"), speed_pct=100, log_root=str(tmp_path))
    svc.connect()
    d.read_loads_signed = lambda: {j: (300 * g.torques(d.read_joints())[i] if j in gravity.GRAVITY_JOINTS else 0.0) for i, j in enumerate(ARM_JOINTS + ["gripper"])}
    monkeypatch.setattr(gravcal, "step_slope", lambda svc, joint: 18.0)
    monkeypatch.setattr(gravcal, "SETTLE_S", 0.05)
    monkeypatch.setattr(gravity, "DEFAULT_PATH", str(tmp_path / "gravity.json"))
    out = []
    params = gravcal.run_gravcal(svc, out=out.append)
    svc.disconnect()
    assert params is not None and params["shoulder_lift"]["k"] == pytest.approx(300, rel=0.02) and params["shoulder_lift"]["s"] == 18.0
    assert (tmp_path / "gravity.json").exists()
