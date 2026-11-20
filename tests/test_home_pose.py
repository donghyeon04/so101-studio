"""REQ-103(피드백 F1): 그리퍼별 poses.home, 에피소드 시작 자세 poses.episode_start(없으면 home)."""
import pytest

from so101_studio.core.config import load_robot, load_workspace

PAR, SJ = load_robot("config/robot.yaml", gripper="parallel"), load_robot("config/robot.yaml", gripper="single_jaw")


def test_parallel_home_rolls_the_wrist_and_episode_start_keeps_the_old_home():
    assert PAR.poses["home"][4] == 90 and PAR.poses["home"][:4] == SJ.poses["home"][:4]
    assert PAR.episode_start() == SJ.poses["home"] == SJ.episode_start()  # ACT v2·비교 실험의 시작 자세 그대로
    assert {k: PAR.poses[k] for k in ("park", "radar_clear")} == {k: SJ.poses[k] for k in ("park", "radar_clear")}  # 키 단위로 덮으니 robot.yaml 값과 같아야 한다


def test_mujoco_driver_starts_at_episode_start_not_home():
    pytest.importorskip("mujoco")
    from so101_studio.arm.mujoco_driver import MuJoCoDriver

    d = MuJoCoDriver(PAR, load_workspace("config/workspace.yaml"), objects=[], realtime=False)
    d.connect()
    try:
        q = d.read_joints()
        assert q.wrist_roll == pytest.approx(PAR.episode_start()[4], abs=1.0) and q.shoulder_pan == pytest.approx(0, abs=1.0)
    finally:
        d.disconnect()


def test_home_goes_to_the_rolled_wrist_in_the_kinematic_sim(tmp_path):
    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.service.arm_service import ArmService

    s = ArmService(SimDriver(PAR.episode_start()), PAR, load_workspace("config/workspace.yaml"), speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        assert s.home().status == "ok"
        assert s.driver.read_joints().wrist_roll == pytest.approx(90, abs=0.5)
    finally:
        s.disconnect()
