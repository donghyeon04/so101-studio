"""REQ-072 (mujoco-spec 13.10.4): 에피소드 기록기. LeRobot 데이터셋으로 저장·버림·다시 읽기. lerobot·mujoco가 없으면 건너뛴다."""
import time

import pytest

pytest.importorskip("mujoco")
pytest.importorskip("lerobot.datasets.lerobot_dataset")

from so101_studio.arm.mujoco_driver import MuJoCoDriver
from so101_studio.arm.mujoco_record import NAMES, EpisodeRecorder, dataset_dir
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.service.arm_service import ArmService


def test_dataset_dir_name_rule(tmp_path):
    assert dataset_dir("pick_58-a", tmp_path) == tmp_path / "runs" / "datasets" / "pick_58-a"
    with pytest.raises(ValueError):
        dataset_dir("../x", tmp_path)


@pytest.mark.slow
def test_records_saves_discards_and_reads_back(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cfg, ws = load_robot("config/robot.yaml"), load_workspace("config/workspace.yaml")
    drv = MuJoCoDriver(cfg, ws)
    svc = ArmService(drv, cfg, ws, log_root=str(tmp_path / "runs"))
    svc.incident_after_s = 0
    assert svc.connect()[0]
    root = tmp_path / "ds"
    rec = EpisodeRecorder(svc, drv, root, fps=25, task="hold still")
    with pytest.raises(ValueError):
        EpisodeRecorder(svc, drv, root)  # 이미 있는 폴더는 거부
    try:
        rec.start()
        time.sleep(1.5)  # 첫 줄은 렌더러(GL 문맥)를 만드느라 느리다
        assert rec.stop(save=True) > 5
        rec.start()
        time.sleep(0.3)
        assert rec.stop(save=False) == 0
        rec.close()
    finally:
        svc.disconnect()
    ds = LeRobotDataset(f"local/{root.name}", root=root)
    assert ds.num_episodes == 1 and ds.fps == 25 and ds.features["action"]["names"] == NAMES
    row = ds[0]
    assert row["observation.images.top"].shape == (3, 240, 320) and row["observation.images.wrist"].shape == (3, 240, 320)
    assert row["observation.state"].shape == (6,) and row["task"] == "hold still"
