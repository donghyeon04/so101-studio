"""REQ-072 (mujoco-spec 13.10.4): 에피소드 기록기. LeRobot 데이터셋으로 저장·버림·다시 읽기. lerobot·mujoco가 없으면 건너뛴다."""
import time
from pathlib import Path

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


def replay(dataset_root, episode_index: int, seed: int, log_dir) -> dict:
    """저장한 에피소드의 action을 같은 씨앗 배치에서 servo_joints로 25Hz 다시 틀고 judge(13.10.4 끝난 기준). 실시간."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    from so101_studio.arm import mujoco_bench as bench
    from so101_studio.core.types import JointState

    root = Path(dataset_root)
    ds = LeRobotDataset(f"local/{root.name}", root=root, episodes=[episode_index])
    actions = [ds.hf_dataset[i]["action"] for i in range(len(ds.hf_dataset))]
    suite = bench.load_suite("collect_pick")
    env = bench.BenchEnv(bench.load_cfg())
    env.reset(options={"episode": bench.episodes(suite, 1, seed_start=seed)[0], "log_dir": Path(log_dir)})
    try:
        t_next = time.perf_counter()
        for a in actions:
            env.svc.servo_joints(JointState(*[float(v) for v in a]))
            t_next += 1.0 / ds.fps
            time.sleep(max(0.0, t_next - time.perf_counter()))
        time.sleep(1.0)
        return bench.judge(env.drv, bench.layout("collect_pick", seed))
    finally:
        env.close()


def _record_one(tmp_path):
    import json

    from so101_studio.arm import mujoco_bench as bench

    out = bench.run("collect_pick", seeds=1, out_root=tmp_path, record="one")
    return json.loads((out / "results.jsonl").read_text(encoding="utf-8")), tmp_path / "runs" / "datasets" / "one"


@pytest.mark.slow
def test_expert_records_placed_ok_episode(tmp_path):
    """스크립트 전문가 한 씨앗(0): placed_ok라 저장되고 데이터셋에 그 에피소드 하나가 있다."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    row, root = _record_one(tmp_path)
    assert row["outcome"] == "placed_ok" and row["recorded_frames"] > 100
    assert LeRobotDataset("local/one", root=root).num_frames == row["recorded_frames"]


@pytest.mark.slow
@pytest.mark.measure
def test_expert_replay_places(tmp_path):
    """저장한 action을 다시 틀면 judge가 placed_ok(13.10.4 끝난 기준). 열린 고리 + 실시간이라 흔들려 measure로 둔다(11-18 측정: 7번 중 4번)."""
    row, root = _record_one(tmp_path)
    assert replay(root, 0, row["seed"], tmp_path / "replay")["outcome"] == "placed_ok"


def _tiny(root, episodes):
    """영상 없는 작은 데이터셋: 에피소드마다 action[t] = (에피소드*100 + t)를 여섯 칸에."""
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    vec = {"dtype": "float32", "shape": (6,), "names": NAMES}
    ds = LeRobotDataset.create(repo_id=f"local/{root.name}", root=root, fps=25, robot_type="so101_follower",
                               features={"observation.state": vec, "action": dict(vec)}, use_videos=False)
    for e, n in episodes:
        for t in range(n):
            v = np.full(6, e * 100 + t, np.float32)
            ds.add_frame({"observation.state": v, "action": v, "task": "t"})
        ds.save_episode()
    ds.finalize()
    return root


def _actions(root):
    import numpy as np
    import pyarrow as pa
    import pyarrow.parquet as pq

    t = pa.concat_tables([pq.read_table(f) for f in sorted((root / "data").rglob("*.parquet"))])
    return t.column("episode_index").to_numpy(), np.stack(t.column("action").to_numpy(zero_copy_only=False))[:, 0]


def test_shift_actions_moves_each_episode_k_frames_ahead(tmp_path):
    """결정 0068: 사본 <이름>-k2의 action[t] = cmd[t+2], 에피소드 끝 2프레임은 마지막 명령. 원본은 그대로, 통계는 새 action으로."""
    import json

    from so101_studio.arm.mujoco_record import shift_actions

    src = _tiny(tmp_path / "d", [(0, 5), (1, 4)])
    dst = shift_actions(src, 2)
    assert dst.name == "d-k2"
    ep, a = _actions(dst)
    assert a[ep == 0].tolist() == [2, 3, 4, 4, 4] and a[ep == 1].tolist() == [102, 103, 103, 103]
    assert _actions(src)[1][:5].tolist() == [0, 1, 2, 3, 4]
    st = json.loads((dst / "meta" / "stats.json").read_text(encoding="utf-8"))["action"]
    assert st["min"][0] == 2 and st["max"][0] == 103
    with pytest.raises(ValueError):
        shift_actions(src, 2)  # 사본이 이미 있다


def test_merge_parts_joins_pieces_and_skips_empty(tmp_path):
    """결정 0068 --jobs N 수집: part-NN 조각을 aggregate_datasets로 합치고 빈 조각은 뺀다. 조각 폴더는 합친 뒤 없다."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    from so101_studio.arm.mujoco_record import merge_parts

    root = tmp_path / "m"
    _tiny(root / "part-00", [(0, 3)])
    _tiny(root / "part-01", [(5, 4), (6, 2)])
    _tiny(root / "part-02", [])
    assert merge_parts(root) == 3
    assert not list(root.glob("part-*"))
    ds = LeRobotDataset("local/m", root=root)
    assert ds.num_episodes == 3 and ds.num_frames == 9


def test_clip_actions_keeps_every_joint_inside_limits_minus_margin(tmp_path):
    """REQ-077 (c): 사본 <이름>-clip은 관절별 [lo+margin, hi−margin]으로 자르고, 자른 프레임 수와 순서(derived)를 남긴다."""
    import json

    from so101_studio.arm.driver import JOINTS
    from so101_studio.arm.mujoco_record import clip_actions, shift_actions

    src = _tiny(tmp_path / "d", [(0, 5), (1, 4)])  # action 0~4, 100~103
    limits = {j: [1.0, 103.0] for j in JOINTS}
    dst, clipped = clip_actions(shift_actions(src, 1), limits, margin=1.0)
    assert dst.name == "d-k1-clip"
    ep, a = _actions(dst)
    assert a[ep == 0].tolist() == [2, 2, 3, 4, 4] and a[ep == 1].tolist() == [101, 102, 102, 102]
    assert clipped[JOINTS[0]] == {"frames": 3, "pct": 33.33}  # 시프트 뒤 1(2 미만) 1개 + 103(102 초과) 2개, 9프레임 중
    info = json.loads((dst / "meta" / "collect.json").read_text(encoding="utf-8"))
    assert [d["op"] for d in info["derived"]] == ["shift", "clip"]


@pytest.mark.slow
def test_step_clock_records_on_sim_time(tmp_path):
    """13.10.2: --clock step 수집은 기록기가 시계의 every(1/25초)로 돈다(스레드 없음). 프레임 수 ≈ 시뮬레이션 길이 × 25."""
    import json

    from so101_studio.arm import mujoco_bench as bench

    out = bench.run("collect_pick2", seeds=1, out_root=tmp_path, record="st", clock="step")
    row = json.loads((out / "results.jsonl").read_text(encoding="utf-8"))
    assert row["outcome"] == "placed_ok" and not row["realtime_lag"]
    assert 100 < row["recorded_frames"] <= row["sim_s"] * 25 + 1  # start_pose로 옮기는 동안은 기록하지 않는다
    info = json.loads((tmp_path / "runs" / "datasets" / "st" / "meta" / "collect.json").read_text(encoding="utf-8"))
    assert info["start_pose"] == bench.load_suite("collect_pick2")["start_pose"]  # 정책 실행이 같은 자세에서 시작하게(결정 0071)


@pytest.fixture(autouse=True)
def _keep_real_runs(monkeypatch):
    """시험이 만드는 실행 기록이 진짜 runs/의 보관 정리를 부르지 않게 한다(2026-11-19 test_cli_status_mujoco가 runs/train 체크포인트를 지웠다)."""
    monkeypatch.setattr("so101_studio.core.logging.prune_runs", lambda *a, **k: [])
