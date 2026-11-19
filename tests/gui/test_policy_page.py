"""정책 실행 페이지 1차 (gui-spec 12.10 ③): render가 없으면 안내만, 있으면 가짜 정책으로 [시작] → [정지]."""

import asyncio
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.pages import policy as policy_page
from so101_studio.service import policy_runner


class HomePolicy:
    def __init__(self, home):
        self.home = home

    def select_action(self, obs):
        return list(self.home)

    def reset(self):
        pass


def open_policy(backend, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/")
            with user.scope(marker="nav"):
                user.find("정책 실행").click()
            await check(user)

    asyncio.run(go())


def test_without_render_shows_text_and_no_buttons(backend):
    async def check(user):
        await user.should_see(marker="policy-unavailable")
        await user.should_not_see(marker="policy-start")

    open_policy(backend, check)


def test_start_then_stop_with_a_fake_policy(backend, monkeypatch, tmp_path):
    driver = backend.sim.driver
    monkeypatch.setattr(driver, "render", lambda camera, width=320, height=240: np.zeros((height, width, 3), np.uint8), raising=False)
    monkeypatch.setattr(policy_runner, "load_act", lambda path: HomePolicy(backend.sim.cfg.poses["home"]))
    monkeypatch.setattr(policy_runner, "JUDGE_DELAY_S", 0.0)
    events = []
    monkeypatch.setattr(backend.sim, "log_event", lambda kind, data=None, level="INFO", code=None: events.append(code))
    ckpt = tmp_path / "run1" / "checkpoints" / "last" / "pretrained_model"
    ckpt.mkdir(parents=True)
    _with_start_pose(ckpt, tmp_path / "ds", [10.0, -30.0, 30.0, 75.0, 0.0, 50.0])
    monkeypatch.setattr(policy_page, "TRAIN_ROOT", tmp_path)
    moves, move = [], backend.sim.move_joints
    monkeypatch.setattr(backend.sim, "move_joints", lambda goal, *a, **k: (moves.append(goal), move(goal, *a, **k))[1])
    backend.policy = None

    async def check(user):
        await user.should_see(marker="camera-top")
        user.find(marker="policy-start").click()
        for _ in range(50):
            if backend.policy is not None and backend.policy._thread is not None:
                break
            await asyncio.sleep(0.05)
        await asyncio.sleep(0.3)
        user.find(marker="policy-stop").click()
        for _ in range(60):
            if backend.policy.result is not None:
                break
            await asyncio.sleep(0.05)
        await user.should_see("끝난 이유: stop")
        assert backend.policy.max_s == policy_page.MAX_S
        assert moves and moves[0].shoulder_pan == 10.0 and moves[0].wrist_flex == 75.0  # 결정 0071: 학습 때 시작 자세로 먼저 옮겼다

    try:
        open_policy(backend, check)
    finally:
        if backend.policy is not None:
            backend.policy.stop()
        backend.policy = None
    assert [c for c in events if c and c.startswith("ARM-POLICY")] == ["ARM-POLICY-START", "ARM-POLICY-STOP"]


def test_checkpoint_list_is_newest_first(tmp_path):
    for name, t in (("a", 100), ("b", 200)):
        p = tmp_path / name / "checkpoints" / "last" / "pretrained_model"
        p.mkdir(parents=True)
        os.utime(p, (t, t))
    assert [Path(s).parts[-4] for s in policy_page.checkpoints(tmp_path)] == ["b", "a"]


def test_stop_while_loading_cancels_and_second_click_makes_no_second_runner(backend, monkeypatch, tmp_path):
    import time

    monkeypatch.setattr(backend.sim.driver, "render", lambda camera, width=320, height=240: np.zeros((height, width, 3), np.uint8), raising=False)
    made = []

    def slow_load(path):
        made.append(path)
        time.sleep(0.6)
        return HomePolicy(backend.sim.cfg.poses["home"])

    monkeypatch.setattr(policy_runner, "load_act", slow_load)
    (tmp_path / "run1" / "checkpoints" / "last" / "pretrained_model").mkdir(parents=True)
    monkeypatch.setattr(policy_page, "TRAIN_ROOT", tmp_path)
    backend.policy = None

    async def check(user):
        await user.should_see(marker="camera-top")
        user.find(marker="policy-start").click()
        await asyncio.sleep(0.1)
        user.find(marker="policy-start").click()  # 불러오는 중 두 번째 누름
        await user.should_see("불러오는 중")
        user.find(marker="policy-stop").click()
        for _ in range(40):
            if not backend.policy_loading:
                break
            await asyncio.sleep(0.05)
        await asyncio.sleep(0.3)
        assert len(made) == 1
        assert backend.policy._thread is None or not backend.policy._thread.is_alive()

    try:
        open_policy(backend, check)
    finally:
        if backend.policy is not None:
            backend.policy.stop()
        backend.policy = None


def _with_start_pose(ckpt, ds, pose):
    """체크포인트 → train_config.json의 dataset.root → meta/collect.json의 start_pose(mujoco_bench가 쓰는 모양)."""
    import json

    (ds / "meta").mkdir(parents=True)
    (ds / "meta" / "collect.json").write_text(json.dumps({"suite": "collect_pick2", "start_pose": pose}), encoding="utf-8")
    (ckpt / "train_config.json").write_text(json.dumps({"dataset": {"root": str(ds)}}), encoding="utf-8")


def test_start_pose_comes_from_the_training_dataset(tmp_path):
    """없거나 깨졌으면 None(지금 자리에서 시작), 있으면 관절 6개."""
    ckpt = tmp_path / "ckpt"
    ckpt.mkdir()
    assert policy_page.start_pose_of(ckpt) is None
    _with_start_pose(ckpt, tmp_path / "ds", [0, -30, 30, 75, 0, 50])
    assert policy_page.start_pose_of(ckpt) == [0.0, -30.0, 30.0, 75.0, 0.0, 50.0]
    (ckpt / "train_config.json").write_text("{", encoding="utf-8")
    assert policy_page.start_pose_of(ckpt) is None


def test_training_preset_is_applied_and_the_old_values_come_back(backend):
    """GUI 기본 속도(30%)면 정책 목표가 잘려 팔이 뒤처진다(11-20): 수집 프리셋을 적용하고 바꾸기 전 값을 돌려준다. 서비스에 없는 키(gripper)는 건너뛴다."""
    svc = backend.sim
    old = {k: v["value"] for k, v in svc.get_params().items()}
    info = {"preset_values": {"gripper": "parallel", "speed_pct_default": 100, "joint_v_max_deg_s": 150, "servo_acceleration": 50}}
    try:
        before = policy_page.apply_training_preset(svc, info)
        now = svc.get_params()
        assert now["speed_pct"]["value"] == 100 and now["joint_v_max_deg_s"]["value"] == 150 and now["servo_acceleration"]["value"] == 50
        assert set(before) <= {"speed_pct", "joint_v_max_deg_s", "servo_acceleration"} and "gripper" not in before
        assert policy_page.apply_training_preset(svc, info) == {}  # 이미 같은 값이면 바꿀 것이 없다
        svc.set_params(before)
        assert {k: v["value"] for k, v in svc.get_params().items()} == old
        assert policy_page.apply_training_preset(svc, {}) == {}  # collect.json이 없는 체크포인트
    finally:
        svc.set_params({k: old[k] for k in ("speed_pct", "joint_v_max_deg_s", "servo_acceleration")})


def test_gripper_mismatch_is_reported_before_start(backend):
    """학습 손과 시뮬레이션 손이 다르면 시작 전에 알린다(11-20: 병렬로 배운 정책이 단동식 손에서 0/6). 모르면 막지 않는다."""
    svc = backend.sim
    have = svc.cfg.gripper
    other = "parallel" if have != "parallel" else "single_jaw"
    why = policy_page.gripper_mismatch(svc, {"preset_values": {"gripper": other}})
    assert why and policy_page.GRIPPER_TEXT[other] in why and "다시 켠" in why
    assert policy_page.gripper_mismatch(svc, {"preset_values": {"gripper": have}}) is None
    assert policy_page.gripper_mismatch(svc, {}) is None


def test_checkpoint_list_includes_sibling_folders(tmp_path, monkeypatch):
    """학습은 MuJoCo 폴더, GUI는 다른 폴더에서 켠다: 옆 폴더의 runs/train도 목록에 넣는다(같은 폴더는 한 번만)."""
    for folder, name, t in (("gui", "a", 100), ("sim", "b", 200)):
        p = tmp_path / folder / "runs" / "train" / name / "checkpoints" / "last" / "pretrained_model"
        p.mkdir(parents=True)
        os.utime(p, (t, t))
    monkeypatch.chdir(tmp_path / "gui")
    root = Path("runs/train")
    assert [Path(s).parts[-4] for s in policy_page.checkpoints(root)] == ["a"]
    assert [Path(s).parts[-4] for s in policy_page.checkpoints(root, siblings=True)] == ["b", "a"]
