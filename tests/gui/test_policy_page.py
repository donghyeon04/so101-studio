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
    (tmp_path / "run1" / "checkpoints" / "last" / "pretrained_model").mkdir(parents=True)
    monkeypatch.setattr(policy_page, "TRAIN_ROOT", tmp_path)
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
