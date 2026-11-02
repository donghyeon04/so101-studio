"""공통 틀과 페이지 이동 (gui-spec G1). 브라우저 없이 NiceGUI user_simulation으로 돈다."""

import argparse
import asyncio

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app

ALL_PAGES = [page for _, pages in gui_app.PAGES for page in pages]


def in_app(check) -> None:
    async def go():
        async with user_simulation(gui_app.root) as user:
            await user.open("/")
            await check(user)

    asyncio.run(go())


def test_thirteen_pages_in_five_groups():
    assert len(gui_app.PAGES) == 5
    assert len(ALL_PAGES) == 13
    assert len({path for path, _, _ in ALL_PAGES}) == 13


@pytest.mark.parametrize("path,title,task", ALL_PAGES)
def test_menu_click_changes_page(path, title, task):
    async def check(user):
        with user.scope(marker="nav"):
            user.find(title).click()
        await user.should_see(marker="page-title", content=title)
        await user.should_see(f"{task}에서 만든다.")

    in_app(check)


def test_frame_stays_on_every_page():
    async def check(user):
        for _, title, _ in ALL_PAGES:
            with user.scope(marker="nav"):
                user.find(title).click()
            for text in ("SO-101 Studio", "시뮬레이션", "조작권", "손끝 좌표", "기록 폴더", "E-STOP"):
                await user.should_see(text)

    in_app(check)


def test_estop_tells_user_nothing_is_connected():
    async def check(user):
        user.find(marker="estop").click()
        await user.should_see("연결된 팔이 없다")

    in_app(check)


def test_main_binds_localhost_and_needs_sim(monkeypatch):
    calls = []
    monkeypatch.setattr(gui_app.ui, "run", lambda *a, **k: calls.append(k))
    gui_app.main(argparse.Namespace(sim=True, port=9000))
    gui_app.main(argparse.Namespace(sim=True))  # so101 gui에는 port가 없다
    assert [(c["host"], c["port"]) for c in calls] == [("127.0.0.1", 9000), ("127.0.0.1", gui_app.DEFAULT_PORT)]
    with pytest.raises(SystemExit):
        gui_app.main(argparse.Namespace(sim=False))
