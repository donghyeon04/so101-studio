"""공통 틀과 페이지 이동 (gui-spec G1·G2). 브라우저 없이 NiceGUI user_simulation으로 돈다."""

import argparse
import asyncio

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app

ALL_PAGES = [page for _, pages in gui_app.PAGES for page in pages]


def in_app(backend, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/")
            await check(user)

    asyncio.run(go())


def test_thirteen_pages_in_five_groups():
    assert len(gui_app.PAGES) == 5
    assert len(ALL_PAGES) == 13
    assert len({path for path, _, _ in ALL_PAGES}) == 13


@pytest.mark.parametrize("path,title,task", ALL_PAGES)
def test_menu_click_changes_page(backend, path, title, task):
    async def check(user):
        with user.scope(marker="nav"):
            user.find(title).click()
        await user.should_see(marker="page-title", content=title)
        if path not in gui_app.BUILDERS:
            await user.should_see(f"{task}에서 만든다.")

    in_app(backend, check)


def test_frame_stays_on_every_page(backend):
    async def check(user):
        for _, title, _ in ALL_PAGES:
            with user.scope(marker="nav"):
                user.find(title).click()
            for text in ("SO-101 Studio", "시뮬레이션", "조작권", "손끝 좌표", "기록 폴더", "E-STOP"):
                await user.should_see(text)

    in_app(backend, check)


def test_status_bars_show_live_values(backend):
    async def check(user):
        await user.should_see("시뮬레이션만")
        await user.should_see(backend.readout()["손끝 좌표"])
        await user.should_see(backend.readout()["토크"])

    in_app(backend, check)


def test_watchdog_state_has_icon_and_text(backend):
    async def check(user):
        await user.should_see("감시")
        await user.should_see(marker="watch-icon")
        await user.should_see("없음")

    in_app(backend, check)


def test_first_browser_owns_control(backend):
    async def check(user):
        await user.should_see("이 브라우저")
        assert backend.owner is not None

    in_app(backend, check)


def test_http_request_without_socket_does_not_take_control(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            assert (await user.http_client.get("/")).status_code == 200
            assert backend.owner is None

    asyncio.run(go())


def test_second_browser_is_view_only_until_it_takes_free_control(backend):
    backend.owner = "다른 브라우저"

    async def check(user):
        await user.should_see("보기 전용")
        await user.should_not_see("조작권 받기")
        backend.owner = None
        await user.should_see("조작권 받기")
        user.find(marker="take").click()
        await user.should_see("이 브라우저")
        await user.should_not_see("조작권 받기")

    in_app(backend, check)


def test_estop_stops_arm_and_shows_state(backend):
    async def check(user):
        await user.should_see(marker="page-state", content="대기")
        user.find(marker="estop").click()
        await user.should_see("비상 정지했습니다")
        await user.should_see(marker="page-state", content="정지")
        assert backend.arm.ctl.state == "stopped"

    in_app(backend, check)


def test_view_only_browser_can_still_estop(backend):
    backend.owner = "다른 브라우저"

    async def check(user):
        user.find(marker="estop").click()
        await user.should_see("비상 정지했습니다")
        assert backend.arm.ctl.state == "stopped"

    in_app(backend, check)


def test_main_binds_localhost_and_connects_the_real_arm_without_sim(monkeypatch):
    class FakeBackend:
        def __init__(self, speed=None, link=None, viser_port=None, arm_name=None, **real):
            self.speed, self.real, self.link = speed, real, link
            self.run_dir = "runs/시험"

        def close(self):
            pass

    class FakeLink:
        def __init__(self, sim):
            self.sim = sim

        def start(self, run_dir=None):
            self.run_dir = run_dir

        def stop(self):
            pass

    calls = []
    monkeypatch.setattr(gui_app, "Backend", FakeBackend)
    monkeypatch.setattr(gui_app, "WatchdogLink", FakeLink)
    monkeypatch.setattr(gui_app.app, "on_startup", lambda fn: None)
    monkeypatch.setattr(gui_app.app, "on_shutdown", lambda fn: None)
    monkeypatch.setattr(gui_app, "make_root", lambda backend: backend)
    monkeypatch.setattr(gui_app.ui, "run", lambda root, **k: calls.append((root, k)))
    gui_app.main(argparse.Namespace(sim=True, port=9000, speed=20))
    gui_app.main(argparse.Namespace(sim=True))  # so101 gui에는 port가 없다
    assert [(k["host"], k["port"]) for _, k in calls] == [("127.0.0.1", 9000), ("127.0.0.1", gui_app.DEFAULT_PORT)]
    assert calls[0][0].speed == 20
    assert calls[0][0].real == {} and calls[0][0].link.sim is True
    assert calls[0][0].link.run_dir == "runs/시험"  # 감시 기록을 실행 폴더에 남긴다

    # 실제 팔: 포트는 보드 시리얼로 고르고, 감시 프로세스는 기록만 하는 --sim이 아니어야 한다
    board = {"device": "COM7", "vid": 0x1A86, "pid": 0x55D3, "serial": "5B8E114816", "description": ""}
    monkeypatch.setattr(gui_app, "list_serial_ports", lambda: [board])
    monkeypatch.setattr(gui_app, "make_driver", lambda sim, cfg: ("driver", sim))
    monkeypatch.setattr(gui_app, "doctor", lambda **kw: {"probed": kw})  # 시험이 실제 포트를 열면 안 된다
    gui_app.main(argparse.Namespace(sim=False))
    real = calls[-1][0]
    assert real.real == {
        "real_driver": ("driver", False), "serials": ["5B8E114816"], "port": "COM7",
        "first_report": {"probed": {"probe": True, "port": "COM7", "arm": None}},
    }
    assert real.link.sim is False

    def broken(**_kw):
        raise OSError("port busy")

    monkeypatch.setattr(gui_app, "doctor", broken)  # 점검이 실패해도 앱은 뜬다
    gui_app.main(argparse.Namespace(sim=False))
    assert calls[-1][0].real["first_report"] is None

    monkeypatch.setattr(gui_app, "list_serial_ports", lambda: [{**board, "serial": "OTHER"}])  # 다른 보드로 추측해 연결하지 않는다
    with pytest.raises(SystemExit, match="안 보인다"):
        gui_app.main(argparse.Namespace(sim=False))
