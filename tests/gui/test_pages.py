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


def test_thirteen_pages_in_six_groups():
    assert len(gui_app.PAGES) == 6  # 맨 위 "안내"(시작 화면) + 다섯 그룹 (gui-spec 11절)
    assert len(ALL_PAGES) == 13  # 카메라는 장치를 달 때까지 메뉴에 없다 (결정 0044)
    assert len({path for path, _, _ in ALL_PAGES}) == 13


@pytest.mark.parametrize("path,title,task", ALL_PAGES)
def test_menu_click_changes_page(backend, path, title, task):
    async def check(user):
        with user.scope(marker="nav"):
            user.find(title).click()
        await user.should_see(marker="page-title", content=title)
        if path not in gui_app.BUILDERS:
            await user.should_see(marker="page-pending")

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


def test_brand_goes_to_start_page(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/logs")
            await user.should_see(marker="page-title", content="로그")
            user.find(marker="brand").click()
            await user.should_see(marker="page-title", content="시작")

    asyncio.run(go())


def test_every_page_has_a_one_line_description():
    assert {path for path, _, _ in ALL_PAGES} <= set(gui_app.DESCRIPTIONS)


def test_page_shows_its_description(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/input")
            await user.should_see(marker="page-desc", content="센서 대신 사람이")

    asyncio.run(go())


def test_start_page_shows_mode_steps_and_guide(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/start")
            await user.should_see(marker="start-mode-title", content="시뮬레이션")
            await user.should_see("직접 조작에서 팔 움직여 보기")
            await user.should_see("센서 대신 사람이")  # 페이지 안내도 같은 설명 문구
            user.find(marker="start-step-/manual").click()
            await user.should_see(marker="page-title", content="직접 조작")

    asyncio.run(go())


def test_top_bar_gripper_speed_applies_now(backend):
    """결정 0042: 상단 막대가 gripper_speed_pct를 바로 바꾼다."""
    before = backend.arm.cfg.gripper_speed_pct

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/start")
            for _ in range(160):
                if backend.owner is not None:
                    break
                await asyncio.sleep(0.05)
            next(iter(user.find(marker="top-grip-speed").elements)).value = 40
            for _ in range(60):
                if backend.arm.cfg.gripper_speed_pct == 40:
                    break
                await asyncio.sleep(0.05)

    try:
        asyncio.run(go())
        assert backend.arm.cfg.gripper_speed_pct == 40
    finally:
        backend.set_params({"gripper_speed_pct": before})


def test_top_bar_run_mode_and_doctor(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/start")
            select = next(iter(user.find(marker="top-run-mode").elements))
            assert select.value == backend.mode == "sim"
            select.set_value("real")  # 시뮬레이션 앱에는 실제 팔이 없어 바뀌지 않는다
            await asyncio.sleep(0.3)
            assert backend.mode == "sim"
            user.find(marker="top-doctor").click()
            await user.should_see(marker="doctor-dialog")
            await user.should_see("점검 (읽기 전용)")

    asyncio.run(go())
