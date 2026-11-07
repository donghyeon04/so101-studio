"""운영자 화면 /operator (gui-spec 12.4): 틀 없이 크게, 보기 전용은 버튼·E-STOP 숨김, 작업 상태 그대로 단계 표시, 시작하면 물체를 옮긴다."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.core.types import DetectedObject, TaskEvent
from so101_studio.gui import app as gui_app
from so101_studio.gui.pages.operator import object_lines


@pytest.fixture
def clean(backend):
    backend.reset_objects()
    backend.state.events.clear()
    backend.state.task_state, backend.state.placed = "IDLE", 0
    yield backend
    backend.stop_task()
    _wait(lambda: not backend.task_running(), 15.0)
    backend.arm.resume()
    backend.reset_objects()
    backend.state.task_state = "IDLE"


def _wait(check, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.05)
    return False


async def _until(check, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


def in_app(backend, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/operator")
            await check(user)

    asyncio.run(go())


def test_object_lines_mark_placed_current_and_waiting():
    objs = [DetectedObject(i, 0, 0, 0, 30, 30, 1.0, "manual") for i in ("a", "b", "c")]
    events = [TaskEvent(0.0, "PLACE", "a", "ok", {})]
    assert object_lines(objs[1:], events, "b", "CHECK") == [("a", "✓ 놓음"), ("b", "확인 중"), ("c", "대기")]
    old_run = [TaskEvent(0.5, "PLACE", "z", "ok"), TaskEvent(1.0, "PREPARE"), TaskEvent(2.0, "PLACE", "a", "ok"), TaskEvent(3.0, "PREPARE")]
    assert object_lines([], old_run, None, "SCAN", since=1.0) == [("a", "✓ 놓음")]  # 지난 실행의 z는 빼고, 물체마다 오는 PREPARE에는 끊기지 않는다


def test_view_only_hides_buttons_and_stage_follows_task_state(clean):
    """조작권: 연결되면 받고, 다른 화면이 가지면 보기 전용(버튼 숨김, E-STOP은 보임), 비면 [조작권 받기]."""
    async def check(user):
        await user.should_see(marker="operator")
        await user.should_not_see(marker="nav")  # 메뉴·상태줄 틀 없이 그린다
        assert await _until(lambda: "시뮬레이션 · 인식: 수동 입력" in _one(user, "operator-title").text)
        await user.should_see(marker="operator-buttons", retries=30)  # 본 화면처럼 연결되면 비어 있는 조작권을 받는다
        await user.should_see(marker="operator-estop", retries=30)
        clean.owner = "someone-else"
        await user.should_not_see(marker="operator-buttons", retries=30)  # 다른 화면이 조작하면 보기 전용
        await user.should_see(marker="operator-view-only", retries=30)
        await user.should_see(marker="operator-estop", retries=30)  # E-STOP은 보기 전용에도 보인다(결정 0047)
        clean.owner = None
        await user.should_see(marker="operator-take", retries=30)
        clean.state.task_state = "CHECK"  # 초음파 확인 단계도 그대로 보인다
        assert await _until(lambda: "operator-chip-active" in _one(user, "operator-chip-CHECK").classes and _one(user, "operator-now").text == "확인")

    in_app(clean, check)


@pytest.mark.slow
def test_start_moves_the_object_and_the_list_shows_it_placed(clean):
    clean.set_speed(100)
    keep = clean.objects()[0].id
    for o in clean.objects()[1:]:
        clean.remove_object(o.id)

    async def check(user):
        await user.should_see(marker="operator-buttons", retries=30)
        await user.should_see(marker="operator-estop", retries=30)
        assert await _until(lambda: _one(user, "operator-start").enabled)
        user.find(marker="operator-start").click()
        assert await _until(lambda: clean.state.placed == 1 and not clean.task_running(), 90.0)
        await user.should_see(f"{keep} ✓ 놓음", retries=30)

    in_app(clean, check)


def test_remote_browser_never_gets_control_but_can_estop(clean, monkeypatch):
    """다른 기기(원격)는 연결해도 조작권을 받지 못하고 [조작권 받기]도 없다. E-STOP은 보이고 누르면 세운다(결정 0047, gui-spec 12.6)."""
    from so101_studio.gui import app as app_module
    from so101_studio.gui.pages import operator as operator_module

    monkeypatch.setattr(operator_module, "is_remote", lambda ip: True)
    monkeypatch.setattr(app_module, "is_remote", lambda ip: True)
    assert clean.claim("remote-client", remote=True) is False and clean.owner is None
    stopped = []
    monkeypatch.setattr(clean, "estop", lambda: stopped.append(1))

    async def check(user):
        await user.should_see(marker="operator-view-only", retries=30)
        assert "다른 기기" in _one(user, "operator-view-only").text
        await user.should_not_see(marker="operator-buttons", retries=30)
        await user.should_not_see(marker="operator-take", retries=30)
        assert clean.owner is None  # 연결해도 조작권을 가져가지 않았다
        user.find(marker="operator-estop").click()
        assert await _until(lambda: stopped == [1])

    in_app(clean, check)


def test_share_badge_shows_on_the_local_screen_only_when_sharing(clean, monkeypatch):
    """--share로 켜면 본 화면 상단에 '공유 중' 배지(누르면 주소). 아니면 없다(gui-spec 12.6)."""

    async def go(sharing):
        monkeypatch.setattr(clean, "share", sharing)
        async with user_simulation(gui_app.make_root(clean)) as user:
            await user.open("/start")
            await user.should_see(marker="page-title")
            if sharing:
                await user.should_see(marker="share-badge")
            else:
                await user.should_not_see(marker="share-badge")

    asyncio.run(go(False))
    asyncio.run(go(True))
