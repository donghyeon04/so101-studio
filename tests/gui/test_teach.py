"""교시 (gui-spec G5). 시뮬레이션에서 wave를 전체·구간별 배속으로 재생한다."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.pages.teach import chart_options, expected_seconds

NAME = "wave_safe1"  # 안전 범위 안의 2초짜리 기록
OUTSIDE = "wave"  # 손으로 작업 영역 밖까지 움직인 기록


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


async def _until(check, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def test_sequence_list_has_wave(backend):
    assert NAME in backend.sequences()


def test_sequence_data_is_limited_and_draws_trajectory(backend):
    data = backend.sequence(NAME)
    assert 2 <= len(data["t"]) <= 2 * 300
    assert len(data["t"]) == len(data["q"])
    assert data["info"]["segments"]
    handles = backend.scene.server.scene._handle_from_node_name
    assert [n for n in handles if n.startswith("/marks/trajectory/")]
    backend.scene.clear_target()
    assert not [n for n in handles if n.startswith("/marks/trajectory/")]


def test_expected_seconds_applies_factors():
    segments = [{"index": 0, "start_s": 0, "end_s": 4}, {"index": 1, "start_s": 4, "end_s": 6}]
    assert expected_seconds(segments, {}, 1.0) == 6
    assert expected_seconds(segments, {0: 2.0}, 1.0) == 4
    assert expected_seconds(segments, {0: 2.0, 1: 0.5}, 2.0) == 3


def test_chart_has_six_joint_series_on_one_axis(backend):
    options = chart_options(backend.sequence(OUTSIDE))
    assert len(options["series"]) == 6
    assert isinstance(options["yAxis"], dict)
    assert len({s["color"] for s in options["series"]}) == 6


def test_replay_follows_total_and_segment_speed(backend):
    segments = backend.arm.record_info(NAME)["segments"]
    plain = backend.arm.replay(NAME, 1.0).duration_s
    assert plain == pytest.approx(expected_seconds(segments, {}, 1.0), abs=0.2)
    assert backend.arm.replay(NAME, 0.5).duration_s == pytest.approx(2 * plain, abs=0.3)
    index = max(segments, key=lambda s: s["end_s"] - s["start_s"])["index"]
    slowed = backend.arm.replay(NAME, 1.0, False, {index: 0.5}).duration_s
    assert slowed == pytest.approx(expected_seconds(segments, {index: 0.5}, 1.0), abs=0.3)


def test_replay_too_fast_gives_reason_and_stays_put(backend):
    before = backend.arm.status().joints
    assert "한계" in backend.replay(OUTSIDE, 3.0)
    assert backend.arm.status().joints == before


def test_outside_record_is_refused_with_advice_then_cut_at_the_limit(backend):
    assert "한계 앞까지만" in backend.replay(OUTSIDE, 1.0)  # 첫 자세까지는 간 뒤 거부된다
    assert backend.replay(OUTSIDE, 1.0, True) is None
    assert "까지 재생했습니다" in backend.replay_cut()


def test_finished_full_replay_has_no_cut(backend):
    assert backend.replay(NAME, 1.0) is None
    assert backend.replay_cut() is None


def test_page_loads_sequence_and_plays(backend):
    async def check(user):
        select = _one(user, "sequence")
        assert NAME in select.options and OUTSIDE in select.options
        select.value = NAME
        assert await _until(lambda: "초" in _one(user, "summary").text)
        await user.should_see(marker="segment-bar")
        await user.should_see(marker="joint-chart")
        expected_before = _one(user, "expected").text
        _one(user, "replay-speed").value = 0.5
        assert await _until(lambda: _one(user, "expected").text != expected_before)
        assert await _until(lambda: _one(user, "play").enabled)
        user.find(marker="play").click()
        assert await _until(lambda: backend.arm.ctl.state == "moving", 3.0)
        user.find(marker="stop").click()
        assert await _until(lambda: backend.arm.ctl.state in ("idle", "stopped"), 3.0)

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/teach")
            await check(user)

    asyncio.run(go())


def test_record_is_off_in_simulation(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/teach")
            assert not _one(user, "record").enabled

    asyncio.run(go())


def test_view_only_browser_cannot_play(backend):
    backend.owner = "다른 브라우저"

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/teach")
            assert await _until(lambda: not _one(user, "play").enabled, 2.0)
            assert await _until(lambda: not _one(user, "replay-speed").enabled, 2.0)

    asyncio.run(go())
