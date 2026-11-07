"""서보 건강 대시보드 (결정 0043-5): 3초 이어진 주의·오류 판정, 최고값, 이벤트 횟수, 팔 화면 카드."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.core.types import ArmStatus, JointState, ToolPose
from so101_studio.gui import app as gui_app
from so101_studio.gui.health import ServoHealth, count_events, describe, level_of
from so101_studio.gui.pages.arm import chart_options, servo_rows


async def _until(check, timeout=5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


def in_app(backend, path, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await check(user)

    asyncio.run(go())


def _st(t, temp=30.0, load=5.0, volt=12.0, current_ma=500.0):
    return ArmStatus(t, JointState(0, 0, 0, 0, 0, 0), ToolPose(0, 0, 0, 0), {"elbow_flex": temp}, {"elbow_flex": load}, volt, True, "idle", current_ma=current_ma)


def test_levels_follow_limits_both_directions():
    assert [level_of("temperature", v) for v in (49, 50, 60)] == ["ok", "warn", "error"]
    assert [level_of("voltage", v) for v in (11.5, 11.4, 10.9)] == ["ok", "warn", "error"]
    assert describe("temperature:elbow_flex") == "elbow_flex 온도 ≥ 50°C"
    assert describe("voltage", "error") == "전압 < 11V"


def test_level_changes_only_after_hold_and_blips_are_ignored():
    h = ServoHealth(hold_s=3.0)
    h.update(_st(0.0, temp=55))
    h.update(_st(1.0, temp=30))  # 한 번 튄 값
    h.update(_st(5.0, temp=30))
    assert h.level("temperature:elbow_flex") == "ok" and not h.changes
    for t in (10.0, 11.0, 13.0):
        h.update(_st(t, temp=55, volt=11.2))
    assert h.level("temperature:elbow_flex") == "warn" and h.level("voltage") == "warn"
    assert h.worst() == ("warn", ["elbow_flex 온도 ≥ 50°C", "전압 < 11.5V"])
    assert h.peak["temperature:elbow_flex"] == 55 and h.peak["voltage"] == 11.2
    for t in (20.0, 23.5):
        h.update(_st(t))
    assert h.level("temperature:elbow_flex") == "ok"
    assert [c[2] for c in h.changes if c[1] == "temperature:elbow_flex"] == ["warn", "ok"]


def test_series_is_one_per_second_relative_to_now():
    h = ServoHealth()
    for i in range(30):
        h.update(_st(i * 0.1 + 100, current_ma=1500))
    s = h.series()
    assert s["t"] == [-2.0, -1.0, 0.0] and s["current"] == [1.5, 1.5, 1.5]
    opts = chart_options("power", s)
    assert [x["name"] for x in opts["series"]] == ["전압", "전류 합"] and opts["series"][1]["yAxisIndex"] == 1
    assert len(chart_options("temperature", s)["series"]) == 1


def test_count_events_does_not_double_count_comm_loss():
    events = [
        {"t": 1_000_000_000, "kind": "comm_lost", "data": {}},
        {"t": 2_000_000_000, "kind": "fault", "data": {"reason": "comm_lost"}},
        {"t": 3_000_000_000, "kind": "torque_lost", "data": {"status_bits": {"elbow_flex": 32}}},
        {"t": 4_000_000_000, "kind": "fault", "data": {"reason": "overheat:elbow_flex"}},
    ]
    c = count_events(events)
    assert c["comm_lost"][:2] == (1, 1.0) and c["torque_lost"][0] == 1 and c["fault"] == (1, 4.0, "overheat:elbow_flex")


def test_servo_rows_show_peak_and_held_level():
    h = ServoHealth(hold_s=0.0)
    h.update(_st(0.0, temp=58))
    rows = servo_rows(_st(1.0, temp=40), h)
    elbow = next(r for r in rows if r["joint"].startswith("3. "))
    assert elbow["temperature"] == "40 (58)" and elbow["state"] == "주의: 온도"


def test_arm_page_shows_health_card(backend, monkeypatch):
    async def check(user):
        await user.should_see(marker="health-card")
        assert await _until(lambda: "주의 기준 안" in _one(user, "health-summary").text)
        assert await _until(lambda: "통신 끊김 0회" in _one(user, "health-events").text)
        monkeypatch.setattr(backend.health, "worst", lambda: ("error", ["elbow_flex 온도 ≥ 60°C"]))
        assert await _until(lambda: "elbow_flex 온도 ≥ 60°C" in _one(user, "health-summary").text, timeout=4.0)

    in_app(backend, "/arm", check)
