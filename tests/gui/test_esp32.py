"""ESP32 센서 보드 페이지 (REQ-028). 가상 ESP32로 연결·hello·sync와 화면 값을 본다."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.pages import esp32


def test_sim_board_connects_and_logs_both_directions():
    board = esp32.Board()
    board.connect(esp32.SIM_PORT, 921600)
    try:
        assert board.error is None and board.connected
        assert board.hello["fw"] and board.hello["k"] == ["r", "a"]
        assert board.sync_error_ms() is not None and board.sync_error_ms() < 2.0
        ways = {(way, text.split('"')[3]) for _, way, text in board.lines}  # 첫 키의 값: cmd 또는 t
        assert {("→", "hello"), ("←", "hello"), ("→", "sync"), ("←", "sync")} <= ways
    finally:
        board.disconnect()
    assert not board.connected


def test_bad_port_shows_reason_and_stays_disconnected():
    board = esp32.Board()
    board.connect("COM_NOT_THERE", 921600)
    assert board.error and not board.connected and board.link is None


def test_sensor_rows_show_last_values_and_servo_angle():
    board = esp32.Board()
    board.hello = {"t": "hello", "fw": "0.2", "servo": "pwm", "k": ["r", "a"]}
    board.last = {"r": {"t": "r", "a": 86, "d": None, "t_ns": time.perf_counter_ns()}}
    rows = esp32.sensor_rows(board)
    assert [(r["name"], r["value"]) for r in rows] == [("레이더", "응답 없음"), ("팔 센서", "측정 전"), ("서보 (pwm)", "86°")]


def test_page_shows_disconnected_state(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/esp32")
            await user.should_see(marker="esp-state", content="연결 안 됨")
            await user.should_see(marker="esp-fw", content="펌웨어 모름")

    asyncio.run(go())
