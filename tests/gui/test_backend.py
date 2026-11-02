"""backend와 state 단위 테스트 (gui-spec G2)."""

import threading
import time
from dataclasses import replace

import pytest

pytest.importorskip("nicegui")

from so101_studio.core.types import ArmStatus, JointState, ToolPose
from so101_studio.gui.state import hottest_text, page_state


def _status(state="idle", temperature_c=None) -> ArmStatus:
    return ArmStatus(0.0, JointState(0, 0, 0, 0, 0, 0), ToolPose(0, 0, 0, 0), temperature_c or {}, {}, 12.0, True, state)


@pytest.mark.parametrize(
    "state,text",
    [("idle", "대기"), ("moving", "실행 중"), ("teleop", "실행 중"), ("teach", "실행 중"), ("stopped", "정지"), ("fault", "오류"), ("?", "오류")],
)
def test_page_state_text(state, text):
    assert page_state(_status(state)) == text


def test_page_state_before_first_status():
    assert page_state(None) == "대기"


def test_hottest_text():
    assert hottest_text(_status(temperature_c={"a": 31.2, "b": 44.6})) == "45 °C"
    assert hottest_text(_status()) == "—"


def _wait_for(check, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.02)
    return False


def test_status_stream_updates_readout_when_arm_moves(backend):
    assert _wait_for(lambda: backend.state.status is not None)
    before = backend.readout()["손끝 좌표"]
    q = backend.state.status.joints
    goal = replace(q, shoulder_pan=q.shoulder_pan + (10 if q.shoulder_pan < 0 else -10))
    assert backend.arm.move_joints(goal, 100).status == "ok"
    assert _wait_for(lambda: backend.readout()["손끝 좌표"] != before)


def test_readout_has_every_live_field(backend):
    assert _wait_for(lambda: backend.state.status is not None)
    values = backend.readout()
    assert values["실행 방식"] == "시뮬레이션만"
    assert values["전압"].endswith(" V")
    assert values["토크"] in ("켜짐", "꺼짐")
    assert values["안전 모드"] == "보통"
    assert values["기록 폴더"] == "—"  # 이 테스트는 기록을 끄고 돈다


def test_first_client_owns_and_second_does_not(backend):
    assert backend.claim("A")
    assert not backend.claim("B")
    assert backend.owner == "A"


def test_owner_disconnect_stops_arm_and_frees_control(backend, monkeypatch):
    calls = []
    monkeypatch.setattr(backend.arm, "stop", lambda: calls.append(1))
    backend.claim("A")
    backend.disconnected("B")
    assert (backend.owner, calls) == ("A", [])
    backend.disconnected("A")
    assert (backend.owner, calls) == (None, [1])
    assert backend.claim("B")


def test_disconnect_during_motion_stops_it_but_keeps_torque(backend):
    q = backend.state.status.joints if backend.state.status else backend.arm.status().joints
    goal = replace(q, shoulder_pan=q.shoulder_pan + (40 if q.shoulder_pan < 0 else -40))
    backend.claim("A")
    results = []
    moving = threading.Thread(target=lambda: results.append(backend.arm.move_joints(goal, 30)))
    moving.start()
    assert _wait_for(lambda: backend.arm.ctl.state == "moving")
    backend.disconnected("A")
    moving.join(timeout=5)
    assert results and results[0].status == "stopped"
    assert backend.arm.driver.torque_enabled()
