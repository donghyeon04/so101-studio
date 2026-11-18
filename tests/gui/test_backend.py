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


def test_close_during_motion_releases_the_waiting_thread():
    from so101_studio.gui.backend import Backend

    backend = Backend(log_root=None)
    q = backend.arm.status().joints
    goal = replace(q, shoulder_pan=q.shoulder_pan + (40 if q.shoulder_pan < 0 else -40))
    results = []
    moving = threading.Thread(target=lambda: results.append(backend.arm.move_joints(goal, 30)), daemon=True)
    moving.start()
    assert _wait_for(lambda: backend.arm.ctl.state == "moving")
    backend.close()
    moving.join(timeout=5)
    assert not moving.is_alive()  # 남아 있으면 앱을 꺼도 프로세스가 끝나지 않는다
    assert results and results[0].status == "stopped"


def test_manual_objects_are_told_to_the_arm_as_known_obstacles(backend):
    """놓기 벌림이 수동 입력 물체에 부딪히지 않게 팔(Skills.known_objects)에 알린다. 지금 집는 물체는 뺀다 (REQ-050)."""
    backend.reset_objects()
    objs = backend.objects()
    assert len(objs) >= 2
    first = objs[0]
    backend._tell_known_objects(backend.arm, near=(first.x_mm, first.y_mm))
    known = backend.arm.skills.known_objects
    assert len(known) == len(objs) - 1 and all(abs(x - first.x_mm) + abs(y - first.y_mm) > 1 for x, y, _ in known)
    assert all(r > 0 for _, _, r in known)
    assert (first.x_mm, first.y_mm) not in backend.arm.skills.known_height  # 집는 물체 높이는 운반 높이에 넣지 않는다(#352)
    backend._tell_known_objects(backend.arm)
    assert len(backend.arm.skills.known_objects) == len(objs)
    assert backend.arm.skills.known_height == {(o.x_mm, o.y_mm): o.height_mm for o in objs}


def test_object_edits_undo_in_order_and_a_run_clears_them(backend):
    """수동 입력 물체 되돌리기(gui-spec 12.9 나): 더하기·옮기기·지우기를 차례로 되돌린다. 0.6초 안의 옮기기 연타는 한 번, 작업을 시작하면 기록을 비운다."""
    backend.reset_objects()
    start = backend.objects()
    backend._object_undo.clear()
    oid = backend.add_object(200, 150, 0, 30, 60, 40)
    time.sleep(0.7)
    backend.nudge_object(oid, 5, 0)
    backend.nudge_object(oid, 5, 0)  # 연타: 한 번으로
    time.sleep(0.7)
    backend.remove_object(oid)
    assert backend.undo_objects() and any(o.id == oid for o in backend.objects())
    assert backend.undo_objects() and next(o for o in backend.objects() if o.id == oid).x_mm == 200
    assert backend.undo_objects() and backend.objects() == start
    assert not backend.undo_objects()
    backend.add_object(220, 150)
    assert backend.start_task() is None
    assert not backend.undo_objects()  # 실행한 작업은 되돌리지 않는다
    backend.stop_all()


def test_object_reachable_gives_radar_candidates_the_same_margin_as_the_task(backend, monkeypatch):
    """REQ-065: 확인 기능이 있는 인식 방식(초음파)의 후보는 집기 범위 + 여유로 '닿음'을 본다(작업 실행과 같은 기준). 수동 입력은 집기 범위 그대로."""
    from types import SimpleNamespace

    from so101_studio.core.types import DetectedObject

    b = backend.arm.ws.base
    far = DetectedObject("us-1", b.x_mm, b.y_mm + 280.0, None, 0.0, 0.0, 0.5, "ultrasonic")  # 팔 원점 정면 반경 280: 집기 범위(250) 밖, 여유(+60) 안
    assert not backend.object_reachable(far)
    monkeypatch.setattr(backend, "source", SimpleNamespace(start_check=lambda *a, **k: None))
    assert backend.object_reachable(far)
