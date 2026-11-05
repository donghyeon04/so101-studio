"""작업 공간과 수동 입력 (gui-spec G6). 시뮬레이션에서 물체를 차례로 옮기고 진행 막대가 따라가는지 본다."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.core.frames import Pose2D
from so101_studio.core.types import DetectedObject, TaskEvent
from so101_studio.gui import app as gui_app
from so101_studio.gui import sensors
from so101_studio.gui.backend import Backend
from so101_studio.gui.sensors.manual_view import plane_svg
from so101_studio.gui.state import PROGRESS, State, progress_index


@pytest.fixture
def clean(backend):
    backend.reset_objects()
    backend.state.events.clear()
    backend.state.task_state = "IDLE"
    yield backend
    backend.stop_task()
    _wait(lambda: not backend.task_running(), 15.0)
    backend.arm.resume()
    _wait(lambda: backend.arm.ctl.state == "idle", 3.0)
    backend.reset_objects()


async def _until(check, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def _wait(check, timeout=60.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.05)
    return False


def _keep_two(backend) -> list[str]:
    ids = [o.id for o in backend.objects()]
    for object_id in ids[2:]:
        backend.remove_object(object_id)
    return ids[:2]


def test_progress_index_follows_states():
    assert PROGRESS[progress_index("PREPARE", 0)] == "준비"
    assert PROGRESS[progress_index("SCAN", 0)] == "스캔"
    assert PROGRESS[progress_index("DECIDE", 1)] == "판단"
    assert PROGRESS[progress_index("PICK", 1)] == "집기"
    assert PROGRESS[progress_index("PLACE", 1)] == "놓기"
    assert PROGRESS[progress_index("SCAN", 1)] == "다시 스캔"
    assert progress_index("DONE", 2) is None


def test_state_tracks_chosen_and_placed():
    state = State()
    state.on_task(TaskEvent(0, "DECIDE", detail={"chosen": "a", "reason": "가까움"}))
    assert state.chosen == "a"
    state.on_task(TaskEvent(1, "PLACE", "a"))
    assert state.placed == 0
    state.on_task(TaskEvent(2, "PLACE", "a", "ok"))
    assert (state.placed, state.chosen) == (1, None)
    assert len(state.events) == 3


def test_registry_has_manual_page_and_pending_sensors():
    assert [path for path, _, _ in sensors.menu()] == ["/ultrasonic", "/camera", "/input"]
    assert {"/input", "/ultrasonic"} <= set(sensors.pages())
    assert sensors.pending_titles() == ["카메라"]
    assert [v.name for v in sensors.mini_views()] == ["manual"]


def test_plane_flips_y_and_marks_selection():
    table = {"x": 600, "y": 400}
    objects = [DetectedObject("obj-1", 100, 300, 45, 30, 100, 1.0, "manual"), DetectedObject("o<b>", 200, 100, 0, 30, 100, 1.0, "manual")]
    svg = plane_svg(table, objects, {"right": {"x_mm": 500, "y_mm": 150}}, Pose2D(300, 50, 90), {"min": 90, "max": 250}, "obj-1")
    assert "translate(100.0 100.0) rotate(-45.0)" in svg  # y는 400 - 300
    assert "o&lt;b&gt;" in svg  # 이름은 이스케이프한다
    assert "fill=\"#2457d6\"" in svg


def test_add_move_remove_reset_objects(clean):
    count = len(clean.objects())
    object_id = clean.add_object(310, 180, 30, 40, 80)
    added = next(o for o in clean.objects() if o.id == object_id)
    assert (added.x_mm, added.y_mm, added.angle_deg, added.width_mm, added.length_mm) == (310, 180, 30, 40, 80)
    clean.move_object(object_id, 320, 190)
    moved = next(o for o in clean.objects() if o.id == object_id)
    assert (moved.x_mm, moved.y_mm, moved.width_mm) == (320, 190, 40)
    clean.remove_object(object_id)
    assert len(clean.objects()) == count
    clean.remove_object(clean.objects()[0].id)
    clean.reset_objects()
    assert len(clean.objects()) == count


def test_two_objects_are_moved_in_turn_and_events_follow(clean):
    clean.set_speed(100)
    first, second = _keep_two(clean)
    clean.start_task()
    seen = set()

    def watch():
        seen.add(clean.state.task_state)
        return not clean.task_running()

    assert _wait(watch, 120.0)
    assert clean.studio.result["end"] == "no_objects"
    assert set(clean.studio.result["picked"]) == {first, second}
    assert clean.state.placed == 2
    assert clean.objects() == []
    assert {"PICK", "PLACE", "DONE"} <= {e.state for e in clean.state.events}
    assert {"PREPARE", "SCAN", "DECIDE", "PICK", "PLACE"} & seen
    assert clean.state.task_state == "DONE"


def test_step_mode_waits_for_each_step(clean):
    clean.set_speed(100)
    clean.step_task()
    assert _wait(lambda: clean.state.task_state == "PREPARE", 3.0)
    time.sleep(0.5)
    assert clean.state.task_state == "PREPARE"  # 다음 단계를 기다린다
    clean.step_task()
    assert _wait(lambda: clean.state.task_state == "SCAN", 15.0)


def test_stop_during_step_mode_ends_the_thread(clean):
    clean.set_speed(100)
    assert clean.step_task() is None
    assert _wait(lambda: clean.state.task_state == "PREPARE", 3.0)
    clean.stop_task()
    assert _wait(lambda: not clean.task_running(), 15.0)


def test_pause_holds_then_continue_runs(clean):
    clean.set_speed(100)
    _keep_two(clean)
    assert clean.start_task() is None
    assert _wait(lambda: clean.state.task_state == "PICK", 15.0)
    clean.toggle_pause()
    assert clean.paused
    assert _wait(lambda: clean.state.task_state == "PLACE", 30.0)  # 지금 단계는 끝나고 다음 상태에서 멈춘다
    time.sleep(1.0)
    assert clean.state.task_state == "PLACE" and clean.state.placed == 0 and clean.task_running()
    clean.toggle_pause()
    assert not clean.paused
    assert _wait(lambda: clean.state.placed == 1, 30.0)


def test_stop_ends_task_and_estop_needs_resume(clean):
    clean.set_speed(100)
    clean.start_task()
    assert _wait(lambda: clean.task_running(), 2.0)
    clean.stop_task()
    assert _wait(lambda: not clean.task_running(), 15.0)
    assert _wait(lambda: clean.arm.ctl.state == "idle", 3.0)
    assert clean.start_task() is None
    assert _wait(lambda: clean.task_running(), 2.0)
    clean.estop()
    assert _wait(lambda: not clean.task_running(), 15.0)
    assert clean.arm.ctl.state in ("stopped", "fault")
    assert clean.start_task() == "정지 상태입니다. 정지 해제를 눌러 주세요"


def test_owner_disconnect_stops_task(clean):
    clean.set_speed(100)
    clean.claim("A")
    clean.start_task()
    assert _wait(lambda: clean.task_running(), 2.0)
    clean.disconnected("A")
    assert _wait(lambda: not clean.task_running(), 15.0)


def test_workspace_page_shows_progress_and_runs(clean):
    clean.set_speed(100)
    _keep_two(clean)

    async def go():
        async with user_simulation(gui_app.make_root(clean)) as user:
            await user.open("/")
            await user.should_see(marker="progress")
            await user.should_see("추가 예정")
            assert await _until(lambda: next(iter(user.find(marker="task-auto").elements)).enabled)
            user.find(marker="task-auto").click()
            assert await _until(clean.task_running, 3.0)
            await user.should_see("옮긴 물체")
            active = next(iter(user.find(marker="progress").elements))
            assert await _until(
                lambda: any("progress-active" in c.classes for c in active.default_slot.children), 10.0
            )
            user.find(marker="task-stop").click()
            assert await _until(lambda: not clean.task_running(), 15.0)

    asyncio.run(go())


def test_manual_controls_blocked_while_task_runs(clean):
    clean.set_speed(100)
    clean.start_task()

    async def go():
        async with user_simulation(gui_app.make_root(clean)) as user:
            await user.open("/manual")
            assert await _until(lambda: not next(iter(user.find(marker="home").elements)).enabled, 3.0)

    asyncio.run(go())


def test_input_page_lists_objects_and_view_only_is_off(clean):
    clean.owner = "다른 브라우저"

    async def go():
        async with user_simulation(gui_app.make_root(clean)) as user:
            await user.open("/input")
            await user.should_see(marker="plane")
            await user.should_see(marker="object-table")
            assert await _until(lambda: not next(iter(user.find(marker="reset-objects").elements)).enabled, 3.0)

    asyncio.run(go())


def test_clicking_plane_adds_then_moves_object(clean):
    async def go():
        async with user_simulation(gui_app.make_root(clean)) as user:
            await user.open("/input")
            assert await _until(lambda: clean.owner is not None)
            plane = next(iter(user.find(marker="plane").elements))
            count = len(clean.objects())
            handlers = plane._event_listeners
            listener = next(v for v in handlers.values() if v.type == "mouse")
            args = {"mouse_event_type": "click", "image_x": 100, "image_y": 100, "button": 0, "buttons": 0,
                    "altKey": False, "ctrlKey": False, "metaKey": False, "shiftKey": False}
            from nicegui.events import GenericEventArguments

            for handler in listener.handler if isinstance(listener.handler, list) else [listener.handler]:
                handler(GenericEventArguments(sender=plane, client=user.client, args=args))
            assert await _until(lambda: len(clean.objects()) == count + 1)
            added = next(o for o in clean.objects() if o.id.startswith("added"))
            assert (added.x_mm, added.y_mm) == (100, clean.arm.ws.table_mm["y"] - 100)

    asyncio.run(go())


def test_backend_without_scene_still_runs_objects():
    backend = Backend(log_root=None)
    try:
        assert backend.objects()
        backend.add_object(300, 200)
    finally:
        backend.close()
