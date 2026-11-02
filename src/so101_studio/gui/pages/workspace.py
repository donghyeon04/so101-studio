"""작업 공간 페이지 (gui-spec 3절): 진행 막대, 3D와 센서 화면, 물체 목록, 관절 막대, 최근 이벤트, 스캔·한 단계·자동 실행·일시정지·정지."""

from dataclasses import astuple
from datetime import datetime

from nicegui import run, ui

from .. import sensors
from ..backend import JOINT_TEXT
from ..state import PROGRESS, TASK_TEXT, progress_index
from .common import Controls, scene_frame

EVENTS_SHOWN = 8


def event_line(event) -> str:
    text = TASK_TEXT.get(event.state, event.state)
    if event.object_id:
        text += f" {event.object_id}"
    reason = event.detail.get("reason")
    if reason:
        text += f": {reason}"
    elif event.result:
        text += f" ({event.result})"
    return f"{datetime.fromtimestamp(event.t):%H:%M:%S}  {text}"


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id, block_during_task=False)
    state = backend.state

    async def scan() -> None:
        count = len(await run.io_bound(backend.scan))
        ui.notify(f"물체 {count}개를 찾았습니다")

    async def auto() -> None:
        reason = backend.start_task()
        if reason:
            ui.notify(reason, type="warning")

    async def step() -> None:
        reason = backend.step_task()
        if reason:
            ui.notify(reason, type="warning")

    async def pause() -> None:
        backend.toggle_pause()

    with actions:
        ui.button("정지", color="negative", on_click=lambda: run.io_bound(backend.stop_task)).props("outline no-caps").mark("task-stop")
        pause_button = ui.button("일시정지", on_click=controls.guard(pause)).props("outline no-caps").mark("task-pause")
        step_button = ui.button("한 단계", on_click=controls.guard(step)).props("outline no-caps").mark("task-step")
        scan_button = ui.button("스캔", on_click=controls.guard(scan)).props("outline no-caps").mark("task-scan")
        auto_button = ui.button("자동 실행", on_click=controls.guard(auto)).props("no-caps").mark("task-auto")

    with ui.row().classes("w-full no-wrap gap-1").mark("progress"):
        chips = [ui.label(text).classes("progress-step flex-1").mark(f"progress-{i}") for i, text in enumerate(PROGRESS)]
    summary = ui.label("").classes("page-body").mark("task-summary")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-3"):
            scene_frame(backend, "scene3d scene3d-short")
            with ui.card().classes("w-full gap-1"):
                ui.label("최근 이벤트").classes("card-title")
                events = ui.column().classes("gap-0 w-full").mark("task-events")
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                for view in sensors.mini_views():
                    ui.label(view.title).classes("card-title")
                    view.mini_view_fn(backend)
                pending = sensors.pending_titles()
                if pending:
                    ui.label(f"{', '.join(pending)}: 추가 예정").classes("page-body").mark("pending-sensors")
            with ui.card().classes("w-full gap-2"):
                ui.label("물체").classes("card-title")
                objects = ui.column().classes("gap-0 w-full").mark("object-list")
            with ui.card().classes("w-full gap-1"):
                ui.label("관절").classes("card-title")
                bars = {}
                for name, text in JOINT_TEXT.items():
                    with ui.row().classes("w-full no-wrap items-center gap-2"):
                        ui.label(text).classes("joint-name")
                        bars[name] = ui.linear_progress(value=0, show_value=False).classes("flex-1")
                        bars[name + "_v"] = ui.label("").classes("stat-value joint-value")

    last_events: list = [None]
    last_objects: list = [None]

    def tick() -> None:
        controls.refresh()
        running = backend.task_running()
        idle = bool(state.status and state.status.state == "idle")
        scan_button.enabled = controls.mine and not running
        auto_button.enabled = controls.mine and idle and not running
        step_button.enabled = controls.mine and (not running or backend.paused)
        pause_button.enabled = controls.mine and running
        pause_button.text = "계속" if backend.paused else "일시정지"
        active = progress_index(state.task_state, state.placed)
        for i, chip in enumerate(chips):
            chip.classes(add="progress-active" if i == active else "", remove="" if i == active else "progress-active")
        summary.text = f"{TASK_TEXT.get(state.task_state, state.task_state)} · 옮긴 물체 {state.placed}개" + (" · 일시정지" if backend.paused else "")
        key = (len(state.events), state.events[-1].t if state.events else 0)
        if key != last_events[0]:
            last_events[0] = key
            events.clear()
            with events:
                for event in list(state.events)[-EVENTS_SHOWN:][::-1]:
                    ui.label(event_line(event)).classes("event-line")
        lines = [
            f"{'▶ ' if o.id == state.chosen else ''}{o.id}  x {o.x_mm:.0f}  y {o.y_mm:.0f}  {'닿음' if backend.object_reachable(o) else '안 닿음'}"
            for o in backend.objects()
        ]
        if lines != last_objects[0]:
            last_objects[0] = lines
            objects.clear()
            with objects:
                for line in lines:
                    ui.label(line).classes("event-line")
        if state.status:
            limits = backend.arm.cfg.joint_limits_deg  # 안전 모드 "끔"에도 막대를 그릴 수 있게 설정값을 쓴다
            for name, value in zip(JOINT_TEXT, astuple(state.status.joints), strict=True):
                lo, hi = limits[name]
                bars[name].value = round(min(max((value - lo) / (hi - lo), 0.0), 1.0), 3)
                bars[name + "_v"].text = f"{value:6.1f}"

    ui.timer(0.2, tick)
