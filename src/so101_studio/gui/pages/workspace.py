"""작업 공간 페이지 (gui-spec 3절): 진행 막대, 3D와 센서 화면, 물체 목록, 관절 막대, 최근 이벤트, 스캔·한 단계·자동 실행·일시정지·정지."""

from dataclasses import astuple
from datetime import datetime

from nicegui import run, ui

from .. import sensors
from ..backend import JOINT_TEXT, PERCEPTION_TEXT
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

    async def perception_changed(e) -> None:
        """인식 소스 바꾸기(gui-spec 12.11). 초음파는 확인 창 뒤 스레드에서 연다(가상은 팔이 대기 자세로 가서 기준값을 잰다)."""
        kind = e.value
        if kind == backend.perception_kind:
            return
        port = None
        if kind == "real":
            from .esp32 import SIM_PORT, board_of

            board = board_of(backend)
            if not (board.connected and board.port and board.port != SIM_PORT):
                ui.notify("ESP32 센서 보드 페이지에서 실제 포트를 먼저 연결하세요", type="warning")
                perception.set_value(backend.perception_kind)
                return
            port = board.port
        if kind != "manual":
            with ui.dialog() as dialog, ui.card().mark("perception-confirm"):
                ui.label("초음파(가상): 팔이 대기 자세로 가서 빈 작업대 기준값을 잽니다(약 5초). 그동안 팔을 움직일 수 없습니다" if kind == "sim"
                         else f"초음파(실물): ESP32 페이지 연결을 닫고 {port}로 엽니다").classes("page-body")
                with ui.row():
                    ui.button("취소", on_click=lambda: dialog.submit(False)).props("outline no-caps").mark("perception-cancel")
                    ui.button("켜기", on_click=lambda: dialog.submit(True)).props("no-caps").mark("perception-ok")
            if not await dialog:
                perception.set_value(backend.perception_kind)
                return
            if kind == "real":
                board.disconnect()
        reason = await run.io_bound(backend.select_perception, kind, port)
        if reason:
            ui.notify(reason, type="warning")
        perception.set_value(backend.perception_kind)

    with ui.row().classes("w-full items-center gap-4").mark("recognition-row"):
        ui.label("인식").classes("stat-label")
        perception = controls.add(ui.toggle(PERCEPTION_TEXT, value=backend.perception_kind, on_change=perception_changed)
                                  .props("no-caps dense").mark("perception"))
        ui.label("모드").classes("stat-label")
        mode = controls.add(ui.toggle({False: "빠른", True: "설명"}, value=backend.explain, on_change=lambda e: setattr(backend, "explain", e.value))
                            .props("no-caps dense").mark("explain-mode"))
        mode.tooltip("설명: 레이더 회전 → 후보 → 팔 확인 → 위치 → 집기를 단계마다 멈추고 설명합니다(속도 30%)")
    caption = ui.label("").classes("recognition-caption").mark("recognition-caption")

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
                objects_title = ui.label("물체").classes("card-title").mark("objects-title")
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
        manual = backend.perception_kind == "manual"
        scan_button.enabled = controls.mine and not running and manual and not backend.move_lock()  # 초음파는 작업 실행으로만 본다(12.11)
        auto_button.enabled = controls.mine and idle and not running and not backend.move_lock()
        step_button.enabled = controls.mine and (not running or backend.paused) and not backend.move_lock()
        step_button.text = "다음" if running and backend.explain else "한 단계"
        pause_button.enabled = controls.mine and running
        pause_button.text = "계속" if backend.paused else "일시정지"
        perception.enabled = mode.enabled = controls.mine and not running and not backend.move_lock()
        objects_title.text = "물체" if manual else "가상 작업대에 놓인 물체(정답)" if backend.perception_kind == "sim" else "물체(정답, 사람이 적은 것)"
        caption.text = backend.caption()
        backend.draw_recognition()  # 3D 인식 겹(F2-f): 기록이 바뀔 때만
        caption.classes(add="caption-explain" if backend.explain else "", remove="" if backend.explain else "caption-explain")
        active = progress_index(state.task_state, state.placed)
        for i, chip in enumerate(chips):
            chip.classes(add="progress-active" if i == active else "", remove="" if i == active else "progress-active")
        summary.text = f"{TASK_TEXT.get(state.task_state, state.task_state)} · 옮긴 물체 {state.placed}개" + (" · 일시정지" if backend.paused else "")
        key = (len(state.events), state.events[-1].t if state.events else 0)
        if key != last_events[0]:
            last_events[0] = key
            events.clear()
            with events:
                if not state.events:
                    ui.label("아직 없습니다. [스캔]이나 [자동 실행]을 누르면 여기에 쌓입니다").classes("page-body")
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
