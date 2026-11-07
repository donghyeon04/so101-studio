"""운영자 화면 /operator (gui-spec 12.4, 결정 0044): 시연·프로젝터용. 메뉴·설정·진단 없이 3D, 지금 단계, 물체 목록, 실행 버튼만 크게.
조작권이 없는 브라우저는 보기 전용이라 버튼 줄이 숨고, E-STOP은 늘 보인다(결정 0047). 다른 기기(원격)는 조작권을 받을 수 없다. 작업 공간 화면과 같은 Backend 작업 함수를 쓴다."""

import time

from nicegui import run, ui

from .. import theme
from ..share import is_remote
from ..state import TASK_TEXT
from ..watchdog_link import BROWSER_HB_JS
from .common import Controls, scene_frame

STAGES = ("SCAN", "DECIDE", "CHECK", "PICK", "PLACE", "RESCAN")  # 작업 실행 상태 그대로 (gui-spec 12.4)
SOURCE_TEXT = {"ManualSource": "수동 입력", "UltrasonicSource": "초음파", "SimUltrasonicSource": "초음파(가상)"}


def object_lines(objects, events, chosen: str | None, task_state: str, since: float = 0.0) -> list[tuple[str, str]]:
    """(물체 id, 상태 글자). 놓은 물체는 이번 실행(since 뒤)의 작업 이벤트에서, 지금 고른 물체는 작업 상태에서 읽는다.
    실행 경계는 시작 시각이다. 수동 입력은 물체마다 PREPARE를 다시 거쳐 PREPARE로는 나눌 수 없다."""
    placed = {e.object_id for e in events if e.t >= since and e.state == "PLACE" and e.result == "ok" and e.object_id}
    out = [(oid, "✓ 놓음") for oid in sorted(placed)]
    for o in objects:
        if o.id in placed:
            continue
        out.append((o.id, f"{TASK_TEXT.get(task_state, task_state)} 중" if o.id == chosen else "대기"))
    return out


def build(backend, client_id: str) -> None:
    theme.apply()
    ui.add_head_html(BROWSER_HB_JS)
    controls = Controls(backend, client_id, block_during_task=False)
    remote = is_remote(ui.context.client.ip)  # 다른 기기(LAN IP로 연 같은 PC 포함)는 늘 보기 전용(gui-spec 12.6)
    state = backend.state
    started: list = [None]  # 이 화면에서 본 작업 시작 시각(걸린 시간 표시)
    beating: list[bool] = []

    async def start() -> None:
        reason = backend.start_task()
        if reason:
            ui.notify(reason, type="warning")

    async def step() -> None:  # 작업이 없으면 단계별로 시작, 멈춰 있으면 한 단계 진행
        reason = backend.step_task()
        if reason:
            ui.notify(reason, type="warning")

    async def pause() -> None:
        backend.toggle_pause()

    async def estop() -> None:
        failed = await run.io_bound(backend.estop)
        ui.notify(failed or "비상 정지했습니다", type="negative", **({"timeout": 0, "close_button": "닫기"} if failed else {}))

    with ui.column().classes("w-full gap-4 p-4 operator").mark("operator"):
        with ui.row().classes("w-full items-center no-wrap gap-4"):
            title = ui.label("").classes("operator-title flex-1 min-w-0").mark("operator-title")
            take = ui.button("조작권 받기", on_click=lambda: backend.claim(client_id, remote)).props("outline no-caps").mark("operator-take")
            ui.button("E-STOP", color="negative", on_click=estop).props("round no-caps").classes("estop").mark("operator-estop")
        with ui.row().classes("w-full items-start gap-4"):
            with ui.column().classes("flex-1 min-w-[420px] gap-4"):
                scene_frame(backend, "scene3d")
                with ui.card().classes("w-full gap-2"):
                    ui.label("물체").classes("operator-card-title")
                    objects = ui.row().classes("w-full gap-x-8 gap-y-2").mark("operator-objects")
            with ui.card().classes("operator-stage gap-3").mark("operator-stage"):
                ui.label("지금 단계").classes("operator-card-title")
                chips = {s: ui.label(TASK_TEXT[s]).classes("operator-chip").mark(f"operator-chip-{s}") for s in STAGES}
                now = ui.label("").classes("operator-now").mark("operator-now")
                count = ui.label("").classes("operator-body").mark("operator-count")
        with ui.row().classes("w-full items-center gap-3").mark("operator-buttons") as buttons:
            start_button = ui.button("시작", on_click=controls.guard(start)).props("no-caps").classes("operator-button").mark("operator-start")
            step_button = ui.button("한 단계", on_click=controls.guard(step)).props("outline no-caps").classes("operator-button").mark("operator-step")
            pause_button = ui.button("일시정지", on_click=controls.guard(pause)).props("outline no-caps").classes("operator-button").mark("operator-pause")
            ui.button("정지", color="negative", on_click=lambda: run.io_bound(backend.stop_task)).props("outline no-caps").classes("operator-button").mark("operator-stop")
        view_only = ui.label("보기 전용: 조작은 조작권이 있는 화면에서 합니다").classes("operator-body").mark("operator-view-only")

    last: list = [None]

    def tick() -> None:
        controls.refresh()
        mine = controls.mine
        if beating != [mine]:  # 조작하는 화면은 감시 프로세스에 heartbeat를 보낸다(본 화면과 같다, 결정 0026)
            beating[:] = [mine]
            ui.run_javascript(f"so101Hb({str(mine).lower()})")
        running = backend.task_running()
        buttons.visible = mine  # E-STOP은 조작권과 상관없이 늘 보인다(결정 0047)
        view_only.visible = not mine and (remote or backend.owner is not None)
        view_only.text = "보기 전용(다른 기기): 조작은 팔이 연결된 PC 화면에서 합니다" if remote else "보기 전용: 조작은 조작권이 있는 화면에서 합니다"
        take.visible = backend.owner is None and not remote
        idle = bool(state.status and state.status.state == "idle")
        start_button.enabled = mine and idle and not running and not backend.move_lock()
        step_button.enabled = mine and (not running or backend.paused) and not backend.move_lock()
        pause_button.enabled = mine and running
        pause_button.text = "계속" if backend.paused else "일시정지"
        source = SOURCE_TEXT.get(type(backend.source).__name__, type(backend.source).__name__)
        title.text = f"SO-101 · {'실물' if backend.real else '시뮬레이션'} · 인식: {source}"
        ts = state.task_state
        if running and started[0] is None:
            started[0] = time.monotonic()
        elif not running:
            started[0] = None
        for s, chip in chips.items():
            active = s == ts
            chip.classes(add="operator-chip-active" if active else "", remove="" if active else "operator-chip-active")
        now.text = TASK_TEXT.get(ts, ts) + (" · 일시정지" if backend.paused else "")
        lines = object_lines(backend.objects(), list(state.events), state.chosen, ts, state.run_t0)
        took = f" · 걸린 시간 {time.monotonic() - started[0]:.0f}초" if started[0] is not None and running else ""
        count.text = f"옮긴 물체 {state.placed}/{len(lines)}{took}" if lines else "물체가 없습니다"
        if lines != last[0]:
            last[0] = lines
            objects.clear()
            with objects:
                for oid, text in lines:
                    ui.label(f"{oid} {text}").classes("operator-body")

    # 본 화면과 같다: 소켓이 열리면 조작권을 받고(비어 있을 때), 끊기면 돌려주며 작업을 세운다
    client = ui.context.client
    client.on_connect(lambda: backend.claim(client_id, remote))
    client.on_disconnect(lambda: backend.disconnected(client_id))
    ui.timer(0.2, tick)
