"""작업 > 프로그램 (결정 0044, UR PolyScope 프로그램 트리 방식): 왼쪽 단계 목록, 가운데 고른 단계 편집과 실행, 오른쪽 자세 즐겨찾기.
실행은 Backend.run_program이 실행 방식(시뮬레이션 먼저 등)과 안전 검사를 그대로 쓴다."""

import copy

from nicegui import run, ui

from .. import programs
from .common import Controls

NUM_LABEL = {
    "x": "x (mm)", "y": "y (mm)", "h": "높이 (mm)", "yaw": "방향 (°)", "pitch": "손끝 각도 (°)", "value": "그리퍼 (0 닫힘 ~ 100 열림)",
    "angle": "물체 각도 (°)", "width": "물체 폭 (mm)", "length": "물체 길이 (mm)", "height": "물체 높이 (mm, 0 = 설정값)",
    "place_x": "놓을 x (mm)", "place_y": "놓을 y (mm)", "s": "초", "speed": "배속", "count": "반복 횟수",
}


def build(backend, client_id: str, actions) -> None:
    gate = Controls(backend, client_id, moves=True)
    state: dict = {"name": "", "steps": [], "sel": None, "running": False}  # sel = (바깥 번호, 반복 안 번호 | None)

    def selected() -> dict | None:
        sel = state["sel"]
        if sel is None:
            return None
        i, j = sel
        if i >= len(state["steps"]):
            return None
        step = state["steps"][i]
        return step if j is None else (step["steps"][j] if j < len(step["steps"]) else None)

    def container(sel) -> list:
        i, j = sel
        return state["steps"] if j is None else state["steps"][i]["steps"]

    def changed() -> None:
        step_list.refresh()
        editor.refresh()

    def add(kind: str, inside: bool) -> None:
        step = {"type": kind, **copy.deepcopy(programs.DEFAULTS[kind])}
        sel = state["sel"]
        if inside and sel and state["steps"][sel[0]]["type"] == "repeat" and kind != "repeat":
            inner = state["steps"][sel[0]]["steps"]
            inner.append(step)
            state["sel"] = (sel[0], len(inner) - 1)
        else:
            state["steps"].append(step)
            state["sel"] = (len(state["steps"]) - 1, None)
        changed()

    def move(sel, d: int) -> None:
        items, k = container(sel), sel[1] if sel[1] is not None else sel[0]
        if 0 <= k + d < len(items):
            items[k], items[k + d] = items[k + d], items[k]
            state["sel"] = (sel[0] + (d if sel[1] is None else 0), None if sel[1] is None else sel[1] + d)
            changed()

    def duplicate(sel) -> None:
        items, k = container(sel), sel[1] if sel[1] is not None else sel[0]
        items.insert(k + 1, copy.deepcopy(items[k]))
        changed()

    def remove(sel) -> None:
        items, k = container(sel), sel[1] if sel[1] is not None else sel[0]
        items.pop(k)
        state["sel"] = None
        changed()

    def load(name: str) -> None:
        if name:
            state["name"], state["steps"], state["sel"] = name, backend.load_program(name), None
            name_input.value = name
            changed()

    def save() -> None:
        reason = backend.save_program(name_input.value or "", state["steps"])
        if reason:
            ui.notify(reason, type="negative")
            return
        state["name"] = name_input.value.strip()
        picker.options = backend.program_names()
        picker.value = state["name"]
        ui.notify(f"프로그램 {state['name']}을 저장했습니다", type="positive")

    async def start(step_mode: bool) -> None:
        state["running"] = True
        try:
            reason = await run.io_bound(backend.run_program, copy.deepcopy(state["steps"]), name_input.value or "이름 없음", step_mode)
        finally:
            state["running"] = False
        if reason:
            ui.notify(reason, type="negative")
        elif backend.pending:
            ui.notify("시뮬레이션에서 끝났습니다. 위 확인 창에서 실제 팔로 실행할지 고르세요", type="info")
        else:
            ui.notify("프로그램을 마쳤습니다", type="positive")

    with actions:
        ui.button("정지", color="negative", on_click=lambda: run.io_bound(backend.stop_program)).props("outline no-caps").mark("program-stop")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        # 단계 목록
        with ui.column().classes("flex-1 min-w-0 gap-3"):
            with ui.card().classes("w-full gap-2"):
                with ui.row().classes("w-full items-center gap-2 no-wrap"):
                    picker = ui.select(backend.program_names(), label="불러오기", on_change=lambda e: load(e.value)).props("outlined dense").classes("w-48").mark("program-picker")
                    name_input = ui.input("프로그램 이름").props("outlined dense").classes("flex-1").mark("program-name")
                    ui.button("저장", on_click=save).props("outline no-caps").mark("program-save")
                with ui.row().classes("w-full items-center gap-2"):
                    kind = ui.select({k: v for k, v in programs.STEP_TEXT.items()}, value="waypoint", label="단계 종류").props("outlined dense").classes("w-40").mark("step-kind")
                    ui.button("끝에 추가", on_click=lambda: add(kind.value, False)).props("outline no-caps").mark("step-add")
                    ui.button("고른 반복 안에 추가", on_click=lambda: add(kind.value, True)).props("outline no-caps").mark("step-add-inside")

            @ui.refreshable
            def step_list() -> None:
                with ui.card().classes("w-full gap-1").mark("step-list"):
                    ui.label("단계 (위에서부터 실행)").classes("card-title")
                    if not state["steps"]:
                        ui.label("단계를 추가하세요. 예: 자세로 이동 → 그리퍼 닫기 → 좌표로 이동 → 그리퍼 열기").classes("page-body")
                    for i, step in enumerate(state["steps"]):
                        _row(f"{i + 1}", step, (i, None))
                        if step["type"] == "repeat":
                            for j, inner in enumerate(step["steps"]):
                                _row(f"{i + 1}.{j + 1}", inner, (i, j), indent=True)

            def _row(no: str, step: dict, sel, indent: bool = False) -> None:
                classes = "w-full items-center gap-1 no-wrap program-step" + (" pl-8" if indent else "")
                with ui.row().classes(classes).mark(f"step-{no}") as row:
                    if state["sel"] == sel:
                        row.classes("program-step-selected")
                    ui.label(no).classes("stat-label w-8 shrink-0")
                    ui.button(programs.describe(step), on_click=lambda s=sel: (state.update(sel=s), changed())).props("flat no-caps align=left").classes("flex-1 min-w-0")
                    for icon, label, fn in (("arrow_upward", "위로", lambda s=sel: move(s, -1)), ("arrow_downward", "아래로", lambda s=sel: move(s, 1)),
                                            ("content_copy", "복사", lambda s=sel: duplicate(s)), ("delete", "삭제", lambda s=sel: remove(s))):
                        ui.button(icon=icon, on_click=fn).props(f'flat round dense aria-label="{label}"')

            step_list()

        # 고른 단계 편집과 실행
        with ui.column().classes("control-column gap-3"):
            @ui.refreshable
            def editor() -> None:
                step = selected()
                with ui.card().classes("w-full gap-2").mark("step-editor"):
                    ui.label("고른 단계").classes("card-title")
                    if step is None:
                        ui.label("왼쪽에서 단계를 고르세요").classes("page-body")
                        return
                    ui.label(programs.STEP_TEXT[step["type"]]).classes("page-body")
                    for key in programs.DEFAULTS[step["type"]]:
                        if key == "steps":
                            continue
                        if key == "name" and step["type"] == "waypoint":
                            ui.select(list(backend.waypoints()), value=step["name"] or None, label="자세",
                                      on_change=lambda e, s=step: (s.update(name=e.value or ""), step_list.refresh())).props("outlined dense").classes("w-full").mark("edit-name")
                        elif key == "name":
                            ui.select(backend.sequences(), value=step["name"] or None, label="교시 기록",
                                      on_change=lambda e, s=step: (s.update(name=e.value or ""), step_list.refresh())).props("outlined dense").classes("w-full").mark("edit-name")
                        else:
                            ui.number(NUM_LABEL.get(key, key), value=step[key],
                                      on_change=lambda e, s=step, k=key: (s.update({k: int(e.value) if k == "count" else float(e.value)}), step_list.refresh()) if e.value is not None else None,
                                      ).props("outlined dense").classes("w-full").mark(f"edit-{key}")

            editor()
            with ui.card().classes("w-full gap-2"):
                ui.label("실행").classes("card-title")
                ui.label("지금 실행 방식(위쪽)을 따릅니다. 시뮬레이션 먼저면 끝난 뒤 실제 팔로 실행할지 묻습니다").classes("page-body")
                with ui.row().classes("w-full gap-2 no-wrap"):
                    gate.add(ui.button("실행", on_click=gate.guard(lambda: start(False))).props("no-caps").classes("flex-1").mark("program-run"))
                    gate.add(ui.button("한 단계씩", on_click=gate.guard(lambda: start(True))).props("outline no-caps").classes("flex-1").mark("program-step-run"))
                next_button = ui.button("다음 단계", on_click=lambda: backend.program_run and backend.program_run.next_step()).props("outline no-caps").classes("w-full").mark("program-next")
                progress = ui.label("").classes("page-body").mark("program-progress")

        # 자세 즐겨찾기
        with ui.column().classes("control-column gap-3"):
            @ui.refreshable
            def poses() -> None:
                with ui.card().classes("w-full gap-1").mark("waypoint-list"):
                    ui.label("자세 (★ 즐겨찾기가 위)").classes("card-title")
                    wps = backend.waypoints()
                    if not wps:
                        ui.label("팔을 원하는 자세로 옮긴 뒤 이름을 넣고 저장하세요").classes("page-body")
                    for name, wp in sorted(wps.items(), key=lambda kv: (not kv[1].get("favorite"), kv[0])):
                        with ui.row().classes("w-full items-center gap-1 no-wrap").mark(f"waypoint-{name}"):
                            fav = wp.get("favorite")
                            ui.button(icon="star" if fav else "star_border", on_click=lambda n=name, f=fav: (backend.set_favorite(n, not f), poses.refresh())).props(
                                f'flat round dense color={"warning" if fav else "grey-8"} aria-label="즐겨찾기"').mark(f"waypoint-fav-{name}")
                            ui.label(name).classes("flex-1 min-w-0 page-body")
                            gate.add(ui.button("이동", on_click=gate.guard(lambda n=name: _goto(n))).props("outline dense no-caps").mark(f"waypoint-go-{name}"))
                            ui.button(icon="delete", on_click=lambda n=name: (backend.delete_waypoint(n), poses.refresh())).props('flat round dense aria-label="삭제"')
                    with ui.row().classes("w-full items-center gap-2 no-wrap mt-2"):
                        new_name = ui.input("새 자세 이름").props("outlined dense").classes("flex-1").mark("waypoint-name")
                        ui.button("지금 자세 저장", on_click=lambda: _save_pose(new_name.value or "")).props("outline no-caps").mark("waypoint-save")

            async def _goto(name: str) -> None:
                reason = await run.io_bound(backend.goto_waypoint, name)
                if reason:
                    ui.notify(reason, type="negative")

            def _save_pose(name: str) -> None:
                reason = backend.save_waypoint(name)
                if reason:
                    ui.notify(reason, type="negative")
                else:
                    poses.refresh()
                    editor.refresh()

            poses()

    def tick() -> None:
        gate.refresh()
        r = backend.program_run
        next_button.visible = bool(state["running"] and r and r.step_mode)
        if state["running"] and r:
            progress.text = f"실행 중: {r.done + 1}/{r.total} 단계" if r.total else "실행 준비 중"
            if r.index is not None and r.index != state["sel"]:
                state["sel"] = r.index
                changed()
        elif r and r.reason:
            progress.text = f"멈춤: {r.reason}"
        elif r:
            progress.text = "마쳤습니다"

    ui.timer(0.2, tick)
