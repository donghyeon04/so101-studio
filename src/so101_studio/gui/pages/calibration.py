"""장치 › 캘리브레이션 마법사 1차 (gui-spec 12.3): 1 준비 › 2 측정 › 3 비교. EEPROM에 쓰지 않는 측정만(measure_only).
4 저장 › 5 0° 확인은 #41(REQ-017)과 AC4 실물 합격 뒤 2차로 만든다. 실제 팔에서만 켜지고, 시뮬레이션에서는 단계 설명만 보인다."""

import threading

from nicegui import run, ui

from .common import Controls

MIN_VOLT = 11.0  # 측정 엔진 기준과 같다(결정 0028, 0043-3)
STEPS = ("1 준비", "2 측정", "3 비교", "4 저장", "5 0° 확인")
CHECKS = ("팔 받침을 클램프로 고정했다", "팔 주변 반경 50cm를 비웠다", "12V 전원 스위치 옆에 사람이 있다")
GUIDE = {
    0: "팔을 쉬는 자세(윗팔 뒤로, 아래팔 접고, 그리퍼가 받침 앞에서 아래로 늘어지게)로 두고 아래 세 가지를 확인하세요. 측정은 관절을 하나씩 끝까지 밀어 범위와 0° 위치를 잽니다.",
    1: "측정 중에는 팔이 관절마다 끝까지 움직입니다. 이상하면 [중단]을 누르세요. 엔진이 토크 한계와 위치를 되돌린 뒤 다시 연결합니다. 전압이 11.0V 아래로 떨어지면 스스로 멈춥니다.",
    2: "기존 캘리브레이션과 관절별 범위·0° 위치 차이를 봅니다. 모든 관절이 ±3° 이내면 합격입니다. 반복 측정 합격(3회 중앙값)과 저장은 2차에서 엽니다.",
}
LATER = "AC4 실물 시험 뒤 열립니다(#41 REQ-017). 지금은 CLI: `so101.cmd calibrate --save`, `so101.cmd calibrate --verify-zero`"
LOG_LINES = 8


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id, block_during_task=True)
    state: dict = {"step": 0, "report": None, "reason": None, "lines": [], "busy": False, "stop": threading.Event()}

    with ui.row().classes("w-full no-wrap gap-1").mark("calib-steps"):
        chips = [ui.label(text).classes("progress-step flex-1").mark(f"calib-step-{i}") for i, text in enumerate(STEPS)]
    guide = ui.label("").classes("page-body").mark("calib-guide")
    sim_note = ui.label("실제 팔이 연결됐을 때만 켜집니다. 시뮬레이션에서는 단계 설명만 볼 수 있습니다").classes("page-body").mark("calib-sim")

    # 1 준비
    with ui.card().classes("w-full gap-2").mark("calib-prepare") as prepare:
        boxes = [ui.checkbox(text).mark(f"calib-check-{i}") for i, text in enumerate(CHECKS)]
        why = ui.label("").classes("page-body").mark("calib-why")
        next_button = controls.add(ui.button("다음: 측정", on_click=lambda: go(1)).props("no-caps").mark("calib-next"))

    # 2 측정
    with ui.card().classes("w-full gap-2").mark("calib-measure") as measure:
        with ui.row().classes("items-center gap-2"):
            start_button = controls.add(ui.button("측정 시작", on_click=controls.guard(lambda: start())).props("no-caps").mark("calib-start"))
            ui.button("중단", color="negative", on_click=lambda: state["stop"].set()).props("outline no-caps").mark("calib-stop")
        now = ui.label("").classes("page-body").mark("calib-now")
        log = ui.column().classes("gap-0 w-full").mark("calib-log")

    # 3 비교
    with ui.card().classes("w-full gap-2").mark("calib-compare") as compare:
        verdict = ui.label("").classes("card-title").mark("calib-verdict")
        table = ui.column().classes("gap-0 w-full").mark("calib-table")
        with ui.row().classes("gap-2"):
            ui.button("다시 측정", on_click=lambda: go(1)).props("outline no-caps").mark("calib-again")
            ui.button("처음으로", on_click=lambda: go(0)).props("outline no-caps").mark("calib-restart")

    later = ui.label(LATER).classes("page-body").mark("calib-later")

    def go(step: int) -> None:
        state["step"] = step

    def ready_reason() -> str | None:
        """1단계 [다음]이 켜지는 조건(gui-spec 12.3 표). 막히면 이유."""
        st = backend.state.status
        if backend.real is None:
            return "실제 팔이 연결되어 있지 않습니다"
        if not controls.mine:
            return "조작권이 있는 화면에서만 할 수 있습니다"
        if backend.task_running() or not (st and st.state == "idle"):
            return "팔이 멈춰 있고 실행 중인 작업이 없어야 합니다"
        if st.voltage_v < MIN_VOLT:
            return f"지금 전압 {st.voltage_v:.1f}V가 {MIN_VOLT:.1f}V 아래입니다. 전원 경로를 먼저 점검하세요(hardware.md)"
        if not all(b.value for b in boxes):
            return "세 가지를 모두 확인해 주세요"
        return None

    async def start() -> None:
        state.update(busy=True, lines=[], report=None, reason=None)
        state["stop"].clear()

        def out(text) -> None:
            state["lines"] = (state["lines"] + [line for line in str(text).splitlines() if line.strip()])[-200:]

        try:
            state["report"], state["reason"] = await run.io_bound(backend.calibration_measure, out, state["stop"].is_set)
        finally:
            state["busy"] = False
        if state["report"] is not None:
            go(2)
        elif state["reason"]:
            ui.notify(state["reason"], type="negative", timeout=0, close_button="닫기")

    shown: list = [None]

    def tick() -> None:
        controls.refresh()
        step = state["step"]
        for i, chip in enumerate(chips):
            chip.classes(add="progress-active" if i == step else "", remove="" if i == step else "progress-active")
        guide.text = GUIDE.get(step, "")
        sim_note.visible = backend.real is None
        prepare.visible, measure.visible, compare.visible = step == 0, step == 1, step == 2
        later.visible = True
        if step == 0:
            reason = ready_reason()
            why.text = reason or "준비됐습니다"
            next_button.enabled = reason is None
        elif step == 1:
            start_button.enabled = not state["busy"] and backend.real is not None and controls.mine
            lines = state["lines"]
            now.text = "측정 중..." if state["busy"] else (state["reason"] or "측정 전")
            if lines != shown[0]:
                shown[0] = list(lines)
                log.clear()
                with log:
                    for line in lines[-LOG_LINES:]:
                        ui.label(line).classes("event-line")
        elif step == 2 and state["report"] is not None and shown[0] != ("report", id(state["report"])):
            shown[0] = ("report", id(state["report"]))
            rep = state["report"]
            verdict.text = (f"합격: 모든 관절이 ±{rep.get('pass_deg', 3):g}° 이내" if rep.get("passed")
                            else "합격 아님: 기존 값과 다르거나 비교할 기존 파일이 없습니다. 반복 측정 합격은 2차(#41)에서 엽니다")
            table.clear()
            with table:
                ui.label("관절 | 기존 범위 | 새 범위 | 범위 차이 | 0° 차이 | 판정").classes("stat-label")
                for r in rep.get("rows", []):
                    num = lambda v: "-" if v is None else f"{v:.1f}"
                    mark = "-" if r.get("ok") is None else ("합격" if r["ok"] else "벗어남")
                    ui.label(f"{r['joint']} | {num(r.get('old_range_deg'))} | {num(r.get('new_range_deg'))} | {num(r.get('range_diff_deg'))} | "
                             f"{num(r.get('zero_diff_deg'))} | {mark}").classes("event-line").mark(f"calib-row-{r['joint']}")
                if rep.get("run_dir"):
                    ui.label(f"기록: {rep['run_dir']}/measure.json").classes("page-body")

    ui.timer(0.2, tick)
