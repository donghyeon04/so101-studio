"""장치 › 캘리브레이션 마법사 (gui-spec 12.3): 1 준비 › 2 측정 › 3 비교 › 4 저장 › 5 0° 확인.
측정은 EEPROM에 쓰지 않는다(measure_only). 저장·0° 확인은 CLI `calibrate --save`·`--verify-zero`와 같은 엔진(#41, 결정 0029).
실제 팔에서만 켜지고, 시뮬레이션에서는 단계 설명만 보인다."""

import threading
import time

from nicegui import run, ui

from ...core.types import JointState
from .common import Controls, scene_frame

MIN_VOLT = 11.0  # 측정 엔진 기준과 같다(결정 0028, 0043-3)
STEPS = ("1 준비", "2 측정", "3 비교", "4 저장", "5 0° 확인")
CHECKS = ("팔 받침을 클램프로 고정했다", "팔 주변 반경 50cm를 비웠다", "12V 전원 스위치 옆에 사람이 있다")
GUIDE = {
    0: "팔을 쉬는 자세(윗팔 뒤로, 아래팔 접고, 그리퍼가 받침 앞에서 아래로 늘어지게)로 두고 아래 세 가지를 확인하세요. 측정은 관절을 하나씩 끝까지 밀어 범위와 0° 위치를 잽니다.",
    1: "측정 중에는 팔이 관절마다 끝까지 움직입니다. 이상하면 [중단]을 누르세요. 엔진이 토크 한계와 위치를 되돌린 뒤 다시 연결합니다. 전압이 11.0V 아래로 떨어지면 스스로 멈춥니다.",
    2: "기존 캘리브레이션과 관절별 범위·0° 위치 차이를 봅니다. 모든 관절이 ±3° 이내면 1회 합격, 아니면 24시간 안 측정 3회가 서로 1° 이내일 때 반복 합격(중앙값을 저장)입니다.",
    3: "서보 EEPROM, LeRobot 캐시, config/calibration 세 곳에 씁니다. 이전 파일은 .bak.json으로 남고, 저장 뒤에는 되돌리기가 없습니다. 저장하면 0° 확인 전까지 움직이는 명령이 잠깁니다.",
    4: "속도 20%로 모든 관절을 0°로 보냅니다. 0° 자세는 팔이 앞으로 곧게 뻗어 작업대 밖으로 나갑니다. 선 뒤 실물이 3D의 0° 자세와 같은지(어깨 정면, 손목 회전, 그리퍼) 보고 고르세요.",
}
LOG_LINES = 8
ANSWER_WAIT_S = 600  # 0° 자세에서 [맞음]·[다름]을 기다리는 시간


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id, block_during_task=True)
    state: dict = {"step": 0, "report": None, "reason": None, "lines": [], "busy": False, "stop": threading.Event(),
                   "saved": False, "zero": "before", "answer": None, "answered": threading.Event()}

    with ui.row().classes("w-full no-wrap gap-1").mark("calib-steps"):
        chips = [ui.label(text).classes("progress-step flex-1").mark(f"calib-step-{i}") for i, text in enumerate(STEPS)]
    guide = ui.label("").classes("page-body").mark("calib-guide")
    sim_note = ui.label("실제 팔이 연결됐을 때만 켜집니다. 시뮬레이션에서는 단계 설명만 볼 수 있습니다").classes("page-body").mark("calib-sim")

    # 1 준비
    with ui.card().classes("w-full gap-2").mark("calib-prepare") as prepare:
        boxes = [ui.checkbox(text).mark(f"calib-check-{i}") for i, text in enumerate(CHECKS)]
        why = ui.label("").classes("page-body").mark("calib-why")
        with ui.row().classes("gap-2"):
            next_button = controls.add(ui.button("다음: 측정", on_click=lambda: go(1)).props("no-caps").mark("calib-next"))
            zero_jump = controls.add(ui.button("0° 확인만 하기", on_click=lambda: go(4)).props("outline no-caps").mark("calib-zero-only"))

    # 2 측정
    with ui.card().classes("w-full gap-2").mark("calib-measure") as measure:
        with ui.row().classes("items-center gap-2"):
            start_button = controls.add(ui.button("측정 시작", on_click=controls.guard(lambda: start())).props("no-caps").mark("calib-start"))
            ui.button("중단", color="negative", on_click=lambda: state["stop"].set()).props("outline no-caps").mark("calib-stop")
        now = ui.label("").classes("page-body").mark("calib-now")

    # 3 비교
    with ui.card().classes("w-full gap-2").mark("calib-compare") as compare:
        verdict = ui.label("").classes("card-title").mark("calib-verdict")
        repeat_line = ui.label("").classes("page-body").mark("calib-repeat")
        table = ui.column().classes("gap-0 w-full").mark("calib-table")
        with ui.row().classes("gap-2"):
            save_next = controls.add(ui.button("다음: 저장", on_click=lambda: go_save()).props("no-caps").mark("calib-to-save"))
            ui.button("다시 측정", on_click=lambda: go(1)).props("outline no-caps").mark("calib-again")
            ui.button("처음으로", on_click=lambda: go(0)).props("outline no-caps").mark("calib-restart")

    # 4 저장
    with ui.card().classes("w-full gap-2").mark("calib-save") as save_card:
        save_table = ui.column().classes("gap-0 w-full").mark("calib-save-table")
        ui.label("위 표대로 바꿉니다. 맞으면 yes를 입력하고 [저장]을 누르세요").classes("page-body")
        with ui.row().classes("items-center gap-2"):
            yes_input = ui.input("yes 입력").props("dense outlined").mark("calib-yes")
            save_button = controls.add(ui.button("저장", on_click=controls.guard(lambda: save())).props("no-caps").mark("calib-save-run"))
            ui.button("저장하지 않고 끝", on_click=lambda: go(0)).props("outline no-caps").mark("calib-save-cancel")

    # 5 0° 확인
    with ui.card().classes("w-full gap-2").mark("calib-zero") as zero_card:
        # 처음 보는 사람은 0° 자세를 모른다(사용자 11-14): 계산 자세(보라 반투명)로 0° 자세를 함께 보인다
        ui.label("3D의 보라색 반투명 팔이 0° 자세입니다(어깨 정면, 팔이 앞으로 곧게, 손목 회전 0, 그리퍼 닫힘). 진한 팔이 지금 실물 자세입니다. 두 팔이 겹치면 [맞음]입니다").classes("page-body").mark("calib-zero-help")
        scene_frame(backend, classes="scene3d scene3d-short")
        zero_now = ui.label("").classes("page-body").mark("calib-zero-now")
        with ui.row().classes("gap-2"):
            zero_start = controls.add(ui.button("0° 자세로 이동", on_click=controls.guard(lambda: verify())).props("no-caps").mark("calib-zero-move"))
            same = ui.button("맞음", on_click=lambda: answer("yes")).props("no-caps").mark("calib-zero-yes")
            differ = ui.button("다름", color="negative", on_click=lambda: answer("no")).props("outline no-caps").mark("calib-zero-no")
            ui.button("중단", color="negative", on_click=lambda: stop_zero()).props("outline no-caps").mark("calib-zero-stop")
            redo = ui.button("캘리브레이션 다시 하기", on_click=lambda: go(0)).props("outline no-caps").mark("calib-zero-redo")
            lower = ui.button("홈으로 내리기", on_click=lambda: lower_home()).props("outline no-caps").mark("calib-zero-home")  # 0° 확인 잠금 중에도 켠다(controls.add는 잠금에 꺼진다)
    for hidden in (same, differ, redo, lower, measure, compare, save_card, zero_card):  # 첫 tick 전에 눌리지 않게(0° 이동 전 [맞음] 등)
        hidden.visible = False

    log = ui.column().classes("gap-0 w-full").mark("calib-log")

    def go(step: int) -> None:
        state["step"] = step
        if step == 4:
            state["zero"] = "before"
        backend.scene.set_preview(JointState(0, 0, 0, 0, 0, 0) if step == 4 else None)

    def out(text) -> None:
        state["lines"] = (state["lines"] + [line for line in str(text).splitlines() if line.strip()])[-200:]

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

    def passed(rep) -> bool:
        return bool(rep and (rep.get("passed") or rep.get("repeat", {}).get("passed")))

    async def start() -> None:
        state.update(busy=True, lines=[], report=None, reason=None)
        state["stop"].clear()
        try:
            state["report"], state["reason"] = await run.io_bound(backend.calibration_measure, out, state["stop"].is_set)
        finally:
            state["busy"] = False
        if state["report"] is not None:
            go(2)
        elif state["reason"]:
            ui.notify(state["reason"], type="negative", timeout=0, close_button="닫기")

    async def save() -> None:
        state.update(busy=True, lines=[])
        try:
            ok, reason = await run.io_bound(backend.calibration_save, yes_input.value or "", out)
        finally:
            state["busy"] = False
        if ok:
            state["saved"] = True
            ui.notify("저장했습니다. 이제 0° 자세를 확인하세요", type="positive")
            go(4)
        if reason:
            ui.notify(reason, type="negative", timeout=0, close_button="닫기")

    def answer(text: str) -> None:
        state["answer"] = text
        state["answered"].set()

    async def lower_home() -> None:
        reason = await run.io_bound(backend.calibration_home)
        ui.notify(reason or "홈으로 내렸습니다", type="negative" if reason else "positive")

    def stop_zero() -> None:
        """0°로 가는 중이면 세운다(그러면 엔진이 되돌아가지 못한다). 0°에 선 뒤에는 세우지 않고 [다름]처럼 기록하지 않고 되돌아가게 한다."""
        if state["zero"] == "moving":
            backend.stop_all()
        answer("no")

    def wait_answer() -> str:
        state["zero"] = "asking"
        end = time.monotonic() + ANSWER_WAIT_S
        while not state["answered"].wait(0.2):
            if backend.real is None or time.monotonic() > end:  # 화면을 닫고 떠나도 스레드가 남지 않게: 답이 없으면 기록하지 않는다
                return "no"
        return state["answer"] or "no"

    async def verify() -> None:
        state.update(busy=True, lines=[], zero="moving", answer=None)
        state["answered"].clear()
        try:
            ok, reason = await run.io_bound(backend.calibration_verify_zero, wait_answer, out)
        finally:
            state["busy"] = False
        state["zero"] = "checked" if ok else "refused"
        if ok:
            ui.notify("0° 자세 확인을 기록했습니다. 움직이는 명령이 풀렸습니다", type="positive")
        if reason:
            ui.notify(reason, type="negative" if not ok else "warning", timeout=0, close_button="닫기")

    shown: list = [None]

    def tick() -> None:
        controls.refresh()
        step = state["step"]
        for i, chip in enumerate(chips):
            chip.classes(add="progress-active" if i == step else "", remove="" if i == step else "progress-active")
        guide.text = GUIDE.get(step, "")
        sim_note.visible = backend.real is None
        prepare.visible, measure.visible, compare.visible = step == 0, step == 1, step == 2
        save_card.visible, zero_card.visible = step == 3, step == 4
        log.visible = step in (1, 3, 4)
        if step == 0:
            reason = ready_reason()
            why.text = reason or "준비됐습니다"
            next_button.enabled = zero_jump.enabled = reason is None
        elif step == 1:
            start_button.enabled = not state["busy"] and backend.real is not None and controls.mine
            step_line = next((line for line in reversed(state["lines"]) if line.startswith("측정 중(")), None)  # 엔진이 단계마다 보내는 "측정 중(i/n): 관절"(REQ-052)
            now.text = (step_line or "측정 중...") if state["busy"] else (state["reason"] or "측정 전")
        elif step == 2:
            save_next.enabled = passed(state["report"]) and controls.mine
        elif step == 3:
            save_button.enabled = not state["busy"] and (yes_input.value or "").strip().lower() == "yes" and controls.mine
        elif step == 4:
            z = state["zero"]
            zero_start.enabled = z in ("before", "refused") and not state["busy"] and backend.real is not None and controls.mine
            same.visible = differ.visible = z == "asking"
            redo.visible = lower.visible = z == "refused"  # 되돌아가지 못했으면(정지·E-STOP 뒤) 잠긴 채라 여기서만 홈으로 내린다
            lower.enabled = controls.mine and not state["busy"]
            zero_now.text = {"before": "0° 자세로 이동을 누르세요", "moving": "0° 자세로 가는 중...", "asking": "실물이 3D의 0° 자세와 같나요?",
                             "checked": "확인을 기록했습니다. 움직이는 명령이 풀렸습니다",
                             "refused": "기록하지 않았습니다(알림에 팔이 어디로 돌아갔는지 나옵니다). 잠금이 유지됩니다. 어긋나면 캘리브레이션을 다시 하세요"}[z]
        if step in (1, 3, 4) and state["lines"] != shown[0]:
            shown[0] = list(state["lines"])
            log.clear()
            with log:
                for line in state["lines"][-LOG_LINES:]:
                    ui.label(line).classes("event-line")
        if step == 2 and state["report"] is not None and shown[0] != ("report", id(state["report"])):
            shown[0] = ("report", id(state["report"]))
            rep = state["report"]
            rep_status = rep.get("repeat") or {}
            verdict.text = (f"1회 합격: 모든 관절이 ±{rep.get('pass_deg', 3):g}° 이내" if rep.get("passed")
                            else "반복 합격: 최근 3회의 중앙값을 저장합니다" if rep_status.get("passed")
                            else "합격 아님: 기존 값과 다르거나 비교할 기존 파일이 없습니다. 측정을 더 해 3회 반복 합격을 보세요")
            repeat_line.text = f"반복 {min(rep_status.get('count', 0), 3)}/3, 최대 차이 {rep_status.get('max_diff_deg', 0):.2f}°" if rep_status else ""
            fill_rows(table, rep.get("rows", []), "calib-row")
            if rep.get("run_dir"):
                with table:
                    ui.label(f"기록: {rep['run_dir']}/measure.json").classes("page-body")

    def fill_rows(box, rows: list[dict], mark: str) -> None:
        box.clear()
        with box:
            ui.label("관절 | 기존 범위 | 새 범위 | 범위 차이 | 0° 차이 | 판정").classes("stat-label")
            for r in rows:
                num = lambda v: "-" if v is None else f"{v:.1f}"
                ok = "-" if r.get("ok") is None else ("합격" if r["ok"] else "벗어남")
                ui.label(f"{r['joint']} | {num(r.get('old_range_deg'))} | {num(r.get('new_range_deg'))} | {num(r.get('range_diff_deg'))} | "
                         f"{num(r.get('zero_diff_deg'))} | {ok}").classes("event-line").mark(f"{mark}-{r['joint']}")

    def go_save() -> None:
        """4단계로: 1회 합격이면 3단계 표 그대로, 반복 합격이면 저장될 중앙값의 차이표(엔진이 같은 규칙으로 고른다)."""
        rep = state["report"] or {}
        rows = rep.get("rows", []) if rep.get("passed") else (backend.calibration_repeat_rows() or [])
        fill_rows(save_table, rows, "calib-save-row")
        go(3)

    ui.timer(0.2, tick)
