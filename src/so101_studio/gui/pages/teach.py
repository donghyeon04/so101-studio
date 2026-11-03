"""교시 페이지 (gui-spec 3절): 기록 목록, 구간 막대와 구간별 배속, 전체 배속, 재생, 손끝 궤적, 관절-시간 그래프."""

import asyncio

from nicegui import run, ui

from .. import theme
from ..backend import JOINT_TEXT
from .common import Controls, scene_frame

SEGMENT_FACTORS = {0.5: "0.5", 1.0: "1", 2.0: "2"}
RECORD_STEPS = ("홈으로 이동", "팔 잡기", "카운트다운", "기록")
COUNTDOWN_S = 3


def expected_seconds(segments: list[dict], factors: dict[int, float], speed: float) -> float:
    """구간별 배율과 전체 배속을 적용한 재생 시간."""
    return sum((s["end_s"] - s["start_s"]) / factors.get(s["index"], 1.0) for s in segments) / speed


def chart_options(data: dict) -> dict:
    series = [
        {
            "name": JOINT_TEXT[name], "type": "line", "showSymbol": False, "lineStyle": {"width": 2},
            "color": theme.SERIES[i], "data": [[t, q[i]] for t, q in zip(data["t"], data["q"], strict=True)],
        }
        for i, name in enumerate(JOINT_TEXT)
    ]
    axis = {"axisLine": {"lineStyle": {"color": theme.CONTROL_BORDER}}, "axisLabel": {"color": theme.MUTED},
            "splitLine": {"lineStyle": {"color": theme.LINE}}, "nameTextStyle": {"color": theme.MUTED}}
    return {
        "animation": False,
        "tooltip": {"trigger": "axis"},
        "legend": {"top": 0, "textStyle": {"color": theme.TEXT}},
        "grid": {"left": 48, "right": 16, "top": 32, "bottom": 32},
        "xAxis": {"type": "value", "name": "초", "min": "dataMin", "max": "dataMax", **axis},
        "yAxis": {"type": "value", "name": "°", **axis},
        "series": series,
    }


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id, moves=True)
    state: dict = {"data": None, "factors": {}, "counting": False}

    async def play() -> None:
        data = state["data"]
        if data is None:
            return
        reason = await run.io_bound(
            backend.replay, select.value, float(speed.value), bool(until_limit.value), dict(state["factors"]) or None
        )
        ui.notify(reason or backend.replay_cut() or "재생을 마쳤습니다", type="negative" if reason else "positive")

    with actions:
        ui.button("정지", color="negative", on_click=lambda: run.io_bound(backend.stop_all)).props("outline no-caps").mark("stop")
        play_button = controls.add(ui.button("재생", on_click=controls.guard(play)).props("no-caps").mark("play"))

    def update_expected() -> None:
        if state["data"]:
            expected.text = f"재생 시간 약 {expected_seconds(state['data']['info']['segments'], state['factors'], float(speed.value)):.1f}초"

    @ui.refreshable
    def segment_panel() -> None:
        data = state["data"]
        if data is None:
            ui.label("기록을 고르세요").classes("page-body")
            return
        segments = data["info"]["segments"]
        with ui.row().classes("segment-bar w-full no-wrap").mark("segment-bar"):
            for s in segments:
                width = max(s["end_s"] - s["start_s"], 0.1)
                ui.label(s["label"]).classes("segment").style(
                    f"flex: {width} 1 0; background: {theme.SEGMENT_COLORS[s['label']]}; color: {theme.SEGMENT_INK[s['label']]}"
                ).tooltip(f"구간 {s['index'] + 1}: {width:.1f}초, 평균 {s['mean_deg_s']:.0f}°/s")
        for s in segments:
            with ui.row().classes("w-full no-wrap items-center gap-2"):
                ui.label(f"{s['index'] + 1}. {s['label']} {s['end_s'] - s['start_s']:.1f}초").classes("flex-1")

                def changed(e, index=s["index"]) -> None:
                    if e.value == 1.0:
                        state["factors"].pop(index, None)
                    else:
                        state["factors"][index] = e.value
                    update_expected()

                controls.add(
                    ui.toggle(SEGMENT_FACTORS, value=state["factors"].get(s["index"], 1.0), on_change=changed)
                    .props("no-caps dense").mark(f"segment-{s['index']}")
                )

    @ui.refreshable
    def chart_panel() -> None:
        if state["data"]:
            ui.echart(chart_options(state["data"])).classes("w-full joint-chart").mark("joint-chart")

    async def load(name: str | None) -> None:
        state["data"], state["factors"] = None, {}
        expected.text = ""
        if name:
            summary.text = "불러오는 중"
            state["data"] = await run.io_bound(backend.sequence, name)
            info = state["data"]["info"]
            summary.text = f"{info['duration_s']:.1f}초 · 표본 {info['points']}개 · 최대 {info['max_deg_s']:.0f}°/s · 구간 {len(info['segments'])}개"
            update_expected()
        segment_panel.refresh()
        chart_panel.refresh()

    async def record_start_pressed() -> None:
        problem = backend.record_name_problem(record_name.value)
        if problem:
            ui.notify(problem, type="warning")
            return
        home_dialog.open()

    async def record_go_home() -> None:
        home_dialog.close()
        record_note.text = "홈으로 이동 중"
        reason = await run.io_bound(backend.record_prepare)
        if reason:
            record_note.text = ""
            ui.notify(reason, type="negative")
            return
        record_note.text = ""
        hold_dialog.open()

    async def record_count_down() -> None:
        hold_dialog.close()
        state["counting"] = True
        for left in range(COUNTDOWN_S, 0, -1):
            record_note.text = f"{left}초 뒤 토크가 꺼집니다. 팔을 잡고 계세요"
            await asyncio.sleep(1)
            if not state["counting"]:
                record_note.text = "기록을 취소했습니다"
                return
        state["counting"] = False
        reason = backend.record_begin(record_name.value)
        record_note.text = "토크가 꺼졌습니다. 손으로 천천히 움직이세요" if not reason else ""
        if reason:
            ui.notify(reason, type="negative")

    async def record_end() -> None:
        name = await run.io_bound(backend.record_finish)
        if name is None:
            return
        cut = backend.record_truncated()
        record_note.text = f"최대 {cut:.0f}초라 앞부분만 저장했습니다" if cut else ""
        ui.notify(f"저장했습니다: {name}", type="positive")
        record_name.value = ""
        select.set_options(backend.sequences(), value=name)

    def record_cancel() -> None:
        state["counting"] = False

    def confirm_dialog(title: str, text: str, go, go_mark: str, cancel_mark: str) -> ui.dialog:
        with ui.dialog() as dialog, ui.card().classes("gap-3"):
            ui.label(title).classes("card-title")
            ui.label(text).classes("page-body")
            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("취소", on_click=dialog.close).props("outline no-caps").mark(cancel_mark)
                ui.button("시작", color="negative", on_click=controls.guard(go)).props("no-caps").mark(go_mark)
        dialog.props("persistent")
        return dialog

    home_dialog = confirm_dialog("새 기록", "팔이 홈 자세로 움직입니다. 주변을 비우고 전원 스위치 옆에서 지켜보세요.", record_go_home, "record-home-go", "record-home-cancel")
    hold_dialog = confirm_dialog("팔을 손으로 잡으세요", f"시작을 누르면 {COUNTDOWN_S}초 뒤 토크가 꺼져 팔이 처집니다. 팔을 단단히 잡은 뒤에 누르세요.", record_count_down, "record-hold-go", "record-hold-cancel")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-3"):
            scene_frame(backend, "scene3d scene3d-short")
            chart_panel()

        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("기록").classes("card-title")
                select = ui.select(backend.sequences(), label="기록", on_change=lambda e: load(e.value)).props("outlined dense").classes("w-full").mark("sequence")
                summary = ui.label("").classes("page-body").mark("summary")
            with ui.card().classes("w-full gap-2"):
                ui.label("구간별 배속").classes("card-title")
                segment_panel()
            with ui.card().classes("w-full gap-2"):
                ui.label("재생").classes("card-title")
                speed = controls.add(
                    ui.slider(min=0.25, max=3, step=0.25, value=1.0, on_change=lambda: update_expected())
                    .props("label-always").classes("mt-6").mark("replay-speed")
                )
                until_limit = controls.add(ui.checkbox("한계 앞까지만 재생").mark("until-limit"))
                expected = ui.label("").classes("page-body").mark("expected")
            with ui.card().classes("w-full gap-2"):
                ui.label("새 기록").classes("card-title")
                ui.label(" → ".join(RECORD_STEPS)).classes("page-body")
                record_name = controls.add(ui.input("기록 이름").props("outlined dense").classes("w-full").mark("record-name"))
                record_note = ui.label("").classes("page-body").mark("record-note")
                record_button = ui.button(
                    "새 기록 시작" if backend.real else "실제 팔에서만 기록할 수 있습니다", on_click=controls.guard(record_start_pressed)
                ).props("outline no-caps").classes("w-full").mark("record")
                # 기록을 끝내는 것은 토크를 다시 켜는 일이라 조작권 검사로 막지 않는다
                record_stop_button = ui.button("기록 끝내고 저장", color="negative", on_click=record_end).props("no-caps").classes("w-full").mark("record-stop")
                record_cancel_button = ui.button("취소", on_click=record_cancel).props("outline no-caps").classes("w-full").mark("record-cancel")
                record_stop_button.visible = record_cancel_button.visible = False
                record_button.enabled = False  # tick이 판단한다. 그 전에 눌리지 않게 꺼 둔다

    def tick() -> None:
        controls.refresh()
        recording = backend.recording()
        idle = bool(backend.state.status and backend.state.status.state == "idle")
        busy = recording or state["counting"]
        record_button.enabled = bool(backend.real and controls.allowed and idle and not busy)
        record_button.visible = not busy
        record_stop_button.visible = recording
        record_cancel_button.visible = state["counting"]
        play_button.enabled = bool(controls.allowed and state["data"] and backend.state.status and backend.state.status.state == "idle")

    ui.timer(0.2, tick)
