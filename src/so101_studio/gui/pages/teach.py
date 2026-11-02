"""교시 페이지 (gui-spec 3절): 기록 목록, 구간 막대와 구간별 배속, 전체 배속, 재생, 손끝 궤적, 관절-시간 그래프."""

from nicegui import run, ui

from .. import theme
from ..backend import JOINT_TEXT
from .common import Controls, scene_frame

SEGMENT_FACTORS = {0.5: "0.5", 1.0: "1", 2.0: "2"}
RECORD_STEPS = ("홈으로 이동", "팔 잡기", "카운트다운", "기록")


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
    controls = Controls(backend, client_id)
    state: dict = {"data": None, "factors": {}}

    async def play() -> None:
        data = state["data"]
        if data is None:
            return
        reason = await run.io_bound(
            backend.replay, select.value, float(speed.value), bool(until_limit.value), dict(state["factors"]) or None
        )
        ui.notify(reason or backend.replay_cut() or "재생을 마쳤습니다", type="negative" if reason else "positive")

    with actions:
        ui.button("정지", color="negative", on_click=lambda: run.io_bound(backend.arm.stop)).props("outline no-caps").mark("stop")
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
                # shortcut: 토크를 끄고 손으로 움직이는 흐름이라 시뮬레이션에서는 시험할 수 없다, G10에서 실제 팔로 붙인다
                ui.button("실제 팔에서만 기록할 수 있습니다").props("outline no-caps").classes("w-full").mark("record").disable()

    def tick() -> None:
        controls.refresh()
        play_button.enabled = bool(controls.allowed and state["data"] and backend.state.status and backend.state.status.state == "idle")

    ui.timer(0.2, tick)
