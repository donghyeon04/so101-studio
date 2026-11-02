"""시스템 > 실행 기록 페이지 (gui-spec 3절): runs/ 목록, 이벤트 시간줄, 전압·온도 그래프. 3D 다시 보기는 자리만 있다."""

from datetime import datetime

from nicegui import run, ui

from .. import theme


def line_options(title: str, unit: str, points: list, color: str) -> dict:
    """계열 하나짜리 시간 그래프. 축을 나누지 않으려고 전압과 온도를 따로 그린다."""
    axis = {"axisLine": {"lineStyle": {"color": theme.CONTROL_BORDER}}, "axisLabel": {"color": theme.MUTED},
            "splitLine": {"lineStyle": {"color": theme.LINE}}, "nameTextStyle": {"color": theme.MUTED}}
    return {
        "animation": False,
        "title": {"text": title, "textStyle": {"fontSize": 14, "color": theme.TEXT}},
        "tooltip": {"trigger": "axis"},
        "grid": {"left": 48, "right": 16, "top": 36, "bottom": 28},
        "xAxis": {"type": "value", "name": "초", "min": "dataMin", "max": "dataMax", **axis},
        "yAxis": {"type": "value", "name": unit, "scale": True, **axis},
        "series": [{"name": title, "type": "line", "showSymbol": False, "lineStyle": {"width": 2}, "color": color, "data": points}],
    }


def build(backend, client_id: str, actions) -> None:
    state: dict = {"data": None}

    @ui.refreshable
    def detail() -> None:
        data = state["data"]
        if data is None:
            ui.label("왼쪽에서 실행을 고르세요").classes("page-body")
            return
        ui.label(f"{data['name']} · 이벤트 {len(data['events'])}개 · 상태 {data['rows']}줄 · {data['duration_s']:.0f}초").classes("page-body").mark("run-summary")
        if data["voltage"]:
            ui.echart(line_options("전압", "V", data["voltage"], theme.SERIES[0])).classes("w-full run-chart").mark("run-voltage")
            ui.echart(line_options("최고 온도", "°C", data["temperature"], theme.SERIES[1])).classes("w-full run-chart").mark("run-temperature")
        else:
            ui.label("이 실행에는 상태 기록이 없습니다").classes("page-body").mark("run-empty")
        with ui.card().classes("w-full gap-1"):
            ui.label("이벤트 시간줄").classes("card-title")
            with ui.scroll_area().classes("w-full run-events").mark("run-events"):
                for event in data["events"][:500]:
                    ui.label(f"{datetime.fromtimestamp(event['t']):%H:%M:%S}  {event['kind']}  {event['text']}").classes("event-line")

    async def pick(name: str | None) -> None:
        if name:
            state["data"] = await run.io_bound(backend.load_run, name)
        detail.refresh()

    with actions:
        ui.label("3D 다시 보기는 추가 예정입니다").classes("page-body").mark("replay-3d")
        ui.button("새로고침", on_click=lambda: setattr(listing, "options", backend.run_names()) or listing.update()).props("outline no-caps").mark("runs-reload")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("run-list gap-2"):
            names = backend.run_names()
            ui.label(f"실행 {len(names)}개").classes("card-title")
            listing = ui.select(names, label="실행", on_change=lambda e: pick(e.value)).props("outlined dense").classes("w-full").mark("run-select")
        with ui.column().classes("flex-1 min-w-0 gap-3"):
            detail()
