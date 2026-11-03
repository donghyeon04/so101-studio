"""시스템 > 실행 기록 페이지 (gui-spec 3절, decisions/0030): runs/ 목록, 이벤트 시간줄, 전압·온도 그래프, 토크·상태·오류 띠, 사고 묶음. 3D 다시 보기는 자리만 있다."""

from datetime import datetime

from nicegui import run, ui

from .. import theme

BAND_COLORS = {"켜짐": theme.OK, "꺼짐": theme.MUTED, "대기": theme.MUTED, "실행 중": theme.ACCENT, "정지": theme.WARN, "오류": theme.DANGER}


def band_options(bands: dict, duration: float, mark: float | None = None) -> dict:
    """토크·상태·오류를 굵은 선 띠로 그린다. 값마다 계열을 나눠 범례와 툴팁에 글자가 보인다."""
    names = list(bands)
    series: dict[str, list] = {}
    for name, spans in bands.items():
        for label, start, end in spans:
            # 짧은 상태도 보이게 최소 길이를 준다. null로 띠를 끊는다
            series.setdefault(label, []).extend([[start, name], [max(end, start + duration / 200), name], None])
    out = [{"name": label, "type": "line", "showSymbol": False, "connectNulls": False, "lineStyle": {"width": 14, "cap": "butt"},
            "color": BAND_COLORS.get(label, theme.DANGER), "data": data} for label, data in series.items()]
    if mark is not None and out:
        out[0]["markLine"] = _mark_line(mark)
    return {
        "animation": False, "tooltip": {"trigger": "item", "formatter": "{a}"}, "legend": {"top": 0, "textStyle": {"color": theme.TEXT}},
        "grid": {"left": 48, "right": 16, "top": 30, "bottom": 24},
        "xAxis": {"type": "value", "min": 0, "max": max(duration, 1.0), "axisLabel": {"color": theme.MUTED}, "splitLine": {"show": False}},
        "yAxis": {"type": "category", "data": names, "axisLabel": {"color": theme.TEXT}},
        "series": out,
    }


def _mark_line(t: float) -> dict:
    return {"symbol": "none", "silent": True, "lineStyle": {"color": theme.TEXT, "type": "dashed"}, "label": {"show": False}, "data": [{"xAxis": t}]}


def line_options(title: str, unit: str, points: list, color: str, mark: float | None = None) -> dict:
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
        "series": [{"name": title, "type": "line", "showSymbol": False, "lineStyle": {"width": 2}, "color": color, "data": points, **({"markLine": _mark_line(mark)} if mark is not None else {})}],
    }


def build(backend, client_id: str, actions) -> None:
    state: dict = {"data": None, "mark": None}
    focus, backend.run_focus = backend.run_focus, None

    @ui.refreshable
    def detail() -> None:
        data = state["data"]
        if data is None:
            ui.label("왼쪽에서 실행을 고르세요").classes("page-body")
            return
        ui.label(f"{data['name']} · 이벤트 {len(data['events'])}개 · 상태 {data['rows']}줄 · {data['duration_s']:.0f}초").classes("page-body").mark("run-summary")
        mark = None if state["mark"] is None else round(state["mark"] - data["t0"], 2)
        if any(data["bands"].values()):
            ui.echart(band_options(data["bands"], data["duration_s"], mark)).classes("w-full run-bands").mark("run-bands")
        elif data["voltage"]:
            ui.label("옛 형식 기록이라 토크·상태·오류 띠가 없습니다").classes("page-body").mark("run-no-bands")
        if data["voltage"]:
            ui.echart(line_options("전압", "V", data["voltage"], theme.SERIES[0], mark)).classes("w-full run-chart").mark("run-voltage")
            ui.echart(line_options("최고 온도", "°C", data["temperature"], theme.SERIES[1], mark)).classes("w-full run-chart").mark("run-temperature")
        else:
            ui.label("이 실행에는 상태 기록이 없습니다").classes("page-body").mark("run-empty")
        with ui.card().classes("w-full gap-1"):
            ui.label("이벤트 시간줄").classes("card-title")
            with ui.scroll_area().classes("w-full run-events").mark("run-events"):
                for event in data["events"][:500]:
                    code = f"{event['code']}  " if event.get("code") else ""
                    ui.label(f"{datetime.fromtimestamp(event['t']):%H:%M:%S}  {code}{event['kind']}  {event['text']}").classes("event-line log-link").on(
                        "click", lambda t=event["t"]: show_mark(t)
                    )

    def show_mark(t: float) -> None:
        state["mark"] = t
        detail.refresh()

    @ui.refreshable
    def incident_list() -> None:
        items = backend.incidents()
        if not items:
            ui.label("사고 묶음이 없습니다").classes("page-body").mark("incident-empty")
        for item in items[:20]:
            with ui.row().classes("w-full items-center no-wrap gap-2"):
                ui.icon("report", color="negative")
                ui.label(f"{item['name']} {item.get('reason') or ''}").classes("page-body flex-1 min-w-0").mark("incident-name")
                ui.button(icon="download", on_click=lambda name=item["name"]: download(name)).props('flat round dense aria-label="zip 내려받기"').mark("incident-zip")

    async def download(name: str) -> None:
        ui.download(await run.io_bound(backend.incident_zip, name))

    async def make_incident() -> None:
        path = await run.io_bound(backend.make_incident)
        if path:
            ui.notify("사고 묶음을 만들었습니다. 뒤 60초 기록은 시간이 지나야 채워집니다", type="positive")
        else:
            ui.notify("기록 중이 아니라 만들 수 없습니다", type="warning")
        incident_list.refresh()

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
            with ui.card().classes("w-full gap-2").mark("incident-card"):
                with ui.row().classes("w-full items-center"):
                    ui.label("사고 묶음").classes("card-title")
                    ui.space()
                    ui.button("지금 사고 묶음 만들기", on_click=make_incident).props("outline no-caps").mark("incident-make")
                incident_list()

    if focus:  # 로그에서 고른 줄로 왔다
        state["mark"] = focus[1]
        listing.value = focus[0]
