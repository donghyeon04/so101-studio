"""시스템 > 로그 페이지 (gui-spec 3절): 수준 필터, 검색, 실시간 표시."""

from datetime import datetime

from nicegui import ui

LEVELS = {"INFO": "모두", "WARNING": "경고 이상", "ERROR": "오류만"}
SHOWN = 200
ORDER = {"INFO": 0, "WARNING": 1, "ERROR": 2}


def matches(record, level: str, text: str) -> bool:
    return ORDER[record.level] >= ORDER[level] and text.lower() in record.text.lower()


def build(backend, client_id: str, actions) -> None:
    with actions:
        live = ui.checkbox("실시간", value=True).mark("log-live")

    with ui.row().classes("w-full no-wrap items-center gap-3"):
        level = ui.toggle(LEVELS, value="INFO").props("no-caps").mark("log-level")
        search = ui.input("검색").props("outlined dense clearable").classes("flex-1").mark("log-search")

    count = ui.label("").classes("page-body").mark("log-count")
    with ui.scroll_area().classes("w-full log-area").mark("log-area"):
        lines = ui.column().classes("w-full gap-0").mark("log-lines")

    previous: list = [None]

    def tick(force: bool = False) -> None:
        if not (live.value or force):
            return
        records = [r for r in backend.logs() if matches(r, level.value, search.value or "")]
        key = (len(records), records[-1].t if records else 0, level.value, search.value)
        if key == previous[0]:
            return
        previous[0] = key
        count.text = f"{len(records)}줄" + (f" 중 최근 {SHOWN}줄" if len(records) > SHOWN else "")
        lines.clear()
        with lines:
            for r in records[-SHOWN:][::-1]:
                classes = "log-line" + (" log-error" if r.level == "ERROR" else " log-warning" if r.level == "WARNING" else "")
                ui.label(f"{datetime.fromtimestamp(r.t):%H:%M:%S}  {r.level:<7}  {r.text}").classes(classes)

    level.on_value_change(lambda: tick(True))
    search.on_value_change(lambda: tick(True))
    tick(True)
    ui.timer(0.5, tick)
