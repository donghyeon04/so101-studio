"""초음파 화면 (REQ-024, ultrasonic-spec 8.2절 이벤트): 레이더 부채꼴·각도별 값·후보·지운 각도, 확인 경로와 높이 점, 결과, 스캔 그래프(E2).
화면은 us.* 이벤트만 보고 그린다. 지금은 가상 ESP32와 가상 팔을 돌려 이벤트를 만든다."""

import copy
import html
import logging
import tempfile
import threading
import time
from pathlib import Path
from weakref import WeakKeyDictionary

from nicegui import ui

from ...core.events import EventBus
from ...perception.ultrasonic import benchmark, radar
from ...perception.ultrasonic.fake_esp32 import SimClock
from ...perception.ultrasonic.link import Link
from ...perception.ultrasonic.model import Box, load_config
from ...perception.ultrasonic.source import UltrasonicSource
from .. import theme
from ..pages.common import Controls
from .manual_view import plane_svg

TOPICS = ("us.sweep", "us.candidates", "us.check_path", "us.check_result", "us.sweep_progress", "us.check_estimate")  # 뒤 둘: gui-spec 12.11·8.9
DEMO_SPEED = 3.0  # 가상 시간을 이만큼 빠르게 돌린다
STATUS_TEXT = {"ok": "확정", "empty": "물체 없음", "unreachable": "못 감"}


class UltrasonicLog:
    """이벤트 스레드에서 받아 화면 타이머가 읽는 상태. 그릴 때는 snapshot()으로 복사본을 쓴다."""

    def __init__(self):
        self._lock = threading.Lock()
        self.clear()

    def clear(self) -> None:
        with self._lock:
            self.version = getattr(self, "version", 0) + 1
            self.sweep: dict | None = None
            self.candidates: list[dict] = []
            self.arcs: list[dict] = []
            self.masked: list = []
            self.paths: list[dict] = []
            self.results: list[dict] = []
            self.baseline: dict = {}
            self.passes: list[dict] = []  # 이번 실행의 바퀴(us.sweep) 목록
            self.progress: dict | None = None  # 이번 바퀴의 진행(us.sweep_progress 마지막 값)
            self.estimates: dict[str, list[dict]] = {}  # 후보 id -> 단계별 us.check_estimate(값이 None인 키도 늘 있다, 8.9)

    def attach(self, bus) -> None:
        for topic in TOPICS:
            bus.subscribe(topic, lambda e, t=topic: self.on_event(t, e))

    def on_event(self, topic: str, e: dict) -> None:
        """작업 스레드의 버스 콜백이다. 예외를 내지 않는다(올라가면 작업 실행이 FAULT가 된다)."""
        try:
            self._on_event(topic, e)
        except Exception:  # 화면용 기록이 작업을 멈추면 안 된다
            logging.getLogger(__name__).exception("초음파 기록 실패: %s", topic)

    def _on_event(self, topic: str, e: dict) -> None:
        with self._lock:
            self.version += 1
            if topic == "us.sweep":
                self.sweep = e
                self.passes.append(e)
            elif topic == "us.sweep_progress":
                self.progress = e
            elif topic == "us.check_estimate":
                self.estimates.setdefault(str(e.get("id")), []).append(e)
            elif topic == "us.candidates":
                self.candidates, self.arcs, self.masked = e["objects"], e["arcs"], e["masked"]
            elif topic == "us.check_path":
                if e.get("phase") == "end" and self.paths and self.paths[-1]["id"] == e["id"] and self.paths[-1]["stage"] == e["stage"] and self.paths[-1].get("phase") == "start":
                    self.paths[-1] = e  # 시작에 그린 경로를 측정이 붙은 값으로 바꾼다
                else:
                    self.paths.append(e)
            elif topic == "us.check_result":
                self.results.append(e)

    def set_baseline(self, d: dict) -> None:
        with self._lock:
            self.version += 1
            self.baseline = dict(d)

    def snapshot(self) -> dict:
        with self._lock:
            return copy.deepcopy({"version": self.version, "sweep": self.sweep, "candidates": self.candidates, "arcs": self.arcs,
                                  "masked": self.masked, "paths": self.paths, "results": self.results, "baseline": self.baseline,
                                  "passes": self.passes, "progress": self.progress, "estimates": self.estimates})


LAYERS = ("바닥 원자료", "팔 원자료", "바닥 거른 데이터", "팔 후보별 원자료", "합치기", "최종 위치")  # 초음파 화면 겹(gui-spec 12.11 F7)
FOLLOW = {"idle": {"바닥 원자료"}, "scan": {"바닥 거른 데이터"}, "check": {"팔 원자료", "팔 후보별 원자료"},
          "check_done": {"합치기", "최종 위치"}, "pick": {"최종 위치"}}  # [단계 따라가기]: 흐름 단계 → 켤 겹
STAGE_COLS = (("L1", ("L1",)), ("L2", ("L2", "GRID")), ("원", ("R",)), ("긴 축·짧은 축", ("L34", "L3", "L4")))  # 타임라인 열 → 확인 단계 이름


def radar_overlay(cfg, snap: dict, table_h: float, hit_mm: float, layers=LAYERS) -> str:
    """plane_svg 위에 얹는 SVG. 좌표는 작업대 mm이고 y만 뒤집는다. layers = 그릴 겹 이름(LAYERS 중, 기본 전부)."""

    def xy(x: float, y: float) -> tuple[float, float]:
        return x, table_h - y

    def at(servo_deg: float, dist: float) -> tuple[float, float]:
        return xy(*radar.polar(cfg, servo_deg, dist))

    def ring(x, y, rad, color, dash="") -> str:
        cx, cy = xy(x, y)
        return f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{rad:.1f}" fill="none" stroke="{color}" stroke-width="2" stroke-dasharray="{dash}"/>'

    on, r, parts = set(layers), cfg.radar, []
    sx, sy = xy(r.station_mm.x, r.station_mm.y)
    fan = [at(a, r.max_mm) for a in range(r.sweep.from_deg, r.sweep.to_deg + 1, r.sweep.step_deg)]
    parts.append('<polygon points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in [(sx, sy), *fan]) + f'" fill="{theme.SENSOR}" fill-opacity="0.07" stroke="{theme.SENSOR}" stroke-opacity="0.4"/>')
    if "바닥 거른 데이터" in on:
        for a in snap["masked"]:
            x, y = at(a, r.max_mm)
            parts.append(f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{x:.1f}" y2="{y:.1f}" stroke="{theme.MUTED}" stroke-opacity="0.5" stroke-dasharray="4 6"/>')
    sweep = snap["sweep"]
    if sweep and "바닥 원자료" in on:
        if sweep["angles"]:
            x, y = at(sweep["angles"][-1], r.max_mm)
            parts.append(f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{x:.1f}" y2="{y:.1f}" stroke="{theme.SENSOR}" stroke-width="2"/>')
        for a, d in zip(sweep["angles"], sweep["d"], strict=True):
            if d is not None:
                x, y = at(a, d)
                parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{theme.SENSOR}" class="raw-floor"/>')
    parts.append(f'<rect x="{sx - 12:.1f}" y="{sy - 12:.1f}" width="24" height="24" fill="{theme.SENSOR}"/>')
    parts.append(f'<text x="{sx + 18:.1f}" y="{sy + 24:.1f}" font-size="18" fill="{theme.SENSOR}">레이더</text>')  # 레이더는 작업대 앞 변에 있어 글자는 안쪽으로
    if "바닥 거른 데이터" in on:
        for c in snap["candidates"]:
            x, y = xy(c["x_mm"], c["y_mm"])
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="14" fill="none" stroke="{theme.SENSOR}" stroke-width="3" stroke-dasharray="6 4"/>')
            if c.get("search_r_mm"):  # 예상 범위(팔이 찾아갈 곳)
                parts.append(ring(c["x_mm"], c["y_mm"], c["search_r_mm"], theme.WARN, "8 6"))
            parts.append(f'<text x="{x + 18:.1f}" y="{y + 6:.1f}" font-size="16" fill="{theme.SENSOR}">{html.escape(c["id"])}</text>')
    if "팔 원자료" in on:  # 모든 확인 점을 높이로 색칠
        for p in snap["paths"]:
            for px, py, h in p.get("points") or []:
                x, y = xy(px, py)
                parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2" fill="{theme.OK if h > hit_mm else theme.MUTED}" class="raw-arm"/>')
    latest = snap["paths"][-1]["id"] if snap["paths"] else None  # 지나간 후보의 경로까지 그리면 알아볼 수 없다. 끝난 후보는 결과 사각형만 남긴다
    if "팔 후보별 원자료" in on:
        for p in (q for q in snap["paths"] if q["id"] == latest):
            line = " ".join(f"{x:.1f},{y:.1f}" for x, y in (xy(*q) for q in p["points_mm"]))
            color = theme.DANGER if p.get("unreachable") else theme.ACCENT
            parts.append(f'<polyline points="{line}" fill="none" stroke="{color}" stroke-width="2" stroke-opacity="0.6"/>')
            for px, py, h in p["points"]:
                x, y = xy(px, py)
                parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{4 if h > hit_mm else 2}" fill="{theme.OK if h > hit_mm else theme.MUTED}"/>')
    if "합치기" in on:  # 단계마다 좁혀지는 범위
        for ests in (snap.get("estimates") or {}).values():
            for e in ests:
                if e.get("center") and e.get("spread_mm"):
                    parts.append(ring(*e["center"], e["spread_mm"], theme.PREVIEW))
    for res in (snap["results"] if "최종 위치" in on else []):
        o = res["object"]
        if not o:
            continue
        x, y = xy(o["x_mm"], o["y_mm"])
        angle = -(o["angle_deg"] or 0.0)  # SVG 회전은 시계 방향이다
        parts.append(f'<g transform="translate({x:.1f} {y:.1f}) rotate({angle:.1f})">'
                     f'<rect x="{-o["length_mm"] / 2:.1f}" y="{-o["width_mm"] / 2:.1f}" width="{o["length_mm"]:.1f}" height="{o["width_mm"]:.1f}" fill="{theme.OK}" fill-opacity="0.2" stroke="{theme.OK}" stroke-width="2"/>')
        if o["angle_deg"] is not None:  # 긴 축 방향. 원형·정사각형은 방향이 없다
            parts.append(f'<line x1="{-o["length_mm"] / 2:.1f}" y1="0" x2="{o["length_mm"] / 2:.1f}" y2="0" stroke="{theme.OK}" stroke-width="2"/>')
        parts.append("</g>")
    return "".join(parts)


def chart_options(cfg, snap: dict) -> dict:
    """각도-거리 그래프(E2): 이번 바퀴와 빈 작업대 기준값. 응답 없음은 끊어 그린다."""
    sweep = snap["sweep"] or {"angles": [], "d": []}
    now = sorted(zip(sweep["angles"], sweep["d"], strict=True), key=lambda p: p[0])
    base = sorted((int(a), d) for a, d in snap["baseline"].items())
    axis = {"axisLine": {"lineStyle": {"color": theme.CONTROL_BORDER}}, "axisLabel": {"color": theme.MUTED},
            "splitLine": {"lineStyle": {"color": theme.LINE}}, "nameTextStyle": {"color": theme.MUTED}}
    return {
        "animation": False,
        "tooltip": {"trigger": "axis"},
        "legend": {"top": 0, "textStyle": {"color": theme.TEXT}},
        "grid": {"left": 56, "right": 16, "top": 32, "bottom": 40},
        "xAxis": {"type": "value", "name": "서보 각도 (°)", "nameLocation": "middle", "nameGap": 26, "min": cfg.radar.sweep.from_deg, "max": cfg.radar.sweep.to_deg, **axis},
        "yAxis": {"type": "value", "name": "mm", "min": 0, "max": cfg.radar.max_mm, **axis},
        "series": [
            {"name": "이번 바퀴", "type": "line", "connectNulls": False, "showSymbol": True, "symbolSize": 6, "lineStyle": {"width": 2}, "color": theme.SENSOR, "data": [list(p) for p in now]},
            {"name": "기준값", "type": "line", "connectNulls": False, "showSymbol": False, "lineStyle": {"width": 2, "type": "dashed"}, "color": theme.MUTED, "data": [list(p) for p in base]},
        ],
    }


def arm_chart_options(snap: dict, hit_mm: float) -> dict:
    """팔 센서 그래프(F7): 지금 후보의 확인 구간마다 점 순서 → 높이 h, 물체 위 기준(hit_mm) 선."""
    latest = snap["paths"][-1]["id"] if snap["paths"] else None
    series = [{"name": p.get("stage") or "구간", "type": "line", "showSymbol": True, "symbolSize": 4,
               "data": [[i, round(h, 1)] for i, (_, _, h) in enumerate(p.get("points") or [])]}
              for p in snap["paths"] if p["id"] == latest and p.get("points")]
    series.append({"name": "물체 위 기준", "type": "line", "showSymbol": False, "lineStyle": {"type": "dashed"}, "color": theme.MUTED,
                   "data": [[0, hit_mm], [max((len(s["data"]) for s in series), default=1), hit_mm]]})
    return {"animation": False, "tooltip": {"trigger": "axis"}, "legend": {"top": 0, "textStyle": {"color": theme.TEXT}},
            "grid": {"left": 48, "right": 16, "top": 32, "bottom": 32},
            "xAxis": {"type": "value", "name": "점 순서", "nameLocation": "middle", "nameGap": 22}, "yAxis": {"type": "value", "name": "높이 mm", "min": 0},
            "series": series}


def timeline_rows(snap: dict) -> list[dict]:
    """후보별 타임라인(F7): 후보 → L1 → L2(격자) → 원 → 긴 축·짧은 축 → 결과. 칸은 '(x, y) ±범위'."""
    results = {r["id"]: r for r in snap["results"]}
    ids = list(dict.fromkeys([*(snap.get("estimates") or {}), *results]))
    rows = []
    for i, cid in enumerate(ids):
        row = {"key": i, "id": cid}
        ests = (snap.get("estimates") or {}).get(cid, [])
        for col, stages in STAGE_COLS:
            e = next((e for e in reversed(ests) if e.get("stage") in stages and e.get("center")), None)
            row[col] = f'({e["center"][0]:.0f}, {e["center"][1]:.0f})' + (f' ±{e["spread_mm"]:.0f}' if e.get("spread_mm") else "") if e else ""
        res = results.get(cid)
        o = (res or {}).get("object") or {}
        if res is None:
            row["result"] = "확인 중"
        elif not o:
            row["result"] = STATUS_TEXT.get(res["status"], res["status"])
        else:
            ang = "방향 없음" if o.get("angle_deg") is None else f'{o["angle_deg"]:.0f}°'
            row["result"] = f'({o["x_mm"]:.0f}, {o["y_mm"]:.0f}) {ang} {o["length_mm"]:.0f}×{o["width_mm"]:.0f} 신뢰 {o.get("confidence", 0):.2f}'
        rows.append(row)
    return rows


def active_log(backend) -> "UltrasonicLog":
    """작업 공간 인식이 초음파면 작업 실행 기록(backend.us_log), 수동이면 가상 데모 기록(12.11)."""
    return backend.us_log if getattr(backend, "perception_source", None) is not getattr(backend, "source", None) else demo_of(backend).log


class Demo:
    """가상 ESP32와 가상 팔로 인식을 돌린다. 진짜 팔과 포트는 쓰지 않는다.
    shortcut: 작업 실행 흉내는 벤치마크의 run_cycle을 빌려 쓴다, REQ-025와 #73이 합쳐지면 Studio.bus의 이벤트를 받는 것으로 바꾼다."""

    def __init__(self, speed: float = DEMO_SPEED, gripper: str | None = None):
        self.log = UltrasonicLog()
        self.speed = speed
        self.gripper = gripper  # 그리퍼별 check.tool_reach(#390): 병렬 팔이면 병렬 도달 범위로 그린다
        self.error: str | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stop(self) -> None:
        self._stop.set()

    def wait(self, timeout: float) -> bool:
        if self._thread:
            self._thread.join(timeout)
        return not self.running

    def start(self, boxes: list[Box], calibrate_only: bool = False) -> None:
        if self.running:
            return
        self._stop.clear()
        self.error = None
        self.log.clear()
        self._thread = threading.Thread(target=self._run, args=(boxes, calibrate_only), daemon=True, name="us-demo")
        self._thread.start()

    def _paced(self, clk: SimClock):
        def sleep(s: float) -> None:
            if self._stop.is_set():
                raise InterruptedError
            SimClock.sleep(clk, s)
            time.sleep(s / self.speed)

        return sleep

    def _run(self, boxes: list[Box], calibrate_only: bool) -> None:
        try:
            cfg, clk, esp, arm = benchmark.build(list(boxes), gripper=self.gripper)
            clk.sleep = self._paced(clk)
            bus = EventBus()
            self.log.attach(bus)
            with tempfile.TemporaryDirectory() as folder:  # 진짜 기준값 파일을 건드리지 않는다
                src = UltrasonicSource(Link(esp, clk), cfg, bus=bus, arm_origin_mm=arm.base, arm_links_fn=arm.links_at,
                                       baseline_path=Path(folder) / "baseline.json")
                src.connect()
                esp.boxes, kept = [], esp.boxes
                self.log.set_baseline(src.calibrate() and src.baseline.d)
                esp.boxes = kept
                if not calibrate_only:
                    benchmark.run_cycle(src, arm, esp)
        except InterruptedError:
            pass
        except Exception as e:  # noqa: BLE001  화면에 이유를 보여 준다. 앱은 계속 돈다
            self.error = repr(e)


_demos: WeakKeyDictionary = WeakKeyDictionary()


def demo_of(backend) -> Demo:
    return _demos.setdefault(backend, Demo(gripper=getattr(getattr(backend.arm, "cfg", None), "gripper", None)))


def demo_boxes(backend) -> list[Box]:
    """수동 입력 물체를 가상 작업대에 놓는다."""
    return [Box(o.x_mm, o.y_mm, o.angle_deg or 0.0, o.width_mm, o.length_mm) for o in backend.objects()]


def page(backend, client_id: str, actions) -> None:
    demo = demo_of(backend)
    cfg = load_config()
    table, ws = backend.arm.ws.table_mm, backend.arm.ws
    controls = Controls(backend, client_id, block_during_task=False)
    seen: dict = {"version": None}

    async def start() -> None:
        demo.start(demo_boxes(backend))

    async def calibrate() -> None:
        demo.start([], calibrate_only=True)

    async def stop() -> None:
        demo.stop()

    with actions:
        if backend.real is None:
            calibrate_button = controls.add(ui.button("기준값 다시 재기", on_click=controls.guard(calibrate)).props("outline no-caps").mark("us-calibrate"))
            start_button = controls.add(ui.button("가상으로 돌리기", on_click=controls.guard(start)).props("no-caps").mark("us-start"))
            stop_button = controls.add(ui.button("멈추기", on_click=controls.guard(stop)).props("outline no-caps").mark("us-stop"))

    note = ui.label("").classes("page-body").mark("us-note")
    status = ui.label("").classes("page-body").mark("us-status")

    layers: set = set(LAYERS)  # 고른 겹(F7). [단계 따라가기]면 흐름 단계가 정한다
    with ui.row().classes("w-full items-center gap-3").mark("us-layers"):
        boxes = {}
        for name in LAYERS:
            boxes[name] = ui.checkbox(name, value=True, on_change=lambda e, n=name: (layers.add(n) if e.value else layers.discard(n), seen.update(version=None)))
            boxes[name].mark(f"us-layer-{name}")
        follow = ui.switch("단계 따라가기", value=False, on_change=lambda e: seen.update(version=None)).mark("us-follow")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-2"):
            plane = ui.interactive_image(size=(table["x"], table["y"]), sanitize=False).classes("w-full plane plane-large").mark("us-plane")
            ui.label("부채꼴 = 레이더 시야, 파란 점 = 바닥 메아리, 점선 원 = 후보·예상 범위, 파란 선 = 확인 경로, 보라 원 = 좁혀지는 범위, 초록 사각형 = 결과").classes("page-body")
        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("각도별 거리 (바닥)").classes("card-title")
                chart = ui.echart(chart_options(cfg, demo.log.snapshot())).classes("w-full joint-chart").mark("us-chart")
            with ui.card().classes("w-full gap-2"):
                ui.label("팔 센서 높이 (지금 후보)").classes("card-title")
                arm_chart = ui.echart(arm_chart_options(demo.log.snapshot(), cfg.check.hit_mm)).classes("w-full joint-chart").mark("us-arm-chart")

    columns = [{"name": k, "label": label, "field": k, "align": "left"} for k, label in (
        ("id", "후보"), *((c, c) for c, _ in STAGE_COLS), ("result", "결과"))]
    timeline = ui.table(rows=[], columns=columns, row_key="key").props('flat bordered dense no-data-label="아직 확인한 후보가 없습니다"').classes("w-full").mark("us-timeline")

    def redraw() -> None:
        controls.refresh()
        manual = active_log(backend) is demo.log
        if backend.real is None:
            calibrate_button.enabled = start_button.enabled = controls.allowed and not demo.running and manual
            stop_button.enabled = controls.allowed and demo.running
        note.text = ("가상 ESP32와 가상 팔의 값입니다. 작업 공간에서 인식을 초음파로 고르면 그 실행의 기록을 보입니다." if manual
                     else "작업 공간 작업 실행의 초음파 기록입니다(다음 실행을 시작할 때까지 남습니다).")
        status.text = (f"오류: {demo.error}" if demo.error else "돌아가는 중" if demo.running else "멈춤") if manual else backend.caption() or "멈춤"
        snap = active_log(backend).snapshot()
        on = FOLLOW.get(getattr(backend, "_flow", ("idle",))[0], set(LAYERS)) if follow.value else layers
        key = (snap["version"], id(active_log(backend)), tuple(sorted(on)))
        if key == seen["version"]:
            return
        seen["version"] = key
        if follow.value:  # 따라가는 동안 체크도 지금 겹을 보인다
            for name, box in boxes.items():
                box.set_value(name in on)
        plane.content = plane_svg(table, [], ws.drop_zones, ws.base, ws.reach_mm) + radar_overlay(cfg, snap, table["y"], cfg.check.hit_mm, on)
        chart.options.update(chart_options(cfg, snap))
        chart.update()
        arm_chart.options.update(arm_chart_options(snap, cfg.check.hit_mm))
        arm_chart.update()
        timeline.rows = timeline_rows(snap)

    redraw()
    ui.timer(0.25, redraw)


def mini_view(backend) -> None:
    """작업 공간용 작은 화면: 레이더 부채꼴, 후보, 확인 경로, 결과(gui-spec 3절). 인식이 초음파면 작업 실행 기록, 아니면 가상 데모 기록."""
    cfg, ws = load_config(), backend.arm.ws
    table = ws.table_mm
    image = ui.interactive_image(size=(table["x"], table["y"]), sanitize=False).classes("w-full plane").mark("us-mini")
    seen: list = [None]

    def redraw() -> None:
        log = active_log(backend)
        snap = log.snapshot()
        if (snap["version"], id(log)) == seen[0]:
            return
        seen[0] = (snap["version"], id(log))
        image.content = plane_svg(table, [], ws.drop_zones, ws.base, ws.reach_mm) + radar_overlay(cfg, snap, table["y"], cfg.check.hit_mm)

    redraw()
    ui.timer(0.5, redraw)


from . import register

register("ultrasonic", "초음파", "/ultrasonic", page, mini_view)
