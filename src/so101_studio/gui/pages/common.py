"""페이지들이 같이 쓰는 조각: 조작권 게이트, 3D 칸."""

import asyncio
import inspect
import time

from nicegui import background_tasks, run, ui

from .. import theme
from ..param_meta import GAPS, META, fmt, hint_text, widget_for
from ..scene3d import LAYERS
from ..view_cube import view_cube

HEIGHT_HINT = ("물체 높이를 알면 꼭 넣으세요. 팔 방향과 어긋난 물체는 손을 기울이지 못해 높이를 모르면 턱이 물체 위 모서리를 칠 수 있습니다. "
               "0이면 설정한 집는 높이로 집습니다")  # REQ-055 sim 근거: 높이를 모르면 384건 중 127건이 위 모서리를 친다


class Controls:
    """조작권이 있어야 켜지는 요소를 모으고, 눌렸을 때 서버에서 한 번 더 막는다."""

    def __init__(self, backend, client_id: str, block_during_task: bool = True, moves: bool = False):
        self.backend, self.client_id, self.block_during_task, self.moves = backend, client_id, block_during_task, moves
        self.elements: list = []

    def add(self, element, needs=()):
        """needs: 이 요소가 읽는 숫자 칸(MetaNumber). 하나라도 허용 밖이면 끈다 (gui-spec 12.7.4)."""
        self.elements.append((element, needs))  # 목록이면 나중에 더한 칸도 본다
        return element

    @staticmethod
    def valid(needs=()) -> bool:
        return all(m.valid for m in needs)

    @property
    def mine(self) -> bool:
        return self.backend.owner == self.client_id

    @property
    def allowed(self) -> bool:
        """작업 실행 중에는 팔을 직접 움직이는 조작을 막는다."""
        return self.mine and not self.locked and not (self.block_during_task and self.backend.task_running())

    @property
    def locked(self) -> str | None:
        """팔을 움직이는 페이지(moves)는 실제 팔이 잠기면 함께 꺼진다 (decisions/0027)."""
        return self.backend.move_lock() if self.moves else None

    def guard(self, fn, needs=()):
        """누른 순간 서버에서 다시 막는다: 조작권·잠김·작업 실행, 그리고 needs 칸이 허용 안인지(버튼은 타이머로만 꺼져 그 사이에 누를 수 있다, 12.7.4).
        동기 함수도 감쌀 수 있다(돌려준 값이 awaitable일 때만 기다린다)."""
        async def call(*args):
            if not self.mine:
                ui.notify("보기 전용입니다", type="warning")
                return None
            if self.locked:
                ui.notify(self.locked, type="warning")
                return None
            if not self.allowed:
                ui.notify("작업 실행 중입니다", type="warning")
                return None
            bad = next((m for m in needs if not m.valid), None)
            if bad is not None:
                ui.notify(f"{bad.name}: 허용 밖이라 실행하지 않습니다", type="warning")
                return None
            result = fn(*args)
            return await result if inspect.isawaitable(result) else result

        return call

    def refresh(self) -> None:
        allowed = self.allowed
        for element, needs in self.elements:
            element.enabled = allowed and self.valid(needs)


class MetaNumber:
    """값 정보표(META)를 따르는 숫자 칸 한 묶음 (gui-spec 12.7.4): 숫자 칸, 규칙에 따라 슬라이더나 간격 버튼, [기본값으로], 칸 아래 줄.
    .value는 마지막 허용 값이고, 허용 밖 글자는 오류로 보이기만 하고 on_change를 부르지 않는다. on_change는 Enter·칸 떠남·간격 버튼·[기본값으로]·슬라이더 놓음에서 한 번."""

    def __init__(self, key: str, value, on_change=None, *, label: str | None = None, default=None, add=None, mark: str | None = None,
                 hard: tuple | None = None, on_valid=None):
        self.key, self.meta = key, META[key]
        self.name = label or key
        self.on_change, self.on_valid, self.default = on_change, on_valid, default
        self.hard = tuple(float(x) for x in (hard or self.meta.hard))
        self.integer = float(self.meta.step).is_integer() and (isinstance(value, int) and not isinstance(value, bool) or key == "program.count")  # 한 칸이 0.5처럼 소수면 YAML 값이 정수여도 소수를 받는다
        self.other_error: str | None = None
        self._enabled, self._syncing = True, False
        mark = mark or f"meta-{key}"
        kind = widget_for(key)
        self.parts: list = []
        with ui.column().classes("gap-0 meta-number").mark(f"{mark}-box") as self.box:
            with ui.row().classes("w-full no-wrap items-center gap-1"):
                self.slider = None
                if kind == "slider":
                    self.slider = ui.slider(min=self.hard[0], max=self.hard[1], step=self.meta.step, value=self._clamp(value)).props("label").classes("flex-1 meta-slider").mark(f"{mark}-slider")
                    self.slider.on("update:model-value", self._slider_moving)
                    self.slider.on("change", self._slider_released)
                    self.parts.append(self.slider)
                self.number = ui.number(value=value, format="%d" if self.integer else None, step=self.meta.step, validation=self._error_text,
                                        on_change=self._typed).props("outlined dense").classes("meta-input").mark(mark)
                self.number.meta = self  # 시험이 _one(user, 표식).meta.set(v)로 닿는다
                self.number.on("blur", self._commit)
                self.number.on("keydown.enter", self._commit)
                self.parts.append(self.number)
                if kind == "gap":
                    for d in GAPS:
                        b = ui.button(f"{d:+d}", on_click=lambda _e, d=d: self._step(d)).props("outline dense no-caps size=sm").classes("meta-gap").mark(f"{mark}-{'m' if d < 0 else 'p'}{abs(d)}")
                        self.parts.append(b)
                if default is not None:
                    b = ui.button(icon="restart_alt", on_click=lambda: self.set(self.default)).props(f'flat dense round size=sm aria-label="기본값 {fmt(default)}으로"').mark(f"{mark}-reset")
                    b.tooltip(f"기본값 {fmt(default)}으로")
                    self.parts.append(b)
            self.hint = ui.label("").classes("stat-label meta-hint").mark(f"{mark}-hint")
        self.value = value if self._ok(value) else None
        self.initial = value
        self._was_valid = self._ok(value)
        self._paint()
        if add is not None:
            add(self)
        if on_valid is not None:
            on_valid(self._was_valid)  # 처음 값이 허용 밖이어도 바로 알린다(프로그램 화면의 state["invalid"])

    # 판정
    def _ok(self, v) -> bool:
        if v is None:
            return self.meta.allow_none
        try:
            v = float(v)
        except (TypeError, ValueError):
            return False
        return self.hard[0] <= v <= self.hard[1]

    def _error_text(self, v) -> str | None:
        if not self._ok(v):
            return f"허용 {fmt(self.hard[0])}~{fmt(self.hard[1])} 밖이라 쓰지 않습니다"
        return self.other_error

    @property
    def valid(self) -> bool:
        return self._ok(self.number.value)

    def _clamp(self, v) -> float:
        try:
            return min(max(float(v), self.hard[0]), self.hard[1])
        except (TypeError, ValueError):
            return self.hard[0]

    def _norm(self, v):
        return None if v is None else (round(float(v)) if self.integer else float(v))

    # 그리기
    def _paint(self) -> None:
        self.hint.text = hint_text(self.meta, self.default, self.hard)
        v, soft = self.number.value, self.meta.soft
        warn = soft is not None and self._ok(v) and v is not None and not soft[0] <= float(v) <= soft[1]
        self.box.classes(add="meta-soft-out" if warn else "", remove="" if warn else "meta-soft-out")
        if self.slider is not None and soft:
            span = self.hard[1] - self.hard[0]
            a, b = ((s - self.hard[0]) / span * 100 for s in soft)
            self.slider.style(f"--soft-a:{a:.1f}%;--soft-b:{max(b, a + 1):.1f}%")
            self.slider.classes(add="meta-soft-band")

    def _check_valid(self) -> None:
        ok = self.valid
        if ok != self._was_valid:
            self._was_valid = ok
            if self.on_valid is not None:
                self.on_valid(ok)

    # 사람이 바꿀 때
    def _typed(self, _e=None) -> None:
        """글자가 바뀔 때마다: 판정만 하고(값을 보내지 않는다) 슬라이더를 따라 옮긴다."""
        self._check_valid()
        self._paint()
        if self.slider is not None and self.valid and self.number.value is not None and not self._syncing:
            self._syncing = True
            self.slider.value = self._clamp(self.number.value)
            self._syncing = False

    def _commit(self, _e=None) -> None:
        """Enter·칸 떠남: 허용 값이 바뀌었으면 한 번 보낸다."""
        v = self.number.value
        if self._ok(v) and self._norm(v) != self._norm(self.value):
            self._accept(v)

    def _slider_moving(self, _e=None) -> None:
        if not self._syncing:
            self._syncing = True
            self.number.value = self._norm(self.slider.value)
            self._syncing = False

    def _slider_released(self, _e=None) -> None:
        self._slider_moving()
        self._commit()

    def _step(self, d: float) -> None:
        base = self.value if self.value is not None else self._clamp(0)
        self.set(self._clamp(float(base) + d))

    def _accept(self, v, quiet: bool = False) -> None:
        self.value = self._norm(v)
        if not quiet and self.on_change is not None:
            result = self.on_change(self.value)
            if inspect.isawaitable(result):  # guard로 감싼 처리 함수(팔을 움직이는 칸)
                background_tasks.create(result, name=f"meta {self.key}")

    # 코드에서
    def set(self, v, quiet: bool = False) -> bool:
        """값 바꾸기. 허용 밖이면 바꾸지 않고 False. 성공하면 quiet가 아닐 때 on_change(v)."""
        if not self._ok(v):
            return False
        same = self._norm(v) == self._norm(self.value)
        self.number.value = self._norm(v)
        if not same:  # 같은 값이면 다시 보내지 않는다(끝값에서 +10, 기본값에서 [기본값으로])
            self._accept(v, quiet)
        return True

    def load(self, v) -> None:
        """저장된 값을 다시 읽어 넣는다: 허용 밖이어도 그대로 보이고, on_change를 부르지 않고 처음 값도 v로 바꾼다."""
        self.number.value = v
        self.value = self._norm(v) if self._ok(v) else self.value
        self.initial = v
        self._typed()

    def set_hard(self, lo: float, hi: float) -> None:
        self.hard = (float(lo), float(hi))
        if self.slider is not None:
            self.slider.props(f"min={lo} max={hi}")
        self.number.validate(return_result=False)
        self._typed()

    def set_error(self, text: str | None) -> None:
        self.other_error = text
        self.number.validate(return_result=False)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, on: bool) -> None:
        if on != self._enabled:
            self._enabled = on
            for p in self.parts:
                p.enabled = on


def meta_number(key: str, value, on_change=None, **kw) -> MetaNumber:
    """gui-spec 12.7.4 공용 부품. 표식: 숫자 칸 meta-<key>(또는 mark), 슬라이더 <mark>-slider, 간격 버튼 <mark>-m10…-p10, [기본값으로] <mark>-reset."""
    return MetaNumber(key, value, on_change, **kw)


def _layer_menu(scene) -> None:
    """3D에 보일 것을 고르는 메뉴. 3D 칸 왼쪽 위에 얹는다."""
    with ui.button(icon="layers").props('round dense size=sm color=white text-color=dark aria-label="3D에 보일 것 고르기"').classes("scene-layers").mark("scene-layers"), ui.menu(), ui.column().classes("gap-0 p-2"):
        for name, text in LAYERS.items():
            ui.checkbox(text, value=scene.layers[name], on_change=lambda e, n=name: scene.set_layer(n, e.value)).mark(f"layer-{name}")


def _request_host() -> str | None:
    """이 브라우저가 앱을 연 호스트 이름(시험처럼 요청이 없으면 None). --share일 때만 쓴다: 아니면 Viser는 127.0.0.1에만 있어 localhost(::1)로 열면 못 닿는다."""
    try:
        return ui.context.client.request.url.hostname
    except (RuntimeError, AttributeError):
        return None


def scene_frame(backend, classes: str = "scene3d") -> None:
    with ui.element("div").classes("scene-wrap"):
        ui.element("iframe").props(f'src={backend.scene.url_for(_request_host()) if getattr(backend, "share", False) else backend.scene.url} title="3D 보기"').classes(classes).mark("scene3d")
        view_cube(backend.scene)
        _layer_menu(backend.scene)


def jpeg_url(frame) -> str:
    """render가 준 RGB 배열을 브라우저에 그릴 JPEG data URL로."""
    import base64
    import io

    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(frame).save(buf, "JPEG", quality=80)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()


CAMERA_TITLES = {"top": "탑뷰 (top)", "wrist": "손목 (wrist)"}  # gui-spec 12.10 화면 제목


def camera_tile(driver, camera: str, title: str):
    """카메라 칸 하나(제목, 그림, 갱신 시각). 돌려준 코루틴 함수를 부를 때마다 driver.render를 한 번 불러 다시 그린다 (gui-spec 12.10)."""
    with ui.row().classes("w-full items-center"):
        ui.label(title).classes("card-title")
        ui.space()
        stamp = ui.label("").classes("stat-label").mark(f"camera-time-{camera}")
    image = ui.image().classes("w-full").mark(f"camera-{camera}")

    async def update() -> None:
        try:
            url = await run.io_bound(lambda: jpeg_url(driver.render(camera)))
        except Exception as e:  # noqa: BLE001 그리기 실패는 그 칸에만 보인다
            stamp.text = f"그리기 실패: {e!r}"[:80]
            return
        image.set_source(url)
        stamp.text = time.strftime("%H:%M:%S")

    return update


def watch_visibility():
    """브라우저 탭이 숨겨졌는지 알려 준다. 숨겨진 동안 카메라를 그리지 않으려고 쓴다(12.10 ①). 돌려준 함수가 True면 보이는 중."""
    client = ui.context.client
    shown = getattr(client, "so101_visible", None)
    if shown is None:  # 처리기는 브라우저 하나에 한 번만 단다(페이지를 다시 열 때마다 쌓이지 않게)
        client.so101_visible = shown = [True]
        ui.on("so101_visible", lambda e: shown.__setitem__(0, bool(e.args)))
        ui.run_javascript("document.addEventListener('visibilitychange', () => emitEvent('so101_visible', !document.hidden));")
    return lambda: shown[0]


def offer_free_edge(backend, retry) -> None:
    """작업대 가장자리에 걸려 멈췄을 때 그 자리에서 묻는다: 가장자리 검사만 끄고(자유 모드) 다시 하기."""
    with ui.dialog() as dialog, ui.card().classes("gap-2").mark("edge-dialog"):
        ui.label("작업대 가장자리 밖이라 움직이지 않았습니다").classes("card-title")
        ui.label("작업대 가장자리 검사만 끄고(자유 모드) 다시 할까요? 관절 한계와 바닥 검사는 그대로 켜져 있습니다. 안전 페이지에서 다시 켤 수 있습니다.").classes("page-body")

        async def go() -> None:
            dialog.close()
            reason = await run.io_bound(backend.free_table_edge)
            if reason:
                ui.notify(reason, type="negative")
                return
            result = retry()  # 다시 할 동작(비동기 함수면 기다린다)
            if asyncio.iscoroutine(result):
                await result

        with ui.row().classes("gap-2"):
            ui.button("가장자리 검사 끄고 다시 하기", on_click=go).props("no-caps").mark("edge-free")
            ui.button("취소", on_click=dialog.close).props("outline no-caps")
    dialog.open()


NUDGE_MM, NUDGE_DEG, NUDGE_REPEAT_S = 5.0, 5.0, 0.15


def nudge_pad(on_step, add=lambda e: e, rotate: bool = True) -> None:
    """고른 것을 옮기고 돌리는 패드 (gui-spec 11절 D11, G16). 누를 때마다 5mm·5°, 누르고 있으면 계속. on_step(dx, dy, dangle).
    위 = 작업대 +y(평면 그림의 위쪽). add는 조작권에 따라 켜고 끄는 Controls.add."""
    repeat = {"timer": None}

    def start(step) -> None:
        stop()
        on_step(*step)
        repeat["timer"] = ui.timer(NUDGE_REPEAT_S, lambda: on_step(*step))

    def stop() -> None:
        if repeat["timer"] is not None:
            repeat["timer"].cancel()
            repeat["timer"] = None

    def button(icon: str, label: str, step) -> None:
        b = add(ui.button(icon=icon).props(f'outline dense aria-label="{label}"').classes("nudge-button").mark(f"nudge-{label}"))
        b.on("mousedown", lambda: start(step))
        b.on("touchstart", lambda: start(step))
        for event in ("mouseup", "mouseleave", "touchend"):
            b.on(event, stop)

    m, d = NUDGE_MM, NUDGE_DEG
    with ui.grid(columns=3).classes("gap-1 nudge-pad").mark("nudge-pad"):
        button("rotate_left", "왼쪽으로 돌리기", (0, 0, d)) if rotate else ui.element()
        button("arrow_upward", "위로", (0, m, 0))
        button("rotate_right", "오른쪽으로 돌리기", (0, 0, -d)) if rotate else ui.element()
        button("arrow_back", "왼쪽으로", (-m, 0, 0))
        ui.label(f"{m:.0f}mm").classes("stat-label self-center text-center")
        button("arrow_forward", "오른쪽으로", (m, 0, 0))
        ui.element()
        button("arrow_downward", "아래로", (0, -m, 0))
        ui.element()


def jog_pads(backend, gate: Controls, prefix: str = "") -> None:
    """G13 조이스틱 두 개(앞뒤·좌우, 높이): 누르는 동안 기울인 만큼의 속도로, 떼면 멈춘다. 위 = 팔 앞쪽(+y).
    직접 조작 화면과 어디서나 여는 조그 패널(결정 0044)이 같이 쓴다. 가만히 쥐고 있으면 이벤트가 안 와서 0.1초마다 이어 보낸다(데드맨)."""
    pad: dict = {"v": None}

    def move(key: str, e) -> None:
        if not gate.allowed or e.x is None:
            return
        v = list(pad["v"] or [0.0, 0.0, 0.0])
        if key == "xy":
            v[0], v[1] = e.x, e.y
        else:
            v[2] = e.y
        pad["v"] = v
        backend.jog_vector(v)

    def end() -> None:
        if pad["v"] is not None:
            pad["v"] = None
            backend.jog_release("cartesian")

    def keep() -> None:
        if pad["v"] and gate.allowed:
            backend.jog_vector(pad["v"])

    with ui.row().classes("w-full no-wrap items-end justify-around gap-2 mb-2").mark(f"{prefix}joystick"):
        for key, label, opts in (("xy", "앞뒤·좌우", {}), ("z", "높이", {"lockY": True})):
            with ui.column().classes("items-center gap-1"):
                ui.joystick(color=theme.ACCENT, size=96, mode="static", restOpacity=0.6, on_move=lambda e, k=key: move(k, e),  # 기본 dynamic은 누른 자리에 조이스틱을 새로 만들어 받침 원과 겹친다
                            on_end=lambda _e: end(), **opts).classes("jog-pad").mark(f"{prefix}pad-{key}")
                ui.label(f"{label} (눌러 끌기)").classes("stat-label")
    ui.timer(0.1, keep)
