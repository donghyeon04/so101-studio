"""ESP32 센서 보드 페이지 (REQ-028, gui-spec 3절, ultrasonic-spec 7절): 포트와 연결 상태, 펌웨어 버전, 센서 2개와 서보 상태,
시계 맞추기 오차, 최근 JSON 메시지. 보내는 명령은 hello와 sync뿐이라 서보를 움직이지 않는다."""

import threading
import time
from collections import deque
from weakref import WeakKeyDictionary

from nicegui import run, ui

from ...core.ports import list_serial_ports
from ...perception.ultrasonic.fake_esp32 import FakeESP32
from ...perception.ultrasonic.link import Link, SerialTransport, UltrasonicLinkError
from ...perception.ultrasonic.model import RealClock, load_config
from .common import Controls

SIM_PORT = "가상 ESP32"
KEEP = 50  # 최근 메시지 줄 수
POLL_S = 0.05
SENSOR_NAMES = {"r": "레이더", "a": "팔 센서"}


class Tap:
    """전송 계층을 감싸 오가는 줄을 기록한다(Link는 sync 응답과 JSON이 아닌 줄을 버린다)."""

    def __init__(self, tx, lines: deque):
        self.tx, self.lines = tx, lines

    def write(self, line: str) -> None:
        self.tx.write(line)
        self.lines.append((time.time(), "→", line.strip()))

    def readlines(self) -> list[str]:
        out = self.tx.readlines()
        self.lines.extend((time.time(), "←", s.strip()) for s in out)
        return out

    def close(self) -> None:
        getattr(self.tx, "close", lambda: None)()


class Board:
    """페이지를 떠나도 연결이 남도록 백엔드마다 하나. 읽기 스레드가 Link.poll을 돌린다.
    shortcut: 작업 실행이 같은 포트를 쓰면(REQ-025) 여기서 먼저 끊어야 한다, 실제 연결이 붙을 때 Studio와 연결을 나눠 쓴다."""

    def __init__(self):
        self.lines: deque = deque(maxlen=KEEP)
        self.port: str | None = None
        self.hello: dict | None = None
        self.error: str | None = None
        self.last: dict[str, dict] = {}  # 센서별 마지막 측정
        self.link: Link | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def sync_error_ms(self) -> float | None:
        """시계 차이의 오차 상한 = 왕복 시간의 절반(7절)."""
        rtt = self.link.sync_rtt_ns if self.link else None
        return None if rtt is None else rtt / 2e6

    def connect(self, port: str, baud: int) -> None:
        """hello를 0.5초마다 다시 보내 최대 5초 기다린 뒤 sync. 실패하면 포트를 닫고 error에 이유를 남긴다."""
        self.disconnect()
        self.error, self.hello, self.last = None, None, {}
        self.lines.clear()
        tx = None
        try:
            tx = Tap(FakeESP32(RealClock(), [], load_config()) if port == SIM_PORT else SerialTransport(port, baud), self.lines)
            self.link = Link(tx)
            for _ in range(10):
                self.link.send(cmd="hello")
                try:
                    self.hello = self.link.wait_for("hello", 0.5)
                    break
                except UltrasonicLinkError:
                    continue
            else:
                raise UltrasonicLinkError("hello 응답 없음(5초): 포트·케이블·펌웨어를 확인하세요")
            self.link.sync()
        except Exception as e:  # noqa: BLE001  포트 열기 실패도 화면에 보여 준다
            self.error = str(e)
            if tx:
                tx.close()
            self.link = None
            return
        self.port = port
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(tx,), daemon=True, name="esp32-page")
        self._thread.start()

    def _run(self, tx) -> None:
        try:
            while not self._stop.is_set():
                for m in self.link.poll():
                    if m.get("t") in SENSOR_NAMES:
                        self.last[m["t"]] = m
                    elif m.get("t") == "err":
                        self.error = m.get("msg", "")
                time.sleep(POLL_S)
        except UltrasonicLinkError as e:
            self.error = str(e)
        except Exception as e:  # noqa: BLE001  예상 못 한 줄(부팅 잡음 등)로 스레드가 조용히 죽지 않게 화면에 보인다
            self.error = f"읽기 중 오류로 멈췄습니다: {e!r}"
        finally:
            tx.close()

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(1.0)
        self._thread = None


_boards: WeakKeyDictionary = WeakKeyDictionary()


def board_of(backend) -> Board:
    return _boards.setdefault(backend, Board())


def port_options(sim: bool) -> list[str]:
    return ([SIM_PORT] if sim else []) + [p["device"] for p in list_serial_ports()]


def sensor_rows(board: Board) -> list[dict]:
    kinds = (board.hello or {}).get("k", [])
    rows = []
    for k in kinds:
        m = board.last.get(k)
        d = "" if m is None else "응답 없음" if m.get("d") is None else f'{m["d"]} mm'
        rows.append({"name": SENSOR_NAMES.get(k, k), "value": d or "측정 전", "age": "" if m is None else f'{(time.perf_counter_ns() - m["t_ns"]) / 1e9:.1f}초 전'})
    servo = (board.hello or {}).get("servo")
    if servo:
        a = board.last.get("r", {}).get("a")
        rows.append({"name": f"서보 ({servo})", "value": "각도 모름" if a is None else f"{a}°", "age": ""})
    return rows


def build(backend, client_id: str, actions) -> None:
    board = board_of(backend)
    cfg = load_config()
    controls = Controls(backend, client_id)
    options = port_options(backend.real is None)

    async def connect() -> None:
        await run.io_bound(board.connect, port.value, cfg.baud)

    async def disconnect() -> None:
        board.disconnect()

    with actions:
        port = ui.select(options, value=board.port or (options[0] if options else None), label="포트").props("outlined dense").classes("w-48").mark("esp-port")
        connect_button = controls.add(ui.button("연결", on_click=controls.guard(connect)).props("no-caps").mark("esp-connect"))
        disconnect_button = controls.add(ui.button("끊기", on_click=controls.guard(disconnect)).props("outline no-caps").mark("esp-disconnect"))

    ui.label("이 페이지는 hello와 시계 맞추기(sync)만 보냅니다. 서보를 움직이지 않습니다. 작업 실행이 ESP32를 쓰기 전에는 [끊기]를 누르세요.").classes("page-body")
    with ui.row().classes("w-full items-start gap-4"):
        with ui.card().classes("gap-1"):
            ui.label("연결").classes("card-title")
            state = ui.label("").classes("page-body").mark("esp-state")
            firmware = ui.label("").classes("page-body").mark("esp-fw")
            sync = ui.label("").classes("page-body").mark("esp-sync")
        with ui.card().classes("flex-1 gap-1"):
            ui.label("센서와 서보").classes("card-title")
            columns = [{"name": k, "label": label, "field": k, "align": "left"} for k, label in (("name", "장치"), ("value", "마지막 값"), ("age", "언제"))]
            sensors = ui.table(rows=[], columns=columns, row_key="name").props('flat dense no-data-label="연결하면 센서와 서보 값이 나옵니다"').classes("w-full").mark("esp-sensors")
    ui.label(f"최근 메시지 ({KEEP}줄, 새 것이 위)").classes("card-title")
    with ui.scroll_area().classes("w-full log-area").mark("esp-lines"):
        lines = ui.column().classes("w-full gap-0")
    seen: list = [None]

    def redraw() -> None:
        controls.refresh()
        in_use = getattr(backend, "perception_kind", "manual") == "real"  # 작업 공간 초음파(실물)가 이 포트를 쓰는 중(12.11)
        connect_button.enabled = controls.allowed and bool(port.value) and not in_use
        disconnect_button.enabled = controls.allowed and board.connected
        state.text = ("작업 공간 초음파(실물)가 사용 중입니다" if in_use else (f"연결됨: {board.port}" if board.connected else "연결 안 됨")) + (f" · 오류: {board.error}" if board.error else "")
        firmware.text = f'펌웨어 {board.hello.get("fw", "?")}' if board.hello else "펌웨어 모름"
        err = board.sync_error_ms()
        sync.text = "시계 오차 모름" if err is None else f"시계 오차 ±{err:.2f} ms (목표 2 ms 이하)"
        if board.hello and not board.connected:
            firmware.text += " (지난 연결)"
            sync.text += " (지난 연결)"
        sensors.rows = sensor_rows(board)
        recent = list(board.lines)
        key = (len(recent), recent[-1][0] if recent else 0)
        if key == seen[0]:
            return
        seen[0] = key
        lines.clear()
        with lines:
            if not recent:
                ui.label("아직 없습니다. 포트를 고르고 [연결]을 누르면 주고받은 메시지가 여기에 나옵니다").classes("page-body")
            for t, way, text in reversed(recent):
                ui.label(f'{time.strftime("%H:%M:%S", time.localtime(t))} {way} {text}').classes("log-line")

    redraw()
    ui.timer(0.25, redraw)

