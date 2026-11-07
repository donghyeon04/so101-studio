"""화면이 읽는 상태. 상태 스레드가 쓰고 화면 타이머가 읽는다 (gui-spec 8절)."""

import logging
from collections import deque
from dataclasses import dataclass, field

from ..core.types import ArmStatus, TaskEvent
from .health import LIMITS

EMPTY = "—"
PAGE_STATE = {"idle": "대기", "moving": "실행 중", "teleop": "실행 중", "teach": "실행 중", "stopped": "정지", "fault": "오류"}
SAFETY_TEXT = {"normal": "보통", "free": "자유", "off": "끔"}
TASK_TEXT = {
    "IDLE": "대기", "PREPARE": "준비", "SCAN": "스캔", "DECIDE": "판단", "PICK": "집기", "PLACE": "놓기",
    "CHECK": "확인", "RESCAN": "다시 스캔",
    "DONE": "완료", "STOPPED": "정지", "FAULT": "오류",
}
PROGRESS = ("준비", "스캔", "판단", "집기", "놓기", "다시 스캔")
_PROGRESS_INDEX = {"PREPARE": 0, "SCAN": 1, "DECIDE": 2, "CHECK": 2, "PICK": 3, "PLACE": 4, "RESCAN": 5}  # 초음파 확인(CHECK)은 판단 칸에 둔다


def progress_index(task_state: str, placed: int) -> int | None:
    """진행 막대에서 켤 칸. 물체를 하나 옮긴 뒤의 준비·스캔은 "다시 스캔"이다."""
    if task_state in ("PREPARE", "SCAN") and placed:
        return 5
    return _PROGRESS_INDEX.get(task_state)


@dataclass
class State:
    status: ArmStatus | None = None
    task_state: str = "IDLE"
    chosen: str | None = None
    placed: int = 0
    run_t0: float = 0.0  # 이번 작업을 시작한 시각(이벤트 t와 같은 epoch 초). 운영자 화면이 이번 실행의 이벤트만 센다
    events: deque = field(default_factory=lambda: deque(maxlen=200))

    def update(self, status: ArmStatus) -> None:
        self.status = status

    def on_task(self, event: TaskEvent) -> None:
        """작업 스레드에서 불린다. 화면은 타이머로 읽는다."""
        self.task_state = event.state
        self.events.append(event)
        if event.state == "DECIDE":
            self.chosen = event.detail.get("chosen")
        elif event.state == "PLACE" and event.result == "ok":
            self.placed += 1
            self.chosen = None
        elif event.state in ("DONE", "STOPPED", "FAULT"):
            self.chosen = None


@dataclass
class LogRecord:
    t: float
    level: str  # "INFO" | "WARNING" | "ERROR"
    text: str
    source: str = "gui"  # 기록 머리의 proc: gui | cli | watchdog
    code: str | None = None
    run: str | None = None  # 이 줄이 있는 실행 폴더. 실행 기록 페이지로 이동할 때 쓴다


class LogBook(logging.Handler):
    """파이썬 로그를 모은다. 실행 기록(events.jsonl)과 합쳐 로그 페이지에 보인다."""

    def __init__(self, maxlen: int = 2000):
        super().__init__(level=logging.WARNING)
        self.records: deque = deque(maxlen=maxlen)

    def emit(self, record: logging.LogRecord) -> None:
        level = "ERROR" if record.levelno >= logging.ERROR else "WARNING"
        self.records.append(LogRecord(record.created, level, f"{record.name}: {record.getMessage()}"))


STALE_S = 1.0  # 상태가 10Hz로 오므로 이만큼 끊기면 값을 믿지 않는다
SERVO_WARN_C = LIMITS["temperature"][1]  # 서보 건강 대시보드와 같은 주의 기준 (결정 0043-5)
SERVO_WARN_LOAD_PCT = LIMITS["load"][1]
FAULT_TEXT = {"grip_overload": "그리퍼 과부하로 열고 멈췄습니다. 물체와 턱을 확인한 뒤 정지 해제를 눌러 주세요"}


def component_states(status: ArmStatus | None, now: float, comm_lost: bool = False) -> dict[str, tuple[str, str]]:
    """상태줄의 구성 요소 상태: {이름: (ok|warn|error|stale, 설명)}. 값이 끊겼으면 정상이라고 보이지 않는다."""
    if comm_lost:  # 통신이 끊기면 상태도 오지 않아, 값이 오래됐다는 표시보다 오류를 먼저 보여 준다 (decisions/0031)
        return {"버스": ("error", "통신 끊김: 다시 연결이 필요합니다"), "서보": ("stale", "통신이 끊겨 값을 읽지 못합니다")}
    if status is None or now - status.t > STALE_S:
        reason = "상태를 아직 받지 못했습니다" if status is None else f"{now - status.t:.1f}초째 상태가 오지 않습니다"
        return {"버스": ("stale", reason), "서보": ("stale", reason)}
    fault = status.fault or ""
    bus = ("error", f"통신 오류: {fault}") if fault.startswith("comm") else ("ok", "상태 수신 중")
    if fault and not fault.startswith("comm"):
        return {"버스": bus, "서보": ("error", getattr(status, "fault_text", None) or f"오류: {FAULT_TEXT.get(fault, fault)}")}
    warned = [f"{name} {status.temperature_c[name]:.0f}°C" for name in status.temperature_c if status.temperature_c[name] >= SERVO_WARN_C]
    warned += [f"{name} 부하 {load:.0f}%" for name, load in status.load_pct.items() if load >= SERVO_WARN_LOAD_PCT]
    return {"버스": bus, "서보": ("warn", ", ".join(warned)) if warned else ("ok", f"서보 {len(status.temperature_c)}개 정상")}


def run_bands(states: list[tuple[float, dict]], t0: float) -> dict[str, list[tuple[str, float, float]]]:
    """토크·상태·오류 띠: {띠 이름: [(글자, 시작 초, 끝 초)]}. 값이 바뀌는 곳에서만 나누므로 줄 수와 상관없이 짧다. 옛 형식에는 칸이 없어 빈 목록이다."""
    fields = {
        "토크": lambda d: None if "torque_enabled" not in d else "켜짐" if d["torque_enabled"] else "꺼짐",
        "상태": lambda d: PAGE_STATE.get(d["state"], d["state"]) if d.get("state") else None,
        "오류": lambda d: d.get("fault"),
    }
    bands: dict[str, list] = {name: [] for name in fields}
    for t, data in states:
        for name, read in fields.items():
            label, spans = read(data), bands[name]
            if spans and spans[-1][0] == label:
                spans[-1][2] = t - t0
            elif label is not None:
                if spans:
                    spans[-1][2] = t - t0
                spans.append([label, t - t0, t - t0])
    return {name: [(label, round(a, 2), round(b, 2)) for label, a, b in spans] for name, spans in bands.items()}


def page_state(status: ArmStatus | None) -> str:
    return PAGE_STATE.get(status.state, "오류") if status else "대기"


def hottest_text(status: ArmStatus) -> str:
    return f"{max(status.temperature_c.values()):.0f} °C" if status.temperature_c else EMPTY
