"""화면이 읽는 상태. 상태 스레드가 쓰고 화면 타이머가 읽는다 (gui-spec 8절)."""

import logging
from collections import deque
from dataclasses import dataclass, field

from ..core.types import ArmStatus, TaskEvent

EMPTY = "—"
PAGE_STATE = {"idle": "대기", "moving": "실행 중", "teleop": "실행 중", "teach": "실행 중", "stopped": "정지", "fault": "오류"}
SAFETY_TEXT = {"normal": "보통", "free": "자유", "off": "끔"}
TASK_TEXT = {
    "IDLE": "대기", "PREPARE": "준비", "SCAN": "스캔", "DECIDE": "판단", "PICK": "집기", "PLACE": "놓기",
    "DONE": "완료", "STOPPED": "정지", "FAULT": "오류",
}
PROGRESS = ("준비", "스캔", "판단", "집기", "놓기", "다시 스캔")
_PROGRESS_INDEX = {"PREPARE": 0, "SCAN": 1, "DECIDE": 2, "PICK": 3, "PLACE": 4}


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


class LogBook(logging.Handler):
    """파이썬 로그를 모은다. 실행 기록(events.jsonl)과 합쳐 로그 페이지에 보인다."""

    def __init__(self, maxlen: int = 2000):
        super().__init__(level=logging.WARNING)
        self.records: deque = deque(maxlen=maxlen)

    def emit(self, record: logging.LogRecord) -> None:
        level = "ERROR" if record.levelno >= logging.ERROR else "WARNING"
        self.records.append(LogRecord(record.created, level, f"{record.name}: {record.getMessage()}"))


def page_state(status: ArmStatus | None) -> str:
    return PAGE_STATE.get(status.state, "오류") if status else "대기"


def hottest_text(status: ArmStatus) -> str:
    return f"{max(status.temperature_c.values()):.0f} °C" if status.temperature_c else EMPTY
