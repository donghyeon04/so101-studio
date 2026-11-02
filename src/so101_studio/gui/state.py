"""화면이 읽는 상태. 상태 스레드가 쓰고 화면 타이머가 읽는다 (gui-spec 8절)."""

from dataclasses import dataclass

from ..core.types import ArmStatus

EMPTY = "—"
PAGE_STATE = {"idle": "대기", "moving": "실행 중", "teleop": "실행 중", "teach": "실행 중", "stopped": "정지", "fault": "오류"}
SAFETY_TEXT = {"normal": "보통", "free": "자유", "off": "끔"}


@dataclass
class State:
    status: ArmStatus | None = None

    def update(self, status: ArmStatus) -> None:
        self.status = status


def page_state(status: ArmStatus | None) -> str:
    return PAGE_STATE.get(status.state, "오류") if status else "대기"


def hottest_text(status: ArmStatus) -> str:
    return f"{max(status.temperature_c.values()):.0f} °C" if status.temperature_c else EMPTY
