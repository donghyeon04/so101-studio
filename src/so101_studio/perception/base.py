"""인식 모듈 공통 규격 (system-architecture.md 4절). 작업 실행 모듈은 어떤 방식인지 모른다."""
from typing import Protocol

from ..core.types import ScanResult


class PerceptionSource(Protocol):
    name: str

    def connect(self) -> None: ...

    def calibrate(self) -> dict: ...  # 빈 작업대 기준값. 결과 요약을 돌려준다

    def scan(self) -> ScanResult: ...

    def notify_picked(self, object_id: str) -> None: ...  # 집어서 옮긴 물체. 센서 없는 방식은 이걸로 다음 스캔에서 뺀다

    def close(self) -> None: ...
