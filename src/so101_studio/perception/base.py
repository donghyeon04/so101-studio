"""인식 모듈 공통 규격 (system-architecture.md 4절). 작업 실행 모듈은 어떤 방식인지 모른다."""
from dataclasses import dataclass
from typing import Protocol

from ..core.types import DetectedObject, ScanResult


class PerceptionSource(Protocol):
    name: str

    def connect(self) -> None: ...

    def calibrate(self) -> dict: ...  # 빈 작업대 기준값. 결과 요약을 돌려준다

    def scan(self) -> ScanResult: ...

    def notify_picked(self, object_id: str) -> None: ...  # 집어서 옮긴 물체. 센서 없는 방식은 이걸로 다음 스캔에서 뺀다

    def close(self) -> None: ...


@dataclass
class PathSegment:  # 작업대 좌표, 손끝은 수직 아래 (ultrasonic-spec 8.2절)
    points_mm: list[tuple[float, float]]  # 손끝 xy (센서 xy 아님)
    tool_z_mm: float  # 작업대 면 위 손끝 높이
    speed_mm_s: float  # 실제 속도 = min(이 값, tool_v_max_mm_s × 속도%)
    yaw_deg: float | None = None  # 경로 동안 고정할 손목 각도. None = 지금 값 유지
    stage: str | None = None  # 확인 단계 이름(L1·L2·R·L34·L3·L4·GRID, check.Probe가 채운다). 작업 실행의 멈춤 고리(pace)가 본다(ultrasonic-spec 8.9)


@dataclass
class PoseSample:  # 팔 50Hz 제어 틱마다 한 개
    t_ns: int  # 제어 시계의 ns(MotionController.clock.now_ns, 기본 WallClock = time.perf_counter_ns, mujoco-spec 13.10.2)
    tool_mm: tuple[float, float, float]  # 손끝 위치(작업대 좌표, z는 작업대 면 위 높이)
    sensor_mm: tuple[float, float, float]  # 팔 센서 위치 = 손끝 + yaw로 돌린 arm_sensor_offset_mm (z는 작업대 면 위 높이)


class CheckProbe(Protocol):
    def next_segment(self) -> PathSegment | None: ...

    def feed(self, poses: list[PoseSample], status: str = "ok") -> None: ...  # status = follow_path 결과 상태

    def result(self) -> DetectedObject | None: ...  # None = 물체 없음


class CheckableSource(PerceptionSource, Protocol):
    def start_check(self, candidate: DetectedObject) -> CheckProbe: ...

    def prefetch(self, arm_links_fn) -> None: ...  # 놓는 동안 레이더 미리 돌리기 (T5)
