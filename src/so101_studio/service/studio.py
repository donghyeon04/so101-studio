"""서비스 계층: 앱(CLI, 나중에 GUI)이 부르는 유일한 입구 (system-architecture.md 7절)."""
import threading

from ..core.events import EventBus
from ..task.runner import TaskRunner
from .arm_service import ArmService
from .perception import PerceptionManager


class Studio:
    """이벤트 구독: studio.bus.subscribe("task", cb) -> cb(TaskEvent). 콜백은 작업 스레드에서 불리므로 GUI는 큐에 넣어 UI 스레드에서 처리한다.
    TaskEvent.state = PREPARE|SCAN|DECIDE|PICK|PLACE|DONE|STOPPED|FAULT, detail(상태별):
      DECIDE: detail_n(스캔된 수), scan_s(스캔 시간), objects(행: id, x_mm, y_mm, angle_deg, width_mm, radial_mm, reachable), chosen(id|None), reason(문장)
      PICK/PLACE의 result가 있는 이벤트: duration_s(마지막으로 실행한 동작 시간, 초). FAULT: detail_state."""

    def __init__(self, arm: ArmService, source, dry_run: bool = False):
        self.arm, self.source, self.bus = arm, source, EventBus()
        self.ctl, self.kin, self.cfg, self.ws = arm.ctl, arm.kin, arm.cfg, arm.ws
        self.skills = arm.skills
        self.task = TaskRunner(arm.skills, source, arm.ws, self.bus, accept_no_object=dry_run)
        self.bus.subscribe("task", lambda e: arm._log("task", e))
        self._thread: threading.Thread | None = None
        self.perception = PerceptionManager(self, source)
        self._step = threading.Event()
        self.task.wait_step = self._wait_step  # 단계별 모드: step()이 올 때까지 기다린다 (CLI는 이 함수를 Enter 입력으로 바꿔 쓴다)
        self.result: dict | None = None

    def start(self, step_mode: bool = False, blocking: bool = False) -> dict | None:
        """작업을 시작한다. blocking=False면 스레드에서 돌고 status()로 본다."""
        run = lambda: setattr(self, "result", self.task.run(step_mode))
        if blocking:
            run()
            return self.result
        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        return None

    def _wait_step(self) -> None:
        self._step.wait()
        self._step.clear()

    def step(self) -> None:
        """단계별 모드(start(step_mode=True))에서 다음 단계로 진행한다. 어느 스레드에서든 부를 수 있다."""
        self._step.set()

    def status(self) -> dict:
        return {"state": self.task.state, "arm": self.ctl.state, "result": self.result}

    def stop(self) -> None:
        self.task.stop()
        self._step.set()  # 단계 대기 중이면 풀어 준다

    def estop(self) -> None:
        """작업 중단 -> 팔 비상 정지 한 번에 (토크는 끄지 않는다)."""
        self.stop()
        self.ctl.estop()

    def close(self) -> None:
        self.arm.disconnect()
