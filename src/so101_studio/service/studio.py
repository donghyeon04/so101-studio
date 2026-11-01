"""서비스 계층: 앱(CLI, 나중에 GUI)이 부르는 유일한 입구 (system-architecture.md 7절)."""
import threading

from ..core.events import EventBus
from ..task.runner import TaskRunner
from .arm_service import ArmService


class Studio:
    def __init__(self, arm: ArmService, source, dry_run: bool = False):
        self.arm, self.source, self.bus = arm, source, EventBus()
        self.ctl, self.kin, self.cfg, self.ws = arm.ctl, arm.kin, arm.cfg, arm.ws
        self.skills = arm.skills
        self.task = TaskRunner(arm.skills, source, arm.ws, self.bus, accept_no_object=dry_run)
        self.bus.subscribe("task", lambda e: arm._log("task", e))
        self._thread: threading.Thread | None = None
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

    def status(self) -> dict:
        return {"state": self.task.state, "arm": self.ctl.state, "result": self.result}

    def stop(self) -> None:
        self.task.stop()

    def estop(self) -> None:
        """작업 중단 -> 팔 비상 정지 한 번에 (토크는 끄지 않는다)."""
        self.task.stop()
        self.ctl.estop()

    def close(self) -> None:
        self.arm.disconnect()
