"""서비스 계층: 앱(CLI, 나중에 GUI)이 부르는 유일한 입구 (system-architecture.md 7절)."""
import threading

from ..arm.kinematics import Kinematics
from ..arm.motion import MotionController
from ..arm.safety import Safety
from ..arm.skills import Skills
from ..core.events import EventBus
from ..core.logging import RunLogger
from ..task.runner import TaskRunner


class Studio:
    def __init__(self, driver, cfg, ws, source, speed_pct: float | None = None, log_root: str | None = "runs"):
        self.cfg, self.ws, self.source, self.bus = cfg, ws, source, EventBus()
        self.kin = Kinematics(cfg.urdf, ws.base)
        self.ctl = MotionController(
            driver, self.kin, Safety(cfg.joint_limits_deg, ws, self.kin, z_floor_mm=cfg.tip_floor_mm, table_z_mm=cfg.table_z_mm), cfg.control_hz
        )
        self.skills = Skills(self.ctl, self.kin, cfg, ws, speed_pct)
        self.task = TaskRunner(self.skills, source, ws, self.bus)
        self.logger = RunLogger(log_root) if log_root else None
        if self.logger:
            self.bus.subscribe("task", lambda e: self.logger.event("task", e))
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
        self.ctl.close()
        if self.logger:
            self.logger.close()
