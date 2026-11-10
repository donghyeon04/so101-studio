"""서비스 계층: 앱(CLI, 나중에 GUI)이 부르는 유일한 입구 (system-architecture.md 7절)."""
import threading

from ..core.events import EventBus
from ..task.runner import TaskRunner
from ..task.summary import RunSummary
from .arm_service import ArmService
from .perception import PerceptionManager


class Studio:
    """이벤트 구독: studio.bus.subscribe("task", cb) -> cb(TaskEvent). 콜백은 작업 스레드에서 불리므로 GUI는 큐에 넣어 UI 스레드에서 처리한다.
    TaskEvent.state = PREPARE|SCAN|DECIDE|CHECK|RESCAN|PICK|PLACE|DONE|STOPPED|FAULT, detail(상태별; CHECK·RESCAN은 초음파처럼 확인 기능이 있는 인식 방식에서만):
      DECIDE: detail_n(스캔된 수), scan_s(스캔 시간), objects(행: id, x_mm, y_mm, angle_deg, width_mm, radial_mm, reachable), chosen(id|None), reason(문장)
      CHECK의 result가 있는 이벤트: result = ok|empty|unreachable, duration_s(확인 시간). 인식 모듈의 us.sweep·us.candidates·us.check_path·us.check_result도 같은 bus로 나간다.
      PICK/PLACE의 result가 있는 이벤트: duration_s(마지막으로 실행한 동작 시간, 초). FAULT: detail_state."""

    def __init__(self, arm: ArmService, source, dry_run: bool = False):
        self.arm, self.source, self.bus = arm, source, EventBus()
        self.ctl, self.kin, self.cfg, self.ws = arm.ctl, arm.kin, arm.cfg, arm.ws
        self.skills = arm.skills
        self.task = TaskRunner(arm.skills, source, arm.ws, self.bus, accept_no_object=dry_run, arm=arm)
        self.bus.subscribe("task", lambda e: arm._log("task", e))
        self.summary = RunSummary(getattr(source, "name", type(source).__name__))
        self.bus.subscribe("task", self._record_summary)
        self.bus.subscribe("us.check_result", self._record_check)
        self._thread: threading.Thread | None = None
        self.perception = PerceptionManager(self, source)
        self._step = threading.Event()
        self._waiting = False
        self.task.wait_step = self._wait_step  # 단계별 모드: step()이 올 때까지 기다린다 (CLI는 이 함수를 Enter 입력으로 바꿔 쓴다)
        self.result: dict | None = None

    def start(self, step_mode: bool = False, blocking: bool = False) -> dict | None:
        """작업을 시작한다. blocking=False면 스레드에서 돌고 status()로 본다."""
        self._step.clear()  # 이전 stop()·step()이 남긴 신호 때문에 첫 단계가 기다리지 않고 지나가지 않게

        def run() -> None:
            self.summary = RunSummary(getattr(self.task.source, "name", type(self.task.source).__name__))  # 실행마다 새로 모은다(인식 방식은 그 사이 바뀔 수 있다)
            try:
                self.result = self.task.run(step_mode)
            except Exception as e:
                if blocking:
                    raise
                self.task._emit("FAULT", result="exception", error=repr(e))
                self.result = {"picked": self.task.picked, "failed": self.task.failed, "cycles": self.task.cycles, "end": "FAULT", "error": repr(e)}
            finally:
                self._write_summary()

        if blocking:
            run()
            return self.result
        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        return None

    def _record_summary(self, e) -> None:
        """기록 실패가 작업을 멈추면 안 된다: 요약을 모으다 난 예외는 남기기만 한다."""
        try:
            self.summary.on_event(e)
        except Exception as ex:  # noqa: BLE001
            self._summary_failed(ex)

    def _record_check(self, e: dict) -> None:
        """물체별 확인 결과·시간을 summary.json의 per_object에 더한다. 레이더 시간은 DECIDE에서 고른 물체에 붙는다."""
        try:
            extra = {} if e.get("status") == "ok" else {"reason": f"check_{e.get('status')}"}  # 확인에서 빠진 후보는 skipped + 이유
            self.summary.note(e["id"], check=e.get("status"), check_s=round((e.get("info") or {}).get("check_s", 0.0), 3), **extra)
        except Exception as ex:  # noqa: BLE001
            self._summary_failed(ex)

    def _summary_failed(self, ex: Exception) -> None:
        try:
            self.arm._log("summary_failed", {"error": repr(ex)}, "WARN", "ARM-SUMMARY-FAILED")
        except Exception:  # noqa: BLE001, S110 기록도 안 되면 조용히 넘어간다
            pass

    def _write_summary(self) -> None:
        """실행이 끝나거나 멈출 때 runs/<시각>/summary.json을 쓴다(기록 폴더가 없는 시험·시뮬레이션은 건너뛴다)."""
        try:
            lg = getattr(self.arm, "logger", None)
            if lg is not None:
                self.summary.write(lg.dir / "summary.json", self.result)
        except Exception as e:  # noqa: BLE001 끝난 작업이 기록 실패로 예외가 되면 안 된다
            self._summary_failed(e)

    def _wait_step(self) -> None:
        if self.task._stop:  # 정지 요청이 있으면 기다리지 않는다 (stop()의 신호는 한 번만 풀어 주므로)
            return
        self._waiting = True
        self._step.wait()
        self._step.clear()
        self._waiting = False

    def pause(self) -> None:
        """다음 상태 변화에서 멈춘다(단계별 모드로 바꾼다). step()으로 한 단계씩, resume()으로 이어 간다. 어느 스레드에서든 부를 수 있다 (REQ-013)."""
        self.task._step_mode = True

    def resume(self) -> None:
        """일시정지나 단계별 모드를 풀고 끝까지 이어 간다. 기다리는 중이면 풀어 준다."""
        self.task._step_mode = False
        if self._waiting:
            self._step.set()

    def step(self) -> None:
        """단계별 모드(start(step_mode=True))에서 다음 단계로 진행한다. 어느 스레드에서든 부를 수 있다."""
        self._step.set()

    def reload_workspace(self):
        """작업대 설정을 다시 읽는다. 작업이 실행 중이면 거부한다(RuntimeError)."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("작업이 실행 중이라 작업대 설정을 다시 읽을 수 없다")
        self.ws = self.task.ws = self.arm.reload_workspace()
        self.task.refresh_ignore_zones()  # 배출 구역이 바뀌었으면 인식 소스도 따라간다
        return self.ws

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
