"""작업 실행: 상태 머신 (system-architecture.md 5절).
IDLE -> PREPARE -> SCAN -> DECIDE -> PICK -> PLACE -> (물체 하나마다 다시 PREPARE) ... -> DONE"""
import time
from collections.abc import Callable

from ..core.events import EventBus
from ..core.types import TaskEvent
from .policy import choose

MAX_CYCLES = 20  # 사이클 상한
MAX_CONSECUTIVE_FAILS = 3  # 같은 물체를 연속으로 이만큼 실패하면 끝낸다


class TaskRunner:
    def __init__(self, skills, source, ws, bus: EventBus | None = None, zone: str = "right", accept_no_object: bool = False):
        self.skills, self.source, self.ws, self.zone = skills, source, ws, zone
        self.accept_no_object = accept_no_object  # 물체 없이 동작만 보일 때(dry-run): 접촉이 없어도(no_object) 성공으로 보고 이어간다
        self.bus = bus or EventBus()
        self.state = "IDLE"
        self._stop = False
        self._step_mode = False
        self.wait_step: Callable[[], None] = lambda: None  # 단계별 모드에서 다음 단계를 기다리는 함수(CLI는 Enter, GUI는 step())

    def stop(self) -> None:
        self._stop = True
        self.skills.ctl.stop()

    def _emit(self, state: str, object_id: str | None = None, result: str | None = None, **detail) -> None:
        self.state = state
        self.bus.publish("task", TaskEvent(time.time(), state, object_id, result, detail))
        if self._step_mode and state not in ("DONE", "STOPPED", "FAULT"):
            self.wait_step()  # 상태가 바뀔 때마다 멈추고 다음 단계를 기다린다 (D3)

    def _stopped(self) -> bool:
        if self._stop:
            self._emit("STOPPED")
        return self._stop

    def _ok(self, r) -> bool:
        return r.status == "ok" or (self.accept_no_object and r.status == "no_object")

    def _end(self, end: str) -> dict:
        return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": end}

    def _fault(self, where: str) -> dict:
        self._emit("FAULT", result=where, detail_state=self.skills.ctl.state)
        return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": "FAULT"}

    def run(self, step_mode: bool = False) -> dict:
        self._step_mode, self._stop = step_mode, False
        self.picked, self.failed, self.cycles, fails = [], [], 0, {}
        self.source.connect()
        while True:
            if self.cycles >= MAX_CYCLES:
                self._emit("DONE", result="cycle_limit")
                return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": "cycle_limit"}
            self._emit("PREPARE")
            if self._stopped():
                return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": "STOPPED"}
            if self.skills.park().status != "ok":
                return self._fault("park")
            self._emit("SCAN")
            scan = self.source.scan()
            self._emit("DECIDE", detail_n=len(scan.objects))
            obj = choose(scan.objects, self.ws)
            if obj is None:
                self._emit("DONE", result="no_objects")
                return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": "no_objects"}
            self.cycles += 1
            self._emit("PICK", obj.id)
            if self._stopped():
                return self._end("STOPPED")
            r = self.skills.pick(obj)
            if not self._ok(r):  # 재시도 1회
                if self._stop:
                    self._stopped()
                    return self._end("STOPPED")
                if self.skills.ctl.state == "fault":
                    return self._fault("pick")
                r = self.skills.pick(obj)
            if not self._ok(r):
                fails[obj.id] = fails.get(obj.id, 0) + 1
                self.failed.append(obj.id)
                self._emit("PICK", obj.id, result=r.status)
                if fails[obj.id] >= MAX_CONSECUTIVE_FAILS:
                    self._emit("DONE", obj.id, result="repeated_failure")
                    return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": "repeated_failure"}
                continue
            if self._stopped():
                return self._end("STOPPED")
            self._emit("PLACE", obj.id)
            r = self.skills.place(self.zone)
            if r.status != "ok":
                return self._fault("place")
            fails.pop(obj.id, None)
            self.picked.append(obj.id)
            self.source.notify_picked(obj.id)
            self._emit("PLACE", obj.id, result="ok")
