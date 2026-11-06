"""실행 요약 summary.json (system-architecture.md 10절). TaskEvent에서 물체 수, 물체별 결과, 단계별 소요 시간을 모은다.
단계 시간: 이벤트 사이의 간격을 앞 이벤트의 상태(PREPARE·SCAN·DECIDE·CHECK·PICK·PLACE…)에 더한다. result가 있는 이벤트는 단계를 끝내는 표시라 간격을 더하지 않는다."""
import json
import threading
from pathlib import Path

from ..core.types import TaskEvent

END_STATES = ("DONE", "STOPPED", "FAULT")


class RunSummary:
    def __init__(self, source_name: str):
        self.source_name = source_name
        self._lock = threading.Lock()
        self._objects: dict[str, dict] = {}
        self._stages: dict[str, float] = {}
        self._prev: TaskEvent | None = None
        self._t0: float | None = None
        self._t1: float | None = None

    def _row(self, oid: str) -> dict:
        return self._objects.setdefault(oid, {"id": oid, "result": "skipped", "reason": None, "stages_s": {}})

    def on_event(self, e: TaskEvent) -> None:
        with self._lock:
            self._t0 = e.t if self._t0 is None else self._t0
            self._t1 = e.t
            p = self._prev
            if p is not None and p.result is None and p.state not in END_STATES:
                dt = e.t - p.t
                self._stages[p.state] = self._stages.get(p.state, 0.0) + dt
                if p.object_id:
                    st = self._row(p.object_id)["stages_s"]
                    st[p.state] = st.get(p.state, 0.0) + dt
            self._prev = e
            if e.state == "DECIDE":
                if e.detail.get("chosen") and e.detail.get("scan_s"):  # 레이더가 이 물체를 고르기까지 걸린 시간(초음파)
                    row = self._row(e.detail["chosen"])
                    row["radar_s"] = round(row.get("radar_s", 0.0) + e.detail["scan_s"], 3)
                for r in e.detail.get("objects", []):
                    row = self._row(r["id"])
                    if row["result"] == "skipped" and not r.get("reachable", True):
                        row["reason"] = "out_of_reach"
            elif e.state == "PICK" and e.object_id and e.result is not None:
                row = self._row(e.object_id)
                row["result"], row["reason"] = "failed", e.result
            elif e.state == "PLACE" and e.object_id and e.result == "ok":
                row = self._row(e.object_id)
                row["result"], row["reason"] = "ok", None

    def note(self, object_id: str, **fields) -> None:
        """물체 행에 값을 더한다(초음파 인식이 물체별 레이더·확인 시간을 넣을 때 쓴다)."""
        with self._lock:
            self._row(object_id).update(fields)

    def to_dict(self, result: dict | None = None) -> dict:
        with self._lock:
            rows = [{**r, "stages_s": {k: round(v, 3) for k, v in r["stages_s"].items()}} for r in self._objects.values()]
            return {
                "source": self.source_name,
                "end": (result or {}).get("end"),
                "objects": len(rows),
                "succeeded": sum(r["result"] == "ok" for r in rows),
                "failed": sum(r["result"] == "failed" for r in rows),
                "skipped": sum(r["result"] == "skipped" for r in rows),
                "total_s": round((self._t1 or 0.0) - (self._t0 or 0.0), 3),
                "stages_s": {k: round(v, 3) for k, v in self._stages.items()},
                "per_object": rows,
            }

    def write(self, path: str | Path, result: dict | None = None) -> None:
        Path(path).write_text(json.dumps(self.to_dict(result), ensure_ascii=False, indent=2), encoding="utf-8")
