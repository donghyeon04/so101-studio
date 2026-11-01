"""실행 기록: runs/YYYYMMDD-HHMMSS/events.jsonl (system-architecture.md 10절)."""
import dataclasses
import json
import time
from datetime import datetime
from pathlib import Path


class RunLogger:
    def __init__(self, root: str | Path = "runs"):
        self.dir = Path(root) / datetime.now().strftime("%Y%m%d-%H%M%S")
        (self.dir / "scans").mkdir(parents=True)
        self._events = (self.dir / "events.jsonl").open("a", encoding="utf-8")

    def event(self, kind: str, data) -> None:
        if dataclasses.is_dataclass(data):
            data = dataclasses.asdict(data)
        self._events.write(json.dumps({"t": time.time(), "kind": kind, "data": data}, ensure_ascii=False) + "\n")
        self._events.flush()

    def close(self) -> None:
        self._events.close()
