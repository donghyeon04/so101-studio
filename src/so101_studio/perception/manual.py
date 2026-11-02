"""ManualSource: 사람이 적어 둔 좌표(config/manual_objects.yaml)를 그대로 돌려준다.
센서 없이 전체 흐름(스캔 -> 집기 -> 반복)을 개발·시연할 수 있다."""
import threading
import time
from pathlib import Path

import yaml

from ..core.types import DetectedObject, ScanResult


class ManualSource:
    name = "manual"

    def __init__(self, path: str | Path = "config/manual_objects.yaml"):
        rows = yaml.safe_load(Path(path).read_text(encoding="utf-8"))["objects"]
        self._objects = [
            DetectedObject(id=f"obj-{i}", confidence=1.0, source="manual", **row) for i, row in enumerate(rows, 1)
        ]
        self._picked: set[str] = set()
        self._lock = threading.Lock()  # GUI 스레드의 add/remove와 작업 스레드의 scan이 동시에 불린다

    def connect(self) -> None:
        pass

    def calibrate(self) -> dict:
        return {"objects": len(self._objects)}

    def scan(self) -> ScanResult:
        t0 = time.perf_counter()
        with self._lock:
            left = [o for o in self._objects if o.id not in self._picked]
            picked = sorted(self._picked)
        return ScanResult(left, time.perf_counter() - t0, {"picked": picked})

    def notify_picked(self, object_id: str) -> None:
        with self._lock:
            self._picked.add(object_id)

    def add(self, obj: DetectedObject) -> None:
        """물체를 더한다. 다음 scan()부터 보인다 (REQ-003)."""
        with self._lock:
            if any(o.id == obj.id for o in self._objects):
                raise ValueError(f"이미 있는 물체 id: {obj.id}")
            self._objects.append(obj)

    def remove(self, object_id: str) -> None:
        with self._lock:
            n = len(self._objects)
            self._objects = [o for o in self._objects if o.id != object_id]
            if len(self._objects) == n:
                raise KeyError(object_id)
            self._picked.discard(object_id)

    def close(self) -> None:
        pass
