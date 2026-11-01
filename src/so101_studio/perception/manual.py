"""ManualSource: 사람이 적어 둔 좌표(config/manual_objects.yaml)를 그대로 돌려준다.
센서 없이 전체 흐름(스캔 -> 집기 -> 반복)을 개발·시연할 수 있다."""
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

    def connect(self) -> None:
        pass

    def calibrate(self) -> dict:
        return {"objects": len(self._objects)}

    def scan(self) -> ScanResult:
        t0 = time.perf_counter()
        left = [o for o in self._objects if o.id not in self._picked]
        return ScanResult(left, time.perf_counter() - t0, {"picked": sorted(self._picked)})

    def notify_picked(self, object_id: str) -> None:
        self._picked.add(object_id)

    def close(self) -> None:
        pass
