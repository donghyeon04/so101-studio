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
        self._status = (self.dir / "status.csv").open("a", encoding="utf-8")
        self._status.write("t,shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper,x_mm,y_mm,z_mm,max_temp_c,max_load_pct,voltage_v,current_a\n")

    def event(self, kind: str, data) -> None:
        if dataclasses.is_dataclass(data):
            data = dataclasses.asdict(data)
        self._events.write(json.dumps({"t": time.time(), "kind": kind, "data": data}, ensure_ascii=False) + "\n")
        self._events.flush()

    def status(self, st) -> None:
        """ArmStatus 한 줄(10Hz): 관절 6개, 끝 위치, 최고 온도, 최대 부하, 최저 전압 (F1·F2 기록의 원천)."""
        j = dataclasses.astuple(st.joints)
        row = [st.t, *j, st.tool.x_mm, st.tool.y_mm, st.tool.z_mm, max(st.temperature_c.values()), max(st.load_pct.values()), st.voltage_v, st.current_ma / 1000]
        self._status.write(",".join(f"{v:.3f}" for v in row) + "\n")
        self._status.flush()

    def close(self) -> None:
        self._events.close()
        self._status.close()
