"""블랙박스 기록 엔진 (decisions/0030, arm-software-spec.md 8절): runs/YYYYMMDD-HHMMSS/events.jsonl, status.jsonl.

모든 줄은 JSON Lines이고 공통 머리 t(벽시계 ns), proc, pid, seq, level, code, kind, data를 가진다.
제어·상태 루프는 큐에만 넣고 기록 스레드가 쓴다. 줄마다 flush, WARN 이상은 fsync.
큐가 차면 상태 줄만 버리고 수를 센다(다음 상태 줄의 data.dropped). 이벤트는 버리지 않는다."""
import csv
import dataclasses
import itertools
import json
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

from .retention import prune_runs

LEVELS = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}
FSYNC_LEVEL = LEVELS["WARN"]
QUEUE_LINES = 10_000
STATUS_ROTATE_BYTES = 50 * 1024 * 1024


def _plain(o):
    if dataclasses.is_dataclass(o) and not isinstance(o, type):
        return dataclasses.asdict(o)
    if hasattr(o, "tolist"):
        return o.tolist()
    return str(o)


def dumps(rec: dict) -> str:
    return json.dumps(rec, ensure_ascii=False, default=_plain)


def default_proc() -> str:
    return os.environ.get("SO101_PROC", "cli")


def make_record(proc: str, seq: int, level: str, code: str, kind: str, data, t_ns: int | None = None) -> dict:
    if level not in LEVELS:
        raise ValueError(f"level은 {list(LEVELS)} 중 하나")
    return {"t": time.time_ns() if t_ns is None else t_ns, "proc": proc, "pid": os.getpid(), "seq": seq, "level": level, "code": code, "kind": kind, "data": data}


class JsonlFile:
    """한 프로세스가 한 파일에 같은 머리로 동기로 쓴다(감시 프로세스처럼 줄 수가 적은 곳). 줄마다 flush, WARN 이상은 fsync."""

    def __init__(self, path: str | Path, proc: str):
        self.path, self.proc, self._seq, self._lock = Path(path), proc, itertools.count(1), threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self.path.open("a", encoding="utf-8")

    def write(self, kind: str, data=None, level: str = "INFO", code: str | None = None) -> int:
        with self._lock:
            seq = next(self._seq)
            self._f.write(dumps(make_record(self.proc, seq, level, code or kind.upper(), kind, data)) + "\n")
            self._f.flush()
            if LEVELS[level] >= FSYNC_LEVEL:
                os.fsync(self._f.fileno())
        return seq

    def close(self) -> None:
        with self._lock:
            self._f.close()


class RunLogger:
    def __init__(self, root: str | Path = "runs", proc: str | None = None, suffix: str | None = None, queue_lines: int = QUEUE_LINES,
                 status_rotate_bytes: int = STATUS_ROTATE_BYTES, prune: bool = True):
        self.proc = proc or default_proc()
        root = Path(root)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S") + (f"-{suffix}" if suffix else "")
        self.dir, n = root / stamp, 1
        root.mkdir(parents=True, exist_ok=True)
        while True:
            try:
                (self.dir / "scans").mkdir(parents=True)
                break
            except FileExistsError:
                n += 1
                self.dir = root / f"{stamp}-{n}"
        if prune:
            try:
                prune_runs(root, protect=[self.dir])
            except Exception:  # noqa: BLE001, S110 보관 정리가 실패해도 기록과 동작은 계속한다
                pass
        self.queue_lines, self.status_rotate_bytes = queue_lines, status_rotate_bytes
        self.dropped_total = 0
        self.write_errors = 0
        self._dropped = 0
        self._pending_status = 0
        self._lock = threading.Lock()
        self._seq = itertools.count(1)
        self._q: queue.Queue = queue.Queue()
        self._events = (self.dir / "events.jsonl").open("a", encoding="utf-8")
        self._part, self._status_bytes = 0, 0
        self._status = (self.dir / "status.jsonl").open("a", encoding="utf-8")
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="RunLogger", daemon=True)
        self._thread.start()

    # ---- 호출하는 쪽(제어·상태 루프 포함): 큐에만 넣는다
    def event(self, kind: str, data=None, level: str = "INFO", code: str | None = None) -> int:
        """이벤트 한 줄. 버리지 않는다. seq를 돌려준다."""
        if dataclasses.is_dataclass(data) and not isinstance(data, type):
            data = dataclasses.asdict(data)
        with self._lock:
            rec = make_record(self.proc, next(self._seq), level, code or kind.upper(), kind, data)
            self._q.put(("events", rec))
        return rec["seq"]

    def status(self, data) -> bool:
        """상태 한 줄(10Hz). 큐가 가득 차 있으면 버리고 False(수는 다음 줄 data.dropped에 남는다)."""
        if dataclasses.is_dataclass(data) and not isinstance(data, type):
            data = dataclasses.asdict(data)
        with self._lock:
            if self._pending_status >= self.queue_lines:
                self._dropped += 1
                self.dropped_total += 1
                return False
            if self._dropped:
                data = {**data, "dropped": self._dropped}
                self._dropped = 0
            self._pending_status += 1
            self._q.put(("status", make_record(self.proc, next(self._seq), "INFO", "STATUS", "status", data)))
        return True

    def flush(self, timeout: float = 5.0) -> None:
        """큐에 있는 줄이 모두 파일에 쓰일 때까지 기다린다(시험과 종료 직전용)."""
        end = time.monotonic() + timeout
        while self._q.unfinished_tasks and time.monotonic() < end:
            time.sleep(0.005)

    # ---- 기록 스레드
    def _run(self) -> None:
        while True:
            item = self._q.get()
            try:
                if item is None:
                    return
                stream, rec = item
                if stream == "status":
                    with self._lock:
                        self._pending_status -= 1
                self._write_line(stream, rec)
            except Exception:  # noqa: BLE001 기록 스레드는 어떤 쓰기 오류에도 죽지 않는다
                self.write_errors += 1
            finally:
                self._q.task_done()

    def _write_line(self, stream: str, rec: dict) -> None:
        line = dumps(rec) + "\n"
        f = self._events if stream == "events" else self._status
        f.write(line)
        f.flush()
        if LEVELS[rec["level"]] >= FSYNC_LEVEL:
            os.fsync(f.fileno())
        if stream == "status":
            self._status_bytes += len(line.encode("utf-8"))
            if self._status_bytes >= self.status_rotate_bytes:  # 50MB마다 나눈다: status.001.jsonl, status.002.jsonl ...
                self._status.close()
                self._part += 1
                self._status = (self.dir / f"status.{self._part:03d}.jsonl").open("a", encoding="utf-8")
                self._status_bytes = 0

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._q.put(None)
        self._thread.join(timeout=5.0)
        self._events.close()
        self._status.close()


CSV_COLUMNS = ["t", "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper", "x_mm", "y_mm", "z_mm", "max_temp_c", "max_load_pct", "voltage_v", "current_a"]


def status_files(run_dir: str | Path) -> list[Path]:
    run_dir = Path(run_dir)
    return ([run_dir / "status.jsonl"] if (run_dir / "status.jsonl").exists() else []) + sorted(run_dir.glob("status.[0-9][0-9][0-9].jsonl"))


def status_to_csv(run_dir: str | Path, out: str | Path | None = None) -> Path:
    """status.jsonl(나눈 파일 포함)을 옛 status.csv 열로 내보낸다(F2 CSV 내보내기). 깨진 줄은 건너뛴다."""
    run_dir = Path(run_dir)
    out = Path(out) if out else run_dir / "status.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLUMNS)
        for p in status_files(run_dir):
            for line in p.read_text(encoding="utf-8").splitlines():
                try:
                    r = json.loads(line)
                    d = r["data"]
                    j, tool = d["joints"], d["tool"]
                    w.writerow([f"{r['t'] / 1e9:.3f}", *(f"{j[n]:.3f}" for n in CSV_COLUMNS[1:7]), f"{tool['x_mm']:.3f}", f"{tool['y_mm']:.3f}", f"{tool['z_mm']:.3f}",
                                f"{max(d['temperature_c'].values()):.3f}", f"{max(d['load_pct'].values()):.3f}", f"{d['voltage_v']:.3f}", f"{d['current_ma'] / 1000:.3f}"])
                except (ValueError, KeyError, TypeError):
                    continue
    return out
