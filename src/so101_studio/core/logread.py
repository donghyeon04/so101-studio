"""기록 읽기 공개 API (decisions/0030, GUI 로그·실행 기록·사고 목록이 쓴다). 깨진 줄은 건너뛴다. 옛 형식(t가 초 단위 실수, status.csv)도 새 머리로 바꿔 돌려준다.

- list_runs(root) -> 실행 폴더 이름(최근 순)
- read_events(run_dir), read_status(run_dir) -> 공통 머리 dict 목록(시간 순)
- merge_timeline(root, start_ns, end_ns) -> 여러 프로세스·실행의 기록을 t로 합친 한 시간축
- list_incidents(root) -> 사고 묶음 목록(최근 순), read_incident(path) -> 묶음 내용
"""
import csv
import json
from pathlib import Path

from .logging import status_files

LOG_NAMES = ("events.jsonl", "watchdog.jsonl")


def _norm(rec: dict, proc: str = "cli") -> dict:
    """옛 형식(t 초 단위 실수, level·code 없음)을 새 공통 머리로 바꾼다."""
    if "level" not in rec:
        t = rec.get("t", 0)
        rec = {"t": int(t * 1e9) if isinstance(t, float) or t < 1e11 else t, "proc": proc, "pid": 0, "seq": 0, "level": "INFO", "code": str(rec.get("kind", "")).upper(),
               "kind": rec.get("kind"), "data": rec.get("data")}
    return rec


def read_jsonl(path: str | Path, proc: str = "cli") -> list[dict]:
    path = Path(path)
    out = []
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict) and "t" in rec:
            out.append(_norm(rec, proc))
    return out


def list_runs(root: str | Path = "runs") -> list[str]:
    root = Path(root)
    if not root.is_dir():
        return []
    ok = ("events.jsonl", "status.jsonl", "status.csv")
    return sorted((d.name for d in root.iterdir() if d.is_dir() and d.name != "incidents" and any((d / n).exists() for n in ok)), reverse=True)


def read_events(run_dir: str | Path) -> list[dict]:
    return read_jsonl(Path(run_dir) / "events.jsonl")


def read_status(run_dir: str | Path) -> list[dict]:
    """상태 줄(나눈 파일 포함). 옛 status.csv만 있으면 같은 모양의 data로 바꿔 돌려준다(joints, tool, max_temp_c, max_load_pct, voltage_v, current_ma)."""
    run_dir = Path(run_dir)
    out: list[dict] = []
    for p in status_files(run_dir):
        out += read_jsonl(p)
    if out or not (run_dir / "status.csv").exists():
        return out
    rows = csv.DictReader((run_dir / "status.csv").read_text(encoding="utf-8", errors="replace").splitlines())
    for i, r in enumerate(rows):
        try:
            joints = {k: float(r[k]) for k in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
            out.append({"t": int(float(r["t"]) * 1e9), "proc": "cli", "pid": 0, "seq": i, "level": "INFO", "code": "STATUS", "kind": "status",
                        "data": {"joints": joints, "tool": {"x_mm": float(r["x_mm"]), "y_mm": float(r["y_mm"]), "z_mm": float(r["z_mm"])}, "max_temp_c": float(r["max_temp_c"]),
                                 "max_load_pct": float(r["max_load_pct"]), "voltage_v": float(r["voltage_v"]), "current_ma": float(r["current_a"]) * 1000}})
        except (ValueError, KeyError, TypeError):
            continue  # 잘린 마지막 줄 등
    return out


def merge_timeline(root: str | Path, start_ns: int, end_ns: int, include_status: bool = True) -> list[dict]:
    """root 아래 모든 실행 폴더(와 감시 기록)에서 [start_ns, end_ns]의 줄을 모아 (t, proc, pid, seq) 순서로 합친다. 줄마다 file(실행 폴더/파일)이 더해진다."""
    root = Path(root)
    out: list[dict] = []
    if not root.is_dir():
        return out
    lo_s = start_ns / 1e9 - 2  # 이 시각 전에 마지막으로 수정된 파일에는 창 안의 줄이 없다
    for d in (d for d in root.iterdir() if d.is_dir() and d.name != "incidents"):
        files = [d / n for n in LOG_NAMES] + (status_files(d) if include_status else [])
        for p in files:
            if not p.exists() or p.stat().st_mtime < lo_s:
                continue
            for rec in read_jsonl(p, "watchdog" if p.name == "watchdog.jsonl" else "cli"):
                if start_ns <= rec["t"] <= end_ns:
                    out.append({**rec, "file": f"{d.name}/{p.name}"})
    return sorted(out, key=lambda r: (r["t"], r["proc"], r["pid"], r["seq"]))


def list_incidents(root: str | Path = "runs") -> list[dict]:
    """사고 묶음 목록(최근 순): [{name, path, code, reason, t, level}]."""
    base = Path(root) / "incidents"
    out = []
    for d in sorted((d for d in base.iterdir() if d.is_dir() and (d / "meta.json").exists()), reverse=True) if base.is_dir() else []:
        try:
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            meta = {}
        out.append({"name": d.name, "path": str(d), "code": meta.get("code"), "reason": meta.get("reason"), "t": meta.get("t"), "proc": meta.get("proc")})
    return out


def read_incident(path: str | Path) -> dict:
    """{meta, ring, timeline, config_sha256, doctor}. 없는 파일은 빈 값."""
    p = Path(path)

    def js(name, default):
        try:
            return json.loads((p / name).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return default

    return {"meta": js("meta.json", {}), "ring": read_jsonl(p / "ring.jsonl"), "timeline": read_jsonl(p / "timeline.jsonl"), "config_sha256": js("config.sha256.json", {}), "doctor": js("doctor.json", None)}
