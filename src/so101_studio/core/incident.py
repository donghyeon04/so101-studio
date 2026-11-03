"""사고 묶음 (decisions/0030): fault·E-STOP·감시 작동·ARM-TORQUE-LOST·처리 안 된 예외 때 runs/incidents/<YYYYmmdd-HHMMSS>-<code>/를 만든다.
내용: ring.jsonl(링 버퍼 최근 10초), timeline.jsonl(모든 프로세스 기록의 앞뒤 60초), robot.yaml·팔 파일·local.yaml·캘리브레이션 사본과 sha256, doctor.json, git 커밋, 패키지 버전.
앞 60초는 만들 때 바로 쓰고, 뒤 60초는 지난 뒤(또는 finalize() 때) timeline.jsonl을 다시 쓴다. 사고 묶음은 자동으로 지우지 않는다."""
import hashlib
import importlib.metadata as md
import json
import platform
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from .logging import dumps
from .logread import merge_timeline

WINDOW_S = 60.0
PACKAGES = ("lerobot", "numpy", "placo", "pyserial", "ruckig", "draccus", "pyyaml", "nicegui", "viser")
REPO_ROOT = Path(__file__).resolve().parents[3]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git(cwd: Path) -> dict:
    def run(*a):
        return subprocess.run(["git", *a], cwd=cwd, capture_output=True, text=True, timeout=3, check=False).stdout.strip()

    try:
        return {"commit": run("rev-parse", "HEAD") or None, "branch": run("rev-parse", "--abbrev-ref", "HEAD") or None, "dirty": bool(run("status", "--porcelain"))}
    except (OSError, subprocess.SubprocessError):
        return {"commit": None, "branch": None, "dirty": None}


def versions() -> dict:
    out = {"python": sys.version.split()[0], "platform": platform.platform()}
    for p in PACKAGES:
        try:
            out[p] = md.version(p)
        except md.PackageNotFoundError:
            out[p] = None
    return out


class Incident:
    def __init__(self, path: Path, runs_root: Path, t_ns: int, window_s: float, after_s: float, background: bool):
        self.path, self._root, self.t_ns, self._win = path, runs_root, t_ns, window_s
        self._stop = threading.Event()
        self._done = threading.Event()
        self._thread = None
        if after_s > 0 and background:
            self._thread = threading.Thread(target=self._after, args=(after_s,), name="Incident", daemon=True)
            self._thread.start()
        else:
            self._done.set()

    def _write_timeline(self, end_ns: int) -> None:
        rows = merge_timeline(self._root, self.t_ns - int(self._win * 1e9), end_ns)
        (self.path / "timeline.jsonl").write_text("".join(dumps(r) + "\n" for r in rows), encoding="utf-8")

    def _after(self, after_s: float) -> None:
        self._stop.wait(after_s)
        self._write_timeline(min(time.time_ns(), self.t_ns + int(self._win * 1e9)))
        self._done.set()

    def finalize(self, timeout: float = 5.0) -> None:
        """뒤 60초를 기다리지 않고 지금까지의 기록으로 timeline.jsonl을 마무리한다(프로그램 종료 직전, 시험)."""
        self._stop.set()
        self._done.wait(timeout)


def create_incident(code: str, runs_root: str | Path = "runs", *, reason: str = "", ring=None, proc: str = "cli", config_dir: str | Path = "config", cfg=None,
                    window_s: float = WINDOW_S, after_s: float = WINDOW_S, now_ns: int | None = None, doctor_fn=None, background: bool = True) -> Incident:
    """사고 묶음을 만든다. ring은 RingBuffer나 이미 덤프한 dict 목록(없으면 빈 ring.jsonl). 제어 스레드에서 부르지 말고 짧은 스레드에서 부른다(파일·git·doctor가 느리다)."""
    runs_root, config_dir = Path(runs_root), Path(config_dir)
    now_ns = time.time_ns() if now_ns is None else now_ns
    stamp = datetime.fromtimestamp(now_ns / 1e9).strftime("%Y%m%d-%H%M%S")
    safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in code)
    d, n = runs_root / "incidents" / f"{stamp}-{safe}", 1
    (runs_root / "incidents").mkdir(parents=True, exist_ok=True)
    while True:
        try:
            d.mkdir()
            break
        except FileExistsError:
            n += 1
            d = runs_root / "incidents" / f"{stamp}-{safe}-{n}"
    ticks = ring.dump(now_ns) if hasattr(ring, "dump") else list(ring or [])
    (d / "ring.jsonl").write_text("".join(json.dumps(t, ensure_ascii=False, default=str) + "\n" for t in ticks), encoding="utf-8")
    # 설정과 캘리브레이션 사본, 해시
    copies: dict[str, dict] = {}

    def copy(name: str, src: Path | None) -> None:
        if src and src.is_file():
            (d / name).write_bytes(src.read_bytes())
            copies[name] = {"src": str(src), "sha256": _sha(src)}

    copy("robot.yaml", config_dir / "robot.yaml")
    arm = getattr(cfg, "arm_name", "") if cfg else ""
    copy("arm.yaml", config_dir / "arms" / f"{arm}.yaml" if arm else None)
    copy("local.yaml", config_dir / "local.yaml")
    if cfg and getattr(cfg, "calibration_id", ""):
        from ..doctor import find_calibration

        _cal, path = find_calibration(cfg.calibration_id, config_dir)
        copy("calibration.json", Path(path) if path else None)
    (d / "config.sha256.json").write_text(json.dumps(copies, indent=2, ensure_ascii=False), encoding="utf-8")
    # doctor (포트를 열지 않는다: 사고 직후 팔과의 통신을 방해하지 않게)
    res = None
    try:
        if doctor_fn:
            res = doctor_fn()
        else:
            from ..doctor import doctor

            res = doctor(cfg=cfg, probe=False, config_path=config_dir / "robot.yaml") if cfg else None
    except Exception as e:  # noqa: BLE001 doctor가 실패해도 사고 묶음은 만든다
        res = {"error": repr(e)}
    (d / "doctor.json").write_text(json.dumps(res, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    meta = {"code": code, "reason": reason, "t": now_ns, "proc": proc, "ring_ticks": len(ticks), "git": _git(REPO_ROOT), "versions": versions(), "cwd": str(Path.cwd()), "window_s": window_s}
    (d / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    inc = Incident(d, runs_root, now_ns, window_s, after_s, background)
    inc._write_timeline(now_ns)  # 앞 60초는 바로
    return inc
