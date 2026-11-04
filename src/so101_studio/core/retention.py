"""실행 기록 보관 (decisions/0030): 시작할 때 runs/가 30일 또는 2GB를 넘으면 오래된 실행부터 지운다. runs/incidents/는 지우지 않는다."""
import shutil
import time
from pathlib import Path

MAX_AGE_DAYS = 30
MAX_BYTES = 2 * 1024**3
KEEP = ("incidents",)  # 사고 묶음은 자동으로 지우지 않는다


def _newest_mtime(d: Path) -> float:
    return max((p.stat().st_mtime for p in [d, *d.rglob("*")]), default=0.0)


def _size(d: Path) -> int:
    return sum(p.stat().st_size for p in d.rglob("*") if p.is_file())


def prune_runs(root: str | Path, max_age_days: float = MAX_AGE_DAYS, max_bytes: int = MAX_BYTES, protect=(), now: float | None = None) -> list[Path]:
    """오래된 실행 폴더를 지우고 지운 목록을 돌려준다. 나이(마지막 수정 기준)가 max_age_days를 넘는 폴더를 지운 뒤, 남은 폴더의 합이
    max_bytes를 넘으면 가장 오래된 것부터 지운다. protect(지금 실행 폴더 등)와 incidents는 지우지 않는다. 폴더가 아닌 파일은 건드리지 않는다."""
    root = Path(root)
    now = time.time() if now is None else now
    protect = {Path(p).resolve() for p in protect}
    runs = sorted((d for d in root.iterdir() if d.is_dir() and d.name not in KEEP), key=_newest_mtime) if root.is_dir() else []
    deleted: list[Path] = []
    left = []
    for d in runs:
        if d.resolve() not in protect and now - _newest_mtime(d) > max_age_days * 86400:
            shutil.rmtree(d, ignore_errors=True)
            deleted.append(d)
        else:
            left.append(d)
    total = sum(_size(d) for d in left)
    for d in left:  # 오래된 것부터
        if total <= max_bytes:
            break
        if d.resolve() in protect:
            continue
        total -= _size(d)
        shutil.rmtree(d, ignore_errors=True)
        deleted.append(d)
    return deleted
