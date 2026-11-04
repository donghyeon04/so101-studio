"""감시 프로세스의 기록 파일(watchdog.jsonl). 팔을 다시 연결하면 실행 폴더가 새로 생기므로, POST /arm의 run_dir로 쓰는 폴더를 바꾼다 (decisions/0030).
run_dir는 처음 폴더와 같은 부모(runs/) 바로 아래 폴더만 받는다: 브라우저가 부를 수 있는 HTTP 입구로 임의 경로에 파일을 만들지 못하게 한다."""
import threading
from pathlib import Path

from ..core.logging import JsonlFile


class RunLog:
    def __init__(self, path: str | Path, root: str | Path | None = None):
        path = Path(path)
        self.root = Path(root if root is not None else path.parent.parent).resolve()  # 기본: 실행 폴더들이 모인 폴더(path가 <실행 폴더>/watchdog.jsonl일 때)
        self.path = path
        self._jl = JsonlFile(path, "watchdog")
        self._lock = threading.Lock()

    def write(self, kind: str, data=None, level: str = "INFO", code: str | None = None) -> None:
        with self._lock:
            self._jl.write(kind, data, level, code)

    def switch(self, run_dir: str | Path) -> Path:
        """기록을 run_dir/watchdog.jsonl로 옮긴다. 같은 폴더면 그대로. 허용 범위 밖이면 ValueError(바뀌지 않음)."""
        if not isinstance(run_dir, str | Path) or not str(run_dir):
            raise ValueError("run_dir는 폴더 경로 문자열이어야 한다")
        d = Path(run_dir).resolve()
        if d.parent != self.root:
            raise ValueError(f"run_dir는 {self.root} 바로 아래 폴더여야 한다")
        new = d / "watchdog.jsonl"
        with self._lock:
            if new == self.path.resolve():
                return new
            d.mkdir(parents=True, exist_ok=True)
            old, self._jl, self.path = self._jl, JsonlFile(new, "watchdog"), new
            old.close()
            return new

    def close(self) -> None:
        with self._lock:
            self._jl.close()
