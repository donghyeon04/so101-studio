"""PostToolUse 훅: 고친 .py 파일 하나만 ruff로 검사한다(결정 0045). 문제가 있으면 exit 2로 Claude에게 결과를 돌려준다.
so101 conda 환경의 python을 so101.cmd와 같은 순서로 찾는다. 못 찾거나 ruff가 없으면 조용히 넘어간다(작업을 막지 않는다)."""
import json
import os
import subprocess
import sys
from pathlib import Path


def env_python() -> Path | None:
    name = os.environ.get("SO101_ENV", "so101")
    if os.environ.get("SO101_ENV_DIR"):
        return Path(os.environ["SO101_ENV_DIR"]) / "python.exe"
    home = Path.home()
    for root in (Path("C:/so101-tools/miniforge3"), Path("C:/miniforge3"), home / "miniforge3", home / "miniconda3", home / "anaconda3"):
        p = root / "envs" / name / "python.exe"
        if p.exists():
            return p
    return None


def main() -> int:
    sys.stderr.reconfigure(encoding="utf-8")  # Windows 기본(cp949)이 아니라 UTF-8로 돌려준다
    try:
        data = json.loads(sys.stdin.buffer.read().decode("utf-8"))  # 경로에 한글(사용자 폴더)이 있다
    except ValueError:
        return 0
    path = (data.get("tool_input") or {}).get("file_path") or (data.get("tool_response") or {}).get("filePath") or ""
    if not path.endswith(".py") or not Path(path).exists():
        return 0
    py = env_python()
    if py is None or not py.exists():
        return 0
    r = subprocess.run([str(py), "-m", "ruff", "check", "--quiet", path], check=False, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode == 1:
        print(f"ruff가 {path}에서 문제를 찾았습니다. 고쳐 주세요:\n{r.stdout}{r.stderr}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
