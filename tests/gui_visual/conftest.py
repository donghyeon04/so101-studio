"""GUI 화면 시험(결정 0054): 시뮬레이션 GUI를 실제 팔 GUI와 겹치지 않는 포트로 띄우고 Playwright(Chromium)로 연다.
웹 8380, 3D 8381, 감시 8490(정지 수신 8491). 무작위 조작 시험은 +2(8382·8383·8492). 브라우저 탭을 열지 않는다(--no-browser)."""

import contextlib
import os
import shutil
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from visual_helpers import BASE, ROOT, VISER, WATCHDOG, WEB


def _up(base: str, timeout: float = 90.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            with urllib.request.urlopen(base + "/start", timeout=2):
                return True
        except OSError:
            time.sleep(0.5)
    return False


@contextlib.contextmanager
def gui_server(cwd: Path, web: int, viser: int, watchdog: int):
    """시뮬레이션 GUI를 cwd에서 띄운다. 설정은 cwd의 config/를 읽고 쓴다."""
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PYTEST", "NICEGUI"))}  # NiceGUI는 이 변수를 보면 시험 모드로 서버를 띄우지 않는다
    env |= {"PYTHONPATH": str(ROOT / "src"), "PYTHONIOENCODING": "utf-8"}
    cmd = [sys.executable, "-m", "so101_studio.gui", "--sim", "--no-browser", "--port", str(web), "--viser-port", str(viser), "--watchdog-port", str(watchdog)]
    proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base = f"http://127.0.0.1:{web}"
    try:
        assert _up(base), f"시험용 GUI가 뜨지 않았다(포트 {web}·{viser}·{watchdog}가 비어 있는지 확인)"
        yield base
    finally:
        if os.name == "nt":  # 감시 프로세스(자식)까지 함께 끈다
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], check=False, capture_output=True)
        else:
            proc.terminate()
        proc.wait(timeout=10)


@pytest.fixture(scope="session")
def server():
    with gui_server(ROOT, WEB, VISER, WATCHDOG) as base:
        assert base == BASE
        yield base


@pytest.fixture(scope="session")
def scratch_server(tmp_path_factory):
    """버튼을 마구 누르는 시험용: config/·assets/ 사본이 있는 임시 폴더에서 띄워 레포 설정 파일을 건드리지 않는다."""
    work = tmp_path_factory.mktemp("gui")
    for d in ("config", "assets"):  # 설정의 URDF·장면 경로가 실행 폴더 기준이라 assets도 복사한다(22MB)
        shutil.copytree(ROOT / d, work / d)
    with gui_server(work, WEB + 2, VISER + 2, WATCHDOG + 2) as base:
        yield base


@pytest.fixture
def errors(page):
    """페이지 스크립트 오류. 시험 끝에 비어 있어야 한다."""
    seen: list[str] = []
    page.on("pageerror", lambda e: seen.append(str(e)))
    return seen
