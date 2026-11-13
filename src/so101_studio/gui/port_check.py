"""GUI를 켜기 전에 GUI가 쓸 포트(웹·3D·감시·정지 수신)를 이미 잡은 프로세스를 찾는다.
포트가 고정이라 이전 GUI가 남아 있으면 새 GUI가 WinError 10048로 죽었다(2026-11-13). 브라우저 탭만 닫으면 GUI 프로세스는 계속 돈다."""
import os
import socket
import sys
from collections.abc import Callable

import psutil

WAIT_S = 5.0


def kind(cmd: list[str]) -> str:
    """프로세스 명령줄로 고른다: gui | watchdog | so101(다른 so101 명령·so101-mcp) | other."""
    s = " ".join(cmd)
    if "so101_studio.watchdog" in s:
        return "watchdog"
    if "so101_studio.gui" in s or ("so101_studio" in s and "gui" in cmd):
        return "gui"
    if "so101_studio" in s or "so101-mcp" in s or "so101_mcp" in s:
        return "so101"
    return "other"


def owners(tcp: set[int], udp: set[int]) -> dict[int, set[int]]:
    """포트 -> 그 포트를 잡은 PID들 (TCP는 LISTEN만). 이 프로세스는 뺀다."""
    found: dict[int, set[int]] = {}
    try:
        conns = psutil.net_connections(kind="inet")
    except psutil.Error:  # 볼 수 없으면 검사를 건너뛴다(예전처럼 포트를 열다 실패하면 그때 알린다)
        return found
    for c in conns:
        if not c.laddr or not c.pid or c.pid == os.getpid():
            continue
        is_tcp = c.type == socket.SOCK_STREAM
        if (is_tcp and c.status == psutil.CONN_LISTEN and c.laddr.port in tcp) or (not is_tcp and c.laddr.port in udp):
            found.setdefault(c.laddr.port, set()).add(c.pid)
    return found


def _info(p: psutil.Process) -> tuple[str, str]:
    try:
        cmd = p.cmdline()
        where = p.cwd()
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        return "other", f"PID {p.pid}"
    return kind(cmd), f"PID {p.pid} ({' '.join(cmd[-3:])}, 폴더 {where})"


def _gui_of(p: psutil.Process) -> psutil.Process | None:
    """감시 프로세스면 그것을 띄운 GUI, GUI면 자신. 부모가 없는 감시 프로세스는 None(고아)."""
    k, _ = _info(p)
    if k == "gui":
        return p
    parent = p.parent()
    return parent if parent is not None and _info(parent)[0] == "gui" else None


def _end(procs: list[psutil.Process]) -> None:
    """자식(감시 프로세스)까지 끝낸다."""
    tree = list(procs)
    for p in procs:
        try:
            tree[:0] = p.children(recursive=True)
        except psutil.NoSuchProcess:
            pass
    for p in tree:
        try:
            p.terminate()
        except psutil.NoSuchProcess:
            pass
    psutil.wait_procs(tree, timeout=WAIT_S)


def check_ports(tcp: set[int], udp: set[int], url: str = "http://127.0.0.1:8080", ask: Callable[[str], str] = input,
                interactive: bool | None = None, find: Callable = owners, proc: Callable = psutil.Process) -> str | None:
    """포트가 비면 None. 고아 감시 프로세스는 끄고, 켜진 GUI는 물어서(Enter = 끄기) 끈다. 끄지 못하면 이유 문장을 돌려준다."""
    interactive = sys.stdin.isatty() if interactive is None else interactive
    busy = find(tcp, udp)
    if not busy:
        return None
    guis: dict[int, psutil.Process] = {}
    orphans, others = [], []
    for port, pids in sorted(busy.items()):
        for pid in pids:
            try:
                p = proc(pid)
                k, text = _info(p)
                gui = _gui_of(p) if k in ("gui", "watchdog") else None
            except psutil.NoSuchProcess:
                continue
            if gui is not None:
                guis[gui.pid] = gui
            elif k == "watchdog":
                orphans.append(p)
            else:
                others.append(f"포트 {port}: {text}")
    if others:
        return "GUI가 쓸 포트를 다른 프로그램이 쓰고 있어 켜지 않았습니다:\n  " + "\n  ".join(others) + "\n그 프로그램을 끈 뒤 다시 실행하세요."
    if orphans:
        print(f"이전 GUI가 남긴 감시 프로세스를 끕니다: {', '.join(str(p.pid) for p in orphans)}", flush=True)
        _end(orphans)
    if guis:
        lines = "\n  ".join(_info(g)[1] for g in guis.values())
        if not interactive:
            return f"이미 켜진 SO-101 Studio GUI가 있습니다:\n  {lines}\n그 화면({url})을 쓰거나 그 창을 닫은 뒤 다시 실행하세요."
        try:
            answer = ask(f"이미 켜진 SO-101 Studio GUI가 있습니다(브라우저 탭을 닫아도 GUI는 계속 돕니다):\n  {lines}\n"
                         "팔이 움직이는 중이면 먼저 그 화면에서 멈추세요. 그 GUI를 끄고 새로 켤까요? [Enter = 끄고 켜기 / n = 그만두기] ")
        except EOFError:  # 입력이 없으면(파이프·서비스) 끄지 않는다
            answer = "n"
        if answer.strip().lower() in ("n", "no", "아니오"):
            return f"새 GUI를 켜지 않았습니다. 켜진 GUI를 쓰세요: {url}"
        _end(list(guis.values()))
    still = find(tcp, udp)
    return f"포트가 아직 비지 않았습니다({', '.join(map(str, sorted(still)))}). 잠시 뒤 다시 실행하세요." if still else None


def so101_processes() -> list[str]:
    """시리얼 포트를 쥐고 있을 수 있는 다른 SO-101 프로그램(GUI·so101 명령·so101-mcp) 목록."""
    me = os.getpid()
    out = []
    for p in psutil.process_iter(["pid", "name"]):
        if p.pid == me or "python" not in (p.info["name"] or "").lower():
            continue
        try:
            k, text = _info(p)
        except psutil.NoSuchProcess:
            continue
        if k in ("gui", "so101"):
            out.append(text)
    return out
