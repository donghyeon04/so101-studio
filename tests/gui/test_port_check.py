"""켤 때 포트 검사(gui/port_check.py): 이전 GUI가 남아 있으면 새 GUI가 포트 오류로 죽던 문제(2026-11-13)."""
import psutil

from so101_studio.gui import port_check
from so101_studio.gui.port_check import check_ports, kind


def test_kind_tells_gui_watchdog_and_others():
    assert kind(["python.exe", "-m", "so101_studio", "gui"]) == "gui"
    assert kind(["python.exe", "-m", "so101_studio", "--sim", "gui"]) == "gui"
    assert kind(["python.exe", "-m", "so101_studio.gui", "--port", "8180"]) == "gui"
    assert kind(["python.exe", "-m", "so101_studio.watchdog", "--port", "8090"]) == "watchdog"
    assert kind(["python.exe", "-m", "so101_studio", "run", "--source", "manual"]) == "so101"
    assert kind(["python.exe", "so101-mcp.exe"]) == "so101"
    assert kind(["chrome.exe"]) == "other"


class FakeProc:
    def __init__(self, pid, cmd, parent=None, children=()):
        self.pid, self.cmd, self._parent, self._children = pid, cmd, parent, list(children)
        self.ended = False

    def cmdline(self):
        return self.cmd

    def cwd(self):
        return "C:/so101"

    def parent(self):
        return self._parent

    def children(self, recursive=False):
        return self._children

    def terminate(self):
        self.ended = True


def _run(monkeypatch, procs, busy, answer="", interactive=True):
    """busy: 포트 -> PID들. 끝난 프로세스의 포트는 다음 검사에서 빈다."""
    monkeypatch.setattr(psutil, "wait_procs", lambda procs, timeout: (procs, []))

    def find(tcp, udp):
        return {port: {p for p in pids if not procs[p].ended} for port, pids in busy.items() if any(not procs[p].ended for p in pids)}

    asked = []
    why = check_ports({8080}, {8091}, ask=lambda q: asked.append(q) or answer, interactive=interactive, find=find, proc=lambda pid: procs[pid])
    return why, asked


def test_free_ports_start_without_asking(monkeypatch):
    assert _run(monkeypatch, {}, {}) == (None, [])


def test_old_gui_is_closed_with_its_watchdog_after_enter(monkeypatch):
    gui = FakeProc(1, ["python", "-m", "so101_studio", "gui"])
    wd = FakeProc(2, ["python", "-m", "so101_studio.watchdog"], parent=gui)
    gui._children = [wd]
    why, asked = _run(monkeypatch, {1: gui, 2: wd}, {8080: {1}, 8091: {1}, 8090: {2}})
    assert why is None and gui.ended and wd.ended
    assert len(asked) == 1 and "이미 켜진" in asked[0]  # 한 GUI는 한 번만 묻는다


def test_old_gui_is_kept_when_user_says_no(monkeypatch):
    gui = FakeProc(1, ["python", "-m", "so101_studio", "gui"])
    why, _ = _run(monkeypatch, {1: gui}, {8080: {1}}, answer="n")
    assert not gui.ended and "켜지 않았습니다" in why


def test_end_of_input_keeps_the_old_gui(monkeypatch):
    gui = FakeProc(1, ["python", "-m", "so101_studio", "gui"])
    monkeypatch.setattr(psutil, "wait_procs", lambda procs, timeout: (procs, []))

    def eof(_):
        raise EOFError

    why = check_ports({8080}, set(), ask=eof, interactive=True, find=lambda t, u: {8080: {1}}, proc=lambda pid: gui)
    assert not gui.ended and "켜지 않았습니다" in why


def test_without_a_console_it_does_not_close_anything(monkeypatch):
    gui = FakeProc(1, ["python", "-m", "so101_studio", "gui"])
    why, asked = _run(monkeypatch, {1: gui}, {8080: {1}}, interactive=False)
    assert not gui.ended and asked == [] and "이미 켜진" in why


def test_orphan_watchdog_is_closed_without_asking(monkeypatch):
    wd = FakeProc(2, ["python", "-m", "so101_studio.watchdog"], parent=None)
    why, asked = _run(monkeypatch, {2: wd}, {8090: {2}})
    assert why is None and wd.ended and asked == []


def test_other_program_is_named_and_left_alone(monkeypatch):
    other = FakeProc(3, ["node.exe", "server.js"])
    why, _ = _run(monkeypatch, {3: other}, {8080: {3}})
    assert not other.ended and "PID 3" in why and "다른 프로그램" in why


def test_so101_processes_lists_this_session_python_only_when_it_is_so101(monkeypatch):
    me = psutil.Process()
    assert all(f"PID {me.pid} " not in line for line in port_check.so101_processes())  # 자신은 빼고
