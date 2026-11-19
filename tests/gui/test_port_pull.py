"""포트 빠짐 네 경우 (decisions/0031, gui-spec G10)를 가짜 드라이버로 미리 돌린다. 실제 USB를 뽑는 시험을 대신하지는 않는다."""

import asyncio
import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("nicegui")
pytestmark = pytest.mark.slow  # 시험마다 실제·시뮬레이션 두 팔을 새로 만들어 각 8~12초 (결정 0046: 하루 마칠 때 전체에서 돈다)

from nicegui.testing import user_simulation

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot
from so101_studio.core.types import JointState
from so101_studio.gui import app as gui_app
from so101_studio.gui import backend as backend_module
from so101_studio.gui.backend import COMM_LOST_TEXT, LINKWATCH_TEXT, Backend

LIMIT_S = 0.6
BUS_CALLS = ("connect", "read_joints", "write_joints", "read_health", "read_power", "read_servo_info", "set_acceleration", "torque_enabled",
             "set_torque", "set_gripper_torque_limit", "gripper_torque_limit")


class PulledPort(SimDriver):
    """뽑힌 동안 버스를 쓰는 호출이 모두 실패한다. 다시 꽂아도 연결하기 전에는 예전 연결로 통신하지 못한다."""

    def __init__(self, home):
        self.unplugged = False
        self.stale = False  # 뽑혔던 연결. 새로 connect해야 풀린다
        self.joint_writes = 0
        super().__init__(home)

    def pull(self):
        self.unplugged = self.stale = True

    def plug(self):
        self.unplugged = False


def _bus_call(name):
    original = getattr(SimDriver, name)

    def call(self, *args, **kw):
        if self.unplugged or (self.stale and name != "connect"):
            raise ConnectionError("Failed to read: no status packet")
        if name == "connect":
            self.stale = False
        if name == "write_joints":
            self.joint_writes += 1
        return original(self, *args, **kw)

    return call


for _name in BUS_CALLS:
    setattr(PulledPort, _name, _bus_call(_name))


def _link():
    return SimpleNamespace(alive=True, text=lambda: "정상", stop=lambda: None, rearm=lambda: None, want_armed=False)


@pytest.fixture
def backend(monkeypatch):
    backend = Backend(log_root=None, viser_port=0, real_driver=PulledPort(load_robot().poses.get("home")))
    backend.link = _link()
    backend.set_speed(30)
    board = {"device": "COM7", "vid": 0x1A86, "pid": 0x55D3, "serial": backend.real.cfg.board_serial, "description": ""}
    monkeypatch.setattr(backend_module, "list_serial_ports", lambda: [board])
    backend.set_run_mode("real")
    backend.real.driver.set_torque(True)
    yield backend
    backend.link = None
    backend.real.driver.plug()
    backend.real.driver.stale = False
    backend.close()


def _extend(backend):  # 정면으로 펴면 레이더 받침(결정 0067, 정면 약 384mm)에 걸려 거부된다: 30° 돌려 편다. 병렬 손(기본, REQ-098)은 길어 어깨 20°면 z_floor
    q = backend.real.status().joints
    r = backend.real.move_joints(JointState(30.0, 10.0, 20.0, q.wrist_flex, q.wrist_roll, q.gripper))
    assert r.status == "ok", r.reason


def _slow_move(backend):
    q = backend.real.status().joints
    goal = JointState(q.shoulder_pan + 40.0, q.shoulder_lift, q.elbow_flex, q.wrist_flex, q.wrist_roll, q.gripper)
    threading.Thread(target=lambda: backend.real.move_joints(goal, speed_pct=5), daemon=True).start()
    assert _wait(lambda: backend.real.ctl.state == "moving", 2.0)


def _jog(backend):
    backend.real.jog("joint", [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert _wait(lambda: backend.real.ctl.state == "teleop", 2.0)


def _wait(check, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.01)
    return False


@pytest.mark.parametrize("before", [lambda _b: None, _extend, _slow_move, _jog], ids=["home-still", "extended-still", "slow-move", "jog"])
def test_port_pull_is_seen_in_time_and_nothing_moves_until_reconnect(backend, before):
    before(backend)
    driver = backend.real.driver
    driver.pull()
    t0 = time.monotonic()
    assert _wait(backend.comm_lost, LIMIT_S), f"{LIMIT_S}초 안에 통신 끊김을 보지 못했다"
    assert backend.move_lock() == COMM_LOST_TEXT
    assert backend.home() == COMM_LOST_TEXT
    assert time.monotonic() - t0 < 2.0  # 끊긴 버스를 기다리느라 화면 호출이 멈추면 안 된다

    driver.plug()  # 다시 꽂기만 하면 움직이지도, 풀리지도 않는다
    writes = driver.joint_writes
    time.sleep(0.5)
    assert driver.joint_writes == writes
    assert backend.comm_lost() and backend.home() == COMM_LOST_TEXT
    assert driver.joint_writes == writes

    assert backend.resume() == COMM_LOST_TEXT
    assert backend.comm_lost()

    pose = SimDriver.read_joints(driver)
    old_run = backend.link.run_dir = "옛 폴더"
    reason, torque_off = backend.reconnect()
    assert reason is None and torque_off == []
    assert backend.link.run_dir == backend.run_dir != old_run  # 감시 기록도 새 실행 폴더로 옮긴다
    assert not backend.comm_lost() and backend.move_lock() is None
    time.sleep(0.2)
    assert driver.read_joints() == pose  # 다시 연결만으로는 다른 자세로 가지 않는다
    assert backend.home() is None


def test_reconnect_while_still_unplugged_says_why_and_can_try_again(backend):
    driver = backend.real.driver
    driver.pull()
    assert _wait(backend.comm_lost, LIMIT_S)
    reason, torque_off = backend.reconnect()
    assert reason and torque_off is None
    assert backend.comm_lost()
    driver.plug()
    assert backend.reconnect()[0] is None
    assert not backend.comm_lost()


@pytest.mark.parametrize("path", ["/", "/manual", "/teach", "/input", "/arm", "/params", "/safety", "/table", "/runs", "/logs"])
def test_every_page_opens_and_shows_the_loss_while_unplugged(backend, path):
    backend.real.driver.pull()
    assert _wait(backend.comm_lost, LIMIT_S)

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await user.should_not_see("produced an error")
            await user.should_see(marker="reconnect", retries=20)
            await user.should_see(marker="lock-banner")
            await user.should_not_see(marker="resume")
            await asyncio.sleep(0.3)  # 페이지 타이머가 끊긴 버스를 읽다 멈추지 않는다
            await user.should_see(marker="reconnect")

    asyncio.run(go())


def test_diagnostics_do_not_touch_a_lost_bus(backend):
    backend.real.driver.pull()
    assert _wait(backend.comm_lost, LIMIT_S)
    assert backend.run_diagnostic("servoinfo") == (None, COMM_LOST_TEXT)


def test_estop_during_the_loss_reaches_every_arm_and_resume_stays_locked(backend, monkeypatch):
    backend.real.driver.pull()
    assert _wait(backend.comm_lost, LIMIT_S)
    called = []
    sim_estop = backend.sim.estop
    monkeypatch.setattr(backend.sim, "estop", lambda: (called.append("sim"), sim_estop()))
    backend.estop()
    assert called == ["sim"]
    assert backend.resume() == COMM_LOST_TEXT
    assert backend.comm_lost()
    backend.real.driver.plug()  # 잠깐 끊긴 경우: 통신이 돌아와도 정지 해제로는 풀리지 않는다
    time.sleep(0.3)
    assert backend.resume() == COMM_LOST_TEXT
    assert backend.comm_lost() and backend.home() == COMM_LOST_TEXT


def test_stop_and_page_calls_survive_the_loss(backend):
    backend.real.driver.pull()
    assert _wait(backend.comm_lost, LIMIT_S)
    backend.stop_all()
    backend.set_speed(20)
    backend.owner = "시험"
    backend.disconnected("시험")
    assert backend.comm_lost()


def test_close_finishes_while_unplugged(monkeypatch):
    backend = Backend(log_root=None, real_driver=PulledPort(load_robot().poses.get("home")))
    backend.real.driver.pull()
    t = threading.Thread(target=backend.close, daemon=True)
    t.start()
    t.join(8.0)
    assert not t.is_alive()


def test_linkwatch_disconnects_keeps_moves_locked_and_reconnects(backend, monkeypatch):
    from so101_studio.cli import linkwatch

    monkeypatch.setattr(backend.real.driver, "simulated", False)  # 진단은 가짜 포트를 시뮬레이션으로 보지 않게 한다(diagnostics는 driver.simulated를 본다)
    monkeypatch.setattr(backend, "_pick_port", lambda: "COM7")
    seen = {}

    def fake_run(port, out, stop, **_kw):
        seen.update(port=port, lock=backend.move_lock(), home=backend.home(), connected=backend.real._connected)
        backend.estop()  # 연결을 끊은 동안 눌러도 앱이 죽지 않는다
        out("끊김 없음")
        return {"drops": 0}

    monkeypatch.setattr(linkwatch, "run_linkwatch", fake_run)
    lines = []
    old_ctl = backend.real.ctl
    result, reason = backend.run_diagnostic("linkwatch", lines.append)
    assert reason is None and result["name"] == "linkwatch"
    assert seen == {"port": "COM7", "lock": LINKWATCH_TEXT, "home": LINKWATCH_TEXT, "connected": False}
    assert lines == ["끊김 없음"]
    assert backend.real._connected and backend.real.ctl is not old_ctl and backend.studio.ctl is backend.real.ctl
    assert backend.move_lock() is None and backend.home() is None


def test_linkwatch_failure_still_reconnects_and_a_lost_reconnect_offers_the_button(backend, monkeypatch):
    from so101_studio.cli import linkwatch

    monkeypatch.setattr(backend.real.driver, "simulated", False)
    monkeypatch.setattr(backend, "_pick_port", lambda: backend.real.cfg.port)

    def busy(*_a, **_kw):
        raise OSError("port busy")

    monkeypatch.setattr(linkwatch, "run_linkwatch", busy)
    result, reason = backend.run_diagnostic("linkwatch")
    assert result is None and "port busy" in reason
    assert backend.real._connected and backend.move_lock() is None

    def unplug_then_finish(*_a, **_kw):
        backend.real.driver.pull()
        return {"drops": 1}

    monkeypatch.setattr(linkwatch, "run_linkwatch", unplug_then_finish)
    result, reason = backend.run_diagnostic("linkwatch")
    assert result["name"] == "linkwatch" and reason
    assert backend.comm_lost() and backend.move_lock() == COMM_LOST_TEXT


def test_linkwatch_refuses_while_moving(backend, monkeypatch):
    monkeypatch.setattr(backend.real.ctl, "state", "moving")
    assert backend.run_diagnostic("linkwatch") == (None, backend_module.BUSY_TEXT)
    assert backend.real._connected


def test_recording_interrupted_by_the_pull_still_lets_estop_finish(backend, tmp_path, monkeypatch):
    monkeypatch.setattr(backend.real.teach, "dir", tmp_path)
    assert backend.record_begin("lost") is None
    backend.real.driver.pull()
    assert _wait(backend.comm_lost, LIMIT_S)
    backend.estop()
    assert not backend.recording()
    backend.real.driver.plug()
    assert backend.reconnect()[0] is None
    assert backend.real.ctl.fault is None
