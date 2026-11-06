"""REQ-032: 버스 오류 뒤 'Port is in use'가 남지 않는다, 이동 중에도 0.5초 안에 comm_lost, 토크 반복·끊김 중 프로브 반복 없음."""
import threading
import time
from types import SimpleNamespace

import pytest

from so101_studio.arm.driver import JOINTS, ArmDriver
from so101_studio.arm.safety import HealthMonitor
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


class SerialTimeoutException(Exception):  # pyserial의 같은 이름 예외 흉내(드라이버는 형식 이름으로 알아본다)
    pass


class FakeBus:
    """scservo_sdk의 is_using 동작을 흉내 낸 가짜 버스: 이미 쓰는 중이면 'Port is in use', 예외로 끝나면 플래그를 끄지 않는다(SDK 결함)."""

    def __init__(self):
        self.port_handler = SimpleNamespace(is_using=False)
        self.motors = {m: SimpleNamespace(id=i + 1) for i, m in enumerate(JOINTS)}
        self.packet_handler = SimpleNamespace(read1ByteTxRx=lambda port, i, addr: (addr, 0, 0))
        self.timeout_once = False
        self.active = self.overlap = 0

    def _use(self, work):
        if self.port_handler.is_using:
            raise ConnectionError("Failed to sync read after 1 tries. [TxRxResult] Port is in use!")
        self.port_handler.is_using = True
        self.active += 1
        self.overlap += self.active > 1
        try:
            if self.timeout_once:
                self.timeout_once = False
                raise SerialTimeoutException("Write timeout")  # SDK처럼 is_using을 켠 채 빠져나간다
            time.sleep(0.0005)
            result = work()
        finally:
            self.active -= 1
        self.port_handler.is_using = False
        return result

    def sync_read(self, name, motors=None, **kw):
        return self._use(lambda: {m: 0.0 for m in JOINTS})

    def sync_write(self, name, values, **kw):
        return self._use(lambda: None)

    def read(self, name, motor, **kw):
        return self._use(lambda: 0)

    def write(self, name, motor, value, **kw):
        return self._use(lambda: None)


def fake_driver():
    d = ArmDriver()
    d._robot = SimpleNamespace(bus=FakeBus())
    d.events = []
    d.event_sink = lambda kind, level, code, data: d.events.append((code, data))
    return d


def test_read_works_again_after_a_write_timeout():
    d = fake_driver()
    d._robot.bus.timeout_once = True
    with pytest.raises(SerialTimeoutException):
        d.read_joints()
    assert d._robot.bus.port_handler.is_using is False
    assert isinstance(d.read_joints(), JointState)  # 예전에는 이 뒤로 영원히 'Port is in use'였다
    assert [c for c, _ in d.events] == ["ARM-WRITE-TIMEOUT"]


def test_threads_never_overlap_on_the_bus():
    d = fake_driver()
    errors = []

    def work(kind):
        try:
            for _ in range(60):
                if kind == 0:
                    d.read_joints()
                elif kind == 1:
                    d.write_joints(JointState(*[0.0] * 6))
                elif kind == 2:
                    d.torque_enabled()
                else:
                    d.read_servo_info()
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    threads = [threading.Thread(target=work, args=(i % 4,)) for i in range(8)]
    [t.start() for t in threads]
    [t.join() for t in threads]
    assert not errors and d._robot.bus.overlap == 0


def test_servoinfo_reads_voltage_limit_registers():
    info = fake_driver().read_servo_info()
    one = next(iter(info.values()))
    assert {"Max_Input_Voltage(14)", "Min_Input_Voltage(15)", "Unloading_Condition", "Status"} <= set(one)


# ---- comm_lost 시계
def test_write_success_does_not_clear_a_read_failure():
    m = HealthMonitor()
    assert m.comm_failed(10.0, "read") is None
    m.comm_ok(10.1, "write")  # 이동 중 제어 틱의 쓰기 성공
    assert m.comm_failed(10.5, "read") == "comm_lost"


def test_read_success_clears_the_read_failure():
    m = HealthMonitor()
    m.comm_failed(10.0, "read")
    m.comm_ok(10.1, "read")
    assert m.comm_failed(10.5, "read") is None


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    yield s
    s.disconnect()


def test_comm_lost_within_half_second_while_a_job_runs(svc):
    """읽기만 계속 실패하고 쓰기는 되는 동안(이동 중)에도 0.5초쯤에 comm_lost (결정 0031). 예전에는 쓰기 성공이 시계를 계속 지워 1초 넘게 걸렸다."""
    def broken():
        raise ConnectionError("no status packet")

    traj = svc.skills._joint_move(JointState(0, -30, 30, 0, 0, 3))
    h = svc.ctl.execute(traj)
    svc.driver.read_health = svc.driver.read_joints = broken  # 읽기는 모두 실패(상태 루프·제어 스레드의 건강 읽기 포함), 쓰기는 계속 성공
    t0 = time.perf_counter()
    while svc.ctl.state != "fault" and time.perf_counter() - t0 < 2.0:
        time.sleep(0.02)
    assert svc.ctl.fault == "comm_lost" and time.perf_counter() - t0 < 0.8
    h.wait()


def test_job_with_a_single_read_glitch_does_not_fault(svc):
    h = svc.ctl.execute(svc.skills._joint_move(JointState(0, -80, 80, 0, 0, 3)))
    svc.ctl.note_comm(False)
    for _ in range(10):  # 곧 읽기가 다시 된다
        time.sleep(0.1)
        svc.ctl.note_comm(True)
    assert svc.ctl.state != "fault"
    svc.ctl.stop()
    h.wait()


# ---- 토크 반복, 끊김 중 프로브
def test_enable_torque_writes_only_once(svc):
    calls = []
    orig = svc.driver.set_torque
    svc.driver.set_torque = lambda on: (calls.append(on), orig(on))[1]
    for _ in range(10):  # 조그 keepalive
        assert svc.enable_torque()
    assert calls == [True]
    svc._torque_lost = True  # 풀린 것을 읽었으면 다시 켠다
    svc.enable_torque()
    assert calls == [True, True]


def test_load_probe_is_silent_while_comm_is_lost(svc):
    reads = []
    svc.driver.read_health = lambda: reads.append(1) or {}
    svc.ctl.state, svc.ctl.fault = "fault", "comm_lost"
    for _ in range(3):
        svc._probe("status_loop_error")
        svc._probe_t = -1e9
    assert reads == []


def test_connect_event_says_sim_for_the_simulator(tmp_path):
    import json

    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        rows = [json.loads(ln) for ln in (s.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines() if ln.strip()]
        assert next(r for r in rows if r["kind"] == "connect")["data"]["port"] == "sim"  # COM3처럼 실제 포트로 보이지 않는다
    finally:
        s.disconnect()
