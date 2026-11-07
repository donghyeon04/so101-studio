"""실행 방식 4가지 (gui-spec G9). 실제 팔 자리에 두 번째 SimDriver를 넣어 시험한다. 실제 장치는 쓰지 않는다."""

import asyncio
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot
from so101_studio.core.types import ArmStatus, JointState, MotionResult, ToolPose
from so101_studio.gui import app as gui_app
from so101_studio.gui import backend as backend_module
from so101_studio.gui.backend import COMM_LOST_TEXT, OUTSIDE_TEXT, RUN_MODES, Backend
from so101_studio.gui.pages.common import Controls
from so101_studio.gui.state import component_states


def _alive_link() -> SimpleNamespace:
    return SimpleNamespace(alive=True, text=lambda: "정상", stop=lambda: None, rearm=lambda: None, want_armed=False, stop_port=0)


@pytest.fixture(scope="module")
def shared_twin():
    backend = Backend(log_root=None, real_driver=SimDriver(load_robot().poses.get("home")))
    backend.link = _alive_link()
    yield backend
    backend.link = None
    backend.close()


@pytest.fixture
def twin(shared_twin):
    backend = shared_twin
    backend.link = _alive_link()
    backend.owner, backend.pending, backend.mode = None, None, "sim"
    backend._reconnect_failed = False  # 다시 연결 실패 시험이 다음 시험으로 새지 않게
    backend.resume()
    backend.set_speed(100)
    for arm in backend._arms():
        assert arm.home().status == "ok"
    return backend


def _pan(arm) -> float:
    return arm.status().joints.shoulder_pan


def _wait(check, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.02)
    return False


def _pans(backend) -> tuple[float, float]:
    return _pan(backend.real), _pan(backend.sim)


def test_without_real_arm_only_simulation_is_allowed(backend):
    for mode in ("real", "both", "sim_first"):
        reason = backend.set_run_mode(mode)
        assert "실제 팔이 연결되어 있지 않습니다" in reason
        assert backend.mode == "sim"
    assert backend.readout()["실행 방식"] == "시뮬레이션만"
    assert backend.readout()["팔"] == "시뮬레이션"
    assert backend.set_run_mode("sim") is None


def test_unknown_mode_is_refused(twin):
    assert twin.set_run_mode("없음")
    assert twin.mode == "sim"


def test_sim_only_moves_only_the_simulated_arm(twin):
    real, sim = _pans(twin)
    assert twin.jog_step("joint", 0, 10) is None
    assert _pan(twin.real) == pytest.approx(real, abs=0.5)
    assert _pan(twin.sim) == pytest.approx(sim + 10, abs=0.5)


def test_real_only_moves_only_the_real_arm(twin):
    assert twin.set_run_mode("real") is None
    real, sim = _pans(twin)
    assert twin.jog_step("joint", 0, 10) is None
    assert _pan(twin.real) == pytest.approx(real + 10, abs=0.5)
    assert _pan(twin.sim) == pytest.approx(sim, abs=0.5)


def test_both_moves_both_arms_together(twin):
    assert twin.set_run_mode("both") is None
    real, sim = _pans(twin)
    assert twin.jog_step("joint", 0, 10) is None
    assert _pan(twin.real) == pytest.approx(real + 10, abs=0.5)
    assert _pan(twin.sim) == pytest.approx(sim + 10, abs=0.5)


def test_sim_first_moves_real_arm_only_after_confirmation(twin):
    assert twin.set_run_mode("sim_first") is None
    real, sim = _pans(twin)
    assert twin.jog_step("joint", 0, 10) is None
    assert _pan(twin.sim) == pytest.approx(sim + 10, abs=0.5)
    assert _pan(twin.real) == pytest.approx(real, abs=0.5)  # 아직 움직이지 않는다
    assert twin.pending
    assert twin.confirm_pending() is None
    assert twin.pending is None
    assert _pan(twin.real) == pytest.approx(real + 10, abs=0.5)


def test_sim_first_cancel_leaves_real_arm(twin):
    assert twin.set_run_mode("sim_first") is None
    real = _pan(twin.real)
    assert twin.jog_step("joint", 0, 10) is None
    twin.cancel_pending()
    assert twin.confirm_pending() is None
    assert _pan(twin.real) == pytest.approx(real, abs=0.5)


def test_sim_first_failed_simulation_asks_nothing(twin):
    assert twin.set_run_mode("sim_first") is None
    assert twin.jog_step("joint", 0, 1000)
    assert twin.pending is None


def test_real_commands_blocked_without_watchdog(twin):
    assert twin.set_run_mode("real") is None
    twin.link = None
    before = _pan(twin.real)
    assert "감시 프로세스" in twin.jog_step("joint", 0, 10)
    assert twin.jog("joint", 0, 1) is False
    assert "감시 프로세스" in twin.set_run_mode("both")
    assert _pan(twin.real) == pytest.approx(before, abs=0.1)
    assert twin.set_run_mode("sim") is None  # 시뮬레이션은 된다
    assert twin.jog_step("joint", 0, 5) is None


def test_watchdog_lost_before_confirm_blocks_real(twin):
    assert twin.set_run_mode("sim_first") is None
    before = _pan(twin.real)
    assert twin.jog_step("joint", 0, 10) is None
    twin.link = None
    assert "감시 프로세스" in twin.confirm_pending()
    assert _pan(twin.real) == pytest.approx(before, abs=0.1)


def test_estop_stops_both_and_resume_frees_both(twin):
    twin.estop()
    assert twin.real.ctl.state == "stopped" and twin.sim.ctl.state == "stopped"
    twin.resume()
    assert twin.real.ctl.state == "idle" and twin.sim.ctl.state == "idle"


def test_speed_and_safety_apply_to_both(twin):
    twin.set_speed(40)
    assert twin.real.speed == 40 and twin.sim.speed == 40
    assert twin.set_safety("free", 0.0, None) is None
    assert twin.real.get_safety()["mode"] == "free" and twin.sim.get_safety()["mode"] == "free"
    assert twin.set_safety("normal", 0.0, None) is None


def test_hold_jog_targets_follow_mode(twin):
    assert twin.set_run_mode("real") is None
    sim = _pan(twin.sim)
    real = _pan(twin.real)
    sign = -1 if real > 0 else 1
    for _ in range(4):
        twin.jog("joint", 0, sign)
        time.sleep(0.1)
    twin.jog_release("joint")
    assert _wait(lambda: twin.real.ctl.state == "idle")
    assert sign * (_pan(twin.real) - real) > 1
    assert _pan(twin.sim) == pytest.approx(sim, abs=0.1)


def test_auto_task_needs_real_only_mode(twin):
    assert "실제만" in twin.start_task()
    assert twin.set_run_mode("both") is None
    assert "실제만" in twin.start_task()


def test_mode_shows_in_status_bar_and_arm_is_real(twin):
    assert twin.set_run_mode("both") is None
    assert twin.readout()["실행 방식"] == "동시"
    assert twin.readout()["팔"] == "실제"


def test_page_lists_modes_and_explains_missing_real_arm(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/params")
            toggle = next(iter(user.find(marker="run-mode").elements))
            assert toggle.options == RUN_MODES
            for _ in range(40):
                if backend.owner:
                    break
                await asyncio.sleep(0.05)
            toggle.value = "both"
            await user.should_see("실제 팔이 연결되어 있지 않습니다")
            for _ in range(20):
                if toggle.value == "sim":
                    break
                await asyncio.sleep(0.05)
            assert toggle.value == "sim"
            assert backend.mode == "sim"

    asyncio.run(go())


LOCK = "캘리브레이션 불일치: 시험"


def test_locked_real_arm_refuses_real_modes_and_big_moves_but_reads(twin, monkeypatch):
    monkeypatch.setattr(twin.real.ctl, "locked_reason", LOCK)
    assert twin.move_lock() == LOCK
    assert twin.set_run_mode("real") == LOCK
    assert twin.mode == "sim"
    assert twin.run_diagnostic("powertest") == (None, LOCK)
    assert twin.run_diagnostic("servoinfo")[1] is None  # 읽기는 된다
    assert twin._run_on(twin.real, lambda _arm: SimpleNamespace(status="locked", reason=None)).startswith("실제 팔이 잠겨")


def test_locked_real_arm_turns_off_moving_buttons_with_reason(twin, monkeypatch):
    monkeypatch.setattr(twin.real.ctl, "locked_reason", LOCK)

    async def wait_off(user, marker):
        for _ in range(100):
            found = list(user.find(marker=marker).elements)
            if found and not found[0].enabled:
                return True
            await asyncio.sleep(0.05)
        return False

    async def go():
        async with user_simulation(gui_app.make_root(twin)) as user:
            await user.open("/arm")
            await user.should_see(marker="lock-banner")
            await user.should_see(marker="new-arm-steps")
            assert await wait_off(user, "diag-powertest")

    asyncio.run(go())


def test_only_moving_pages_are_locked(backend, monkeypatch):
    backend.owner = "me"
    moving, reading = Controls(backend, "me", moves=True), Controls(backend, "me")
    assert moving.allowed and reading.allowed
    monkeypatch.setattr(backend, "move_lock", lambda: LOCK)
    assert not moving.allowed and moving.locked == LOCK
    assert reading.allowed


def test_locked_workspace_cannot_start_a_task(backend, monkeypatch):
    monkeypatch.setattr(backend, "move_lock", lambda: LOCK)

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/")
            await user.should_see(marker="lock-banner")
            for _ in range(100):
                if backend.owner and not next(iter(user.find(marker="task-auto").elements)).enabled:
                    break
                await asyncio.sleep(0.05)
            assert not next(iter(user.find(marker="task-auto").elements)).enabled

    asyncio.run(go())


def _lose_comm(backend, monkeypatch):
    monkeypatch.setattr(backend.real.ctl, "fault", "comm_lost")
    monkeypatch.setattr(backend.real.ctl, "state", "fault")


def test_comm_loss_turns_moves_off_and_resume_does_not_clear_it(twin, monkeypatch):
    _lose_comm(twin, monkeypatch)
    assert twin.comm_lost() and twin.move_lock() == COMM_LOST_TEXT
    assert twin.set_run_mode("real") == COMM_LOST_TEXT
    assert twin.resume() == COMM_LOST_TEXT
    assert twin.comm_lost()  # 코드가 없어 전부 시뮬레이션일 때도 그대로


def test_component_states_show_bus_error_even_without_fresh_status():
    assert component_states(None, 10.0, comm_lost=True)["버스"][0] == "error"


def test_reconnect_clears_the_loss_and_gives_a_working_task_runner(tmp_path, monkeypatch):
    backend = Backend(log_root=None, real_driver=SimDriver(load_robot().poses.get("home")))
    backend.link = _alive_link()
    try:
        old_ctl, old_studio = backend.real.ctl, backend.studio
        old_ctl.fault, old_ctl.state = "comm_lost", "fault"
        board = {"device": "COM7", "vid": 0x1A86, "pid": 0x55D3, "serial": backend.real.cfg.board_serial, "description": ""}
        monkeypatch.setattr(backend_module, "list_serial_ports", lambda: [board])
        monkeypatch.setattr(backend.real.driver, "read_health", lambda: {**SimDriver().read_health(), "torque": {"shoulder_pan": 1, "elbow_flex": 0}})
        reason, torque_off = backend.reconnect()
        assert reason is None and torque_off == ["elbow_flex"]
        assert not backend.comm_lost() and backend.move_lock() is None
        assert backend.real.ctl is not old_ctl and backend.studio is not old_studio and backend.studio.ctl is backend.real.ctl
        assert backend.set_run_mode("real") is None
        assert backend.home() is None
    finally:
        backend.link = None
        backend.close()


def test_watchdog_is_registered_again_even_when_connect_fails(twin, monkeypatch):
    _lose_comm(twin, monkeypatch)
    monkeypatch.setattr(twin, "_reconnect_failed", False)
    board = {"device": "COM7", "vid": 0x1A86, "pid": 0x55D3, "serial": twin.real.cfg.board_serial, "description": ""}
    monkeypatch.setattr(backend_module, "list_serial_ports", lambda: [board])
    order = []
    monkeypatch.setattr(twin.link, "rearm", lambda: order.append("rearm"))
    monkeypatch.setattr(twin.real, "connect", lambda _port: (order.append("connect"), (False, "포트 없음"))[1])
    assert twin.reconnect() == ("포트 없음", None)
    assert order == ["connect", "rearm"]


def test_reconnect_refuses_a_different_board(twin, monkeypatch):
    _lose_comm(twin, monkeypatch)
    other = {"device": "COM7", "vid": 0x1A86, "pid": 0x55D3, "serial": "OTHER", "description": ""}
    monkeypatch.setattr(backend_module, "list_serial_ports", lambda: [other])
    reason, torque_off = twin.reconnect()
    assert "안 보인다" in reason and torque_off is None


def test_reconnect_leaves_a_healthy_link_alone(twin, monkeypatch):
    monkeypatch.setattr(twin.real, "connect", lambda _port: pytest.fail("끊긴 적 없는데 다시 연결했다"))
    assert twin.reconnect()[0] == "통신이 끊긴 상태가 아닙니다"


def test_failed_read_after_reconnect_still_offers_reconnect(twin, monkeypatch):
    _lose_comm(twin, monkeypatch)
    monkeypatch.setattr(twin, "_reconnect_failed", False)  # 다른 테스트로 새지 않게 되돌린다
    board = {"device": "COM7", "vid": 0x1A86, "pid": 0x55D3, "serial": twin.real.cfg.board_serial, "description": ""}
    monkeypatch.setattr(backend_module, "list_serial_ports", lambda: [board])

    def half_connect(_port):
        monkeypatch.setattr(twin.real.ctl, "fault", None)  # 새 컨트롤러에는 fault가 없다
        raise ConnectionError("no status packet")

    monkeypatch.setattr(twin.real, "connect", half_connect)
    assert twin.reconnect() == ("no status packet", None)
    assert twin.comm_lost()


def test_out_of_bounds_explains_when_starting_outside_the_table(twin, monkeypatch):
    result = SimpleNamespace(status="out_of_bounds", reason="workspace")
    assert twin._run_on(twin.real, lambda _arm: result) == backend_module.EDGE_TEXT  # 가장자리 거부는 따로 알린다(그 자리에서 끌지 묻는다)
    monkeypatch.setattr(twin.real.safety, "box", lambda: {"x": [0, 1], "y": [0, 1], "z": [0, None]})
    assert twin._run_on(twin.real, lambda _arm: result) == OUTSIDE_TEXT
    floor = SimpleNamespace(status="out_of_bounds", reason="z_floor")  # 밖에서 출발해도 바닥에 걸린 것은 이 문장이 아니다
    assert twin._run_on(twin.real, lambda _arm: floor) == backend_module.MOTION_TEXT["out_of_bounds"]


def test_estop_still_stops_when_saving_a_recording_fails(twin, tmp_path, monkeypatch):
    monkeypatch.setattr(twin.real.teach, "dir", tmp_path)
    assert twin.record_begin("lost") is None
    calls = []
    real_stop = twin.real.record_stop

    def broken(_name):
        raise ConnectionError("no status packet")

    monkeypatch.setattr(twin.real, "record_stop", broken)
    monkeypatch.setattr(twin.real, "estop", lambda: calls.append("estop"))
    twin.estop()
    assert calls == ["estop"]
    monkeypatch.setattr(twin.real, "record_stop", real_stop)
    twin.real.record_stop("lost")


def test_page_confirm_dialog_runs_pending_on_real(twin):
    async def go():
        async with user_simulation(gui_app.make_root(twin)) as user:
            await user.open("/logs")
            for _ in range(40):
                if twin.owner:
                    break
                await asyncio.sleep(0.05)
            assert twin.set_run_mode("sim_first") is None
            real = _pan(twin.real)
            assert await asyncio.to_thread(twin.jog_step, "joint", 0, 10) is None
            await user.should_see(marker="pending-confirm")
            user.find(marker="pending-confirm").click()
            for _ in range(100):
                if abs(_pan(twin.real) - (real + 10)) < 0.5:
                    break
                await asyncio.sleep(0.05)
            assert _pan(twin.real) == pytest.approx(real + 10, abs=0.5)

    asyncio.run(go())


def test_sim_first_preview_follows_the_moving_sim_arm_even_after_planning(shared_twin):
    """gui-spec 11절 B8: 목표를 고른 뒤(계획 중)에도 시뮬레이션 팔이 움직이는 동안은 계산 자세가 그것을 따라간다."""
    shown = []
    shared_twin.scene, shared_twin._planning = SimpleNamespace(set_preview=shown.append), True
    try:
        shared_twin._on_twin_status(SimpleNamespace(state="idle", joints="goal"))
        shared_twin._on_twin_status(SimpleNamespace(state="moving", joints="step"))
        assert shown == ["step"]  # 멈춰 있을 때는 고른 목표 자세를 지킨다
    finally:
        shared_twin.scene, shared_twin._planning = None, False


def test_table_edge_refusal_is_told_apart_and_can_be_switched_off(shared_twin):
    """작업대 가장자리(workspace) 거부는 따로 알리고, 그 자리에서 가장자리 검사만 끌 수 있다(자유 모드)."""
    from so101_studio.core.types import MotionResult, ToolPose

    arm = shared_twin.sim
    edge = MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0), reason="workspace")
    assert Backend._run_on(arm, lambda a: edge) in (backend_module.EDGE_TEXT, OUTSIDE_TEXT)
    before = shared_twin.get_safety()
    try:
        assert shared_twin.free_table_edge() is None
        assert shared_twin.get_safety()["mode"] == "free"
        assert shared_twin.get_safety()["joint_margin_deg"] == before["joint_margin_deg"]
    finally:
        shared_twin.set_safety(before["mode"], before["joint_margin_deg"], before["box_mm"])


def test_component_states_show_the_fault_sentence():
    """REQ-034: 원인과 해결법 문장이 오면 상태줄이 그 문장을 보인다."""
    status = ArmStatus(t=10.0, joints=JointState(0, 0, 0, 0, 0, 0), tool=ToolPose(0, 0, 0, 0), temperature_c={}, load_pct={},
                       voltage_v=12.0, torque_enabled=True, state="fault", fault="step:elbow_flex", fault_text="elbow_flex가 한 제어 주기에 ...")
    assert component_states(status, 10.0)["서보"] == ("error", "elbow_flex가 한 제어 주기에 ...")


def test_fault_result_shows_the_cause_sentence():
    result = MotionResult("fault", 0.0, ToolPose(0, 0, 0, 0), reason="서보와의 통신이 0.5초 넘게 끊겼습니다.")
    assert Backend._run_on(None, lambda arm: result) == "서보와의 통신이 0.5초 넘게 끊겼습니다."


def test_estop_reaches_the_arm_even_when_earlier_steps_fail(backend, monkeypatch):
    """감사 2026-11-07: 기록 저장·작업 정지가 예외를 내도 팔 정지는 반드시 간다."""
    def broken():
        raise RuntimeError("기록 저장 실패")

    monkeypatch.setattr(backend, "record_finish", broken)
    backend.estop()
    assert backend.arm.ctl.state == "stopped"
    backend.resume()


def test_run_on_shows_the_real_reason_unless_the_arm_is_busy():
    """감사 2026-11-07: 동작 중이 아닐 때의 RuntimeError(잠김, 연결 안 됨)는 '아직 움직이는 중'으로 바꾸지 않는다."""
    from so101_studio.gui.backend import BUSY_TEXT

    def fail(_arm):
        raise RuntimeError("연결되지 않음")

    idle = SimpleNamespace(ctl=SimpleNamespace(busy=False))
    busy = SimpleNamespace(ctl=SimpleNamespace(busy=True))
    assert Backend._run_on(idle, fail) == "연결되지 않음"
    assert Backend._run_on(busy, fail) == BUSY_TEXT


def test_estop_reports_when_the_servo_write_failed(backend, monkeypatch):
    """REQ-043: 팔이 정지 자세를 쓰지 못했다고 하면 화면이 성공 대신 그 이유와 전원을 끄라는 안내를 보인다."""
    monkeypatch.setattr(backend.arm, "estop", lambda: "SerialException('port gone')")
    reason = backend.estop()
    assert reason and "port gone" in reason and "전원 스위치" in reason
