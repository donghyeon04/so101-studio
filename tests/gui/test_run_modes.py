"""실행 방식 4가지 (gui-spec G9). 실제 팔 자리에 두 번째 SimDriver를 넣어 시험한다. 실제 장치는 쓰지 않는다."""

import asyncio
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot
from so101_studio.gui import app as gui_app
from so101_studio.gui.backend import RUN_MODES, Backend


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
