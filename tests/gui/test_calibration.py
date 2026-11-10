"""캘리브레이션 마법사 (gui-spec 12.3): 시뮬레이션은 설명만, 실제 팔이면 준비 체크 → 측정(연결을 끊고 다시 연결) → 비교표 → 저장(yes) → 0° 확인."""

import asyncio
import time
from pathlib import Path

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.arm.autocal import cli as autocal_cli
from so101_studio.gui import app as gui_app

REPORT = {"passed": True, "pass_deg": 3.0, "run_dir": "runs/20261110-autocal",
          "rows": [{"joint": "shoulder_pan", "old_range_deg": 220.0, "new_range_deg": 221.0, "range_diff_deg": 1.0, "zero_diff_deg": 0.5, "ok": True}]}


async def _until(check, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


@pytest.fixture
def real_like(backend, monkeypatch):
    """시뮬레이션 팔을 실제 자리에 두고 포트·연결은 가짜로: 측정 흐름만 본다."""
    calls = []
    monkeypatch.setattr(backend, "real", backend.sim)
    monkeypatch.setattr(backend, "_pick_port", lambda: "COM9")
    monkeypatch.setattr(backend.sim, "disconnect", lambda: calls.append("disconnect"))
    monkeypatch.setattr(backend, "_connect_real", lambda: (calls.append("connect"), (None, None))[1])
    return calls


def test_measure_disconnects_runs_and_reconnects(backend, real_like, monkeypatch):
    seen = {}

    def fake(cfg, *, out, stop, runs_dir, **kw):
        seen["port"] = cfg.port
        assert backend.move_lock()  # 측정하는 동안 움직이는 버튼은 잠긴다
        out("측정 중: shoulder_pan")
        return dict(REPORT)

    monkeypatch.setattr(autocal_cli, "measure_once", fake, raising=False)
    lines = []
    report, reason = backend.calibration_measure(lines.append)
    assert reason is None and report["passed"] and seen["port"] == "COM9"
    assert real_like == ["disconnect", "connect"] and lines == ["측정 중: shoulder_pan"] and backend.move_lock() is None


def test_measure_error_still_reconnects(backend, real_like, monkeypatch):
    from so101_studio.arm.autocal.engine import AutoCalibrationError

    def fake(cfg, **kw):
        raise AutoCalibrationError("전압 10.6V가 11.0V 아래라 멈췄다")

    monkeypatch.setattr(autocal_cli, "measure_once", fake, raising=False)
    report, reason = backend.calibration_measure()
    assert report is None and "10.6V" in reason and real_like == ["disconnect", "connect"]


def test_simulation_shows_steps_only(backend):
    assert backend.calibration_measure()[1] == "캘리브레이션 측정은 실제 팔에서만 합니다"

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/calibration")
            await user.should_see(marker="calib-sim", retries=30)
            assert await _until(lambda: not _one(user, "calib-next").enabled)

    asyncio.run(go())


def test_wizard_prepares_measures_and_compares(backend, real_like, monkeypatch):
    monkeypatch.setattr(autocal_cli, "measure_once", lambda cfg, **kw: dict(REPORT), raising=False)

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/calibration")
            await user.should_see(marker="calib-prepare", retries=30)
            assert await _until(lambda: backend.owner is not None and backend.arm.ctl.state == "idle")
            assert await _until(lambda: "세 가지" in _one(user, "calib-why").text)  # 체크 전에는 [다음]이 꺼져 있다
            for i in range(3):
                _one(user, f"calib-check-{i}").value = True
            assert await _until(lambda: _one(user, "calib-next").enabled)
            user.find(marker="calib-next").click()
            await user.should_see(marker="calib-start", retries=30)
            user.find(marker="calib-start").click()
            await user.should_see(marker="calib-verdict", retries=30)
            assert await _until(lambda: "합격" in _one(user, "calib-verdict").text)
            await user.should_see(marker="calib-row-shoulder_pan", retries=30)
            user.find(marker="calib-to-save").click()  # 1회 합격이면 3단계 표 그대로 저장 단계에 보인다
            await user.should_see(marker="calib-save-row-shoulder_pan", retries=30)
            assert await _until(lambda: not _one(user, "calib-save-run").enabled)  # yes를 넣기 전에는 꺼져 있다
            user.find(marker="calib-yes").type("yes")
            assert await _until(lambda: _one(user, "calib-save-run").enabled)

    asyncio.run(go())


def test_save_needs_yes_and_uses_the_cli_engine_with_port_released(backend, real_like, monkeypatch):
    seen = {}

    def fake(a, cfg, *, input_fn, out, runs_dir, **kw):
        seen.update(save=a.save, port=cfg.port, answer=input_fn("yes를 입력"), config_dir=kw.get("config_dir"))
        assert backend.move_lock()  # 저장하는 동안 움직이는 버튼은 잠긴다
        return 0

    monkeypatch.setattr(autocal_cli, "run_calibrate", fake)
    assert backend.calibration_save("네") == (False, "yes를 입력해야 저장합니다") and real_like == []
    assert backend.calibration_save(" YES ") == (True, None)
    assert seen == {"save": True, "port": "COM9", "answer": "yes", "config_dir": Path(backend.real.config_dir)}  # 저장 위치는 팔 설정 폴더(현재 폴더가 아님)
    assert real_like == ["disconnect", "connect"]
    backend._calibrating = True  # 다른 탭이 측정·저장 중이면 같은 포트를 열지 않는다
    try:
        assert backend.calibration_save("yes")[1] == "팔이 아직 움직이고 있습니다" and backend.calibration_measure()[1] == "팔이 아직 움직이고 있습니다"
    finally:
        backend._calibrating = False
    monkeypatch.setattr(autocal_cli, "run_calibrate", lambda *a, **kw: 1)
    ok, reason = backend.calibration_save("yes")
    assert not ok and "저장하지 못했" in reason


def test_verify_zero_asks_after_the_move_and_reconnects_only_when_recorded(backend, real_like, monkeypatch):
    from so101_studio.arm.autocal import verify

    monkeypatch.setattr(backend, "real_allowed", lambda: True)
    asked = []

    def fake(svc, cfg, *, input_fn, out, **kw):
        input_fn("준비됐으면 Enter")  # 마법사는 바로 넘긴다
        reply = input_fn("yes를 입력")
        asked.append(reply)
        return 0 if reply == "yes" else 1

    monkeypatch.setattr(verify, "run_verify_zero", fake)
    assert backend.calibration_verify_zero(lambda: "no") == (False, "확인하지 않았습니다. 움직이는 명령은 계속 잠겨 있습니다")
    assert real_like == []
    assert backend.calibration_verify_zero(lambda: "yes") == (True, None)
    assert asked == ["no", "yes"] and real_like == ["disconnect", "connect"]  # 기록한 뒤 다시 연결해 잠금을 푼다


def test_zero_step_moves_then_asks_and_records(backend, real_like, monkeypatch):
    def fake(answer, out=print):
        out("팔이 0° 자세로 섰다")
        return answer() == "yes", None

    monkeypatch.setattr(backend, "calibration_verify_zero", fake)

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/calibration")
            await user.should_see(marker="calib-prepare", retries=30)
            assert await _until(lambda: backend.owner is not None and backend.arm.ctl.state == "idle")
            for i in range(3):
                _one(user, f"calib-check-{i}").value = True
            assert await _until(lambda: _one(user, "calib-zero-only").enabled)
            user.find(marker="calib-zero-only").click()
            await user.should_see(marker="calib-zero-move", retries=30)
            user.find(marker="calib-zero-move").click()
            await user.should_see(marker="calib-zero-yes", retries=30)  # 선 뒤에만 [맞음]·[다름]이 보인다
            user.find(marker="calib-zero-yes").click()
            ok = await _until(lambda: "기록했습니다" in _one(user, "calib-zero-now").text)
            assert ok, _one(user, "calib-zero-now").text

    asyncio.run(go())
