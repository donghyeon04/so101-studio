"""장치·시스템 페이지 (gui-spec G8): 팔, 실행 기록, 로그."""

import asyncio
import json
import logging
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.core.types import ArmStatus, JointState, ToolPose
from so101_studio.gui import app as gui_app
from so101_studio.gui import backend as backend_module
from so101_studio.gui.pages.arm import doctor_summary, servo_rows
from so101_studio.gui.pages.logs import matches
from so101_studio.gui.pages.runs import line_options
from so101_studio.gui.state import LogRecord


async def _until(check, timeout=5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


def in_app(backend, path, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await check(user)

    asyncio.run(go())


@pytest.fixture
def runs_dir(backend, tmp_path, monkeypatch):
    run = tmp_path / "20260101-100000"
    run.mkdir()
    events = [{"t": 1000.0, "kind": "connect", "data": {"port": "COM3"}}, {"t": 1001.0, "kind": "estop", "data": {}}]
    (run / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events) + "\n{깨진 줄", encoding="utf-8")
    header = "t,shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper,x_mm,y_mm,z_mm,max_temp_c,max_load_pct,voltage_v,current_a\n"
    rows = "".join(f"{1000 + i * 0.1},0,0,0,0,0,0,0,0,0,{30 + i},0,{12 - i * 0.1},0\n" for i in range(5))
    (run / "status.csv").write_text(header + rows + "1000.5,잘린줄", encoding="utf-8")
    empty = tmp_path / "20260101-090000"
    empty.mkdir()
    (empty / "events.jsonl").write_text("", encoding="utf-8")
    (tmp_path / "watchdog-1.jsonl").write_text("{}", encoding="utf-8")
    monkeypatch.setattr(backend, "runs_root", str(tmp_path))
    return tmp_path


def test_servo_rows_show_six_servos():
    status = ArmStatus(0.0, JointState(1, 2, 3, 4, 5, 6), ToolPose(0, 0, 0, 0), {"shoulder_pan": 41.0}, {"shoulder_pan": 12.5}, 12.0, True, "idle")
    rows = servo_rows(status)
    assert len(rows) == 6
    assert rows[0]["position"] == "1.0" and rows[0]["temperature"] == "41" and rows[0]["load"] == "12.5" and rows[0]["state"] == "정상"
    assert rows[5]["joint"].startswith("6. ")
    assert servo_rows(None) == []
    status.fault = "overheat"
    assert servo_rows(status)[0]["state"] == "오류"


def test_arm_page_shows_table_and_power(backend):
    async def check(user):
        await user.should_see(marker="servo-table")
        assert await _until(lambda: "V" in _one(user, "power").text)
        assert await _until(lambda: len(_one(user, "servo-table").rows) == 6)

    in_app(backend, "/arm", check)


def test_doctor_card_shows_result_without_opening_ports(backend, monkeypatch):
    calls = []

    def fake(**kw):
        calls.append(kw)
        return {"ok": False, "ports": [], "arm": None, "esp32": None, "checks": [
            {"name": "numpy", "state": "ok", "text": "numpy 2.2.6", "fix": ""},
            {"name": "포트 목록", "state": "fail", "text": "포트 0개", "fix": "USB 케이블을 확인한다"},
            {"name": "서보 응답", "state": "skip", "text": "탐침하지 않았다", "fix": ""},
        ]}

    monkeypatch.setattr(backend_module, "doctor", fake)

    async def check(user):
        assert await _until(lambda: _one(user, "doctor-state").text == "정상 1 · 실패 1 · 건너뜀 1")
        await user.should_see("고치기: USB 케이블을 확인한다")
        assert "so101_follower_01" in _one(user, "arm-file").text
        assert "준비 중" in _one(user, "zero-check").text  # REQ-017 임시
        await user.should_not_see(marker="new-arm-steps")

    in_app(backend, "/arm", check)
    assert calls and all(kw["probe"] is False for kw in calls)


def test_arm_can_be_changed_for_the_next_start_when_there_are_several(backend, tmp_path, monkeypatch):
    local = tmp_path / "local.yaml"
    monkeypatch.setattr(backend, "local_path", str(local))
    monkeypatch.setattr(backend, "arm_names", lambda: ["so101_follower_01", "other"])

    async def check(user):
        assert await _until(lambda: backend.owner is not None and _one(user, "arm-change").enabled)
        user.find(marker="arm-change").click()
        assert await _until(local.exists)
        await user.should_see("다시 시작하면 적용됩니다")

    in_app(backend, "/arm", check)
    assert local.read_text(encoding="utf-8") == "arm: so101_follower_01\n"


def test_doctor_summary_ignores_skipped_checks():
    assert doctor_summary({"checks": [{"state": "ok"}, {"state": "skip"}]}) == ("ok", "정상 1 · 건너뜀 1")
    assert doctor_summary({"checks": [{"state": "warn"}, {"state": "fail"}]})[0] == "fail"


def test_doctor_failure_is_reported_as_a_check(backend, monkeypatch):
    def broken(**_kw):
        raise OSError("없음")

    monkeypatch.setattr(backend_module, "doctor", broken)
    report = backend.run_doctor()
    assert report["ok"] is False and "없음" in report["checks"][0]["text"]


def test_choose_arm_keeps_local_comments_and_refuses_unknown(backend, tmp_path, monkeypatch):
    local = tmp_path / "local.yaml"
    monkeypatch.setattr(backend, "local_path", str(local))
    assert backend.choose_arm("no_such_arm") and not local.exists()
    assert backend.choose_arm("so101_follower_01") is None
    assert local.read_text(encoding="utf-8") == "arm: so101_follower_01\n"
    local.write_text("# 주석\n# arm: 예시\nport: COM9\narm: other\n", encoding="utf-8")
    assert backend.choose_arm("so101_follower_01") is None
    assert local.read_text(encoding="utf-8") == "# 주석\n# arm: 예시\nport: COM9\narm: so101_follower_01\n"
    local.write_text("port: COM9", encoding="utf-8")
    assert backend.choose_arm("so101_follower_01") is None
    assert local.read_text(encoding="utf-8") == "port: COM9\narm: so101_follower_01\n"


def test_servoinfo_runs_and_linkwatch_is_off_in_sim(backend):
    result, reason = backend.run_diagnostic("servoinfo")
    assert reason is None and result["name"] == "servoinfo"
    assert not backend.diagnostic_available("linkwatch")
    assert backend.diagnostic_available("powertest")
    result, reason = backend.run_diagnostic("linkwatch")
    assert result is None and "시뮬레이션" in reason


def test_powertest_can_be_stopped_right_away(tmp_path):
    from so101_studio.gui.backend import Backend

    backend = Backend(log_root=str(tmp_path))  # powertest는 실행 기록 폴더에 결과를 쓴다
    try:
        before = backend.arm.status().joints
        result, reason = backend.run_diagnostic("powertest", None, lambda: True)
        assert reason is None and result["aborted"] is True
        assert abs(backend.arm.status().joints.shoulder_pan - before.shoulder_pan) < 90  # 크게 휘두르지 않는다
    finally:
        backend.close()


def test_big_move_needs_confirmation_on_page(backend):
    calls = []
    original = backend.run_diagnostic
    backend.run_diagnostic = lambda name, *a, **k: calls.append(name) or ({"name": name}, None)
    try:
        async def check(user):
            assert await _until(lambda: backend.state.status is not None and _one(user, "diag-powertest").enabled)
            user.find(marker="diag-powertest").click()
            await user.should_see(marker="diag-confirm")
            assert calls == []
            user.find(marker="diag-cancel").click()
            await asyncio.sleep(0.1)
            assert calls == []
            user.find(marker="diag-powertest").click()
            await user.should_see(marker="diag-confirm")
            user.find(marker="diag-confirm").click()
            assert await _until(lambda: calls == ["powertest"])
            user.find(marker="diag-servoinfo").click()  # 설정 읽기는 확인 창이 없다
            assert await _until(lambda: calls == ["powertest", "servoinfo"])

        in_app(backend, "/arm", check)
    finally:
        del backend.run_diagnostic
        assert backend.run_diagnostic == original


def test_view_only_arm_page_cannot_run_diagnostics(backend):
    backend.owner = "다른 브라우저"

    async def check(user):
        assert await _until(lambda: backend.state.status is not None)
        assert await _until(lambda: not _one(user, "diag-servoinfo").enabled)

    in_app(backend, "/arm", check)


def test_run_list_is_newest_first_and_skips_other_files(backend, runs_dir):
    assert backend.run_names() == ["20260101-100000", "20260101-090000"]


def test_load_run_reads_events_and_charts_and_survives_cut_lines(backend, runs_dir):
    data = backend.load_run("20260101-100000")
    assert [e["kind"] for e in data["events"]] == ["connect", "estop"]
    assert data["rows"] == 5
    assert data["voltage"][0] == [0.0, 12.0] and data["temperature"][-1][1] == 34
    assert data["duration_s"] == pytest.approx(0.4)


def test_load_run_refuses_other_paths(backend, runs_dir):
    with pytest.raises(ValueError):
        backend.load_run("../secret")


def test_run_charts_are_single_series_on_one_axis():
    options = line_options("전압", "V", [[0, 12.0]], "#000000")
    assert len(options["series"]) == 1 and isinstance(options["yAxis"], dict)


def test_runs_page_shows_charts_when_a_run_is_picked(backend, runs_dir):
    async def check(user):
        select = _one(user, "run-select")
        assert select.options == backend.run_names()
        select.value = "20260101-100000"
        await user.should_see(marker="run-voltage")
        await user.should_see(marker="run-temperature")
        await user.should_see(marker="run-events")
        await user.should_see("추가 예정")
        select.value = "20260101-090000"
        await user.should_see(marker="run-empty")

    in_app(backend, "/runs", check)


def test_log_filter_by_level_and_text():
    record = LogRecord(0.0, "WARNING", "Estop 눌림")
    assert matches(record, "INFO", "") and matches(record, "WARNING", "estop")
    assert not matches(record, "ERROR", "") and not matches(record, "INFO", "없는말")


def test_logbook_collects_python_warnings(backend):
    logging.getLogger("so101.test").warning("경고 시험")
    logging.getLogger("so101.test").info("정보는 모으지 않음")
    texts = [r.text for r in backend.logs()]
    assert any("경고 시험" in t for t in texts)
    assert not any("정보는 모으지 않음" in t for t in texts)


def test_logs_read_run_events_incrementally(tmp_path):
    from so101_studio.gui.backend import Backend

    backend = Backend(log_root=str(tmp_path))
    try:
        first = [r.text for r in backend.logs() if r.text.startswith("connect")]
        assert len(first) == 1
        backend.estop()
        records = backend.logs()
        assert [r.level for r in records if r.text.startswith("estop")] == ["WARNING"]
        assert len([r for r in records if r.text.startswith("connect")]) == 1  # 다시 읽어도 겹치지 않는다
    finally:
        backend.close()


def test_logs_page_filters_live(backend):
    logging.getLogger("so101.test").warning("찾을 말 abc")

    async def check(user):
        await user.should_see("찾을 말 abc")
        _one(user, "log-search").value = "없는말"
        await user.should_not_see("찾을 말 abc")
        _one(user, "log-search").value = "abc"
        await user.should_see("찾을 말 abc")
        _one(user, "log-level").value = "ERROR"
        await user.should_not_see("찾을 말 abc")

    in_app(backend, "/logs", check)
