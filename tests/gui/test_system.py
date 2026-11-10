"""장치·시스템 페이지 (gui-spec G8): 팔, 실행 기록, 로그."""

import asyncio
import json
import logging
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.core.types import ArmStatus, JointState, ToolPose
from so101_studio.gui import app as gui_app
from so101_studio.gui import backend as backend_module
from so101_studio.gui.backend import with_first_report
from so101_studio.gui.pages import params as params_page
from so101_studio.gui.pages.arm import doctor_summary, servo_rows
from so101_studio.gui.pages.logs import matches
from so101_studio.gui.pages.runs import line_options
from so101_studio.gui.state import LogRecord, component_states, run_bands


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
        return {"ok": False, "ports": [], "arm": None, "esp32": None,
                "zero_check": {"state": "checked", "reason": "", "date": "2026-11-10", "sha256": "ab"}, "checks": [
            {"name": "numpy", "state": "ok", "text": "numpy 2.2.6", "fix": ""},
            {"name": "팔 파일", "state": "fail", "text": "팔 파일 없음", "fix": "USB 케이블을 확인한다"},
            {"name": "서보 응답", "state": "skip", "text": "탐침하지 않았다", "fix": ""},
        ]}

    monkeypatch.setattr(backend_module, "doctor", fake)

    async def check(user):
        assert await _until(lambda: _one(user, "doctor-state").text == "정상 1 · 실패 1 · 건너뜀 1")
        await user.should_see("고치기: USB 케이블을 확인한다")
        assert "so101_follower_01" in _one(user, "arm-file").text
        assert "확인됨 (2026-11-10)" in _one(user, "zero-check").text
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
    assert report["zero_check"]["state"] == "needed"  # 0° 카드는 늘 이 필드를 읽는다


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


def test_log_filter_by_source_and_code():
    record = LogRecord(0.0, "ERROR", "trip: {}", "watchdog", "WATCHDOG-TRIP")
    assert matches(record, "INFO", "", "watchdog") and matches(record, "INFO", "watchdog-trip")
    assert not matches(record, "INFO", "", "gui")
    assert matches(LogRecord(0.0, "ERROR", "connect"), "INFO", "", "gui")


def _line(t_s, kind, proc, level="INFO", code=None):
    return json.dumps({"t": int(t_s * 1e9), "proc": proc, "pid": 1, "seq": 1, "level": level, "code": code, "kind": kind, "data": {}})


def test_logs_merge_processes_on_one_time_line(tmp_path):
    from so101_studio.gui.backend import Backend

    backend = Backend(log_root=str(tmp_path))
    try:
        now = time.time()
        other = tmp_path / "cli-run"
        other.mkdir()
        (other / "events.jsonl").write_text(_line(now - 0.1, "goto", "cli") + "\n", encoding="utf-8")
        (other / "watchdog.jsonl").write_text(_line(now - 0.2, "trip", "watchdog", "ERROR", "WATCHDOG-TRIP") + "\n{잘린 줄", encoding="utf-8")
        records = [r for r in backend.logs() if r.run == "cli-run"]
        assert [(r.source, r.level, r.code) for r in records] == [("watchdog", "ERROR", "WATCHDOG-TRIP"), ("cli", "INFO", None)]
        assert any(r.source == "gui" and r.text.startswith("connect") for r in backend.logs())
    finally:
        backend.close()


def test_run_bands_split_only_where_values_change():
    states = [(10.0 + i, {"torque_enabled": i < 2, "state": "idle" if i < 3 else "fault", "fault": "overheat" if i >= 3 else None}) for i in range(5)]
    bands = run_bands(states, 10.0)
    assert bands["토크"] == [("켜짐", 0.0, 2.0), ("꺼짐", 2.0, 4.0)]
    assert bands["상태"] == [("대기", 0.0, 3.0), ("오류", 3.0, 4.0)]
    assert bands["오류"] == [("overheat", 3.0, 4.0)]
    assert run_bands([(1.0, {"voltage_v": 12.0})], 1.0) == {"토크": [], "상태": [], "오류": []}  # 옛 형식


def test_load_run_reads_new_format_with_bands(backend, tmp_path, monkeypatch):
    monkeypatch.setattr(backend, "runs_root", str(tmp_path))
    run = tmp_path / "20260102-100000"
    run.mkdir()
    (run / "events.jsonl").write_text(_line(1000.5, "fault", "gui", "ERROR", "ARM-FAULT") + "\n", encoding="utf-8")
    rows = [{"t": int((1000 + i) * 1e9), "proc": "gui", "pid": 1, "seq": i, "level": "INFO", "code": None, "kind": "status",
             "data": {"voltage_v": 12.0, "temperature_c": {"a": 30.0 + i}, "torque_enabled": i == 0, "state": "idle", "fault": None}} for i in range(3)]
    (run / "status.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
    data = backend.load_run(run.name)
    assert data["events"][0]["code"] == "ARM-FAULT" and data["events"][0]["t"] == pytest.approx(1000.5)
    assert data["temperature"][-1][1] == 32.0
    assert data["bands"]["토크"] == [("켜짐", 0.0, 1.0), ("꺼짐", 1.0, 2.0)]


def test_old_format_run_has_no_bands_but_still_opens(backend, runs_dir):
    data = backend.load_run("20260101-100000")
    assert data["rows"] == 5 and data["voltage"] and not any(data["bands"].values())


def test_incident_zip_refuses_other_paths_and_packs_the_folder(backend, tmp_path, monkeypatch):
    monkeypatch.setattr(backend, "runs_root", str(tmp_path))
    folder = tmp_path / "incidents" / "20260101-MANUAL"
    folder.mkdir(parents=True)
    (folder / "meta.json").write_text(json.dumps({"code": "MANUAL", "reason": "시험"}), encoding="utf-8")
    assert [i["name"] for i in backend.incidents()] == ["20260101-MANUAL"]
    with pytest.raises(ValueError):
        backend.incident_zip("../../secret")
    names = zipfile.ZipFile(backend.incident_zip("20260101-MANUAL")).namelist()
    assert "meta.json" in names


def test_make_incident_on_a_recording_run(tmp_path):
    from so101_studio.gui.backend import Backend

    backend = Backend(log_root=str(tmp_path))
    try:
        path = backend.make_incident()
        assert path and Path(path).is_dir()
        assert backend.incidents()[0]["name"] == Path(path).name
    finally:
        backend.close()


def test_run_page_opens_at_the_focused_event_and_lists_incidents(backend, runs_dir):
    backend.run_focus = ("20260101-100000", 1001.0)

    async def check(user):
        await user.should_see(marker="run-voltage", retries=60)
        await user.should_see(marker="run-no-bands")
        await user.should_see(marker="incident-empty")
        chart = _one(user, "run-voltage")
        assert chart.options["series"][0]["markLine"]["data"] == [{"xAxis": 1.0}]

    in_app(backend, "/runs", check)
    assert backend.run_focus is None


def _status(t, fault=None, temp=30.0, load=5.0):
    return SimpleNamespace(t=t, fault=fault, temperature_c={"elbow_flex": temp}, load_pct={"elbow_flex": load})


def test_component_states_never_show_ok_for_old_or_missing_values():
    assert component_states(None, 10.0)["버스"][0] == "stale"
    assert component_states(_status(1.0), 10.0)["서보"][0] == "stale"
    assert {k: v[0] for k, v in component_states(_status(10.0), 10.1).items()} == {"버스": "ok", "서보": "ok"}


@pytest.mark.parametrize(("kw", "bus", "servo"), [
    ({"temp": 55.0}, "ok", "warn"), ({"load": 70.0}, "ok", "warn"),
    ({"fault": "overheat:elbow_flex"}, "ok", "error"), ({"fault": "comm_lost"}, "error", "ok"),
])
def test_component_states_levels(kw, bus, servo):
    states = component_states(_status(10.0, **kw), 10.0)
    assert (states["버스"][0], states["서보"][0]) == (bus, servo)


def test_top_bar_shows_component_states(backend):
    async def check(user):
        assert await _until(lambda: _one(user, "component-버스").text == "정상")
        assert await _until(lambda: _one(user, "component-서보").text == "정상")

    in_app(backend, "/logs", check)


def test_top_bar_and_banner_follow_comm_loss_without_reload(backend, monkeypatch):
    lost = [False]
    monkeypatch.setattr(backend, "comm_lost", lambda: lost[0])
    monkeypatch.setattr(backend, "move_lock", lambda: "통신 끊김 시험" if lost[0] else None)

    async def check(user):
        assert await _until(lambda: _one(user, "component-버스").text == "정상")
        await user.should_not_see(marker="lock-banner")
        await user.should_not_see(marker="reconnect")
        lost[0] = True
        assert await _until(lambda: _one(user, "component-버스").text == "오류")
        await user.should_see(marker="lock-banner")
        await user.should_see("통신 끊김 시험")
        await user.should_see(marker="reconnect")
        await user.should_not_see(marker="resume")

    in_app(backend, "/logs", check)


def _doctor_with(backend, monkeypatch, **fields):
    report = {"ok": True, "checks": [{"name": "포트", "state": "ok", "text": "COM3", "fix": ""}], "ports": [],
              "zero_check": {"state": "needed", "reason": "확인 기록이 없다", "date": None, "sha256": None}, **fields}
    monkeypatch.setattr(backend, "run_doctor", lambda: report)


@pytest.mark.parametrize(("zero", "text"), [
    ({"state": "checked", "reason": "", "date": "2026-11-04", "sha256": "ab"}, "확인됨 (2026-11-04)"),
    ({"state": "needed", "reason": "확인 기록이 없다", "date": None, "sha256": None}, "필요 (확인 기록이 없다)"),
])
def test_zero_check_card_reads_the_doctor_field(backend, monkeypatch, zero, text):
    _doctor_with(backend, monkeypatch, zero_check=zero)

    async def check(user):
        assert await _until(lambda: text in _one(user, "zero-check").text)

    in_app(backend, "/arm", check)


def test_zero_check_lock_points_to_verify_zero_not_new_arm_steps(backend, monkeypatch):
    _doctor_with(backend, monkeypatch)
    monkeypatch.setattr(backend, "setup_lock", lambda: "0° 자세 확인 필요: 기록 없음")

    async def check(user):
        await user.should_see(marker="zero-check-step")
        await user.should_not_see(marker="new-arm-steps")

    in_app(backend, "/arm", check)


def test_comm_loss_does_not_show_new_arm_steps(backend, monkeypatch):
    _doctor_with(backend, monkeypatch)
    monkeypatch.setattr(backend, "move_lock", lambda: "통신 끊김 시험")

    async def check(user):
        await user.should_not_see(marker="new-arm-steps")

    in_app(backend, "/arm", check)


def test_first_report_fills_only_skipped_checks():
    first = {"checks": [
        {"name": "서보 응답", "state": "ok", "text": "6/6", "fix": ""},
        {"name": "온도", "state": "skip", "text": "못 읽음", "fix": ""},
        {"name": "포트", "state": "fail", "text": "옛 값", "fix": ""},
    ]}
    now = {"ok": True, "checks": [
        {"name": "서보 응답", "state": "skip", "text": "탐침하지 않았다", "fix": ""},
        {"name": "온도", "state": "skip", "text": "탐침하지 않았다", "fix": ""},
        {"name": "포트", "state": "ok", "text": "COM3", "fix": ""},
    ]}
    checks = {c["name"]: c for c in with_first_report(now, first)["checks"]}
    assert checks["서보 응답"]["state"] == "ok" and "앱을 시작할 때" in checks["서보 응답"]["text"]
    assert checks["온도"]["state"] == "skip" and checks["포트"]["text"] == "COM3"
    assert with_first_report(now, None)["checks"] == now["checks"]
    bad = {"checks": [{"name": "서보 응답", "state": "fail", "text": "응답 없음", "fix": ""}]}
    assert with_first_report(now, bad)["ok"] is False


def test_grip_overload_reason_is_readable():
    assert "그리퍼 과부하" in component_states(_status(10.0, fault="grip_overload"), 10.0)["서보"][1]


def test_gripper_settings_have_their_own_labels(backend):
    listed = {key for _, group in params_page.CARDS for key in group}
    assert {"grip_preload_units", "gripper_hold_torque", "gripper_load_warn_pct", "gripper_load_s"} <= listed
    assert {"grip_preload_units", "gripper_hold_torque", "gripper_load_warn_pct", "gripper_load_s"} <= set(backend.arm.get_params())


def test_simulation_doctor_does_not_fail_on_missing_port(backend, monkeypatch):
    """시뮬레이션은 포트를 쓰지 않으니 포트가 없어도 실패로 보이지 않는다."""
    monkeypatch.setattr(backend_module, "doctor", lambda **kw: {"ok": False, "ports": [], "arm": None, "esp32": None, "checks": [
        {"name": "포트 목록", "state": "fail", "text": "포트 0개", "fix": "USB 케이블을 확인한다"}]})
    report = backend.run_doctor()
    assert report["ok"] and report["checks"][0]["state"] == "skip" and "시뮬레이션" in report["checks"][0]["text"]


def test_gripper_type_can_be_changed_for_the_next_start(backend, tmp_path, monkeypatch):
    """결정 0043-1·REQ-038: 지금 종류를 보이고, 고른 종류를 local.yaml에 적는다(주석 유지). 없는 종류는 거부."""
    from so101_studio.gui.pages.arm import gripper_text

    local = tmp_path / "local.yaml"
    local.write_text("# 주석\nport: COM9\n", encoding="utf-8")
    monkeypatch.setattr(backend, "local_path", str(local))
    assert backend.gripper_info()["type"] == "single_jaw"
    assert backend.choose_gripper("nope") and "gripper" not in local.read_text(encoding="utf-8")
    assert gripper_text({"type": "parallel", "max_open_mm": 84}) == "그리퍼: 병렬(두 턱이 대칭으로 움직임) · 최대 벌림 84 mm"

    async def check(user):
        assert "단동식" in _one(user, "gripper-type").text
        assert await _until(lambda: backend.owner is not None and _one(user, "gripper-change").enabled)
        _one(user, "gripper-picker").value = "parallel"
        user.find(marker="gripper-change").click()
        assert await _until(lambda: "gripper: parallel" in local.read_text(encoding="utf-8"))
        await user.should_see("다시 시작하면 적용됩니다")

    in_app(backend, "/arm", check)
    assert local.read_text(encoding="utf-8") == "# 주석\nport: COM9\ngripper: parallel\n"
