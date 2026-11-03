"""블랙박스 기록 (decisions/0030): 공통 머리, 큐 가득 참, 분할·보관, 링 버퍼, 사고 묶음, 토크 풀림, 예외, 감시 기록, incident 명령."""
import json
import os
import threading
import time
from pathlib import Path

import pytest

from so101_studio import bringup
from so101_studio.arm.logged_bus import LoggedBus
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.trajectory import move_j
from so101_studio.core.blackbox import RingBuffer, install_excepthooks
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.incident import create_incident
from so101_studio.core.logging import JsonlFile, RunLogger, status_to_csv
from so101_studio.core.logread import (
    list_incidents,
    list_runs,
    merge_timeline,
    read_events,
    read_incident,
    read_status,
)
from so101_studio.core.retention import prune_runs
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
HEAD = {"t", "proc", "pid", "seq", "level", "code", "kind", "data"}


def status_data(**kw):
    return {"joints": {n: 1.0 for n in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}, "tool": {"x_mm": 1, "y_mm": 2, "z_mm": 3},
            "temperature_c": {"a": 30.0}, "load_pct": {"a": 5.0}, "voltage_v": 12.0, "current_ma": 100.0, **kw}


def make_svc(tmp_path, **kw):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path), **kw)
    s.incident_after_s = 0
    s.config_dir = Path("config")
    ok, msg = s.connect()
    assert ok, msg
    return s


def wait_for(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_common_head_and_proc(tmp_path):
    log = RunLogger(tmp_path, proc="gui", prune=False)
    log.event("a")
    log.event("b", {"x": 1}, "WARN", "B-CODE")
    log.status(status_data())
    log.close()
    ev = read_events(log.dir)
    assert [e["kind"] for e in ev] == ["a", "b"] and all(HEAD <= set(e) and e["proc"] == "gui" and e["pid"] == os.getpid() for e in ev)
    assert ev[0]["seq"] < ev[1]["seq"] and ev[1]["code"] == "B-CODE" and ev[1]["level"] == "WARN" and ev[0]["t"] > 1e18
    assert read_status(log.dir)[0]["data"]["voltage_v"] == 12.0


def test_status_is_dropped_when_queue_full_and_counted_but_events_are_not(tmp_path):
    log = RunLogger(tmp_path, queue_lines=3, prune=False)
    gate = threading.Event()
    real = log._write_line
    log._write_line = lambda s, r: (gate.wait(5), real(s, r))  # 기록 스레드를 막아 큐를 채운다
    results = [log.status(status_data()) for _ in range(10)]
    assert results.count(False) >= 6 and log.dropped_total == results.count(False)
    log.event("kept")
    gate.set()
    log.flush()
    log.status(status_data())
    log.close()
    assert [e["kind"] for e in read_events(log.dir)] == ["kept"]
    assert read_status(log.dir)[-1]["data"]["dropped"] == results.count(False)


def test_warn_and_above_are_fsynced(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(os, "fsync", lambda fd: calls.append(fd))
    log = RunLogger(tmp_path, prune=False)
    log.event("info")
    log.flush()
    n = len(calls)
    log.event("bad", None, "ERROR")
    log.flush()
    log.close()
    assert n == 0 and len(calls) == 1


def test_status_rotates_and_csv_export(tmp_path):
    log = RunLogger(tmp_path, status_rotate_bytes=600, prune=False)
    for _ in range(12):
        log.status(status_data())
    log.close()
    assert (log.dir / "status.001.jsonl").exists()
    assert len(read_status(log.dir)) == 12
    csv_path = status_to_csv(log.dir)
    lines = csv_path.read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith("t,") and len(lines) == 13


def test_old_format_runs_still_read(tmp_path):
    run = tmp_path / "20200101-000000"
    run.mkdir()
    (run / "events.jsonl").write_text(json.dumps({"t": 1000.5, "kind": "connect", "data": {}}) + "\n잘린줄", encoding="utf-8")
    ev = read_events(run)
    assert ev[0]["t"] == 1_000_500_000_000 and ev[0]["level"] == "INFO" and list_runs(tmp_path) == [run.name]


def test_retention_prunes_old_and_big_but_never_incidents(tmp_path):
    now = time.time()
    for name, age, size in (("old", 40, 10), ("mid", 10, 1000), ("new", 1, 1000), ("incidents", 400, 10)):
        d = tmp_path / name
        d.mkdir()
        (d / "f").write_bytes(b"x" * size)
        os.utime(d / "f", (now - age * 86400,) * 2)
        os.utime(d, (now - age * 86400,) * 2)
    deleted = {p.name for p in prune_runs(tmp_path, max_age_days=30, max_bytes=1500, now=now)}
    assert deleted == {"old", "mid"} and (tmp_path / "incidents").exists() and (tmp_path / "new").exists()


def test_ring_buffer_keeps_ten_seconds():
    r = RingBuffer(seconds=10, hz=50)
    now = time.time_ns()
    r.add(now - int(11e9), None, 0.02, None, "idle")
    r.add(now - int(5e9), None, 0.05, None, "moving")
    r.add(now, None, 0.01, "err", "moving")
    d = r.dump(now)
    assert len(d) == 2 and d[0]["dt_ms"] == 50.0 and d[1]["err"] == "err"
    assert r.max_dt(1.0) == 0.01


def test_incident_bundle_contents_and_listing(tmp_path):
    run = RunLogger(tmp_path, prune=False)
    run.event("before", None, "WARN")
    run.flush()
    ring = [{"t": time.time_ns(), "dt_ms": 20.0, "cmd": None, "actual": None, "err": None, "state": "idle"}]
    inc = create_incident("ARM-FAULT", tmp_path, reason="시험", ring=ring, config_dir="config", cfg=CFG, after_s=0, doctor_fn=lambda: {"ok": True})
    run.close()
    d = inc.path
    assert d.parent == tmp_path / "incidents" and d.name.endswith("-ARM-FAULT")
    for n in ("ring.jsonl", "timeline.jsonl", "robot.yaml", "config.sha256.json", "doctor.json", "meta.json"):
        assert (d / n).exists(), n
    info = read_incident(d)
    assert info["meta"]["code"] == "ARM-FAULT" and info["meta"]["git"]["commit"] and "python" in info["meta"]["versions"]
    assert len(info["config_sha256"]["robot.yaml"]["sha256"]) == 64 and [r["kind"] for r in info["timeline"]] == ["before"]
    assert list_incidents(tmp_path)[0]["code"] == "ARM-FAULT"
    assert prune_runs(tmp_path, max_age_days=0, max_bytes=0, now=time.time() + 10) and d.exists()


def test_timeline_merges_processes_by_time(tmp_path):
    run = tmp_path / "20260101-000000"
    run.mkdir()
    a = JsonlFile(run / "events.jsonl", "gui")
    w = JsonlFile(run / "watchdog.jsonl", "watchdog")
    for f, k in ((a, "one"), (w, "two"), (a, "three")):
        f.write(k)
        time.sleep(0.01)  # 윈도 시계 해상도보다 크게 띄운다
    rows = merge_timeline(tmp_path, 0, time.time_ns() + int(1e9))
    assert [r["kind"] for r in rows] == ["one", "two", "three"] and {r["proc"] for r in rows} == {"gui", "watchdog"}


def test_logged_bus_records_register_writes_and_torque_but_not_goal_position():
    seen, calls = [], []

    class Bus:
        def write(self, *a, **k):
            calls.append(("write", a))

        def sync_write(self, *a, **k):
            calls.append(("sync_write", a))

        def disable_torque(self, *a, **k):
            calls.append(("disable", a))

        def sync_read(self, *a, **k):
            return {"x": 1}

    bus = LoggedBus(Bus(), lambda: lambda *a: seen.append(a))
    bus.sync_write("Goal_Position", {"a": 1})
    bus.write("Lock", "a", 0)
    bus.disable_torque()
    assert bus.sync_read("Present_Position") == {"x": 1} and len(calls) == 3
    assert [s[2] for s in seen] == ["ARM-REG-WRITE", "ARM-TORQUE-OFF"] and seen[1][1] == "WARN" and seen[0][3]["caller"]


def test_service_logs_connect_status_first_line_and_estop_incident(tmp_path):
    s = make_svc(tmp_path)
    try:
        s.logger.flush()
        assert read_status(s.logger.dir)  # 연결하자마자 첫 줄
        s.estop()
        assert wait_for(lambda: list_incidents(tmp_path))
        s.logger.flush()
        ev = read_events(s.logger.dir)
        assert any(e["code"] == "ARM-ESTOP" for e in ev) and [e["kind"] for e in ev].count("estop") == 1
        assert list_incidents(tmp_path)[0]["code"] == "ARM-ESTOP"
    finally:
        s.disconnect()


def test_torque_lost_is_detected_logged_and_makes_incident(tmp_path):
    s = make_svc(tmp_path)
    try:
        s.driver.set_torque(True)
        s.driver._torque = False  # 서보가 스스로 토크를 푼 상황: 기대는 켜짐, 읽기는 꺼짐
        assert wait_for(lambda: any(e["code"] == "ARM-TORQUE-LOST" for e in (s.logger.flush(), read_events(s.logger.dir))[1]))
        assert wait_for(lambda: any(i["code"] == "ARM-TORQUE-LOST" for i in list_incidents(tmp_path)))
        e = next(e for e in read_events(s.logger.dir) if e["code"] == "ARM-TORQUE-LOST")
        assert e["level"] == "ERROR" and e["data"]["expected"] is True
    finally:
        s.disconnect()


def test_status_loop_error_is_logged_once_with_traceback_and_loop_continues(tmp_path):
    s = make_svc(tmp_path)
    try:
        real, n = s.driver.read_health, {"c": 0}

        def flaky():
            n["c"] += 1
            if n["c"] <= 3:
                raise RuntimeError("버스 오류")
            return real()

        s.driver.read_health = flaky
        got = []
        s.status_stream(got.append)
        assert wait_for(lambda: n["c"] > 3 and (s.logger.flush(), any(e["code"] == "ARM-STATUS-LOOP-RECOVERED" for e in read_events(s.logger.dir)))[1], 8)  # 오류 뒤에도 루프가 계속 돈다
        assert len(got) >= 5
        errs = [e for e in read_events(s.logger.dir) if e["code"] == "ARM-STATUS-LOOP-ERROR"]
        assert len(errs) == 1 and "Traceback" in errs[0]["data"]["traceback"] and any(e["code"] == "ARM-STATUS-LOOP-RECOVERED" for e in read_events(s.logger.dir))
    finally:
        s.disconnect()


def test_unhandled_thread_exception_is_reported_and_original_hook_still_runs():
    got, orig = [], []
    old = threading.excepthook
    threading.excepthook = lambda a: orig.append(a.exc_value)
    restore = install_excepthooks(lambda t, v, tb, where: got.append((t, where)))
    try:
        th = threading.Thread(target=lambda: 1 / 0, name="Boom")
        th.start()
        th.join()
    finally:
        restore()
        threading.excepthook = old
    assert got == [(ZeroDivisionError, "Boom")] and isinstance(orig[0], ZeroDivisionError)


def test_watchdog_trip_and_manual_incident_api(tmp_path):
    s = make_svc(tmp_path, proc="gui")
    try:
        s.on_watchdog_stop("단축키 F9")
        assert wait_for(lambda: any(i["code"] == "WATCHDOG-TRIP" for i in list_incidents(tmp_path)))
        path = s.create_incident("버튼", last_s=30)
        assert path and Path(path).is_dir() and "MANUAL" in Path(path).name
        s.log_event("operator", {"holder": "gui"})
        s.logger.flush()
        assert any(e["kind"] == "operator" and e["proc"] == "gui" for e in read_events(s.logger.dir))
    finally:
        s.disconnect()


def test_watchdog_writes_run_dir_log_with_common_head(tmp_path):
    from so101_studio.watchdog import __main__ as wm

    t = threading.Thread(target=lambda: None)
    t.start()
    t.join()
    import sys

    argv = ["--run-dir", str(tmp_path), "--sim", "--port", "0"]
    th = threading.Thread(target=wm.main, args=(argv,), daemon=True)
    th.start()
    assert wait_for(lambda: (tmp_path / "watchdog.jsonl").exists() and (tmp_path / "watchdog.jsonl").stat().st_size > 0, 8)
    rec = json.loads((tmp_path / "watchdog.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert HEAD <= set(rec) and rec["proc"] == "watchdog" and rec["kind"] == "start" and sys


def test_incident_command_creates_bundle(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()
    src = Path(__file__).resolve().parents[1] / "config"
    (cfg_dir / "robot.yaml").write_bytes((src / "robot.yaml").read_bytes())
    run = tmp_path / "runs" / "20260101-000000"
    run.mkdir(parents=True)
    JsonlFile(run / "events.jsonl", "cli").write("hello")
    assert bringup.main.__name__ == "main"
    with pytest.raises(SystemExit) as e:
        bringup.main(["--config", str(cfg_dir / "robot.yaml"), "--last", "60", "incident"])
    assert e.value.code == 0 and "사고 묶음" in capsys.readouterr().out
    inc = list_incidents(tmp_path / "runs")
    assert len(inc) == 1 and any(r["kind"] == "hello" for r in read_incident(inc[0]["path"])["timeline"])


class FlakyPort(SimDriver):
    """USB가 빠진 상황: down이 True면 읽기·쓰기가 모두 예외. 쓰기 호출은 모두 기록한다."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.down = False
        self.writes = []

    def read_joints(self):
        if self.down:
            raise ConnectionError("no status packet")
        return super().read_joints()

    def read_health(self):
        if self.down:
            raise ConnectionError("no status packet")
        return super().read_health()

    def write_joints(self, q):
        self.writes.append((time.perf_counter(), q))
        if self.down:
            raise ConnectionError("no status packet")
        return super().write_joints(q)

    def set_torque(self, on):
        self.writes.append(("torque", on))
        return super().set_torque(on)


def test_port_pulled_makes_comm_lost_fault_within_0_6s_then_refuses_until_reconnect(tmp_path):
    d = FlakyPort(CFG.poses["home"])
    s = ArmService(d, CFG, WS, speed_pct=100, log_root=str(tmp_path))
    s.incident_after_s = 0
    assert s.connect()[0]
    try:
        time.sleep(0.3)  # 정지 중(홈): 쓰기가 없어도 상태 루프의 읽기로 끊김을 본다
        t0 = time.perf_counter()
        d.down = True
        assert wait_for(lambda: s.ctl.state == "fault", 1.0)
        assert time.perf_counter() - t0 < 0.6 and s.ctl.fault == "comm_lost"
        n = len(d.writes)
        assert s.move_joints(JointState(10, -90, 93, 0, 0, 3)).status == "fault" and s.resume() is False and s.ctl.state == "fault"  # 다시 연결 전에는 거부
        assert s.ctl.jog("joint", [10, 0, 0, 0, 0, 0]) is False
        time.sleep(0.2)
        assert len(d.writes) == n  # fault 뒤 아무 명령도 보내지 않는다
        s.logger.flush()
        assert any(e["code"] == "ARM-COMM-LOST" and e["level"] == "ERROR" for e in read_events(s.logger.dir))
        assert wait_for(lambda: any(i["code"] == "ARM-COMM-LOST" for i in list_incidents(tmp_path)))
        d.down = False  # 다시 꽂아도 사용자가 연결하기 전에는 움직이지 않는다
        time.sleep(0.2)
        assert len(d.writes) == n and s.ctl.state == "fault"
        ok, msg = s.connect()  # 다시 연결
        assert ok, msg
        assert s.ctl.state == "idle" and s.ctl._last == d.read_joints()  # 명령값이 실제값에 맞춰짐
        assert not [w for w in d.writes if w[0] == "torque" and w[1] is False]  # 토크를 끄는 쓰기가 없다
        assert s.move_joints(JointState(10, -90, 93, 0, 0, 3)).status == "ok"
    finally:
        s.disconnect()


def test_port_pulled_during_a_move_faults_within_0_6s(tmp_path):
    d = FlakyPort(CFG.poses["home"])
    s = ArmService(d, CFG, WS, speed_pct=30, log_root=str(tmp_path))
    s.incident_after_s = 0
    assert s.connect()[0]
    try:
        h = s.ctl.execute(move_j(d.read_joints(), JointState(40, -90, 93, 0, 0, 3), 20, 100))
        time.sleep(0.4)
        t0 = time.perf_counter()
        d.down = True
        assert wait_for(lambda: s.ctl.state == "fault", 1.0) and time.perf_counter() - t0 < 0.6
        assert h.wait(1).status == "fault"
    finally:
        s.disconnect()
