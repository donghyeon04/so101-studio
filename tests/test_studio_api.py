"""GUI 세션이 쓰는 Studio/ArmService 추가 API (session-requests.md REQ-NNN)."""
import threading
import time
import types

from test_task import WS, ListSource, StubSkills, obj

from so101_studio.service.studio import Studio


def make_studio(objs=None, source=None):
    arm = types.SimpleNamespace(ctl=StubSkills().ctl, kin=None, cfg=None, ws=WS, skills=StubSkills(), _log=lambda *a: None)
    return Studio(arm, source or ListSource(objs or [obj(1, 300, 170)]))


def test_req001_step_advances_one_state_at_a_time():
    st = make_studio()
    seen = []
    st.bus.subscribe("task", lambda e: seen.append(e.state))
    st.start(step_mode=True)
    time.sleep(0.2)
    assert seen == ["PREPARE"]  # 첫 상태에서 멈춰 기다린다
    st.step()
    time.sleep(0.2)
    assert seen == ["PREPARE", "SCAN"]
    for _ in range(30):
        st.step()
        time.sleep(0.02)
    st._thread.join(2)
    assert st.result["picked"] == ["obj-1"]


def test_req001_stop_releases_a_waiting_step():
    st = make_studio()
    st.start(step_mode=True)
    time.sleep(0.2)
    st.stop()
    st._thread.join(2)
    assert not st._thread.is_alive() and st.result["end"] == "STOPPED"


def test_req002_perception_select_calibrate_scan():
    st = make_studio([obj(1, 300, 170)])
    other = ListSource([obj(9, 300, 200)])
    other.name = "other"
    st.perception.register(other)
    assert st.perception.list() == ["list", "other"] and st.perception.active == "list"
    assert [o.id for o in st.perception.scan().objects] == ["obj-1"]
    st.perception.select("other")
    assert st.task.source is other and [o.id for o in st.perception.scan().objects] == ["obj-9"]
    assert st.perception.calibrate() == {}
    try:
        st.perception.select("nope")
    except ValueError:
        pass
    else:
        raise AssertionError


def test_req002_select_refused_while_running():
    st = make_studio()
    st.start(step_mode=True)
    time.sleep(0.2)
    try:
        st.perception.select("list")
    except RuntimeError:
        pass
    else:
        raise AssertionError
    st.stop()
    st._thread.join(2)


def test_req003_manual_source_add_remove_thread_safe():
    from so101_studio.perception.manual import ManualSource

    src = ManualSource("config/manual_objects.yaml")
    new = obj(9, 300, 200)
    src.add(new)
    assert "obj-9" in [o.id for o in src.scan().objects]
    try:
        src.add(new)
    except ValueError:
        pass
    else:
        raise AssertionError
    src.notify_picked("obj-9")
    src.remove("obj-9")
    src.add(new)  # 지운 뒤 같은 id를 다시 쓰면 집힌 표시도 사라져 있다
    assert "obj-9" in [o.id for o in src.scan().objects]
    try:
        src.remove("nope")
    except KeyError:
        pass
    else:
        raise AssertionError
    errs = []

    def worker(k):
        try:
            for i in range(50):
                o = obj(100 * k + i, 300, 200)
                src.add(o)
                src.scan()
                src.remove(o.id)
        except Exception as e:  # noqa: BLE001 스레드 안의 어떤 오류든 모은다
            errs.append(e)

    ts = [threading.Thread(target=worker, args=(k,)) for k in range(1, 5)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert not errs


# ---- ArmService 쪽 추가 API (시뮬레이션 드라이버)
import dataclasses

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), dataclasses.replace(CFG), WS, speed_pct=50, log_root=str(tmp_path))
    s.preset_dir = str(tmp_path / "presets")
    ok, msg = s.connect()
    assert ok, msg
    yield s
    s.disconnect()


def test_req004_get_set_params_with_timing(svc):
    p = svc.get_params()
    assert p["speed_pct"] == {"value": 50, "default": 30, "timing": "now"}
    assert p["servo_acceleration"]["timing"] == "reconnect" and p["approach_height_mm"]["timing"] == "next_motion"
    assert svc.set_params({"speed_pct": "70", "approach_height_mm": 55}) == {"speed_pct": "now", "approach_height_mm": "next_motion"}
    assert svc.speed == 70 and svc.skills.speed == 70 and svc.cfg.approach_height_mm == 55.0


def test_req004_invalid_changes_apply_nothing(svc):
    for bad in ({"approach_height_mm": 10, "nope": 1}, {"approach_height_mm": 10, "trajectory_profile": "x"},
                {"approach_height_mm": "abc"}, {"speed_pct": 0}):
        with pytest.raises(ValueError):
            svc.set_params(bad)
    assert svc.cfg.approach_height_mm == CFG.approach_height_mm and svc.speed == 50


def test_req004_presets_roundtrip(svc):
    svc.set_params({"approach_height_mm": 61, "grasp_open_policy": "fit"})
    svc.save_preset("fast")
    svc.set_params({"approach_height_mm": 20, "grasp_open_policy": "max"})
    assert svc.list_presets() == ["fast"]
    svc.load_preset("fast")
    assert svc.cfg.approach_height_mm == 61 and svc.cfg.grasp_open_policy == "fit"
    for bad in ("../x", "a/b"):
        with pytest.raises(ValueError):
            svc.save_preset(bad)
    with pytest.raises(FileNotFoundError):
        svc.load_preset("none")


def test_req005_gui_command_dispatches_or_explains(monkeypatch):
    import sys
    import types as t

    from so101_studio.cli import main

    monkeypatch.setitem(sys.modules, "so101_studio.gui", None)  # None이면 import가 ModuleNotFoundError: GUI 모듈이 없는 경우를 흉내
    with pytest.raises(SystemExit) as e:  # GUI 모듈이 없으면 안내
        main(["--sim", "gui"])
    assert "GUI" in str(e.value)
    got = []
    fake = t.ModuleType("so101_studio.gui")
    fake.main = lambda args: got.append(args)
    monkeypatch.setitem(sys.modules, "so101_studio.gui", fake)
    main(["--sim", "--speed", "20", "gui"])
    assert got[0].sim and got[0].speed == 20 and got[0].command == "gui"


def test_req006_diagnostics_progress_result_and_stop(svc, monkeypatch):
    from so101_studio.cli import powertest, shaketest

    monkeypatch.setattr(powertest, "SETTLE_S", 0.2)
    monkeypatch.setattr(shaketest, "SPEEDS", (200.0,))
    info = svc.run_diagnostic("servoinfo")
    assert info["name"] == "servoinfo" and not info["aborted"] and info["result"]
    lines = []
    r = svc.run_diagnostic("powertest", progress=lines.append, repeats=1)
    assert "peak_total_ma" in r["result"] and lines and not r["aborted"]
    # 중단: 첫 이동 전에 멈추면 이동이 없고 aborted가 켜진다. 팔 설정(가속 한계)은 복원된다
    acc = svc.driver.acceleration
    lines.clear()
    r = svc.run_diagnostic("shaketest", progress=lines.append, stop=lambda: True, reps=1)
    assert r["aborted"] and r["result"] == [] and svc.driver.acceleration == acc
    with pytest.raises(ValueError):
        svc.run_diagnostic("linkwatch")  # 시뮬레이션
    with pytest.raises(ValueError):
        svc.run_diagnostic("nope")


def test_req007_task_events_carry_decision_details():
    st = make_studio([obj(1, 300, 250), obj(2, 300, 170), obj(3, 300, 600)])
    seen = []
    st.bus.subscribe("task", seen.append)
    st.start(blocking=True)
    d = next(e for e in seen if e.state == "DECIDE")
    assert d.detail["detail_n"] == 3 and d.detail["chosen"] == "obj-2" and "가장 가까움" in d.detail["reason"]
    rows = {r["id"]: r for r in d.detail["objects"]}
    assert rows["obj-3"]["reachable"] is False and rows["obj-2"]["radial_mm"] < rows["obj-1"]["radial_mm"]
    done = [e for e in seen if e.state == "PLACE" and e.result == "ok"]
    assert done and "duration_s" in done[0].detail
    last = [e for e in seen if e.state == "DECIDE"][-1]
    assert last.detail["chosen"] is None and last.detail["reason"]


def test_req008_preview_path_does_not_move_and_matches_goal(svc):
    before = svc.driver.read_joints()
    pose = svc.point_pose(300, 170, 40)
    r = svc.preview_path(pose)
    assert svc.driver.read_joints() == before  # 움직이지 않는다
    assert r["ok"] and r["reason"] is None and r["duration_s"] > 0 and len(r["tool"]) == len(r["joints"]) > 2
    end = r["tool"][-1]
    assert abs(end["x_mm"] - pose.x_mm) < 2 and abs(end["y_mm"] - pose.y_mm) < 2 and abs(end["z_mm"] - pose.z_mm) < 2
    assert r["tool"][0]["t"] == 0 and abs(r["tool"][-1]["t"] - r["duration_s"]) < 1e-9
    far = svc.preview_path(svc.point_pose(900, 900, 40))
    assert not far["ok"] and far["reason"]


def test_req009_resume_after_estop(svc):
    goal = JointState(10, -80, 90, 0, 0, 3)
    svc.estop()
    assert svc.move_joints(goal).status == "stopped"
    before = svc.driver.read_joints()
    svc.resume()
    assert svc.driver.read_joints() == before  # 해제만으로는 움직이지 않는다
    assert svc.move_joints(goal).status == "ok"


def test_req012_jog_turns_torque_on_once_without_bus_reads(svc):
    svc.driver.set_torque(False)
    reads = []
    real = svc.driver.torque_enabled
    svc.driver.torque_enabled = lambda: reads.append(1) or real()
    assert svc.jog("joint", [5, 0, 0, 0, 0, 0]) and svc.driver.torque_enabled()
    n = len(reads)
    svc.ctl.jog_stop()
    for _ in range(20):  # 10Hz로 계속 불러도 버스를 다시 읽지 않는다
        svc.jog("joint", [5, 0, 0, 0, 0, 0])
    assert len(reads) == n
    svc._torque_on = False  # 교시 등으로 토크가 꺼진 뒤에는 다시 확인한다
    svc.driver.set_torque(False)
    svc.jog("joint", [5, 0, 0, 0, 0, 0])
    assert svc.driver.torque_enabled()
def test_req013_start_after_stop_waits_at_first_step():
    st = make_studio()
    st.start(step_mode=True)
    time.sleep(0.2)
    st.stop()  # step 신호를 남긴다
    st._thread.join(2)
    seen = []
    st.bus.subscribe("task", lambda e: seen.append(e.state))
    st.start(step_mode=True)
    time.sleep(0.3)
    assert seen == ["PREPARE"]  # 남은 신호로 지나가지 않고 기다린다
    st.stop()
    st._thread.join(2)


def test_req013_pause_stops_at_next_state_and_resume_continues():
    st = make_studio()
    seen = []

    def on_event(e):
        seen.append(e.state)
        if e.state == "SCAN" and seen.count("SCAN") == 1:  # 첫 스캔에서만
            st.pause()  # 작업 중 다른 스레드에서 불러도 되는 함수

    st.bus.subscribe("task", on_event)
    st.start()
    time.sleep(0.4)
    assert seen[-1] == "SCAN"  # pause한 상태 변화(이벤트 처리 직후)에서 멈추고 다음으로 가지 않는다
    st.resume()
    st._thread.join(3)
    assert not st._thread.is_alive() and st.result["picked"] == ["obj-1"]


def test_req013_stop_in_step_mode_ends_the_thread():
    st = make_studio()
    st.start(step_mode=True)
    time.sleep(0.2)
    st.stop()  # PREPARE에서 기다리는 중
    st._thread.join(2)
    assert not st._thread.is_alive() and st.result["end"] == "STOPPED"


def test_req013_thread_exception_becomes_fault_event():
    st = make_studio()
    st.task.skills.park = lambda: (_ for _ in ()).throw(RuntimeError("이미 동작 중"))
    seen = []
    st.bus.subscribe("task", seen.append)
    st.start()
    st._thread.join(2)
    assert st.result["end"] == "FAULT" and "이미 동작 중" in st.result["error"]
    assert seen[-1].state == "FAULT" and seen[-1].result == "exception"

