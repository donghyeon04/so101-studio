"""REQ-061 (arm-software-spec 13.7 (c)(e), M2): MuJoCoDriver 동작과 CLI 엔진 선택. mujoco가 없으면 건너뛴다."""
import sys
import time

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from so101_studio.arm.driver import JOINTS
from so101_studio.arm.mujoco_driver import MuJoCoDriver
from so101_studio.arm.mujoco_model import JOINT_MAP, load_model
from so101_studio.cli import main
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.scene import load_scene
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml", gripper="single_jaw")  # 단동식 기하로 잰 기대값 (REQ-098: 기본은 parallel)
WS = load_workspace("config/workspace.yaml")
HOME = JointState(*CFG.poses["home"])


def make(realtime=False) -> MuJoCoDriver:
    d = MuJoCoDriver(CFG, WS, scene=load_scene(), realtime=realtime)
    d.connect()
    return d


@pytest.fixture
def drv():
    d = make()
    yield d
    d.disconnect()


def test_attributes_and_shapes(drv):
    assert drv.simulated and drv.physical_objects and drv.locked_reason is None and drv.event_sink is None
    assert drv.acceleration == 10 and drv.connect_info == {"engine": "mujoco", "scenario": None} and drv.gripper_torque_limit() == 1000
    h = drv.read_health()
    assert set(h) == {"temperature_c", "load_pct", "voltage_v", "current_ma", "current_ma_by_joint", "torque", "status_bits"}
    assert h["temperature_c"]["gripper"] == 25.0 and h["voltage_v"] == 12.0 and set(h["torque"]) == set(JOINTS)
    assert len(drv.probe_reset()) == 8
    v, ma, load = drv.read_power()
    assert v == 12.0 and set(ma) == set(load) == set(JOINTS)
    assert drv.read_servo_info() == {m: {"Acceleration": 10} for m in JOINTS}
    assert drv.sim_state()["objects"] == {} and drv.sim_state()["held"] is None


def test_connect_home(drv):
    q = drv.read_joints()
    assert all(abs(getattr(q, j) - getattr(HOME, j)) < 1.0 for j in JOINTS), q
    assert drv.torque_enabled()
    drv.advance(1.0)
    assert abs(drv.read_joints().shoulder_lift - HOME.shoulder_lift) < 1.0  # 중력에도 처지지 않는다


def test_torque_off_sags_and_holds(drv):
    drv.write_joints(JointState(0, -30, 60, 0, 0, 3))  # home은 접힌 균형 자세라 처지지 않는다: 팔을 앞으로 뻗은 자세에서
    drv.advance(2.0)
    drv.set_torque(False)
    drv.advance(1.0)
    assert drv.read_joints().shoulder_lift - (-30) > 5.0
    drv.set_torque(True)
    q1 = drv.read_joints()
    drv.advance(1.0)
    assert abs(drv.read_joints().shoulder_lift - q1.shoulder_lift) < 2.0


def test_write_while_torque_off(drv):
    drv.set_torque(False)
    drv.write_joints(JointState(60, 0, 0, 0, 0, 3))
    drv.advance(0.2)
    assert abs(drv.read_joints().shoulder_pan) < 5.0  # 힘이 없어 목표 쪽으로 가지 않는다
    drv.set_torque(True)
    q1 = drv.read_joints()
    drv.advance(1.0)
    q2 = drv.read_joints()
    assert abs(q2.shoulder_pan - q1.shoulder_pan) < 2.0 and abs(q2.shoulder_pan - 60) > 30


@pytest.mark.parametrize("v", [0, 50, 100])
def test_gripper_0_50_100(drv, v):
    drv.write_joints(JointState(*[*CFG.poses["home"][:5], v]))
    drv.advance(1.0)
    assert abs(drv.read_joints().gripper - v) < 2.0


def test_not_realtime_needs_advance(drv):
    t = drv.sim_state()["t_s"]
    drv.write_joints(JointState(30, -90, 93, 0, 0, 3))
    import time

    time.sleep(0.1)
    assert drv.sim_state()["t_s"] == t
    drv.advance(0.1)
    assert drv.sim_state()["t_s"] == pytest.approx(t + 0.1)


def test_physics_fail_event(drv):
    events = []
    drv.event_sink = lambda *e: events.append(e)
    drv.data.qpos[:] = np.nan
    drv.advance(0.01)
    assert [e[2] for e in events] == ["SIM-PHYSICS-FAIL"] and events[0][1] == "ERROR"
    for call in (drv.read_joints, drv.read_health, lambda: drv.write_joints(HOME)):
        with pytest.raises(RuntimeError, match="물리 시뮬레이션이 멈췄습니다"):
            call()
    drv.connect()  # 되살리기
    assert drv.read_joints().shoulder_pan == pytest.approx(HOME.shoulder_pan, abs=1.0)


def test_base_box_no_contact(drv):
    for pose in ("home", "radar_clear"):
        drv.write_joints(JointState(*CFG.poses[pose]))
        drv.advance(1.0)
        base_box = drv.model.geom("base_box").id
        for c in drv.data.contact[: drv.data.ncon]:
            assert base_box not in (c.geom1, c.geom2), pose


def test_ranges_widened_and_default_warning():
    m, w = load_model(base=WS.base, table_z_mm=CFG.table_z_mm, joint_limits=CFG.physical_limits_deg)
    assert w == [] and np.degrees(m.joint("shoulder_pan").range[1]) == pytest.approx(CFG.physical_limits_deg["shoulder_pan"][1])
    assert np.degrees(m.actuator("shoulder_pan").ctrlrange[1]) == pytest.approx(CFG.physical_limits_deg["shoulder_pan"][1])
    assert np.degrees(m.actuator("gripper").ctrlrange[1]) == pytest.approx(-10 + 1.214 * 100 + 1)
    assert load_model(base=WS.base, table_z_mm=CFG.table_z_mm)[1] == [("SIM-JOINT-RANGE-DEFAULT", {})]
    assert JOINT_MAP["gripper"][0] == "gripper"


def test_pending_events_flush_when_sink_is_set():
    d = MuJoCoDriver(CFG, WS, realtime=False)
    d.cfg = CFG.__class__(**{**CFG.__dict__, "physical_limits_deg": {}})
    d.connect()
    got = []
    d.event_sink = lambda *e: got.append(e[2])
    d._emit("sim", "WARN", "X", {})
    assert got == ["SIM-JOINT-RANGE-DEFAULT", "X"]


def test_cli_engine_requires_sim():
    with pytest.raises(SystemExit) as e:
        main(["--sim-engine", "mujoco", "status"])
    assert "--sim과 함께" in str(e.value)


def test_cli_dry_run_rejected():
    with pytest.raises(SystemExit) as e:
        main(["--sim", "--sim-engine", "mujoco", "run", "--dry-run"])
    assert "--dry-run" in str(e.value)


def test_cli_gui_passes_engine(monkeypatch):
    import importlib
    import types

    got = []
    real = importlib.import_module
    monkeypatch.setattr(importlib, "import_module", lambda name, *a, **k: types.SimpleNamespace(main=got.append) if name == "so101_studio.gui" else real(name, *a, **k))
    main(["--sim", "--sim-engine", "mujoco", "--sim-viewer", "gui"])
    assert got[0].sim_engine == "mujoco" and got[0].sim_viewer is True and got[0].sim is True


def test_cli_viewer_requires_engine():
    for argv in (["--sim-viewer", "status"], ["--sim", "--sim-viewer", "status"]):
        with pytest.raises(SystemExit) as e:
            main(argv)
        assert e.value.code == "--sim-viewer는 --sim --sim-engine mujoco와 함께 쓴다"


def test_viewer_flag_passed_and_requires_realtime(monkeypatch):
    import mujoco.viewer

    from so101_studio.arm.sim_driver import make_sim_driver

    calls = []

    class Handle:
        def is_running(self):
            return True

        def sync(self):
            calls.append("sync")

        def close(self):
            calls.append("close")

    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda *a, **k: calls.append(("launch", k)) or Handle())
    d = make_sim_driver(CFG, WS, None, viewer=True)
    assert d.viewer is True
    d.connect()
    time.sleep(0.2)
    d.disconnect()
    assert calls[0] == ("launch", {"show_left_ui": False, "show_right_ui": False}) and "sync" in calls and calls[-1] == "close"
    with pytest.raises(ValueError, match="실시간"):
        MuJoCoDriver(CFG, WS, realtime=False, viewer=True)


def test_failed_reason(drv):
    assert drv.failed_reason is None
    drv.data.qpos[:] = np.nan
    drv.advance(0.01)
    assert drv.failed_reason == "관절값이 NaN"
    drv.connect()
    assert drv.failed_reason is None


def test_sim_state_size_default_objects_stay_put():
    from so101_studio.arm.sim_driver import make_sim_driver

    d = make_sim_driver(CFG, WS, None)
    d.connect()
    try:
        time.sleep(1.0)
        st = d.sim_state()["objects"]
        for o, name in zip(d.objects, ("obj-1", "obj-2", "obj-3")):
            s = st[name]
            assert s["size_mm"] == [o.length_mm, o.width_mm, o.height_mm] and s["tilt_deg"] < 5
            assert np.hypot(s["x_mm"] - o.x_mm, s["y_mm"] - o.y_mm) < 3.0, (name, s)
    finally:
        d.disconnect()


def test_objects_overlap_rejected(tmp_path):
    from so101_studio.arm.sim_driver import make_sim_driver

    f = tmp_path / "over.yaml"
    f.write_text("objects: [{name: a, x_mm: 300, y_mm: 190, length_mm: 100, width_mm: 30, height_mm: 40}, {name: b, x_mm: 330, y_mm: 190, angle_deg: 90, length_mm: 100, width_mm: 30, height_mm: 40}]", encoding="utf-8")
    with pytest.raises(ValueError, match="a과 b이 겹친다"):
        make_sim_driver(CFG, WS, None, scenario=str(f))
    f.write_text("objects: [{name: a, x_mm: 300, y_mm: 190, length_mm: 100, width_mm: 30}, {name: b, x_mm: 300, y_mm: 240, length_mm: 100, width_mm: 30}]", encoding="utf-8")
    make_sim_driver(CFG, WS, None, scenario=str(f))  # 20mm 떨어져 있으면 통과


def test_cli_import_error_exit2(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "mujoco", None)  # import mujoco -> ImportError
    with pytest.raises(SystemExit) as e:
        main(["--sim", "--sim-engine", "mujoco", "status"])
    assert e.value.code == 2 and "tooling-setup.md" in capsys.readouterr().out


def test_cli_status_mujoco(capsys):
    main(["--sim", "--sim-engine", "mujoco", "status"])
    assert "shoulder_pan" in capsys.readouterr().out


@pytest.mark.slow
@pytest.mark.measure  # 벽시계 속도 측정이라 다른 프로세스가 CPU를 쓰면 놓침이 난다(결정 0064)
def test_realtime_60s():
    """13.7 (e) 실시간 속도: 쉬기 30초 + home<->radar_clear 속도 30% 왕복 30초. 결과와 CPU는 measurements.md에 손으로 옮긴다."""
    import platform
    import time

    from so101_studio.service.arm_service import ArmService

    d = MuJoCoDriver(CFG, WS, scene=load_scene(), realtime=True)
    svc = ArmService(d, CFG, WS, 30)
    ok, msg = svc.connect()  # 드라이버 connect는 ArmService가 부른다
    assert ok, msg
    drops0 = d.dropped_count  # connect 직후 ArmService 준비(기구학 로드 등)가 GIL을 길게 쥐어 생기는 버림은 측정 밖
    try:
        time.sleep(30)
        t0 = time.time()
        while time.time() - t0 < 30:
            svc.skills.home()
            svc.skills.radar_clear()
        print(f"CPU {platform.processor()} dropped_count {d.dropped_count - drops0} (connect 직후 {drops0}) dropped_s {d.dropped_s:.3f} step_ms_avg {d.step_ms_avg:.3f}")
        assert d.dropped_count == drops0 and d.step_ms_avg < 1.0
    finally:
        svc.disconnect()


def test_cameras_and_render(drv):
    """REQ-071 (mujoco-spec 13.10.4): 카메라 top·wrist, render 모양, 다른 스레드에서 불러도 되고, 모르는 이름·연결 전은 오류. top은 판 중심 위 550mm."""
    import threading

    from so101_studio.arm.mujoco_model import TOP_CAM_H_MM, table_to_model_mm

    m = drv.model
    top = np.array(m.cam("top").pos) * 1000
    exp = table_to_model_mm(WS.table_mm["x"] / 2, WS.table_mm["y"] / 2, TOP_CAM_H_MM, WS.base, CFG.table_z_mm)
    assert np.allclose(top, exp, atol=0.01) and m.cam("wrist").bodyid[0] == m.body("gripper").id
    a = drv.render("top")
    assert a.shape == (240, 320, 3) and a.dtype == np.uint8 and a.std() > 0
    out = {}
    t = threading.Thread(target=lambda: out.setdefault("img", drv.render("wrist", 160, 120)))
    t.start()
    t.join()
    assert out["img"].shape == (120, 160, 3)
    with pytest.raises(ValueError):
        drv.render("nope")
    drv.disconnect()
    with pytest.raises(RuntimeError):
        drv.render("top")


def test_render_from_two_threads_at_once():
    """REQ-071: GUI(스레드 풀, 매번 다른 스레드)와 PolicyRunner가 동시에 render를 불러도 된다(그리기는 드라이버의 렌더 전용 스레드 하나가 한다)."""
    from concurrent.futures import ThreadPoolExecutor

    d = make(realtime=True)
    try:
        with ThreadPoolExecutor(4) as ex:  # 풀 스레드 넷 = 부르는 스레드가 매번 다르다
            futs = [ex.submit(d.render, cam, *size) for _ in range(20) for cam, size in (("top", (320, 240)), ("wrist", (160, 120)))]
            shapes = {f.result().shape for f in futs}
        assert shapes == {(240, 320, 3), (120, 160, 3)}
    finally:
        d.disconnect()


def test_render_thread_ends_on_disconnect():
    """code-review: 에피소드마다 드라이버를 새로 연결해도 렌더 스레드가 쌓이지 않는다(disconnect가 끝내고, 다음 render가 다시 만든다)."""
    import threading

    d = make()
    try:
        for _ in range(3):
            d.connect()
            assert d.render("top").shape == (240, 320, 3)
            d.disconnect()
            assert d._rpool is None and not [t for t in threading.enumerate() if t.name.startswith("mujoco-render")]
    finally:
        d.disconnect()


def test_set_objects_rebuilds_the_world_on_next_connect():
    """REQ-092(13.10.4 세계 다시 짓기): set_objects 뒤 connect하면 새 물체만 있다. 연결 중에 불러도 지금 세계는 그대로다. 바닥이 겹치면 ValueError이고 목록은 그대로다."""
    from so101_studio.arm.mujoco_driver import MuJoCoDriver
    from so101_studio.arm.mujoco_model import SimObject
    from so101_studio.core.config import load_robot, load_workspace

    cfg, ws = load_robot("config/robot.yaml"), load_workspace("config/workspace.yaml")
    d = MuJoCoDriver(cfg, ws, objects=[SimObject("a", 200, 200, 0, 30, 30, 30)], realtime=False)
    d.connect()
    try:
        d.set_objects([SimObject("b", 250, 180, 0, 40, 40, 30), SimObject("c", 150, 180, 0, 40, 40, 30)])
        assert set(d.sim_state()["objects"]) == {"a"}  # 연결 중에는 그대로
        with pytest.raises(ValueError):
            d.set_objects([SimObject("x", 200, 200, 0, 40, 40, 30), SimObject("y", 210, 200, 0, 40, 40, 30)])
        d.connect()
        assert set(d.sim_state()["objects"]) == {"b", "c"}
    finally:
        d.disconnect()


@pytest.fixture(autouse=True)
def _keep_real_runs(monkeypatch):
    """시험이 만드는 실행 기록이 진짜 runs/의 보관 정리를 부르지 않게 한다(2026-11-19 test_cli_status_mujoco가 runs/train 체크포인트를 지웠다)."""
    monkeypatch.setattr("so101_studio.core.logging.prune_runs", lambda *a, **k: [])
