"""REQ-061 (arm-software-spec 13.7 (c)(e), M2): MuJoCoDriver 동작과 CLI 엔진 선택. mujoco가 없으면 건너뛴다."""
import sys

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

CFG = load_robot("config/robot.yaml")
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
    assert load_model(base=WS.base, table_z_mm=CFG.table_z_mm)[1] == ["SIM-JOINT-RANGE-DEFAULT"]
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


def test_cli_gui_rejected():
    with pytest.raises(SystemExit) as e:
        main(["--sim", "--sim-engine", "mujoco", "gui"])
    assert "M4부터" in str(e.value)


def test_cli_import_error_exit2(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "mujoco", None)  # import mujoco -> ImportError
    with pytest.raises(SystemExit) as e:
        main(["--sim", "--sim-engine", "mujoco", "status"])
    assert e.value.code == 2 and "tooling-setup.md" in capsys.readouterr().out


def test_cli_status_mujoco(capsys):
    main(["--sim", "--sim-engine", "mujoco", "status"])
    assert "shoulder_pan" in capsys.readouterr().out


@pytest.mark.slow
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
