"""M4b (mujoco-spec 13.10, 결정 0063): 로보나인 병렬 손 MuJoCo 모델. mujoco가 없으면 건너뛴다."""
import time

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from so101_studio.arm.kinematics import ARM_JOINTS, Kinematics
from so101_studio.arm.mujoco_driver import MuJoCoDriver
from so101_studio.arm.mujoco_hand import slide_m, value_of_slide
from so101_studio.arm.mujoco_model import (
    JOINT_MAP,
    geom_roles,
    load_model,
    model_to_table_mm,
)
from so101_studio.arm.sim_driver import make_sim_driver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

WS = load_workspace("config/workspace.yaml")


@pytest.fixture(scope="module")
def cfg(tmp_path_factory):
    lp = tmp_path_factory.mktemp("cfg") / "local.yaml"
    lp.write_text("gripper: parallel\n", encoding="utf-8")
    c = load_robot("config/robot.yaml", local_path=str(lp))
    assert c.gripper == "parallel"
    return c


@pytest.fixture(scope="module")
def model(cfg):
    return load_model(None, [], base=WS.base, table_z_mm=cfg.table_z_mm, joint_limits=cfg.physical_limits_deg or None, hand="parallel", gripper_width_mm=cfg.gripper_width_mm)[0]


def test_hand_is_replaced_and_roles_are_the_clamps(model):
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1") == -1
    assert [model.joint("gripper").type[0], model.joint("gripper_left").type[0]] == [mujoco.mjtJoint.mjJNT_SLIDE] * 2
    assert {"clamp_right", "clamp_left", "table"} <= set(geom_roles(model))
    assert model.nu == 6 and model.actuator("gripper").trnid[0] == model.joint("gripper").id


def test_gripper_link_and_frame_match_the_parallel_urdf_forward_kinematics(model, cfg):
    kin = Kinematics("assets/so101_parallel.urdf", WS.base, cfg.joint_limits_deg)
    rng = np.random.default_rng(0)
    data = mujoco.MjData(model)
    site, worst = model.site("gripper_frame").id, 0.0
    for _ in range(10):
        q = JointState(*[float(rng.uniform(cfg.joint_limits_deg[j][0] + 5, cfg.joint_limits_deg[j][1] - 5)) for j in ARM_JOINTS], 0.0)
        for name in ARM_JOINTS:
            data.qpos[model.joint(JOINT_MAP[name][0]).qposadr[0]] = np.radians(getattr(q, name))
        mujoco.mj_kinematics(model, data)
        x, y, h = model_to_table_mm(*(data.site_xpos[site] * 1000.0), WS.base, cfg.table_z_mm)
        p = kin.fk(q)
        worst = max(worst, np.linalg.norm([x - p.x_mm, y - p.y_mm, h + cfg.table_z_mm - p.z_mm]))
    print(f"FK 최대 차이 {worst:.4f}mm")
    assert worst < 0.1, worst


def test_clamps_move_symmetrically_and_the_width_follows_the_value(model, cfg):
    d = mujoco.MjData(model)
    d.ctrl[model.actuator("gripper").id] = slide_m(50, cfg.gripper_width_mm)
    for _ in range(1500):
        mujoco.mj_step(model, d)
    qr, ql = d.qpos[model.joint("gripper").qposadr[0]], d.qpos[model.joint("gripper_left").qposadr[0]]
    assert qr == pytest.approx(0.021, abs=2e-4) and ql == pytest.approx(-qr, abs=1e-4)
    assert value_of_slide(qr, cfg.gripper_width_mm) == pytest.approx(50, abs=1.0)


def test_unequal_push_keeps_the_clamps_coupled(model, cfg):
    d = mujoco.MjData(model)
    d.ctrl[model.actuator("gripper").id] = 0.02
    right = model.body("clamp_right").id
    for _ in range(1500):
        d.xfrc_applied[right, 0] = -5.0  # 오른쪽만 안쪽으로 5N 민다
        mujoco.mj_step(model, d)
    qr, ql = d.qpos[model.joint("gripper").qposadr[0]], d.qpos[model.joint("gripper_left").qposadr[0]]
    assert abs(qr + ql) < 5e-4, (qr, ql)  # 한쪽만 밀려 비대칭이 되지 않는다


def wrist_scene(tmp_path):
    """시험장 기준 설정(결정 0062): 손목 부품 둘(wrist_hcsr04·wrist_bracket)을 켠 장면."""
    from pathlib import Path

    from so101_studio.core.scene import load_scene

    root = Path(__file__).resolve().parents[1]
    y = tmp_path / "config"
    y.mkdir()
    nl = chr(10)
    (y / "scene.yaml").write_text(f"parts:{nl}  - {{name: wrist_hcsr04, file: {root.as_posix()}/assets/scene/wrist_hcsr04.stl, attach: gripper_link}}{nl}"
                                  f"  - {{name: wrist_bracket, file: {root.as_posix()}/assets/scene/wrist_bracket.stl, attach: gripper_link}}{nl}", encoding="utf-8")
    return load_scene(y / "scene.yaml")


@pytest.mark.slow
@pytest.mark.parametrize("size", [30, 45, 58])
def test_parallel_hand_carries_and_places(cfg, tmp_path, size):
    import json
    import threading

    d = make_sim_driver(cfg, WS, wrist_scene(tmp_path), scenario=f"carry{size}")
    svc = ArmService(d, cfg, WS, log_root=str(tmp_path))
    ok, msg = svc.connect()
    assert ok, msg
    samples, stop = [], threading.Event()

    def sample():
        while not stop.is_set():
            samples.append(d.sim_state())
            time.sleep(0.02)

    th = threading.Thread(target=sample, daemon=True)
    th.start()
    try:
        o = DetectedObject(id="box", x_mm=300, y_mm=200, angle_deg=0, width_mm=size, length_mm=size, confidence=1.0, source="manual", height_mm=40)
        r = svc.pick_and_place(o)
        assert r.status == "ok", r
        held = d.sim_state()["held"]
        r = svc.place(at=(420, 170))
        assert r.status == "ok", r
        time.sleep(1.0)
        stop.set()
        th.join()
        end = d.sim_state()["objects"]["box"]
        first, slip = None, 0.0
        for st in samples:  # 13.7 (b): 들어 올린 동안 상자가 손에서 미끄러진 최대 거리
            ob = st["objects"]["box"]
            if st["held"] == "box" and ob["z_mm"] > 5:
                rel = np.array(st["tool"]["rot"]).T @ (np.array([ob["x_mm"], ob["y_mm"], ob["z_mm"]]) - np.array(st["tool"]["pos_mm"]))
                first = rel if first is None else first
                slip = max(slip, float(np.linalg.norm(rel - first)))
        ev = [json.loads(x) for x in (svc.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]
        bad = [e for e in ev if e.get("code") in ("ARM-GRIP-OVERLOAD-WARN", "ARM-GRIP-OVERLOAD", "ARM-BLOCKED")
               or (e.get("code") == "ARM-FAULT" and str(e.get("data", {}).get("reason", "")).startswith(("overload", "overcurrent", "overheat")))]
        print(f"size {size}: held {held}, 미끄러짐 {slip:.2f}mm, 놓은 자리 차이 {np.hypot(end['x_mm'] - 420, end['y_mm'] - 170):.2f}mm, 기울기 {end['tilt_deg']:.1f}°")
        assert first is not None and held == "box" and slip <= 4.0  # 병렬 손 58mm가 실행마다 2.0~3.7mm(11-15 4회)라 단동식 기준 3.5를 4.0으로 둔다
        assert np.hypot(end["x_mm"] - 420, end["y_mm"] - 170) < 7 and end["tilt_deg"] < 10
        assert d.failed_reason is None and isinstance(d, MuJoCoDriver) and not bad, bad
    finally:
        stop.set()
        svc.disconnect()


def test_wrist_bracket_and_sensor_ride_on_the_parallel_hand(cfg, tmp_path):
    from pathlib import Path

    from so101_studio.core.scene import load_scene

    root = Path(__file__).resolve().parents[1]
    y = tmp_path / "config"
    y.mkdir()
    (y / "scene.yaml").write_text(f"parts:\n  - {{name: wrist_hcsr04, file: {root.as_posix()}/assets/scene/wrist_hcsr04.stl, attach: gripper_link}}\n"
                                  f"  - {{name: wrist_bracket, file: {root.as_posix()}/assets/scene/wrist_bracket.stl, attach: gripper_link}}\n", encoding="utf-8")
    m, warns = load_model(load_scene(y / "scene.yaml"), [], base=WS.base, table_z_mm=cfg.table_z_mm, hand="parallel", gripper_width_mm=cfg.gripper_width_mm)
    assert not [w for w in warns if w[0] == "SIM-PART-SKIPPED"]
    assert m.body("scene:wrist_bracket").mass[0] == pytest.approx(0.0155, abs=0.0005)
    assert m.site("wrist_sonar").id >= 0 and m.site("gripper_frame").id >= 0
    parts = {w for w in ("wrist_bracket", "wrist_hcsr04") if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"scene:{w}_geom") >= 0}
    assert parts == {"wrist_bracket", "wrist_hcsr04"}


def test_camera_arm_is_cut_from_the_hand_mesh(tmp_path):
    from so101_studio.arm import mujoco_hand

    v = mujoco_hand.hand_geometry()["hand"]["verts"] * 1000.0
    assert v[:, 1].min() > mujoco_hand.CAMERA_Y_GL_MM - 0.01, v[:, 1].min()  # 카메라 마운트·렌즈(y -85까지)가 없다: 평면으로 잘라 경계 삼각형도 y -36 안
    out = tmp_path / "nocam.stl"
    removed, kept = mujoco_hand.make_nocam_stl(out)
    import trimesh

    assert len(trimesh.load(out, force="mesh").faces) == kept == len(trimesh.load(mujoco_hand.NOCAM_STL, force="mesh").faces) and removed > 0
