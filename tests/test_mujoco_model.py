"""REQ-059 (arm-software-spec 13.7 (d), M1): MuJoCo 모델 대조. 드라이버 없이 `mujoco` 모듈과 `arm/mujoco_model.py`만 쓴다. mujoco가 없으면 건너뛴다."""
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from so101_studio.arm.kinematics import ARM_JOINTS, Kinematics
from so101_studio.arm.mujoco_model import (
    JOINT_MAP,
    SimObject,
    gripper_value_to_deg,
    load_model,
    model_to_table_mm,
    table_to_model_mm,
)
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.scene import load_scene
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml", gripper="single_jaw")  # 단동식 기하로 잰 기대값 (REQ-098: 기본은 parallel)
WS = load_workspace("config/workspace.yaml")


@pytest.fixture(scope="module")
def model():
    return load_model(load_scene("config/scene.yaml"), [SimObject("box", 300, 200, 0, 30, 100, 39)], base=WS.base, table_z_mm=CFG.table_z_mm)[0]


def _set(model, data, q: JointState) -> None:
    for name, (mj_name, sign, offset) in JOINT_MAP.items():
        deg = gripper_value_to_deg(q.gripper) if name == "gripper" else getattr(q, name)
        data.qpos[model.joint(mj_name).qposadr[0]] = np.radians(sign * deg + offset)


def test_gripper_frame_site_matches_forward_kinematics_for_ten_random_poses(model):
    kin = Kinematics(CFG.urdf, WS.base, CFG.joint_limits_deg)
    rng = np.random.default_rng(0)
    data = mujoco.MjData(model)
    site = model.site("gripper_frame").id
    worst = 0.0
    for _ in range(10):
        q = JointState(*[float(rng.uniform(CFG.joint_limits_deg[j][0] + 5, CFG.joint_limits_deg[j][1] - 5)) for j in ARM_JOINTS], 0.0)
        _set(model, data, q)
        mujoco.mj_kinematics(model, data)
        x, y, h = model_to_table_mm(*(data.site_xpos[site] * 1000.0), WS.base, CFG.table_z_mm)
        p = kin.fk(q)
        worst = max(worst, np.linalg.norm([x - p.x_mm, y - p.y_mm, h + CFG.table_z_mm - p.z_mm]))
    assert worst < 2.0, worst


def test_the_base_has_a_collision_geom(model):
    base = model.body("base").id
    assert any(model.geom_bodyid[g] == base and model.geom_contype[g] > 0 for g in range(model.ngeom))


def test_table_and_model_coordinates_are_inverse_of_each_other():
    for x, y, h in [(300.0, 200.0, 0.0), (480.0, 150.0, 40.0), (100.0, 300.0, 12.5)]:
        assert model_to_table_mm(*table_to_model_mm(x, y, h, WS.base, CFG.table_z_mm), WS.base, CFG.table_z_mm) == pytest.approx((x, y, h))


def test_jaw_collision_geoms_get_the_contact_values_and_the_global_options(model):
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC and model.opt.impratio == 10 and model.opt.timestep == pytest.approx(0.002)
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for body in ("gripper", "moving_jaw_so101_v1"):
        jaw = [g for g in range(model.ngeom) if model.geom_bodyid[g] == model.body(body).id and model.geom_priority[g] == 1 and model.geom_contype[g] > 0]  # 원본 collision_gripper 클래스(priority 1)만. 손목 서보 상자(priority 0)는 그대로
        assert jaw and all(model.geom_condim[g] == 4 and tuple(model.geom_friction[g]) == pytest.approx((1.0, 0.02, 0.001)) for g in jaw)


def test_objects_and_scene_parts_are_in_the_world_and_names_must_differ(model):
    box = model.body("obj:box")
    x, y, h = model_to_table_mm(*(model.body_pos[box.id] * 1000.0), WS.base, CFG.table_z_mm)
    assert (x, y) == pytest.approx((300.0, 200.0), abs=1e-6) and h == pytest.approx(19.5)  # 상자 가운데 높이 = 39/2
    radar = [n for n in ("radar_station", "radar_base", "radar_head") if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"scene:{n}") >= 0]  # 레이더 받침은 STL 하나(radar_station) 또는 둘(radar_base·radar_head, F2-d). 둘 다 고정 body다
    assert radar in (["radar_station"], ["radar_base", "radar_head"])
    with pytest.raises(ValueError):
        load_model(None, [SimObject("a", 1, 1), SimObject("a", 2, 2)], base=WS.base, table_z_mm=CFG.table_z_mm)


WRIST = """parts:
  - {name: wrist_hcsr04, file: assets/scene/wrist_hcsr04.stl, attach: gripper_link}
  - {name: wrist_bracket, file: assets/scene/wrist_bracket.stl, attach: gripper_link}
"""


def test_wrist_parts_are_skipped_on_the_single_jaw_hand(tmp_path):
    sc = load_scene(_write(tmp_path))
    m, warns = load_model(sc, [], base=WS.base, table_z_mm=CFG.table_z_mm)
    assert {w[1]["part"] for w in warns if w[0] == "SIM-PART-SKIPPED"} == {"wrist_bracket", "wrist_hcsr04"}
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "scene:wrist_bracket") == -1 and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "wrist_sonar") == -1


def _write(tmp_path):
    root = __import__("pathlib").Path(__file__).resolve().parents[1]
    (tmp_path / "config").mkdir(exist_ok=True)
    (tmp_path / "config" / "scene.yaml").write_text(WRIST.replace("file: assets", f"file: {root.as_posix()}/assets"), encoding="utf-8")
    return tmp_path / "config" / "scene.yaml"


def test_parallel_hand_gets_wrist_mass_inertia_and_sonar_site(tmp_path):
    m, warns = load_model(load_scene(_write(tmp_path)), [], base=WS.base, table_z_mm=CFG.table_z_mm, parallel_hand=True)
    assert not [w for w in warns if w[0] == "SIM-PART-SKIPPED"]
    b = m.body("scene:wrist_bracket")
    assert b.mass[0] == pytest.approx(0.0155, abs=0.0005)  # parts_eval 합계 15.5 g
    assert np.allclose(b.ipos * 1000, [0, -62.2, -19.2], atol=0.3)
    assert b.inertia.min() > 0 and b.inertia.max() < 1e-5  # kg·m², 수 천 g·mm² 크기
    assert m.body("scene:wrist_hcsr04").mass[0] == 0
    gripper_mass = m.body("gripper").subtreemass[0]
    base_m, _ = load_model(None, [], base=WS.base, table_z_mm=CFG.table_z_mm)
    assert gripper_mass - base_m.body("gripper").subtreemass[0] == pytest.approx(0.0155, abs=0.0005)
    site = m.site("wrist_sonar")
    assert np.allclose(site.pos * 1000, [0, -66.5, -32.3], atol=1e-6)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    R = d.site_xmat[site.id].reshape(3, 3)
    Rg = d.xmat[m.body("gripper").id].reshape(3, 3)
    assert np.allclose(R[:, 2], -Rg[:, 2], atol=1e-6)  # 광선 방향 = gripper_link -z


def test_wrist_parts_must_come_as_a_pair(tmp_path):
    p = _write(tmp_path)
    p.write_text("\n".join(p.read_text(encoding="utf-8").splitlines()[:2]) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="둘 다"):
        load_model(load_scene(p), [], base=WS.base, table_z_mm=CFG.table_z_mm, parallel_hand=True)


def test_menagerie_camera_mount_is_removed(model):
    for kind, name in ((mujoco.mjtObj.mjOBJ_BODY, "camera_mount"), (mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam"), (mujoco.mjtObj.mjOBJ_GEOM, "camera_box1"), (mujoco.mjtObj.mjOBJ_GEOM, "camera_box2")):
        assert mujoco.mj_name2id(model, kind, name) == -1, name
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper_frame") >= 0


def test_new_j1_mount_replaces_the_old_base_plate():
    """결정 0067(H-3 ①): 옛 받침 visual(base_so101_v2)은 없고, 새 받침 메쉬와 충돌 상자가 있다. 충돌 상자 바닥은 작업 면(아래 핀은 뺀다)."""
    import mujoco

    from so101_studio.arm.mujoco_model import load_model

    m, _ = load_model(base=load_workspace("config/workspace.yaml").base, table_z_mm=-2.9)
    assert "base_so101_v2" not in {m.mesh(m.geom_dataid[g]).name for g in range(m.ngeom) if m.geom_dataid[g] >= 0}
    g = m.geom("base_mount_box").id
    bottom = m.geom_pos[g][2] - m.geom_size[g][2]
    assert abs(bottom * 1000 - (-2.9)) < 0.01 and m.geom("base_mount_visual").contype == 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "base_box") >= 0


def test_radar_split_into_base_and_head_loads_as_two_fixed_bodies(tmp_path):
    stl = __import__("pathlib").Path(__file__).resolve().parents[1].as_posix() + "/assets/scene/radar_station.stl"
    (tmp_path / "scene.yaml").write_text(f"parts:\n  - {{name: radar_base, file: {stl}, attach: table, xyz_mm: [207.3, 405.5, 0], collide: true}}\n"
                                         f"  - {{name: radar_head, file: {stl}, attach: table, xyz_mm: [207.3, 405.5, 0], collide: true}}\n", encoding="utf-8")
    m, _ = load_model(load_scene(tmp_path / "scene.yaml"), [], base=WS.base, table_z_mm=CFG.table_z_mm)
    for n in ("radar_base", "radar_head"):
        b = m.body(f"scene:{n}")
        assert m.body_dofnum[b.id] == 0 and m.body_jntadr[b.id] < 0  # 돌리지 않는 고정 body
