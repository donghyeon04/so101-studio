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

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")


@pytest.fixture(scope="module")
def model():
    return load_model(load_scene("config/scene.yaml"), [SimObject("box", 300, 200, 0, 30, 100, 39)], base=WS.base, table_z_mm=CFG.table_z_mm)


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
        jaw = [g for g in range(model.ngeom) if model.geom_bodyid[g] == model.body(body).id and model.geom_group[g] in (3, 4) and model.geom_contype[g] > 0]
        assert jaw and all(model.geom_condim[g] == 4 and tuple(model.geom_friction[g]) == pytest.approx((1.0, 0.02, 0.001)) for g in jaw)


def test_objects_and_scene_parts_are_in_the_world_and_names_must_differ(model):
    box = model.body("obj:box")
    x, y, h = model_to_table_mm(*(model.body_pos[box.id] * 1000.0), WS.base, CFG.table_z_mm)
    assert (x, y) == pytest.approx((300.0, 200.0), abs=1e-6) and h == pytest.approx(19.5)  # 상자 가운데 높이 = 39/2
    assert model.body("scene:radar_station").id > 0
    with pytest.raises(ValueError):
        load_model(None, [SimObject("a", 1, 1), SimObject("a", 2, 2)], base=WS.base, table_z_mm=CFG.table_z_mm)
