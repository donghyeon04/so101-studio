# ruff: noqa: F811
"""REQ-038: 병렬 그리퍼 선택(config/grippers), 보정 0·기울기 0, W <= G - 4, --sim 집기·놓기."""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from test_skills import CFG, WS, kin, obj, rig  # noqa: F401  (kin은 픽스처)

from so101_studio.arm.gripper import ParallelGripper, SingleJawGripper
from so101_studio.core.config import load_robot
from so101_studio.core.types import JointState


def load_parallel():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        (Path(d) / "local.yaml").write_text("gripper: parallel\n", encoding="utf-8")
        return load_robot("config/robot.yaml", local_path=Path(d) / "local.yaml")


def test_default_is_single_jaw_and_parallel_is_selected_by_local_yaml(tmp_path):
    assert load_robot("config/robot.yaml", local_path=tmp_path / "none.yaml").gripper == "single_jaw"
    local = tmp_path / "local.yaml"
    local.write_text("gripper: parallel\n", encoding="utf-8")
    c = load_robot("config/robot.yaml", local_path=local)
    assert c.gripper == "parallel" and c.grasp_tilt_deg == 0 and max(mm for _, mm in c.gripper_width_mm) == 84
    local.write_text("gripper: nope\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_robot("config/robot.yaml", local_path=local)


def test_skills_pick_the_gripper_class(kin):
    _d, ctl, sk = rig(kin)
    assert isinstance(sk.gripper, SingleJawGripper)
    ctl.close()
    _d, ctl, sk = rig(kin, **{k: getattr(load_parallel(), k) for k in ("gripper", "gripper_width_mm", "grasp_max_open_mm", "grasp_tilt_deg")})
    assert isinstance(sk.gripper, ParallelGripper) and kin.gripper is sk.gripper
    ctl.close()


def test_parallel_geometry_has_no_correction(kin):
    g = ParallelGripper(kin, load_parallel())
    assert g.center_offset(-90.0, 30.0, 50.0) == pytest.approx([0, 0, -g.cfg.gripper_tip_offset_mm], abs=1e-3)  # 손끝은 TCP 아래
    f, m = g.jaw_tips(-90.0, 0.0, 100.0)
    assert np.linalg.norm(f - m) == pytest.approx(84.0) and g.tilt(100) == 0 and g.narrowing(100, True) == 0
    h = g.hand_pose(300, 200, 40.0, 100.0, True, 20.0)
    assert (h.x_mm, h.y_mm, h.z_mm, h.pitch_deg) == pytest.approx((300, 200, 20 + g.cfg.gripper_tip_offset_mm, -90), abs=0.01)


def pkin():
    from so101_studio.arm.kinematics import Kinematics
    cfg = load_parallel()
    return Kinematics(cfg.urdf, WS.base, cfg.joint_limits_deg)


@pytest.mark.slow
def test_parallel_pick_and_place_in_sim(kin):
    kin = pkin()  # 병렬 URDF: TCP와 메쉬가 맞아야 바닥 검사가 맞는다
    kw = {k: getattr(load_parallel(), k) for k in ("gripper", "gripper_width_mm", "grasp_max_open_mm", "grasp_tilt_deg", "gripper_tip_offset_mm", "urdf")}
    pts = sorted((mm, v) for v, mm in load_parallel().gripper_width_mm)
    block = float(np.interp(30.0, [p[0] for p in pts], [p[1] for p in pts]))
    d, ctl, sk = rig(kin, block, **kw)
    r = sk.pick(obj(width=30.0))
    assert r.status == "ok" and d.log[-1].gripper == pytest.approx(block, abs=0.6)
    assert sk.place_at(300.0, 150.0).status == "ok" and sk.last_place_correction_mm == pytest.approx((0, 0), abs=1e-3)
    ctl.close()


def test_width_rule_w_le_g_minus_4(kin):
    kw = {k: getattr(load_parallel(), k) for k in ("gripper", "gripper_width_mm", "grasp_max_open_mm", "grasp_tilt_deg")}
    _d, ctl, sk = rig(kin, **kw)
    o = replace(obj(width=81.0), height_mm=30.0)
    assert sk._grasp_height(o, 84.0, sk.gripper_for_width(84.0), False)[1] is not None  # 81 > 84 - 4
    o = replace(obj(width=80.0), height_mm=30.0)
    assert sk._grasp_height(o, 84.0, sk.gripper_for_width(84.0), False)[1] is None
    ctl.close()


def test_gripper_info_in_sim(tmp_path):
    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.service.arm_service import ArmService
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    info = s.gripper_info()
    s.disconnect()
    assert info["type"] == "single_jaw" and "parallel" in info["available"] and info["needs_centering"] is True


def test_parallel_urdf_and_mesh_lowest_z(kin):
    """REQ-041: 병렬 URDF의 gripper 관절·TCP, 메쉬 lowest_z가 턱 끝 계산과 맞는다."""
    from so101_studio.arm.kinematics import Kinematics
    cfg = load_parallel()
    assert cfg.urdf == "assets/so101_parallel.urdf"
    k = Kinematics(cfg.urdf, WS.base, cfg.joint_limits_deg)
    k.gripper = ParallelGripper(k, cfg)
    names = k._k.robot.joint_names()
    assert "gripper" in names and "gripper_left" in names
    zs = []
    for g in (0.0, 50.0, 100.0):
        pose = k.gripper.hand_pose(300, 250, 90.0, g, False, 20.0)
        q = next((q for q in (k.ik(pose, JointState(0, a, b, c, 0, g)) for a, b, c in ((-30, 30, 60), (-60, 60, 30), (0, 60, 30))) if q), None)
        assert q is not None
        q = replace(q, gripper=g)
        zs.append(k.gripper.lowest_z(q))
    assert max(zs) - min(zs) < 1.5 and zs[0] == pytest.approx(20.0, abs=1.0)  # 수직이면 간격이 바뀌어도 가장 낮은 점은 같고, 손끝 높이 20mm 근처


def test_scene_collision_with_the_parallel_urdf_moves_both_clamps():
    """#168과 함께: 병렬 URDF에서도 충돌 거리가 gripper 관절(직선 m)로 계산되고, 벌리면 클램프가 장면 부품에 더 가까워진다."""
    import trimesh

    from so101_studio.arm.collision import SceneCollision
    from so101_studio.core.scene import Scene, ScenePart
    cfg, k = load_parallel(), pkin()
    k.gripper = ParallelGripper(k, cfg)
    q = k.ik(k.gripper.hand_pose(300, 230, 90.0, 0.0, False, 30.0), JointState(0, -30, 30, 60, 0, 0))
    assert q is not None
    tip = k.fk(q)
    side = trimesh.creation.box(extents=[10, 10, 80])
    T = np.eye(4)
    T[:3, 3] = [tip.x_mm, tip.y_mm + 60, cfg.table_z_mm]  # 손끝 옆(yaw 90이면 집게가 y 방향으로 열린다) 60mm
    coll = SceneCollision(k, cfg.urdf, WS, cfg.table_z_mm, cfg.tip_floor_mm)
    coll.set_scene(Scene(10.0, [ScenePart("wall", "table", T, side)]), WS)
    closed, opened = coll.distance(replace(q, gripper=0.0))[0], coll.distance(replace(q, gripper=100.0))[0]
    assert np.isfinite(closed) and opened < closed - 10  # 84mm 열리면 한쪽 클램프가 40mm 가까이 간다
