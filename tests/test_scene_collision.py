"""REQ-040 2단계: 장면 부품과 팔의 충돌 거리 검사."""
# ruff: noqa: F811
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import trimesh
from test_skills import CFG, WS, kin  # noqa: F401  (kin은 픽스처)

from so101_studio.arm.collision import SceneCollision
from so101_studio.arm.safety import Safety, SafetySettings
from so101_studio.core.scene import Scene, ScenePart, load_scene
from so101_studio.core.types import JointState
from so101_studio.service.targets import reason_text


def box(name, xyz, size=(40, 40, 40), attach="table", margin_scene=None, collide=True):
    m = trimesh.creation.box(extents=size)
    m.apply_translation([0, 0, size[2] / 2])
    T = np.eye(4)
    T[:3, 3] = xyz
    return ScenePart(name, attach, T, m, collide)


@pytest.fixture(scope="module")
def coll(kin):
    return SceneCollision(kin, CFG.urdf, WS, CFG.table_z_mm, CFG.tip_floor_mm)


def safety(kin, coll, parts, margin=10.0):
    coll.set_scene(Scene(margin, parts), WS)
    s = Safety(CFG.joint_limits_deg, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm)
    s.collision = coll
    return s


HOME = JointState(*CFG.poses["home"])


def test_far_part_is_ok_and_near_part_is_rejected(kin, coll):
    tip = kin.fk(HOME)
    s = safety(kin, coll, [box("far", (50, 400, 0))])
    assert s.check_goal(HOME) is None
    # 손끝이 있는 자리 옆에 상자를 두면(작업대 면에서 위로 솟아 있으므로) 팔이 margin 안으로 들어온다
    s = safety(kin, coll, [box("near", (tip.x_mm, tip.y_mm, CFG.table_z_mm), (60, 60, 300))])
    assert s.check_goal(HOME) == "scene:near"
    assert "near" in reason_text("scene:near")


def test_margin_and_modes(kin, coll):
    tip = kin.fk(HOME)
    parts = [box("p", (tip.x_mm, tip.y_mm, CFG.table_z_mm), (60, 60, 300))]
    s = safety(kin, coll, parts)
    s.settings = SafetySettings(mode="off")
    assert s.check_goal(HOME) is None
    s.settings = SafetySettings(mode="free")
    assert s.check_goal(HOME) == "scene:p"


def test_collide_false_and_empty_scene(kin, coll):
    tip = kin.fk(HOME)
    s = safety(kin, coll, [box("p", (tip.x_mm, tip.y_mm, CFG.table_z_mm), (60, 60, 300), collide=False)])
    assert s.check_goal(HOME) is None
    s = safety(kin, coll, [])
    assert s.check_goal(HOME) is None


def test_leaving_the_margin_is_allowed(kin, coll):
    tip = kin.fk(HOME)
    s = safety(kin, coll, [box("p", (tip.x_mm, tip.y_mm, CFG.table_z_mm), (60, 60, 300))])
    away = replace(HOME, shoulder_pan=HOME.shoulder_pan + 30)
    d_home, d_away = coll.distance(HOME)[0], coll.distance(away)[0]
    assert d_away > d_home
    assert s.check_goal(away, HOME) is None  # 안에서 출발해 멀어지는 움직임
    assert s.check_goal(HOME, away) is None or d_away >= 10  # 반대로 다가가는 움직임은 (멀리서 왔으면) 거부된다
    assert s.check_goal(HOME, away) == "scene:p"


def test_attached_part_counts_as_arm(kin, coll):
    tip = kin.fk(HOME)
    far = box("far", (tip.x_mm, tip.y_mm + 150, CFG.table_z_mm), (40, 40, 400))
    s = safety(kin, coll, [far])
    assert s.check_goal(HOME) is None
    d0 = coll.distance(HOME)[0]
    long = box("tool", (0, 0, 0), (20, 20, 20), attach="gripper_link")
    long.mesh = trimesh.creation.box(extents=[20, 20, 600])  # 링크에 붙은 긴 부품은 팔처럼 상자에 가까워진다
    safety(kin, coll, [far, long])
    assert coll.distance(HOME)[0] < d0


def test_attached_part_touching_floor_is_z_floor(kin, coll):
    tool = box("tool", (0, 0, 0), (20, 20, 20), attach="gripper_link")
    tool.mesh = trimesh.creation.box(extents=[1600, 1600, 1600])  # 어느 방향으로 뻗어도 작업대 아래로 내려가는 긴 부품
    s = safety(kin, coll, [box("far", (50, 400, 0)), tool])
    assert s.check_goal(HOME) == "scene_floor:tool"
    assert "tool" in reason_text("scene_floor:tool")
    assert s.check_goal(HOME, HOME) is None  # 이미 닿은 채 출발해 더 낮아지지 않으면 허용
    assert coll._k is not kin._k  # 제어 스레드가 쓰는 로봇과 따로


def test_check_path_speed(kin, coll):
    from so101_studio.arm.trajectory import cfg_move_j
    s = safety(kin, coll, load_scene("config/scene.yaml").parts)
    traj = cfg_move_j(CFG, HOME, replace(HOME, shoulder_pan=40), 100)
    t0 = time.time()
    s.check_path(traj, n=20)
    assert time.time() - t0 < 0.1  # 20점 이상을 100ms 안에 (실측 약 0.2ms/점)


def test_service_scene_and_reload(tmp_path):
    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.service.arm_service import ArmService
    (tmp_path / "workspace.yaml").write_text(Path("config/workspace.yaml").read_text(encoding="utf-8"), encoding="utf-8")
    (tmp_path / "scene.yaml").write_text("margin_mm: 5\nparts:\n  - {name: wall, box_mm: [10, 10, 10], xyz_mm: [1, 1, 0]}\n", encoding="utf-8")
    svc = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert svc.connect()[0]
    assert [p.name for p in svc.scene.parts] == ["radar_station"] and svc.safety.collision.scene is svc.scene
    svc.reload_workspace(tmp_path / "workspace.yaml")
    assert [p.name for p in svc.scene.parts] == ["wall"] and svc.safety.collision.scene is svc.scene
    svc.disconnect()


BAD_SCENE = "parts:\n  - {name: a, file: nope.stl}\n"


def test_reload_failure_changes_nothing_and_bad_scene_does_not_block_start(tmp_path, monkeypatch):
    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.service.arm_service import ArmService
    (tmp_path / "workspace.yaml").write_text(Path("config/workspace.yaml").read_text(encoding="utf-8").replace("x_mm: 300", "x_mm: 310", 1), encoding="utf-8")
    (tmp_path / "scene.yaml").write_text(BAD_SCENE, encoding="utf-8")
    svc = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert svc.connect()[0]
    ws0, scene0 = svc.ws, svc.scene
    with pytest.raises(RuntimeError):
        svc.reload_workspace(tmp_path / "workspace.yaml")
    assert svc.ws is ws0 and svc.scene is scene0 and svc.skills.ws is ws0
    svc.disconnect()
    monkeypatch.chdir(tmp_path)
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "scene.yaml").write_text(BAD_SCENE, encoding="utf-8")
    with pytest.warns(UserWarning):
        bad = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=None)
    assert bad.scene.parts == [] and bad.scene_error
