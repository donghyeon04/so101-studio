"""REQ-040 1단계: config/scene.yaml 읽기."""
import numpy as np
import pytest

from so101_studio.core.scene import load_scene


def test_missing_file_is_empty_scene(tmp_path):
    s = load_scene(tmp_path / "none.yaml")
    assert s.parts == [] and s.margin_mm == 10.0


def test_repo_scene_has_radar_box():
    s = load_scene("config/scene.yaml")
    p = s.parts[0]
    assert p.name == "radar_station" and p.attach == "table" and p.collide
    assert p.mesh.bounds[0][2] == pytest.approx(0) and p.mesh.bounds[1][2] == pytest.approx(20)  # 아래면이 원점
    assert p.T[:3, 3] == pytest.approx([300, 400, 0])


def test_rpy_and_errors(tmp_path):
    f = tmp_path / "config" / "scene.yaml"
    f.parent.mkdir()
    f.write_text("parts:\n  - {name: a, box_mm: [10, 10, 10], xyz_mm: [1, 2, 3], rpy_deg: [0, 0, 90], attach: wrist_link}\n", encoding="utf-8")
    p = load_scene(f).parts[0]
    assert p.attach == "wrist_link" and np.allclose(p.T @ [1, 0, 0, 1], [1, 3, 3, 1])
    f.write_text("parts:\n  - {name: a, file: nope.stl}\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_scene(f)
    f.write_text("parts:\n  - {name: a, box_mm: [1, 1, 1]}\n  - {name: a, box_mm: [1, 1, 1]}\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_scene(f)


def _cfg(tmp_path, gripper):
    from so101_studio.core.config import load_robot

    lp = tmp_path / f"{gripper}.yaml"
    lp.write_text(f"gripper: {gripper}\n", encoding="utf-8")
    return load_robot("config/robot.yaml", local_path=str(lp))


def test_for_gripper_filters_parts():
    s = load_scene("config/scene.yaml")
    assert {p.name for p in s.for_gripper("parallel").parts} == {"radar_station", "wrist_hcsr04", "wrist_bracket"}
    assert [p.name for p in s.for_gripper("single_jaw").parts] == ["radar_station"]  # 키 없는 부품은 늘 쓴다


@pytest.mark.parametrize("gripper, wrist", [("parallel", {"wrist_hcsr04", "wrist_bracket"}), ("single_jaw", set())])
def test_service_and_mujoco_driver_share_the_filtered_scene(tmp_path, gripper, wrist):
    """mujoco-spec 13.10.1 시험 4: 드라이버 장면 = svc.scene."""
    pytest.importorskip("mujoco")
    from so101_studio.arm.sim_driver import make_sim_driver
    from so101_studio.core.config import load_workspace
    from so101_studio.service.arm_service import ArmService

    cfg, ws = _cfg(tmp_path, gripper), load_workspace("config/workspace.yaml")
    d = make_sim_driver(cfg, ws, load_scene())
    svc = ArmService(d, cfg, ws, 30)
    names = [p.name for p in svc.scene.parts]
    assert [p.name for p in d.scene.parts] == names
    assert wrist == set(names) & {"wrist_hcsr04", "wrist_bracket"}
