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
