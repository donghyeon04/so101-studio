"""M7a 부품 사전 평가(arm/parts_eval.py): 센서 오프셋이 H4 평가값과 같고, 질량·간섭이 말이 되는 범위인지."""
import pytest

from so101_studio.arm import parts_eval as pe


@pytest.fixture(scope="module")
def kin():
    return pe._kin()  # torch를 메쉬보다 먼저 읽는다(Windows DLL)


@pytest.fixture(scope="module")
def parts(kin):
    return pe.load_parts()


def test_sensor_offset_matches_the_h4_evaluation(parts):
    o = pe.sensor_offset(parts)["diff_mm"]
    assert abs(o["x"]) < 1.0 and abs(o["y"]) < 0.5 and abs(o["z"]) < 0.5  # {0.6, 66.5, 99.5}


def test_mass_is_estimated_and_static_torque_grows_a_little(parts, kin):
    mp = pe.mass_properties(parts)
    assert 10 < mp["mass_g"] < 40 and mp["parts"]["sensor"]["mass_g"] == pe.SENSOR_G
    t = pe.static_torque(mp, kin)
    for row in t.values():
        for j in row.values():
            assert 0 <= j["added_nm"] < 0.1 and j["with_nm"] < pe.STALL_NM


def test_bracket_keeps_clear_of_the_arm_links_at_the_wrist_limits(parts, kin):
    assert pe.joint_end_interference(parts, kin)["min_mm"] > 5


def test_overhang_has_a_zero_support_orientation(parts):
    o = pe.overhang(parts)
    assert o["best_pct"] < 5 and o["best_floor_face"] == "+z" and o["support_area_pct_by_floor_face"]["-z"] > 30  # +z(손목 쪽 평평한 면)를 바닥에 두면 서포트 0%


def test_scene_yaml_can_attach_the_parts_to_gripper_link(tmp_path):
    from so101_studio.core.scene import load_scene
    cfg = tmp_path / "config"
    cfg.mkdir()
    (cfg / "scene.yaml").write_text("parts:\n" + "".join(f"  - {{name: {n}, file: {pe.SCENE_DIR / (n + '.stl')}, attach: gripper_link}}\n".replace("\\", "/") for n in pe.NAMES.values()), encoding="utf-8")
    sc = load_scene(cfg / "scene.yaml")
    assert [p.attach for p in sc.parts] == ["gripper_link"] * 2 and all(len(p.mesh.faces) > 10 for p in sc.parts)
