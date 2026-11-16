"""M5 집기 시험장 완료 기준 (mujoco-spec 13.10.1). 실시간이라 대부분 slow. mujoco가 없으면 건너뛴다."""
import json
from dataclasses import replace

import pytest

pytest.importorskip("mujoco")

from so101_studio.arm import mujoco_bench as bench
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.service.arm_service import ArmService

KEYS = {"reported", "picked", "placed", "outcome", "agree", "slip_mm", "place_err_mm", "tilt_end_deg", "other_pushed_mm", "events", "sim_s", "wall_s",
        "dropped_count", "realtime_lag", "gap0_mm", "jaw_yaw_deg"}
WRIST = {"wrist_hcsr04", "wrist_bracket"}


def kin_skills(cfg):
    svc = ArmService(SimDriver(), cfg, bench.BenchEnv(cfg, realtime=False).ws, log_root=None)
    svc.connect()
    return svc.skills


@pytest.mark.slow
def test_pick_smoke_runs_and_every_row_has_the_judgement_keys(tmp_path):
    out = bench.run("pick_smoke", jobs=3, out_root=tmp_path)
    rows = [json.loads(x) for x in (out / "results.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 12
    for r in rows:
        assert KEYS <= set(r), KEYS - set(r)
    assert (out / "summary.md").exists() and json.loads((out / "summary.json").read_text(encoding="utf-8"))["episodes"] == 12


def test_same_suite_and_seed_give_the_same_world_and_given():
    cfg = bench.load_cfg()
    skills = kin_skills(cfg)
    s = bench.load_suite("inject_grid")
    a, b = bench.layout(s), bench.layout(s)
    assert [e.truth for e in a] == [e.truth for e in b] and [e.id for e in a] == [e.id for e in b]
    assert [bench.make_given(e.truth, e.inject, skills) for e in a] == [bench.make_given(e.truth, e.inject, skills) for e in b]
    sp = bench.layout({**bench.load_suite("pick_smoke"), "spread": {"xy_mm": 5, "angle_deg": 3}}, seeds=2)
    assert len(sp) == 24 and sp[0].truth != sp[1].truth and sp[0].truth == bench.layout({**bench.load_suite("pick_smoke"), "spread": {"xy_mm": 5, "angle_deg": 3}}, seeds=2)[0].truth


@pytest.mark.slow
def test_an_episode_does_not_leak_into_the_next(tmp_path):
    cfg = bench.load_cfg()
    env = bench.BenchEnv(cfg)
    e1, e2 = bench.layout(bench.load_suite("pick_smoke"))[:2]
    row = bench.run_episode(e1, cfg, tmp_path / "e1", env)
    assert row["outcome"] == "placed_ok", row
    first = env.svc
    try:
        env.reset(options={"episode": e2, "log_dir": tmp_path / "e2"})
        assert env.svc is not first and not env.svc.skills.placed and not env.svc.skills.known_objects and env.svc.skills.grip is None
    finally:
        env.close()


@pytest.mark.parametrize("gripper", ["parallel", "single_jaw"])
def test_driver_and_service_see_the_same_scene_parts(tmp_path, gripper):
    cfg = bench.load_cfg(bench.gripper_override(gripper, tmp_path))
    env = bench.BenchEnv(cfg, realtime=False)
    try:
        env.reset(options={"episode": bench.layout(bench.load_suite("pick_smoke"))[0], "log_dir": tmp_path / "log"})
        drv_parts, svc_parts = {p.name for p in env.drv.scene.parts}, {p.name for p in env.svc.scene.parts}
        assert drv_parts == svc_parts
        if gripper == "parallel":
            assert WRIST <= drv_parts
        else:
            assert not WRIST & drv_parts and cfg.grasp_tilt_deg != 0
    finally:
        env.close()


@pytest.mark.slow
def test_offset_beyond_half_the_spare_gap_lands_a_clamp_on_the_top(tmp_path):
    cfg = bench.load_cfg()
    ep = bench.layout(bench.load_suite("inject_grid"))[0]
    ep = replace(ep, truth={**ep.truth, "x_mm": 300.0, "y_mm": 230.0})  # 정면 180mm(팔 원점 300,50)
    _, gap, _ = bench.make_given(ep.truth, {}, kin_skills(cfg))
    off = (gap - ep.truth["width_mm"]) / 2 + 8
    ep = replace(ep, id="inject-one", inject={"pos_offset_mm": [off, 0], "height_error_mm": 0})
    samples = []
    row = bench.run_episode(ep, cfg, tmp_path, samples_out=samples)
    h = ep.truth["height_mm"]
    top = [c for st in samples for c in st["contacts"] if {c["a"], c["b"]} & {"clamp_right", "clamp_left"} and "obj:box" in (c["a"], c["b"]) and abs(c["pos_mm"][2] - h) <= 5]
    print(f"G {gap:.1f}mm, 어긋남 {off:.1f}mm: outcome {row['outcome']}, reported {row['reported']}, 윗면 접촉 {len(top)}개")
    assert top and row["outcome"] != "placed_ok", row
