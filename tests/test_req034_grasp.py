"""REQ-034: 물체 높이에 맞춘 집는 높이(height_mm), 그리퍼 속도 배율(gripper_speed_pct)."""
import dataclasses
from pathlib import Path

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service import params
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", gripper="single_jaw")  # 단동식 기하로 잰 기대값 (REQ-098: 기본은 parallel)
WS = load_workspace("tests/workspace_600x400.yaml")


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    yield s
    s.disconnect()


def obj(width, height, **kw):
    return DetectedObject("o", 300.0, 210.0, 0.0, width, 100.0, 1.0, "manual", **{"height_mm": height, **kw})


def test_height_defaults_to_zero_and_is_the_last_field():
    assert [f.name for f in dataclasses.fields(DetectedObject)][-1] == "height_mm"
    assert DetectedObject("o", 1, 2, 0, 3, 4, 1.0, "manual").height_mm == 0.0  # 위치 인자로 만들던 쪽이 그대로 동작


def test_unknown_height_uses_the_configured_grasp_height(svc):
    t, why = svc.skills._grasp_height(obj(30, 0), 90.0, 66.0, True)
    assert (t, why) == (CFG.grasp_height_mm, None)


def test_wide_tall_object_raises_the_grasp_height(svc):
    # 66x39 물체: 설정 t(20)로는 G100에서도 위 모서리를 친다 -> 더 높게 집는다
    sk = svc.skills
    v0 = sk.gripper_for_width(100.0)
    t, why = sk._grasp_height(obj(66, 39), 100.0, v0, True)
    assert why is None and t > CFG.grasp_height_mm and 39 - t >= CFG.grasp_min_contact_mm


def test_too_wide_for_the_opening_is_rejected_with_reason(svc):
    sk = svc.skills
    r = sk.pick(obj(80, 39))  # 최대 벌림 안에서 모서리를 못 지난다
    assert r.status == "out_of_bounds" and r.reason.startswith("grasp_blocked")


def test_low_object_lowers_the_grasp_height(svc):
    t, why = svc.skills._grasp_height(obj(30, 18), 90.0, svc.skills.gripper_for_width(90.0), True)
    assert why is None and t <= 18 - CFG.grasp_min_contact_mm + 1e-9


def test_gripper_speed_scales_gripper_only_motions_and_is_a_now_param(svc):
    base = svc.skills._gripper(60.0).duration_s
    params.set_params(svc, {"gripper_speed_pct": 50})
    assert params.timing("gripper_speed_pct") == params.NOW
    assert svc.skills._gripper(60.0).duration_s > base * 1.3  # 절반 속도면 시간이 늘어난다(가속 한도가 있어 정확히 두 배는 아니다)
    with pytest.raises(ValueError):
        params.set_params(svc, {"gripper_speed_pct": 0})
    with pytest.raises(ValueError):
        params.set_params(svc, {"gripper_speed_pct": 101})


# ---- 교시 재생 변환본 (fitted)
def _write(tmp_path, name, rows, times):
    import json

    (tmp_path / f"{name}.json").write_text(json.dumps({"hz": 20, "t": list(times), "joints": rows}), encoding="utf-8")


def test_fit_times_slows_only_the_segments_over_the_limit():
    import numpy as np

    from so101_studio.arm.teach import fit_times

    t = np.arange(0, 2.0, 0.05)  # 20Hz
    q = np.zeros((len(t), 6))
    q[:, 0] = np.where(t < 1.0, t * 40.0, 40.0 + (t - 1.0) * 600.0)  # 앞은 40°/s(천천히), 뒤는 600°/s(너무 빠름)
    out = fit_times(q, t, v_max=237.0, a_max=1e9)
    dt0, dt1 = np.diff(t), np.diff(out)
    assert np.allclose(dt1[:19], dt0[:19])  # 한도 안 구간은 그대로
    assert (np.abs(np.diff(q[:, 0])) / dt1).max() <= 237.0 + 1e-6 and out[-1] > t[-1]


def test_info_reports_fit_and_fitted_replay_slows_a_too_fast_recording(tmp_path):
    svc = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert svc.connect()[0]
    try:
        svc.teach.dir = tmp_path
        slow = [[p, -90, 93, 0, 0, 3] for p in range(11)]
        _write(tmp_path, "slow", slow, [i * 0.2 for i in range(11)])  # 5°/0.2s = 25°/s
        fast = [[0, -90, 93, 0, 0, 3], [12, -90, 93, 0, 0, 3]]  # 한 표본 간격에 12° (0.02초 = 600°/s)
        _write(tmp_path, "fast", fast, [0.0, 0.02])
        assert svc.record_info("slow")["fit"]["fits_1x"] is True
        fit = svc.record_info("fast")["fit"]
        assert fit["fits_1x"] is False and fit["slowed_segments"] >= 1 and fit["added_s"] > 0
        # 배율을 낮추면 한도 안으로 들어온다
        assert svc.record_info("fast", speed=0.1)["fit"]["fits_1x"] is True
        with pytest.raises(ValueError):
            svc.replay("fast", mode="bogus")
        r = svc.replay("slow", mode="fitted")
        assert r.status == "ok"
    finally:
        svc.disconnect()


# ---- 작업대 설정 다시 읽기
def test_reload_workspace_updates_skills_safety_kinematics_and_rejects_while_moving(svc, tmp_path):
    import yaml

    from so101_studio.core.types import JointState

    raw = yaml.safe_load(Path("tests/workspace_600x400.yaml").read_text(encoding="utf-8"))
    raw["robot_base_in_table"]["x_mm"] = 350
    raw["table_mm"] = {"x": 700, "y": 400}
    f = tmp_path / "ws.yaml"
    f.write_text(yaml.safe_dump(raw), encoding="utf-8")
    ws = svc.reload_workspace(f)
    assert ws.base.x_mm == 350 and svc.skills.ws is ws and svc.safety.ws is ws and svc.kin.base == ws.base and svc.ws is ws
    h = svc.ctl.execute(svc.skills._joint_move(JointState(0, -30, 30, 0, 0, 3)))
    with pytest.raises(RuntimeError):
        svc.reload_workspace(f)  # 동작 중
    svc.ctl.stop()
    h.wait()
    svc.ctl.wait_idle()
    with pytest.raises(FileNotFoundError):
        svc.reload_workspace(tmp_path / "none.yaml")
    assert svc.ws is ws  # 못 읽으면 그대로


def test_fit_limit_is_the_per_tick_step_limit(tmp_path):
    """fit·fitted의 속도 한도는 한 주기 이동량 한계의 95% (약 237°/s). 틱이 늦어도 컨트롤러가 그 안으로만 따라잡는다(test_motion)."""
    svc = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert svc.connect()[0]
    try:
        svc.teach.dir = tmp_path
        svc.cfg.replay_a_max_deg_s2 = 1e9  # 속도 한도만 본다(일정 속도로 시작하는 기록은 가속이 무한대)
        for name, step in (("v213", 10.65), ("v300", 15.0)):  # 20Hz 표본마다 step도
            _write(tmp_path, name, [[i * step, -90, 93, 0, 0, 3] for i in range(11)], [i * 0.05 for i in range(11)])
        assert svc.teach._limits()[0] == pytest.approx(0.95 * 5.0 / svc.ctl.dt)
        assert svc.record_info("v213", speed=0.5)["fit"]["fits_1x"] is True
        assert svc.record_info("v300", speed=1.0)["fit"]["fits_1x"] is False
    finally:
        svc.disconnect()
