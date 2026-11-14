"""REQ-064 (결정 0059): MuJoCo에서 막힘 감지 판정 확인. 장애물(config/sim_scenarios/blocker.yaml)에 막힌 이동은 멈추고, 장애물 없는 빈 이동은 오탐 0.
쥔 채 이동의 오탐 0은 test_mujoco_scenarios.py의 carry 시험이 ARM-BLOCKED 0건으로 본다. mujoco가 없으면 건너뛴다."""
import json
from dataclasses import replace

import pytest

pytest.importorskip("mujoco")

from so101_studio.arm.sim_driver import make_sim_driver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
RADAR = JointState(*CFG.poses["radar_clear"])


def start(scenario, tmp_path, speed):
    svc = ArmService(make_sim_driver(CFG, WS, None, scenario=scenario), CFG, WS, speed_pct=speed, log_root=str(tmp_path))
    ok, msg = svc.connect()
    assert ok, msg
    return svc


def blocked_events(svc):
    return [e for e in (json.loads(x) for x in (svc.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()) if e.get("code") == "ARM-BLOCKED"]


@pytest.mark.slow
@pytest.mark.parametrize("speed", [30, 50])
def test_move_into_a_tall_box_is_stopped_and_held(tmp_path, speed):
    svc = start("blocker", tmp_path, speed)
    try:
        assert svc.move_joints(RADAR).status == "ok"
        r = svc.move_tool(svc.point_pose(300, 200, 10))  # 상자(높이 120) 한가운데로 내려간다
        ev = blocked_events(svc)
        assert r.status == "stopped" and r.reason.startswith("blocked:"), r
        assert len(ev) == 1 and ev[0]["level"] == "WARN" and ev[0]["data"]["profile"] == "empty"
        assert svc.ctl.state == "idle" and svc.ctl._job is None
    finally:
        svc.disconnect()


@pytest.mark.slow
@pytest.mark.parametrize("speed", [30, 50])
def test_free_empty_moves_never_trigger_blocked(tmp_path, speed):
    svc = start("carry30", tmp_path, speed)  # 상자(40 높이)는 (300,200) 한 곳: 80mm 위로만 지난다
    try:
        for name, go in (("radar", lambda: svc.move_joints(RADAR)), ("a", lambda: svc.move_tool(svc.point_pose(300, 200, 70))),
                         ("b", lambda: svc.move_tool(svc.point_pose(340, 200, 70))), ("home", svc.home)):
            r = go()
            assert r.status == "ok", (name, r)
        assert not blocked_events(svc)
    finally:
        svc.disconnect()


def test_sim_blocked_applies_at_judgement_only_for_mujoco_and_rejects_unknown_keys(tmp_path):
    from so101_studio.arm.sim_driver import SimDriver

    assert CFG.sim_blocked  # robot.yaml 값
    mj = ArmService(make_sim_driver(CFG, WS, None, scenario="carry30"), CFG, WS, log_root=None)
    mj.connect()
    assert mj.cfg is CFG and mj.cfg.blocked_empty_cruise_deg == 15.5  # cfg·프리셋 저장값은 실제 설정 그대로
    assert mj.ctl.blocked_cfg.blocked_empty_cruise_deg == CFG.sim_blocked["blocked_empty_cruise_deg"]  # 판정은 sim 값
    assert mj.ctl.blocked_cfg.blocked_grip_units == CFG.blocked_grip_units  # sim_blocked에 없는 키는 cfg
    mj.disconnect()
    kin = ArmService(SimDriver(), CFG, WS, log_root=None)
    kin.connect()
    assert kin.ctl.blocked_cfg is kin.cfg  # 기구학 시뮬레이션은 실물 값 그대로
    kin.disconnect()
    with pytest.raises(ValueError, match="sim_blocked"):
        ArmService(make_sim_driver(CFG, WS, None, scenario="carry30"), replace(CFG, sim_blocked={"speed_pct_default": 5}), WS, log_root=None)
