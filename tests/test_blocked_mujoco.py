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
    svc.ctl.blocked_cfg = replace(CFG, blocked_empty_accel_deg=6.0, blocked_empty_cruise_deg=4.0 * (0.25 + 0.75 * speed / 100))  # MuJoCo 액추에이터의 오차는 실물 서보보다 작고 속도에 비례한다(결정 0059): 실물 문턱(10/15.5)으로는 장애물 시험이 잡히지 않아 시뮬레이션 값을 쓴다
    return svc


def blocked_events(svc):
    return [e for e in (json.loads(x) for x in (svc.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()) if e.get("code") == "ARM-BLOCKED"]


@pytest.mark.slow
@pytest.mark.parametrize("speed", [30, 100])
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
@pytest.mark.parametrize("speed", [30, 100])
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
