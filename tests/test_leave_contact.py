"""실물 2-2(11-13): 놓은 직후 단동식 고정 턱이 상자 옆에 닿아(간격 0~2mm) 기울기 풀기가 box:1mm로 거절되던 것, 쥔 틈보다 넓은 물체 폭, PICK 기록."""
from types import SimpleNamespace

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", arm="so101_follower_01") if __import__("pathlib").Path("config/arms/so101_follower_01.yaml").exists() else load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


def make(tmp_path, block=None):
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    return s


@pytest.mark.slow
def test_untilt_allows_starting_against_the_box_but_not_closing_in(tmp_path):
    """실물 2-2 기록(runs/20261113-201146): 놓은 직후 자세(wrist_flex 102.7°)에서 고정 턱이 상자 옆에 0.5mm로 붙어 있다. 멀어지는 풀기는 허용, 가까워지는 풀기는 거절."""
    s = make(tmp_path)
    try:
        sk = s.skills
        q = JointState(20.89, 17.41, 7.87, 102.68, 0.40, 55.81)
        assert sk._run([sk._joint_move(q)]).status == "ok"
        held = SimpleNamespace(center=SimpleNamespace(x_mm=360.0, y_mm=200.0), box_w=58.0, obj_h=39.0)
        assert sk._untilt_goal(sk._q(), 39.0, held) is not None, sk.untilt_rejected
        orig = sk._box_gap
        for profile, ok in (([0.5, 1.0, 3.0], True), ([0.5, 0.5, 0.4, 3.0], True), ([1.5, 0.3, 3.0], False), ([0.5, 1.0, 1.5], False)):  # 시작보다 0.5mm 넘게 가까워지거나 끝이 2mm 미만이면 거절
            n = {"i": 0}

            def gap(st, p, h, profile=profile, n=n):
                i = n["i"]
                n["i"] += 1
                return profile[min(i, len(profile) - 1)] if i < 3 else profile[-1]

            sk._box_gap = gap
            assert (sk._untilt_goal(sk._q(), 39.0, held) is not None) == ok, (profile, sk.untilt_rejected)
        sk._box_gap = orig
    finally:
        s.disconnect()


@pytest.mark.slow
def test_leave_uses_the_measured_gap_when_it_is_narrower_than_the_object(tmp_path):
    """물체 폭 58인데 쥔 틈이 54.4mm였다(실물): 상자 폭은 쥔 틈 이하여야 한다(넓게 잡으면 고정 턱이 상자 안에 있는 것으로 계산된다)."""
    s = make(tmp_path)
    try:
        sk = s.skills
        seen = {}
        sk._leave_plan = lambda q, need, held=None: seen.setdefault("w", held.box_w) and None
        sk._low_lift = lambda lifted, need: SimpleNamespace(status="stopped")
        sk._leave(54.4, {"need_lift": 49.0, "target": (360.0, 200.0), "obj_w": 58.0, "obj_h": 39.0})
    finally:
        s.disconnect()
    assert seen["w"] == pytest.approx(54.4)


@pytest.mark.slow
def test_pick_event_records_where_the_grasp_closed(tmp_path):
    """집기 기록에 닫은 자세(기울기·목표 높이·관절)가 남는다: 실물에서 덜 내려갔는지 나중에 판정."""
    import json

    block = float(np.interp(58, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = make(tmp_path, block)
    try:
        assert s.pick_and_place(DetectedObject("b", 300, 200, 0, 58, 58, 1.0, "manual", 39)).status == "ok"
        s.logger.flush()
        ev = [json.loads(x) for x in (s.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    finally:
        s.disconnect()
    g = next(e for e in ev if e["kind"] == "pick")["data"]["grasp"]
    assert set(g) >= {"tilted", "target_tip_h_mm", "tool_z_mm", "pitch_deg", "joints", "contact"} and len(g["joints"]) == 6 and g["contact"] is True


