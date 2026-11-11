"""REQ-036: 놓을 때 opening path가 쓰이고(안 쓰이면 경고 이벤트), 집기·놓기 전 구간에서 턱 최저점이 작업대 위 5mm 이상, 폭·높이에 맞춘 벌림."""
import math

import numpy as np
import pytest
from test_skills import BLOCK, CFG, TABLE, WS, Recorder, kin, rig  # noqa: F401  (kin은 픽스처)

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService


def block_for(width):
    return float(np.interp(width, [p[0] for p in TABLE], [p[1] for p in TABLE]))


@pytest.fixture
def svc(tmp_path):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert s.connect()[0]
    s.events = []
    s.ctl.on_event = lambda kind, level, code, data: s.events.append((code, data))
    yield s
    s.disconnect()


def midpoint(s, row):
    q = JointState(*row)
    p = s.kin.fk(q)
    f, m = s.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, q.gripper)
    return p.x_mm + (f[0] + m[0]) / 2, p.y_mm + (f[1] + m[1]) / 2


@pytest.mark.parametrize("target", ["zone", (300.0, 200.0)])
def test_opening_path_is_used_and_keeps_jaw_midpoint_on_the_point(svc, target):
    sk = svc.skills
    sk._run([sk._gripper(block_for(58.0))])  # 58mm 물체를 쥔 값
    seen = []
    orig = sk._closing_path
    sk._closing_path = lambda *a, **k: seen.append(orig(*a, **k)) or seen[-1]
    r = svc.place("right") if target == "zone" else svc.place(at=target)
    assert r.status == "ok"
    assert not [e for e in svc.events if e[0] == "ARM-OPENING-PATH-FALLBACK"]
    path = seen[-1]
    assert path is not None and len(path.pts) > 5
    goal = WS.zone_slots("right")[0] if target == "zone" else target  # 배출 구역은 비어 있는 칸 가운데 로봇 원점에 가장 가까운 칸 (11.3.5)
    assert max(math.dist(midpoint(svc, row), goal) for row in path.pts) < 1.0  # 매 단계 턱 중간점이 놓을 점 1mm 안


def test_fallback_is_reported_with_a_reason_when_the_gripper_is_already_open(svc):
    svc.skills._run([svc.skills._gripper(CFG.joint_limits_deg["gripper"][1])])  # 쥔 것이 없고 그리퍼는 이미 최대로 열려 있다(11.3: 벌림 끝은 쥔 틈 + 양쪽 30mm, 최대까지)
    r = svc.place(at=(300.0, 200.0))
    assert r.status == "ok"
    ev = [d for c, d in svc.events if c == "ARM-OPENING-PATH-FALLBACK"]
    assert ev and ev[0]["reason"] == "gripper_already_open"


def test_floor_check_sees_the_jaw_body(svc):
    from dataclasses import replace

    from so101_studio.core.types import ToolPose

    q = replace(svc.skills._ik(ToolPose(300, 200, CFG.table_z_mm + 40, 90, -115.0)), gripper=66.0)
    assert svc.safety._tip_height(q) <= svc.kin.jaw_lowest_z(q) - CFG.table_z_mm + 1e-6


@pytest.mark.parametrize("w,h", [(58, 39), (30, 30), (40, 20), (58, 0)])
@pytest.mark.slow
def test_jaw_lowest_point_stays_above_the_table_through_pick_and_place(kin, w, h):  # noqa: F811
    d, ctl, sk = rig(kin, block_for(w))
    o = DetectedObject("b", 300, 200, 0, w, w, 1.0, "manual", h)
    r1 = sk.pick(o)
    r2 = sk.place("right")
    ctl.close()
    assert r1.status == "ok" and r2.status == "ok"
    low = min(kin.jaw_lowest_z(q) - CFG.table_z_mm for q in d.log[::3])
    assert low >= CFG.tip_floor_mm  # 접근·하강·닫기·들기·놓기·열기·들기 전 구간


def test_open_only_as_much_as_needed_when_width_and_height_are_known(kin):  # noqa: F811
    _d, ctl, sk = rig(kin)
    ctl.close()
    small = DetectedObject("s", 300, 200, 0, 30, 30, 1.0, "manual", 20)
    assert sk._needed_gap(small, True) < CFG.grasp_max_open_mm  # 작은 물체는 최대까지 안 벌린다
    wide_tall = DetectedObject("w", 300, 200, 0, 58, 58, 1.0, "manual", 39)
    g = sk._needed_gap(wide_tall, True)
    t, why = sk._grasp_height(wide_tall, g, sk.gripper_for_width(g), True)
    assert g == 86.0 and g < CFG.grasp_max_open_mm and why is None and t == 20.0  # 58×39: 최대(90)까지 안 벌려도 지나는 가장 작은 벌림 86mm에서 설정 높이 20mm로 집는다
