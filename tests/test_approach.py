"""병렬 그리퍼 접근 각도 후보 고르기 (arm-software-spec 6.6.1, 결정 0069, REQ-078) ①~⑨."""
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from test_skills import WS, Recorder

from so101_studio.arm import approach
from so101_studio.arm.kinematics import Kinematics, _rotation
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.skills import Skills
from so101_studio.core import frames
from so101_studio.core.config import load_robot
from so101_studio.core.types import DetectedObject, ToolPose


def load_cfg(tmp_dir: Path, extra: str = ""):
    (tmp_dir / "local.yaml").write_text("gripper: parallel\n" + extra, encoding="utf-8")
    return load_robot("config/robot.yaml", local_path=tmp_dir / "local.yaml")


@pytest.fixture(scope="module")
def pcfg(tmp_path_factory):
    return load_cfg(tmp_path_factory.mktemp("cfg"), "grasp_approach: auto\n")


@pytest.fixture(scope="module")
def pkin(pcfg):
    return Kinematics(pcfg.urdf, WS.base, pcfg.joint_limits_deg)


def prig(kin, cfg, block_mm=None, **kw):
    cfg = replace(cfg, **kw)
    block = None
    d = Recorder(cfg.poses["home"])
    ctl = MotionController(d, kin, Safety(cfg.joint_limits_deg, WS, kin, z_floor_mm=cfg.tip_floor_mm, table_z_mm=cfg.table_z_mm), cfg.control_hz)
    sk = Skills(ctl, kin, cfg, WS, speed_pct=100)
    if block_mm is not None:
        block = sk.gripper_for_width(block_mm)
        d.grip_block = block
    return d, ctl, sk


def box(x=300.0, y=230.0, angle=0.0, w=58.0, length=58.0, h=39.0):
    return DetectedObject("b", x, y, angle, w, length, 1.0, "manual", h)


def table_xy(r, az_robot):
    return frames.robot_to_table(r * math.cos(math.radians(az_robot)), r * math.sin(math.radians(az_robot)), WS.base)


SLOT = (350.0, 230.0)  # 놓을 점: 반경 약 187mm


def angle_deg(u, v) -> float:
    return math.degrees(math.acos(min(1.0, abs(float(np.dot(u, v))))))


# ① 닫힘 기울기 γ
def test_closing_tilt_values():
    assert approach.closing_tilt(0.0, 35.0)[1] == pytest.approx(0.0)
    assert approach.closing_tilt(90.0, 35.0)[1] == pytest.approx(35.0)
    assert approach.closing_tilt(45.0, 35.0)[1] == pytest.approx(26.4, abs=0.1)


# ② (β, φ = 90°) 프레임이 IK 목표 _rotation(-90 + β, 방위) @ _C와 같다
@pytest.mark.parametrize("az,beta", [(0.0, 20.0), (30.0, 35.0), (-40.0, 50.0)])
def test_phi_90_frame_matches_tool_pose_rotation(pkin, az, beta):
    a, c = approach.axes(az, beta, 90.0)
    R = _rotation(-90.0 + beta, az) @ pkin._C  # 로봇 좌표(작업대 회전 없이 같은 방위로 비교)
    assert angle_deg(R[:, 2], a) < 0.01  # _C는 거의 도구 축(z) 둘레 회전이라 손끝 축은 그대로
    assert angle_deg(_rotation(-90.0 + beta, az)[:, 0], c) < 1e-6  # 닫힘 방향 = 이상적 도구 x축(턱 축)


# ③ 풀린 자세: 위치 1mm, a·c 1° (tool_axes로 따로 확인). 첫 PR FK 확인 (a)(b)와 총괄 계산의 pitch -70 경우 포함
@pytest.mark.slow
def test_wrist_roll_keeps_tcp_and_tool_axis_stays_radial(pkin, pcfg):
    _d, ctl, sk = prig(pkin, pcfg)
    try:
        for r in (120.0, 250.0):
            for az_r in (-40.0, 0.0, 40.0):
                x, y = table_xy(r, az_r)
                az = WS.base.yaw_deg + az_r
                q = sk._ik(ToolPose(x, y, 40.0, az, -70.0))
                p = pkin.fk(q)
                assert math.dist((p.x_mm, p.y_mm, p.z_mm), (x, y, 40.0)) < 1.0  # pitch -70·높이 40 해의 위치 오차(총괄 계산 34.4는 재현되지 않음)
                for dr in (-90.0, -45.0, 45.0, 90.0):  # (a) wrist_roll을 돌려도 TCP가 1mm 안
                    p2 = pkin.fk(replace(q, wrist_roll=q.wrist_roll + dr))
                    assert math.dist((p.x_mm, p.y_mm, p.z_mm), (p2.x_mm, p2.y_mm, p2.z_mm)) < 1.0
                a = pkin.tool_axes(q)[:, 2]  # (b) 손끝 축의 수평 성분이 r̂와 1° 안
                rh = np.array([math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0])
                assert angle_deg(np.r_[a[:2], 0.0] / np.linalg.norm(a[:2]), rh) < 1.0
    finally:
        ctl.close()


@pytest.mark.slow
@pytest.mark.parametrize("beta,phi", [(20.0, 0.0), (35.0, 45.0), (20.0, -60.0), (35.0, 90.0)])
def test_solved_tilted_pose_matches_position_and_axes(pkin, pcfg, beta, phi):
    _d, ctl, sk = prig(pkin, pcfg)
    try:
        x, y = table_xy(200.0, 20.0)
        az = WS.base.yaw_deg + 20.0
        q, _ = sk._tilt_solve(ToolPose(x, y, 40.0, az, -90.0 + beta), phi)
        p = pkin.fk(q)
        assert math.dist((p.x_mm, p.y_mm, p.z_mm), (x, y, 40.0)) < 1.0
        A = pkin.tool_axes(q)
        a, c = approach.axes(az, beta, phi)
        assert angle_deg(A[:, 2], a) < 1.0 and angle_deg(A[:, 0], c) < 1.0
    finally:
        ctl.close()


# ④ 손 높이 규칙: 58×58×39에서 γ 20°는 통과, 25°는 거부
def test_height_rule_58_box(pkin, pcfg):
    _d, ctl, sk = prig(pkin, pcfg)
    try:
        t, gc, go = sk._tilt_height(58.0, 39.0, 20.0)
        s = math.sin(math.radians(20.0))
        assert t + gc * s <= 39.0 - pcfg.grasp_min_contact_mm + 1e-6 and t - (go - gc) / 2 * s >= pcfg.tip_floor_mm - 1e-6
        assert sk._tilt_height(58.0, 39.0, 25.0) is None
        assert sk._tilt_height(58.0, 39.0, 0.0)[0] == pytest.approx(pcfg.grasp_height_mm)  # γ = 0이면 6.6과 같은 높이
    finally:
        ctl.close()


# ⑤ 높이 0이면 β = 0만 ⑨ place_targets가 없으면 β = 0 ⑥ vertical이면 이 절을 타지 않는다
def test_vertical_is_used_without_height_targets_or_auto(pkin, pcfg):
    _d, ctl, sk = prig(pkin, pcfg)
    try:
        sk._tilted_pick_plan = lambda *a: pytest.fail("후보를 계산하면 안 된다")
        assert sk._approach(box(h=0.0), [SLOT]) is None
        assert sk._approach(box(), None) is None
        sk.cfg = replace(sk.cfg, grasp_approach="vertical")
        assert sk._approach(box(), [SLOT]) is None
    finally:
        ctl.close()


def test_config_and_params(tmp_path):
    from so101_studio.service.params import CHOICES, LIMITS
    c = load_cfg(tmp_path)
    assert c.grasp_approach == "vertical" and c.grasp_approach_tilts_deg == [0, 20, 35, 50]
    assert c.grasp_contact_tilt_max_deg == 20 and c.grasp_approach_tilt_pref_deg == 35
    assert load_cfg(tmp_path, "grasp_approach: auto\n").grasp_approach == "auto"  # 덮어쓰기 파일 한 줄로 바뀐다(mujoco_bench --approach)
    assert CHOICES["grasp_approach"] == ("vertical", "auto")
    assert LIMITS["grasp_contact_tilt_max_deg"] == (0, 45) and LIMITS["grasp_approach_tilt_pref_deg"] == (0, 60)


# ⑦ 기운 집기 -> place_at이 같은 β·φ로 놓고 옛 방식·jaw_segments·기울기 풀기를 쓰지 않는다
@pytest.mark.slow
@pytest.mark.parametrize("angle,want", [(0.0, (35.0, "length", 0.0)), (30.0, (20.0, "length", -31.6))])  # 정면 상자: γ = 0인 길이 쪽 β 35. 30°: |γ| 10.3°(β 20)가 19.3°(β 35)보다 먼저
def test_tilted_pick_then_place_keeps_beta_phi(pkin, pcfg, angle, want):
    d, ctl, sk = prig(pkin, pcfg)
    events = []
    ctl.on_event = lambda kind, level, code, data: events.append((code, data))
    sk._untilt_goal = lambda *a, **k: pytest.fail("기울여 쥔 병렬은 기울기를 풀지 않는다")
    try:
        o = box(angle=angle)
        d.grip_block = sk.gripper_for_width(sk._approach(o, [SLOT]).gap_c - 2.0)  # 닿는 틈보다 2mm 좁은 곳에서 막힌다
        r = sk.pick(o, [SLOT])
        assert r.status == "ok", r
        ap = sk.grip["approach"]
        assert ap["beta"] > 0 and abs(ap["gamma"]) <= pcfg.grasp_contact_tilt_max_deg and sk.last_pick_tilted is True
        assert sk.last_pick_info["approach"] == ap and [dd for c, dd in events if c == "ARM-PICK-APPROACH"] == [ap]
        assert (ap["beta"], ap["face"]) == want[:2] and ap["phi"] == pytest.approx(want[2], abs=0.5) and ap["pick_az_deg"] == pytest.approx(90.0, abs=1.0)
        n_pick = len(d.log)
        f, m = pkin.jaw_tips_at(min(d.log, key=lambda q: pkin.fk(q).z_mm))  # 집을 때 낮은 턱 끝 = 계획 높이 t
        assert abs(min(f[2], m[2]) - pcfg.table_z_mm - sk.last_pick_info["target_tip_h_mm"]) < 2.0
        r = sk.place_at(*SLOT)
        assert r.status == "ok", r
        assert not any(c == "ARM-PLACE-LEGACY" for c, _ in events)
        az = sk._az(*SLOT)
        a_want, c_want = approach.axes(az, ap["beta"], ap["phi"])
        low = min(d.log[n_pick:], key=lambda q: pkin.fk(q).z_mm)  # 가장 낮은 자세 = 놓는 자세
        A = pkin.tool_axes(low)
        assert angle_deg(A[:, 2], a_want) < 1.0 and angle_deg(A[:, 0], c_want) < 1.0
        f, m = pkin.jaw_tips_at(low)
        assert abs(min(f[2], m[2]) - pcfg.table_z_mm - (sk.last_pick_info["target_tip_h_mm"] + pcfg.place_drop_mm)) < 2.0  # 낮은 턱 끝 기준 높이로 놓는다(모델 z - table_z)
        assert sk.last_place_angle_deg == pytest.approx((angle + az - ap["pick_az_deg"]) % 180.0, abs=0.5)  # 코드는 TCP 자리의 방위를 쓴다(물체 중심 방위와 0.1°대 차이)
    finally:
        ctl.close()


# ⑧ 놓을 칸이 모두 안 닿으면 ik_fail이고 집기 기억이 남는다(옛 방식으로 가지 않는다)
@pytest.mark.slow
def test_tilted_grip_with_no_reachable_slot_is_ik_fail_and_keeps_the_grip(pkin, pcfg):
    _d, ctl, sk = prig(pkin, pcfg, block_mm=56.0)
    events = []
    ctl.on_event = lambda kind, level, code, data: events.append(code)
    try:
        assert sk.pick(box(), [SLOT]).status == "ok" and sk.grip["approach"]["beta"] > 0
        tried = []
        sk._tilted_place_plan = lambda x, y, *a: tried.append((x, y))  # 모든 칸에서 안 풀린다
        r = sk.place("right")
        assert r.status == "ik_fail" and r.reason.startswith("tilted_place_unreachable")
        assert len(tried) == len(WS.zone_slots("right")) and sk.grip is not None and "ARM-PLACE-LEGACY" not in events
    finally:
        ctl.close()
