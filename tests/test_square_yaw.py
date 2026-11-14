"""사용자 결정 B(11-14): 정사각형은 턱 축 후보 둘(각도+90, 각도) 가운데 반경 방향에 가까운 쪽을 고른다. 직사각형은 긴 변 법선을 유지한다."""
import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", arm="so101_follower_01") if __import__("pathlib").Path("config/arms/so101_follower_01.yaml").exists() else load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


@pytest.fixture
def sk():
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=30, log_root=None)
    assert s.connect()[0]
    yield s.skills
    s.disconnect()


def obj(angle, w=58.0, length=58.0, x=300.0, y=200.0):
    return DetectedObject("b", x, y, angle, w, length, 1.0, "manual", 39.0)


@pytest.mark.parametrize("angle, tilted", [(0, True), (10, True), (20, False), (30, False), (45, False), (60, False), (80, True), (90, True), (100, True)])
def test_square_axis_is_the_candidate_closer_to_radial(sk, angle, tilted):
    o = obj(angle)  # 정면 (300,200): 반경 방향은 작업대 yaw 90
    assert sk._tilt_ok(o, sk._pick_yaw(o)) is tilted


def test_square_at_90_uses_the_same_jaw_axis_as_at_0(sk):
    assert sk._pick_yaw(obj(90)) == pytest.approx(sk._pick_yaw(obj(0))) == pytest.approx(90.0)


def test_same_rule_at_a_point_off_the_front(sk):
    """(442.5,112.5): 반경 방향 yaw 약 23.7°. 정사각형 각도 23.7°(또는 113.7°)는 턱 축이 반경이 되어 기울일 수 있고, 0°는 아니다. B 이전에는 23.7°가 기울지 못했다."""
    x, y = 442.5, 112.5
    radial_yaw = sk._jaw_yaw(sk.kin.base.yaw_deg + float(np.degrees(np.arctan2(*reversed(__import__("so101_studio.core.frames", fromlist=["x"]).table_to_robot(x, y, sk.kin.base))))))
    assert abs(radial_yaw - 23.7) < 0.5
    assert sk._tilt_ok(obj(23.7, x=x, y=y), sk._pick_yaw(obj(23.7, x=x, y=y))) is True
    assert sk._tilt_ok(obj(113.7, x=x, y=y), sk._pick_yaw(obj(113.7, x=x, y=y))) is True
    assert sk._tilt_ok(obj(0, x=x, y=y), sk._pick_yaw(obj(0, x=x, y=y))) is False
    assert sk._tilt_ok(obj(23.7, x=x, y=y), sk._jaw_yaw(23.7 + 90.0)) is False  # B 이전


def test_square_at_45_is_unchanged_and_other_angles_never_get_worse(sk):
    for angle in range(0, 180, 5):
        o = obj(angle)
        old = sk._jaw_yaw(angle + 90.0)
        assert sk._radial_off(o.x_mm, o.y_mm, sk._pick_yaw(o)) <= sk._radial_off(o.x_mm, o.y_mm, old) + 1e-6
    assert sk._pick_yaw(obj(45)) == sk._jaw_yaw(135.0)  # 45°는 두 후보가 같은 만큼 벗어나 지금과 같다


@pytest.mark.parametrize("angle", [0, 30, 90, 120])
def test_rectangle_keeps_the_long_side_normal(sk, angle):
    o = obj(angle, w=58.0, length=100.0)
    assert sk._pick_yaw(o) == sk._jaw_yaw(angle + 90.0)


@pytest.mark.slow
def test_placed_angle_follows_the_chosen_yaw(tmp_path):
    """각도 90 정사각형: 고른 yaw(= 각도)로 집어 기울이고, 놓은 각도 = 집을 때 각도 + 놓을 때 yaw - 집을 때 yaw."""
    block = float(np.interp(58, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        o = obj(90)  # 정확히 반경일 때만 기울인 계획이 풀린다(5~15°만 벗어나도 손목 한계로 안 풀려 수직으로 집는다)
        yaw = s.skills._pick_yaw(o)
        assert s.pick_and_place(o, zone="right").status == "ok"
        assert s.skills.last_pick_tilted is True and s.skills.last_place_yaw_deg is not None
        want = (90.0 + s.skills.last_place_yaw_deg - yaw) % 180.0
        assert s.last_placed_angle_deg == pytest.approx(want, abs=0.1)
    finally:
        s.disconnect()
