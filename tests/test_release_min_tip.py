"""실물 2-2a(11-14): 낮은 높이(15)로 알려 7mm에 쥔 상자를 놓을 때 턱 끝이 바닥을 긁었다. 놓는 순간 낮은 턱 끝은 tip_floor_mm + 8(13mm) 아래로 내리지 않는다."""
import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", arm="so101_follower_01") if __import__("pathlib").Path("config/arms/so101_follower_01.yaml").exists() else load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


def low_tip(sk, st):
    p = sk.kin.fk(st)
    f, m = sk.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, st.gripper)
    return p.z_mm + float(min(f[2], m[2])) - CFG.table_z_mm


@pytest.mark.slow
@pytest.mark.parametrize("obj_h", [15, 39])
def test_release_keeps_the_low_jaw_tip_above_the_floor_margin(tmp_path, obj_h):
    block = float(np.interp(55.5, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        sk = s.skills
        ev, cap = [], []
        s.ctl.on_event = lambda k, lv, c, d: ev.append(c)
        orig = sk._run
        sk._run = lambda trajs: (cap.append(list(trajs)), orig(trajs))[1]
        assert sk.pick(DetectedObject("b", 300, 275, 45, 58, 58, 1.0, "manual", obj_h)).status == "ok"
        tip_h = sk.grip["tip_h"]
        n0 = len(cap)
        assert sk.place_at(300.0, 275.0).status == "ok"
        lows = [low_tip(sk, JointState(*pt)) for tl in cap[n0:] for tr in tl if getattr(tr, "pts", None) is not None for pt in tr.pts]
    finally:
        s.disconnect()
    floor = CFG.tip_floor_mm + 8.0
    assert min(lows) >= min(floor, tip_h + CFG.place_drop_mm) - 0.3
    assert ("ARM-PLACE-RELEASE-RAISED" in ev) == (tip_h + CFG.place_drop_mm < floor)  # 낮게 쥔 물체(7mm)만 올려 놓는다, 39mm 물체(20mm)는 그대로
    if obj_h == 15:
        assert min(lows) >= floor - 0.3
