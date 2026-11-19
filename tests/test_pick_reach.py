"""REQ-090(H-2 뒤): workspace.yaml reach_mm(집기 범위)의 양 끝에서 수직 하강 집기 계획이 두 그리퍼 모두 풀린다(table_z -2.9 CAD 값 기준)."""
import math
from pathlib import Path

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

WS = load_workspace("config/workspace.yaml")


@pytest.mark.slow
@pytest.mark.parametrize("gripper", ["parallel", "single_jaw"])
def test_reach_ends_are_pickable_for_both_grippers(tmp_path, gripper):
    local = Path(tmp_path) / "local.yaml"
    local.write_text(f"gripper: {gripper}\n", encoding="utf-8")
    cfg = load_robot("config/robot.yaml", local_path=local)
    s = ArmService(SimDriver(cfg.poses["home"]), cfg, WS, log_root=None)
    assert s.connect()[0]
    try:
        sk, b = s.skills, WS.base
        for r in (WS.reach_mm["min"], 220.0, WS.reach_mm["max"]):  # 220 = 시연 배치 바깥(결정 0067)
            for az in (45.0, 90.0, 135.0):
                for h in (0.0, 40.0):
                    o = DetectedObject("t", b.x_mm + r * math.cos(math.radians(az)), b.y_mm + r * math.sin(math.radians(az)), 0.0, 30.0, 30.0, 1.0, "manual", h)
                    yaw = sk._pick_yaw(o)
                    m = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
                    plan = sk._plan_pick(o, yaw, m, sk._tilt_ok(o, yaw))
                    if not hasattr(plan, "down") and h > 0:
                        plan = sk._plan_pick(o, yaw, m, False)  # 기울인 계획이 안 되면 높이를 알 때 수직으로 (REQ-055)
                    assert hasattr(plan, "down"), (gripper, r, az, h, plan)
    finally:
        s.disconnect()
