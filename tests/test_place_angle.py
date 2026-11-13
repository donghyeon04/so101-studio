"""실물 2-2(11-13): 쥔 채 놓는 손의 방향(반경 방향)이 집을 때와 달라 놓은 물체 각도가 바뀐다. PLACE 기록에 놓은 뒤 물체 각도를 남긴다."""
import json

import numpy as np
import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", arm="so101_follower_01") if __import__("pathlib").Path("config/arms/so101_follower_01.yaml").exists() else load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)


@pytest.mark.slow
def test_place_event_records_the_object_angle_after_placing(tmp_path):
    block = float(np.interp(58, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        r = s.pick_and_place(DetectedObject("b", 300, 200, 0, 58, 58, 1.0, "manual", 39), zone="right")
        assert r.status == "ok", r
        sx, sy = WS.zone_slots("right")[0]  # 놓는 칸(배출 구역 중심이 아니라 첫 빈 칸)
        place_yaw = s.skills._place_yaw(sx, sy)
        angle = s.last_placed_angle_deg
        s.logger.flush()
        place = next(json.loads(x) for x in (s.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines() if json.loads(x).get("kind") == "place")["data"]
        # 그 각도로 다시 집으면 턱 축이 상자 변과 나란하다(정사각형은 90° 주기)
        again = s.skills._jaw_yaw(angle + 90.0)
        aligned = (again - 90.0 - angle) % 90.0
    finally:
        s.disconnect()
    assert place["place_yaw_deg"] == pytest.approx(place_yaw, abs=0.1) and place["placed_angle_deg"] == pytest.approx(angle, abs=0.1)
    assert angle == pytest.approx((0.0 + place_yaw - 90.0) % 180.0, abs=0.2) and angle == pytest.approx(113.7, abs=0.5)  # 집을 때 yaw 90, 놓을 때 약 23.7
    assert min(aligned, 90.0 - aligned) < 0.5
