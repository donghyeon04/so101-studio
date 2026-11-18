"""실물 2-2(11-13): 쥐었는데 닫는 동안 지연이 문턱(4)에 못 미쳐(3.9) contact false -> no_object로 올라가 접근 높이에서 열어 떨어뜨림."""
import json

import pytest

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml", arm="so101_follower_01") if __import__("pathlib").Path("config/arms/so101_follower_01.yaml").exists() else load_robot("config/robot.yaml")
WS = load_workspace("tests/workspace_600x400.yaml")
OBJ = DetectedObject("b", 300, 200, 0, 58, 58, 1.0, "manual", 39)


def pick(tmp_path, lag):
    """닫는 목표(v_end)보다 lag만큼 앞에서 그리퍼가 막히는 상황(lag None이면 막힘 없음)."""
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    try:
        v_end = s.skills.gripper_for_width(max(OBJ.width_mm - 10.0, min(mm for _, mm in CFG.gripper_width_mm)))
        s.driver.grip_block = None if lag is None else v_end + lag
        r = s.skills.pick(OBJ)
        s.logger.flush()
        codes = [json.loads(x).get("code") for x in (s.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]
        return r, s.skills.grip, codes
    finally:
        s.disconnect()


@pytest.mark.slow
@pytest.mark.parametrize("lag", [3.9, 2.5])
def test_blocked_just_below_the_contact_threshold_is_still_a_grasp(tmp_path, lag):
    r, grip, codes = pick(tmp_path, lag)
    assert r.status == "ok" and grip and grip["obj_h"] == 39.0 and "ARM-PICK-LATE-CONTACT" in codes


@pytest.mark.slow
@pytest.mark.parametrize("lag", [None, 1.0])
def test_closing_on_nothing_is_still_no_object(tmp_path, lag):
    r, grip, codes = pick(tmp_path, lag)
    assert r.status == "no_object" and grip is None and "ARM-PICK-LATE-CONTACT" not in codes
