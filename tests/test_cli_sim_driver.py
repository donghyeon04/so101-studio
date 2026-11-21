from so101_studio.cli import make_driver
from so101_studio.core.config import load_robot


def test_cli_sim_driver_stops_the_gripper_at_the_object_width():
    # REQ-115: 없으면 병렬 쥠 확인이 CLI --sim 집기를 늘 no_object로 끝냈다(GUI는 REQ-114)
    assert make_driver(True, load_robot()).object_grip is True
