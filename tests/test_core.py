import json
import math

import pytest

from so101_studio.core import frames
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.events import EventBus
from so101_studio.core.logging import RunLogger
from so101_studio.core.types import ToolPose

BASE = frames.Pose2D(300, 50, 90)


def test_table_to_robot_known_point():
    # 로봇 원점 (300,50), yaw 90°: 로봇 x축이 작업대 y축을 향한다 → 작업대 (300,150)은 로봇 (100,0)
    x, y = frames.table_to_robot(300, 150, BASE)
    assert (x, y) == pytest.approx((100, 0), abs=1e-9)


@pytest.mark.parametrize("yaw", [0, 90, 37.5, -120])
def test_roundtrip(yaw):
    base = frames.Pose2D(120, 80, yaw)
    x, y = frames.robot_to_table(*frames.table_to_robot(410, 275, base), base)
    assert (x, y) == pytest.approx((410, 275))


def test_tool_pose_yaw_wraps_and_roundtrips():
    p = ToolPose(300, 150, 20, 170)
    r = frames.tool_table_to_robot(p, BASE)
    assert r.yaw_deg == pytest.approx(80)
    back = frames.tool_robot_to_table(r, BASE)
    assert (back.x_mm, back.y_mm, back.yaw_deg) == pytest.approx((300, 150, 170))
    assert frames._wrap_deg(190) == pytest.approx(-170)


def test_units():
    assert frames.mm_to_m(250) == 0.25 and frames.m_to_mm(0.25) == 250
    assert math.isclose(frames.mm_to_m(frames.m_to_mm(1.23)), 1.23)


def test_load_configs():
    robot = load_robot("config/robot.yaml")
    assert robot.port == "COM3" and robot.pid == {"p": 16, "i": 0, "d": 32}
    assert len(robot.poses["home"]) == 6
    ws = load_workspace("config/workspace.yaml")
    assert ws.base == BASE and ws.drop_zones["right"]["x_mm"] == 500


def test_unknown_key_is_rejected(tmp_path):
    f = tmp_path / "robot.yaml"
    f.write_text("port: COM3\ncalibration_id: x\nurdf: y\nprot: typo\n")
    from draccus.utils import DecodingError

    with pytest.raises(DecodingError):
        load_robot(f)


def test_event_bus():
    bus, got = EventBus(), []
    bus.subscribe("a", got.append)
    bus.publish("a", 1)
    bus.publish("b", 2)  # 구독자 없는 토픽은 조용히 무시
    assert got == [1]


def test_run_logger_writes_jsonl(tmp_path):
    log = RunLogger(tmp_path)
    log.event("cmd", ToolPose(1, 2, 3, 4))
    log.close()
    line = json.loads((log.dir / "events.jsonl").read_text(encoding="utf-8"))
    assert line["kind"] == "cmd" and line["data"]["x_mm"] == 1
    assert (log.dir / "scans").is_dir()
