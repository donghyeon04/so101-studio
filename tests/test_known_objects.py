"""CLI run(작업 실행)도 GUI처럼 남은 물체를 팔에 알린다: 놓는 칸 옆 물체에 부딪혀 FAULT 나던 3개 순서가 MuJoCo에서 끝까지 돈다 (실물 시연 11-14)."""
from types import SimpleNamespace

import pytest

from so101_studio.core.types import DetectedObject
from so101_studio.task.runner import TaskRunner


def test_runner_tells_the_arm_the_scanned_objects_with_the_gui_radius():
    sk = SimpleNamespace(known_objects=[])
    objs = [DetectedObject(id="a", x_mm=400, y_mm=250, angle_deg=90, width_mm=30, length_mm=100, confidence=1.0, source="manual"),
            DetectedObject(id="b", x_mm=170, y_mm=170, angle_deg=45, width_mm=30, length_mm=0, confidence=1.0, source="manual")]
    TaskRunner._tell_known(SimpleNamespace(skills=sk), objs)
    assert sk.known_objects == [(400, 250, 50.0), (170, 170, 15.0)]
    TaskRunner._tell_known(SimpleNamespace(skills=SimpleNamespace()), objs)  # known_objects가 없는 가짜 팔은 건드리지 않는다


@pytest.mark.slow
def test_manual_run_places_all_three_objects_in_mujoco(tmp_path):
    pytest.importorskip("mujoco")
    from so101_studio.arm.sim_driver import make_sim_driver
    from so101_studio.core.config import load_robot, load_workspace
    from so101_studio.perception.manual import ManualSource
    from so101_studio.service.arm_service import ArmService
    from so101_studio.service.studio import Studio

    cfg, ws = load_robot("config/robot.yaml", local_path="config/no-local.yaml"), load_workspace("config/workspace.yaml")
    svc = ArmService(make_sim_driver(cfg, ws, None), cfg, ws, log_root=str(tmp_path))
    ok, msg = svc.connect()
    assert ok, msg
    res = Studio(svc, ManualSource()).start(blocking=True)
    assert res["end"] == "no_objects" and sorted(res["picked"]) == ["obj-1", "obj-2", "obj-3"] and res["failed"] == [], res


def test_drop_slot_next_to_a_remaining_object_is_tried_last():
    """쥔 물체(길이 100)가 남은 물체(반지름 50)에서 100mm 안인 칸은 맨 뒤로: 442.5,187.5는 obj-3(400,250)에서 75mm라 뒤로, 517.5,112.5가 먼저."""
    from so101_studio.arm.skills import Skills
    from so101_studio.core.config import load_workspace
    from so101_studio.core.types import MotionResult, ToolPose
    ws, tried = load_workspace("config/workspace.yaml"), []

    def place_at(x, y):
        tried.append((x, y))
        return MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))

    fake = SimpleNamespace(ws=ws, placed=[(442.5, 112.5, 15.0)], known_objects=[(400, 250, 50.0)], grip={"obj_w": 30.0, "obj_l": 100.0}, place_at=place_at, ctl=None)
    assert Skills.place(fake, "right").status == "ok" and tried == [(517.5, 112.5)]
    tried.clear()
    fake.placed, fake.known_objects = [], []  # 남은 물체가 없으면 예전과 같이 가까운 칸부터
    Skills.place(fake, "right")
    assert tried == [(442.5, 112.5)]
