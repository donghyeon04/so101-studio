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
@pytest.mark.parametrize("gripper", ["single_jaw", "parallel"])
def test_manual_run_places_all_three_objects_in_mujoco_without_touching(tmp_path, gripper):
    """기본 3개(100×30)를 끝까지 옮기고, 그 사이 물체끼리 닿지 않고 놓인 물체가 기울지 않는다(칸이 물체보다 좁던 문제, 실물 11-14)."""
    pytest.importorskip("mujoco")
    import math
    import threading
    import time

    from so101_studio.arm.sim_driver import make_sim_driver
    from so101_studio.core.config import load_robot, load_workspace
    from so101_studio.perception.manual import ManualSource
    from so101_studio.service.arm_service import ArmService
    from so101_studio.service.studio import Studio

    local = tmp_path / "local.yaml"
    local.write_text(f"gripper: {gripper}\n", encoding="utf-8")
    cfg, ws = load_robot("config/robot.yaml", local_path=local), load_workspace("config/workspace.yaml")  # ManualSource는 config/manual_objects.yaml(실물 배치, 결정 0067)을 읽는다
    drv = make_sim_driver(cfg, ws, None)
    svc = ArmService(drv, cfg, ws, log_root=str(tmp_path))
    ok, msg = svc.connect()
    assert ok, msg
    touches, stop = set(), threading.Event()
    (bx, by, br), = ws.ignore_zones()  # 배출 통(결정 0067): 통 안에서는 떨어뜨린 물체끼리 닿는다(쌓임). 통 밖의 닿음만 본다

    def sample():
        while not stop.is_set():
            for c in drv.sim_state()["contacts"]:
                if c["a"].startswith("obj:") and c["b"].startswith("obj:") and math.hypot(c["pos_mm"][0] - bx, c["pos_mm"][1] - by) > br:
                    touches.add(tuple(sorted((c["a"], c["b"]))))
            time.sleep(0.02)

    t = threading.Thread(target=sample, daemon=True)
    t.start()
    try:
        res = Studio(svc, ManualSource()).start(blocking=True)
    finally:
        stop.set()
        t.join()
    assert res["end"] == "no_objects" and sorted(res["picked"]) == ["obj-1", "obj-2", "obj-3"] and res["failed"] == [], res
    assert not touches, touches  # 통에 떨어뜨린 물체는 쌓여 기울 수 있어 기울기는 보지 않는다


def test_drop_slot_next_to_a_remaining_object_is_tried_last():
    """쥔 물체(40×40)가 남은 물체(반지름 60)에서 80mm 안인 칸은 맨 뒤로: 442.5,187.5는 (400,250)에서 75.6mm라 뒤로, 517.5,112.5가 먼저(칸에 들어가는 크기라 칸 격자를 쓴다)."""
    from so101_studio.arm.skills import Skills
    from so101_studio.core.config import load_workspace
    from so101_studio.core.types import MotionResult, ToolPose
    ws, tried = load_workspace("tests/workspace_600x400.yaml"), []

    def place_at(x, y):
        tried.append((x, y))
        return MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))

    fake = SimpleNamespace(ws=ws, placed=[(442.5, 112.5, 20.0)], placed_size={(442.5, 112.5): (40.0, 40.0)}, placed_angle={}, known_objects=[(400, 250, 60.0)],
                           grip={"obj_w": 40.0, "obj_l": 40.0}, place_at=place_at, ctl=None, _place_yaw=lambda *_: 0.0)
    fake._fits = lambda p: Skills._fits(fake, p)
    assert Skills.place(fake, "right").status == "ok" and tried == [(517.5, 112.5)]
    tried.clear()
    fake.placed, fake.placed_size, fake.known_objects = [], {}, []  # 남은 물체가 없으면 예전과 같이 가까운 칸부터
    Skills.place(fake, "right")
    assert tried == [(442.5, 112.5)]


def test_carry_lift_clears_the_tallest_remaining_object():
    from so101_studio.arm.skills import Skills
    fake = SimpleNamespace(cfg=SimpleNamespace(approach_height_mm=40.0), known_objects=[], known_height={})
    assert Skills._carry_lift(fake) == 40.0  # 남은 물체가 없으면 예전 높이
    fake.known_objects, fake.known_height = [(400, 250, 50.0), (300, 190, 50.0)], {(400, 250): 40.0, (300, 190): 55.0}
    assert Skills._carry_lift(fake) == 65.0  # 가장 높은 55 + 여유 10
    fake.known_height = {}  # 높이를 모르면 40으로 보고
    assert Skills._carry_lift(fake) == 50.0


def test_rects_overlap_uses_rotation_and_margin():
    from so101_studio.arm.skills import _rects_overlap
    a = (0.0, 0.0, 30.0, 100.0, 90.0)  # 길이 100이 y 방향
    assert _rects_overlap(a, (0.0, 75.0, 30.0, 100.0, 90.0), 0.0)  # 칸 75 간격이면 길이 방향으로 겹친다(실물 11-14)
    assert not _rects_overlap(a, (60.0, 0.0, 30.0, 100.0, 90.0), 25.0)  # 옆으로 60이면 폭 30 + 간격 30
    assert _rects_overlap(a, (50.0, 0.0, 30.0, 100.0, 90.0), 25.0)  # 50은 간격 20이라 부족
    assert not _rects_overlap(a, (55.0, 0.0, 30.0, 100.0, 90.0), 25.0)  # 간격 25 딱은 허용


def test_place_uses_fine_candidates_when_the_object_does_not_fit_a_slot():
    """길이 100 물체는 칸 75에 안 들어가 촘촘한 후보에서 놓인 물체와 겹치지 않는 자리를 고른다."""
    from so101_studio.arm.skills import Skills
    from so101_studio.core.config import load_workspace
    from so101_studio.core.types import MotionResult, ToolPose
    ws, tried = load_workspace("tests/workspace_600x400.yaml"), []

    def place_at(x, y):
        tried.append((x, y))
        return MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))

    grip = {"obj_w": 30.0, "obj_l": 100.0, "angle": 0.0, "yaw": 90.0}
    fake = SimpleNamespace(ws=ws, placed=[(430.0, 110.0, 15.0)], placed_size={(430.0, 110.0): (30.0, 100.0)}, placed_angle={(430.0, 110.0): 90.0}, known_objects=[], grip=grip,
                           place_at=place_at, ctl=None, _place_yaw=lambda *_: 0.0)
    fake._fits = lambda p: Skills._fits(fake, p)
    assert Skills.place(fake, "right").status == "ok"
    x, y = tried[0]
    assert (x, y) == (470.0, 110.0)  # 폭 30 + 간격 10 = 40만큼 옆, 기본 칸(75 간격)이 아니라 촘촘한 후보
    fake.placed, fake.placed_size, fake.placed_angle = [(430.0, 110.0, 15.0), (485.0, 110.0, 15.0)], {(430.0, 110.0): (30.0, 100.0), (485.0, 110.0): (30.0, 100.0)}, {(430.0, 110.0): 90.0, (485.0, 110.0): 90.0}
