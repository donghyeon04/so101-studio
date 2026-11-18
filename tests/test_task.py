import types
from dataclasses import replace

import numpy as np
import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.skills import Skills
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.events import EventBus
from so101_studio.core.types import DetectedObject, MotionResult, ToolPose
from so101_studio.perception.manual import ManualSource
from so101_studio.task.policy import choose, reachable
from so101_studio.task.runner import TaskRunner

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
OK = MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))


def obj(i, x, y, angle=0, w=30.0):
    return DetectedObject(f"obj-{i}", x, y, angle, w, 100.0, 1.0, "manual")


class ListSource:
    name = "list"

    def __init__(self, objs):
        self.objs, self.done = list(objs), set()

    def connect(self): ...
    def calibrate(self): return {}
    def close(self): ...

    def scan(self):
        from so101_studio.core.types import ScanResult

        return ScanResult([o for o in self.objs if o.id not in self.done], 0.0)

    def notify_picked(self, object_id):
        self.done.add(object_id)


class StubSkills:
    """동작은 하지 않고 결과만 돌려준다: 상태 머신 논리를 빠르게 시험한다."""

    def __init__(self, pick_status="ok"):
        self.ctl = types.SimpleNamespace(state="idle", stop=lambda: None)
        self.pick_status, self.calls = pick_status, []

    def park(self): self.calls.append("park"); return OK
    def place(self, zone): self.calls.append("place"); return OK

    def place_targets(self, zone=None, at=None): return None

    def pick(self, o, place_targets=None):
        self.calls.append(f"pick:{o.id}")
        return OK if self.pick_status == "ok" else MotionResult(self.pick_status, 0.0, ToolPose(0, 0, 0, 0))


def run(runner, **kw):
    seen = []
    runner.bus.subscribe("task", lambda e: seen.append(e))
    return runner.run(**kw), seen


# ---- S1: ManualSource
def test_manual_source_returns_the_configured_objects():
    src = ManualSource("config/manual_objects.yaml")
    scan = src.scan()
    assert [o.id for o in scan.objects] == ["obj-1", "obj-2", "obj-3"] and scan.duration_s >= 0
    assert all(o.source == "manual" and o.confidence == 1.0 for o in scan.objects)
    assert src.calibrate() == {"objects": 3}


def test_manual_source_drops_picked_objects_from_the_next_scan():
    src = ManualSource("config/manual_objects.yaml")
    src.notify_picked("obj-2")
    assert [o.id for o in src.scan().objects] == ["obj-1", "obj-3"]


# ---- S2: policy
def test_policy_skips_unreachable_and_prefers_the_nearest():
    near, far, out = obj(1, 300, 170), obj(2, 300, 250), obj(3, 300, 600)
    assert reachable(near, WS) and not reachable(out, WS)
    assert choose([far, out, near], WS).id == "obj-1"
    assert choose([out], WS) is None
    edge = obj(4, 300, 330)  # 반경 280: 집기 범위(250) 밖, 확인 전 후보 여유(+60) 안 (REQ-065)
    assert not reachable(edge, WS) and reachable(edge, WS, unconfirmed=True) and choose([edge], WS, unconfirmed=True).id == "obj-4"
    assert not reachable(obj(5, 300, 100), WS, unconfirmed=True)  # 최소 반경(90) 쪽은 넓히지 않는다


# ---- S2: state machine (fast, stubbed arm)
def test_runner_picks_everything_nearest_first_and_ends():
    src = ListSource([obj(1, 300, 250), obj(2, 300, 170), obj(3, 300, 200)])
    sk = StubSkills()
    res, seen = run(TaskRunner(sk, src, WS, EventBus()))
    assert res["end"] == "no_objects" and res["picked"] == ["obj-2", "obj-3", "obj-1"]
    states = [e.state for e in seen]
    assert states[:6] == ["PREPARE", "SCAN", "DECIDE", "PICK", "PLACE", "PLACE"] and states[-1] == "DONE"
    assert sk.calls.count("park") == 4  # 물체 하나를 옮길 때마다 대기 자세로 가서 다시 스캔


def test_runner_ignores_unreachable_objects():
    res, _ = run(TaskRunner(StubSkills(), ListSource([obj(1, 300, 600)]), WS, EventBus()))
    assert res["end"] == "no_objects" and res["picked"] == []


def test_runner_retries_once_then_gives_up_after_three_failed_cycles():
    sk = StubSkills(pick_status="no_object")
    res, _ = run(TaskRunner(sk, ListSource([obj(1, 300, 170)]), WS, EventBus()))
    assert res["end"] == "repeated_failure" and res["failed"] == ["obj-1"] * 3
    assert sk.calls.count("pick:obj-1") == 6  # 사이클마다 재시도 1회


def test_dry_run_treats_no_object_as_success():
    src = ListSource([obj(1, 300, 170), obj(2, 300, 200)])
    res, _ = run(TaskRunner(StubSkills(pick_status="no_object"), src, WS, EventBus(), accept_no_object=True))
    assert res["end"] == "no_objects" and res["picked"] == ["obj-1", "obj-2"] and res["failed"] == []


def test_runner_stops_on_request():
    sk = StubSkills()
    runner = TaskRunner(sk, ListSource([obj(1, 300, 170), obj(2, 300, 200)]), WS, EventBus())
    runner.bus.subscribe("task", lambda e: runner.stop() if e.state == "PICK" else None)
    res = runner.run()
    assert res["end"] == "STOPPED" and res["picked"] == []


def test_step_mode_waits_at_every_state_change():
    runner = TaskRunner(StubSkills(), ListSource([obj(1, 300, 170)]), WS, EventBus())
    waits = []
    runner.wait_step = lambda: waits.append(runner.state)
    res, _ = run(runner, step_mode=True)
    assert res["picked"] == ["obj-1"] and waits[:5] == ["PREPARE", "SCAN", "DECIDE", "PICK", "PLACE"]


def test_cycle_limit_ends_the_task():
    from so101_studio.task import runner as mod

    class Sticky(ListSource):
        def notify_picked(self, object_id):  # 집어도 스캔에 계속 남는 경우
            pass

    old, mod.MAX_CYCLES = mod.MAX_CYCLES, 3
    try:
        res, _ = run(TaskRunner(StubSkills(), Sticky([obj(1, 300, 170)]), WS, EventBus()))
    finally:
        mod.MAX_CYCLES = old
    assert res["end"] == "cycle_limit" and res["cycles"] == 3


# ---- S2 완료 기준: SimDriver + ManualSource로 물체 3개 자동 사이클이 끝까지 돈다
@pytest.mark.slow
def test_full_cycle_in_simulation_with_three_objects():
    tbl = sorted((mm, v) for v, mm in CFG.gripper_width_mm)
    block = float(np.interp(30.0, [p[0] for p in tbl], [p[1] for p in tbl]))  # 폭 30mm 물체가 있는 것처럼
    d = SimDriver(CFG.poses["home"], grip_block=block)
    kin = Kinematics(CFG.urdf, WS.base)
    ctl = MotionController(d, kin, Safety(CFG.joint_limits_deg, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm), CFG.control_hz)
    sk = Skills(ctl, kin, replace(CFG), WS, speed_pct=100)
    res, seen = run(TaskRunner(sk, ManualSource("config/manual_objects.yaml"), WS, EventBus()))
    ctl.close()
    assert res["end"] == "no_objects", res
    assert sorted(res["picked"]) == ["obj-1", "obj-2", "obj-3"] and res["failed"] == []
    placed = [e for e in seen if e.state == "PLACE" and e.result]
    assert [e.result for e in placed] == ["ok"] * 3
    assert all({"x_mm", "y_mm", "place_yaw_deg", "placed_angle_deg"} <= set(e.detail) for e in placed)  # 놓인 물체를 다시 집을 때 쓴다
    pytest.approx(0)
