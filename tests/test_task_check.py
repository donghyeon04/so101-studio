"""REQ-025: 작업 실행의 CHECK·RESCAN·prefetch·note_failure·give_up (스텁으로 논리를, 마지막 시험은 --sim 팔과 FakeESP32로 끝까지)."""
import dataclasses
import math
import threading
import time
import types

from test_task import OK, WS, StubSkills, run

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot
from so101_studio.core.events import EventBus
from so101_studio.core.types import DetectedObject, MotionResult, ScanResult, ToolPose
from so101_studio.perception.base import PathSegment
from so101_studio.perception.ultrasonic.model import Box
from so101_studio.service.arm_service import ArmService
from so101_studio.service.studio import Studio
from so101_studio.service.ultrasonic import make_ultrasonic_source
from so101_studio.task.runner import TaskRunner

OFFSET = {"x": 0.0, "y": 30.0, "z": 55.0}


def cand(i, x=300.0, y=200.0):
    return DetectedObject(f"us-{i}", x, y, None, 0.0, 0.0, 0.2, "ultrasonic")


class StubProbe:
    def __init__(self, src, c):
        self.src, self.c, self.sent, self.fed, self.aborted = src, c, 0, [], False

    def next_segment(self):
        self.sent += 1
        return PathSegment([(0.0, 0.0), (10.0, 0.0)], 55.0, 100.0) if self.sent == 1 else None

    def feed(self, poses): self.fed.append(poses)
    def abort(self): self.aborted = True

    @property
    def status(self): return "ok" if self.src.exists.get(self.c.id, True) else "empty"

    def result(self):
        return DetectedObject(self.c.id, self.c.x_mm, self.c.y_mm, 10.0, 30.0, 100.0, 0.9, "ultrasonic") if self.status == "ok" else None


class StubSource:
    """scans: scan()이 차례로 돌려줄 후보 목록, rescans: rescan()이 차례로 돌려줄 목록."""
    name = "stub-us"
    cfg = types.SimpleNamespace(radar=types.SimpleNamespace(rescan_home=True), arm_sensor_offset_mm=OFFSET)
    bus = None

    def __init__(self, scans, rescans=(), exists=None, give_up_after=None):
        self.scans, self.rescans, self.exists, self.give_up_after = list(scans), list(rescans), exists or {}, give_up_after
        self.calls, self.failures, self.probes, self.prefetch_fn = [], [], [], None

    def connect(self): ...
    def calibrate(self): return {}
    def close(self): ...
    def begin_cycle(self): self.calls.append("begin_cycle")
    def notify_picked(self, oid): self.calls.append(f"picked:{oid}")
    def scan(self): self.calls.append("scan"); return ScanResult(self.scans.pop(0) if self.scans else [], 0.5)
    def rescan(self): self.calls.append("rescan"); return ScanResult(self.rescans.pop(0) if self.rescans else [], 0.5)
    def start_check(self, c): self.calls.append(f"check:{c.id}"); p = StubProbe(self, c); self.probes.append(p); return p
    def prefetch(self, fn): self.calls.append("prefetch"); self.prefetch_fn = fn
    def note_failure(self, x, y): self.failures.append((x, y))

    @property
    def give_up(self): return self.give_up_after is not None and len(self.failures) >= self.give_up_after


class StubArm:
    def __init__(self, status="ok", on_follow=None):
        self.ctl, self.status, self.on_follow, self.offset, self.paths = types.SimpleNamespace(trace=None), status, on_follow, None, []
        self.last_path = None

    def set_sensor_offset(self, off): self.offset = off

    def arm_links(self, q=None): return [(1.0, 2.0)]


    def follow_path(self, pts, z, v, yaw):
        self.paths.append((pts, z, v, yaw))
        if self.on_follow:
            self.on_follow()
        self.last_path = MotionResult(self.status, 0.0, ToolPose(0, 0, 0, 0))
        return [] if self.status != "ok" else [object()]


class UsSkills(StubSkills):
    def radar_clear(self): self.calls.append("radar_clear"); return OK


def runner(src, arm=None, skills=None, **kw):
    return TaskRunner(skills or UsSkills(), src, WS, EventBus(), arm=arm or StubArm(), **kw)


def states(seen): return [(e.state, e.object_id, e.result) for e in seen]


def test_check_confirms_then_picks_with_prefetch_and_offset():
    src, arm = StubSource([[cand(1)], []]), StubArm()
    res, seen = run(runner(src, arm, UsSkills()))
    assert arm.offset == OFFSET and src.calls[0] == "begin_cycle"
    assert [s for s in states(seen) if s[0] in ("CHECK", "PICK", "PLACE")] == [
        ("CHECK", "us-1", None), ("CHECK", "us-1", "ok"), ("PICK", "us-1", None), ("PLACE", "us-1", None), ("PLACE", "us-1", "ok")]
    assert arm.paths == [([(0.0, 0.0), (10.0, 0.0)], 55.0, 100.0, None)] and len(src.probes[0].fed) == 1 and src.probes[0].fed[0]
    assert src.calls.index("prefetch") < src.calls.index("picked:us-1") and callable(src.prefetch_fn)
    assert src.prefetch_fn(0) == [(1.0, 2.0)]  # 기록이 비어도(틱 전) 지금 자세로 답한다
    assert res["picked"] == ["us-1"] and arm.ctl.trace is None  # 끝나면 틱 기록을 뗀다


def test_empty_check_drops_candidate_and_redecides_without_rescan():
    src = StubSource([[cand(1, 300, 190), cand(2, 300, 230)], []], exists={"us-1": False})
    res, seen = run(runner(src))
    assert res["picked"] == ["us-2"]
    assert src.calls.count("scan") == 2 and "rescan" not in src.calls[: src.calls.index("check:us-2")]
    assert ("CHECK", "us-1", "empty") in states(seen)


def test_rescan_when_no_candidate_once_then_done():
    src, sk = StubSource([[]], rescans=[[]]), UsSkills()
    res, seen = run(runner(src, skills=sk))
    assert sk.calls == ["radar_clear", "radar_clear"] and "park" not in sk.calls  # 첫 사이클 PREPARE와 RESCAN 모두 radar_clear (결정 0039)
    assert res["end"] == "no_objects" and src.calls.count("rescan") == 1
    assert [s[0] for s in states(seen)].count("RESCAN") == 1


def test_rescan_finds_candidate_then_may_rescan_again_later():
    src = StubSource([[], []], rescans=[[cand(1)], []])
    res, _ = run(runner(src))
    assert res["picked"] == ["us-1"] and src.calls.count("rescan") == 2  # 찾은 뒤에는 다시 할 수 있다


def test_rescan_off_goes_straight_to_done():
    src = StubSource([[]])
    src.cfg = types.SimpleNamespace(radar=types.SimpleNamespace(rescan_home=False), arm_sensor_offset_mm=OFFSET)
    res, _ = run(runner(src))
    assert res["end"] == "no_objects" and "rescan" not in src.calls


def test_pick_failure_calls_note_failure_and_give_up_ends_run():
    src = StubSource([[cand(1)], [cand(2)], [cand(3)], [cand(4)]], give_up_after=2)
    res, seen = run(runner(src, skills=UsSkills(pick_status="no_object")))
    assert res["end"] == "give_up" and len(src.failures) == 2 and src.failures[0] == (300.0, 200.0)
    assert seen[-1].state == "DONE" and seen[-1].result == "give_up"


def test_unreachable_segment_feeds_empty_and_goes_on():
    src, arm = StubSource([[cand(1)], []]), StubArm(status="out_of_bounds")
    run(runner(src, arm))
    assert src.probes[0].fed == [[]] and not src.probes[0].aborted


def test_stop_during_check_aborts_probe_and_ends_stopped():
    src, arm = StubSource([[cand(1)]]), StubArm(status="stopped")
    r = runner(src, arm)
    arm.on_follow = r.stop
    res, seen = run(r)
    assert res["end"] == "STOPPED" and src.probes[0].aborted and seen[-1].state == "STOPPED"
    assert "picked:us-1" not in src.calls


def test_fault_during_check_aborts_probe_and_ends_fault():
    src, arm = StubSource([[cand(1)]]), StubArm(status="fault")
    res, seen = run(runner(src, arm))
    assert res["end"] == "FAULT" and src.probes[0].aborted and seen[-1].state == "FAULT"


def test_exception_in_check_aborts_probe_and_propagates():
    src, arm = StubSource([[cand(1)]]), StubArm()
    arm.on_follow = lambda: (_ for _ in ()).throw(RuntimeError("읽기 실패"))
    r = runner(src, arm)
    try:
        run(r)
    except RuntimeError:
        pass
    else:
        raise AssertionError
    assert src.probes[0].aborted and arm.ctl.trace is None


def test_source_without_check_runs_as_before():
    from test_task import ListSource, obj

    sk = StubSkills()
    res, seen = run(TaskRunner(sk, ListSource([obj(1, 300, 190)]), WS, EventBus()))
    assert res["picked"] == ["obj-1"] and not {"CHECK", "RESCAN"} & {e.state for e in seen}


# ---- --sim 팔 + FakeESP32 (실제 시간으로 돈다: 레이더 회전과 팔 이동)
CFG = load_robot("config/robot.yaml")
SPREAD = [(420, 190, 0), (330, 250, 90), (330, 150, 0)]  # 팔 원점(300, 50) 기준. 대기 자세 팔 끝(300, 269)이 레이더 정면을 가려서 가운데(방위 90°) 물체는 못 찾는다: 비켜 둔다


class ReadFails(SimDriver):
    fail = False

    def read_joints(self):
        if self.fail:
            raise ConnectionError("read timeout")
        return super().read_joints()


def sim_studio(tmp_path, driver=SimDriver, spread=SPREAD):
    boxes = [Box(x, y, a, 30, 100) for x, y, a in spread]
    arm = ArmService(driver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert arm.connect()[0]
    src = make_ultrasonic_source(arm, sim=True, boxes=boxes)
    return Studio(arm, src, dry_run=True), src, arm


def test_sim_three_objects_end_to_end(tmp_path):
    st, src, arm = sim_studio(tmp_path)
    seen, us = [], []
    st.bus.subscribe("task", seen.append)
    for t in ("us.candidates", "us.check_path", "us.check_result"):
        st.bus.subscribe(t, lambda e, t=t: us.append(t))
    res = st.start(blocking=True)
    assert res["end"] in ("no_objects", "give_up") and len(res["picked"]) == 3, res
    kinds = {e.state for e in seen}
    assert {"CHECK", "PICK", "PLACE", "DONE"} <= kinds
    assert [e.state for e in seen].count("PREPARE") == 1  # radar_clear 이동은 첫 사이클 한 번뿐(끝의 RESCAN은 따로)
    assert {"us.candidates", "us.check_path", "us.check_result"} <= set(us)  # source에 bus가 이어졌다
    assert not src.esp.boxes  # 집어 간 물체는 가상 작업대에서도 사라졌다
    rows = [r for r in st.summary.to_dict(res)["per_object"] if r.get("check") == "ok"]  # 확인에 실패한 후보 행도 있을 수 있다
    assert len(rows) == 3 and all(r["check_s"] > 0 and r["radar_s"] > 0 for r in rows)
    arm.disconnect()


def _in_check(arm):
    end = time.time() + 60
    while arm.ctl.tap is None and time.time() < end:
        time.sleep(0.01)
    time.sleep(0.2)


def test_sim_stop_during_check_sends_stop_to_esp32(tmp_path):
    st, src, arm = sim_studio(tmp_path)
    th = threading.Thread(target=lambda: st.start(blocking=False) or None)
    th.start(); th.join()
    _in_check(arm)
    st.stop()
    st._thread.join(30)
    assert st.result["end"] == "STOPPED" and src.esp._stream is None and not src._checking
    arm.disconnect()


def test_sim_read_failure_during_check_faults_and_stops_esp32(tmp_path):
    st, src, arm = sim_studio(tmp_path, ReadFails)
    st.start(blocking=False)
    _in_check(arm)
    arm.driver.fail = True
    st._thread.join(30)
    assert st.result["end"] == "FAULT" and src.esp._stream is None and not src._checking
    arm.driver.fail = False
    arm.disconnect()


def test_sim_front_object_is_found_by_first_scan_with_radar_clear(tmp_path):
    """레이더 정면(방위 90°, 앞 200mm) 물체: park 자세의 팔 끝(300,269)에는 가려지지만 radar_clear에서는 첫 SCAN에서 후보로 나온다 (결정 0039)."""
    st, src, arm = sim_studio(tmp_path, spread=[(300, 200, 90)])
    seen = []
    st.bus.subscribe("task", seen.append)
    first = []
    st.bus.subscribe("us.candidates", lambda e: first.append(e) if not first else None)
    res = st.start(blocking=True)
    assert len(first[0]["objects"]) == 1 and res["picked"], res
    assert not src.esp.boxes
    arm.disconnect()


def test_radar_clear_only_on_first_cycle_rescan_and_after_pick_failure():
    # 물체 3개: PLACE 뒤에는 PREPARE 없이 바로 SCAN. radar_clear는 첫 사이클과 끝의 RESCAN 두 번뿐
    sk = UsSkills()
    src = StubSource([[cand(1)], [cand(2)], [cand(3)], []], rescans=[[]])
    res, seen = run(runner(src, skills=sk))
    assert res["picked"] == ["us-1", "us-2", "us-3"] and sk.calls.count("radar_clear") == 2
    assert [e.state for e in seen].count("PREPARE") == 1
    first_place = sk.calls.index("place")
    assert "radar_clear" not in sk.calls[first_place:sk.calls.index("pick:us-3")]  # 놓은 뒤 다음 후보로 바로 간다


def test_pick_failure_returns_to_radar_clear_before_next_scan():
    sk = UsSkills(pick_status="no_object")
    src = StubSource([[cand(1)], [cand(2)]], give_up_after=1)
    run(runner(src, skills=sk))
    assert sk.calls.count("radar_clear") == 2 and sk.calls.index("radar_clear", 1) > sk.calls.index("pick:us-1")


def test_manual_source_still_parks_every_cycle():
    from test_task import ListSource, obj

    sk = StubSkills()
    run(TaskRunner(sk, ListSource([obj(1, 300, 190), obj(2, 250, 210)]), WS, EventBus()))
    assert sk.calls.count("park") == 3  # 물체 2개 + 마지막 빈 스캔 앞


def test_sim_left_far_object_check_goes_through_from_radar_clear(tmp_path):
    """방위 약 145°·반경 170mm 물체: radar_clear에서 출발하는 첫 CHECK의 접근 IK가 풀려 확인이 끝까지 간다 (REQ-030, plan_goto 여러 시드)."""
    st, _src, arm = sim_studio(tmp_path, spread=[(161, 148, 0)])
    res = []
    st.bus.subscribe("us.check_result", res.append)
    out = st.start(blocking=True)
    ok = [r for r in res if r["status"] == "ok"]  # 레이더가 유령 후보를 함께 낼 수 있어 확인 결과 중 성공이 있는지 본다
    assert ok and out["picked"], (res, out)
    assert math.dist((ok[0]["object"]["x_mm"], ok[0]["object"]["y_mm"]), (161, 148)) < 30
    arm.disconnect()


def test_drop_zone_is_not_a_candidate_and_follows_reload(tmp_path):
    """REQ-045: 배출 구역에 놓인 물체가 다음 스캔에서 후보가 되지 않는다(전에는 같은 물체를 다시 집으러 갔다). 구역을 옮기면 예전 구역은 풀린다."""
    zone = WS.drop_zones["right"]
    boxes = [Box(zone["x_mm"], zone["y_mm"], 0, 30, 100)]
    arm = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert arm.connect()[0]
    src = make_ultrasonic_source(arm, sim=True, boxes=boxes)
    st = Studio(arm, src, dry_run=True)
    src.ignore_zones = [(10.0, 10.0, 5.0)]  # 소스 고유 영역은 남는다
    assert any(math.hypot(o.x_mm - zone["x_mm"], o.y_mm - zone["y_mm"]) < 40 for o in src.scan().objects)  # 넘기기 전에는 후보(시험이 의미 있다)
    st.task.refresh_ignore_zones()
    st.task.refresh_ignore_zones()  # 두 번 불러도 겹치지 않는다
    mine = (zone["x_mm"], zone["y_mm"], WS.drop_zone_ignore_radius_mm)
    assert src.ignore_zones == [(10.0, 10.0, 5.0), mine]
    assert src.scan().objects == []  # 구역 안이라 무시
    st.task.ws = dataclasses.replace(WS, drop_zones={"right": {"x_mm": 100.0, "y_mm": 300.0}})  # reload_workspace가 하는 일
    st.task.refresh_ignore_zones()
    assert src.ignore_zones == [(10.0, 10.0, 5.0), (100.0, 300.0, WS.drop_zone_ignore_radius_mm)]  # 예전 (500,150)은 사라졌다
    arm.disconnect()
