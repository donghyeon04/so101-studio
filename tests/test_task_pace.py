"""REQ-111(F2-c, ultrasonic-spec 8.9 멈춤 고리): TaskRunner.pace 자리·순서, 정지, prefetch_enabled, 팔 지우기, make_ultrasonic_source의 시계·radar_clear 실패."""
import threading
import time
from dataclasses import astuple

import pytest
from test_task import run
from test_task_check import (
    CFG,
    WS,
    StubArm,
    StubProbe,
    StubSource,
    UsSkills,
    cand,
    runner,
    sim_studio,
)

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.types import MotionResult, ToolPose
from so101_studio.perception.base import PathSegment
from so101_studio.service import ultrasonic
from so101_studio.service.arm_service import ArmService

STAGES = ["L1", "L2", "L2", "R", "L34"]  # 두 번째 L2 = 잘린 조각(같은 이름으로 다시 나간다)


class StageProbe(StubProbe):
    def next_segment(self):
        self.sent += 1
        return PathSegment([(0.0, 0.0), (10.0, 0.0)], 55.0, 100.0, stage=STAGES[self.sent - 1]) if self.sent <= len(STAGES) else None


class StageSource(StubSource):
    def start_check(self, c):
        self.calls.append(f"check:{c.id}")
        p = StageProbe(self, c)
        self.probes.append(p)
        return p


def test_pace_is_called_once_per_step_in_order_and_prefetch_runs_by_default():
    src, arm = StageSource([[cand(1)], []]), StubArm()
    r = runner(src, arm, UsSkills())
    calls = []
    r.pace = lambda step, stage: calls.append((step, stage))
    res, _ = run(r)
    assert res["picked"] == ["us-1"]
    assert calls == [("scan", None), ("check", "L1"), ("check", "L2"), ("check", "R"), ("check", "L34"), ("check_done", None), ("pick", None),
                     ("scan", None), ("scan", None)]  # 둘째 SCAN, 비어서 RESCAN 뒤
    assert len(arm.paths) == len(STAGES) and "prefetch" in src.calls
    assert src._links_fn == r._trail.links_at  # _begin_checkable이 늘 팔 지우기 기록을 넣는다


def test_prefetch_disabled_skips_prefetch():
    src = StageSource([[cand(1)], []])
    r = runner(src, StubArm(), UsSkills())
    r.prefetch_enabled = False
    res, _ = run(r)
    assert res["picked"] == ["us-1"] and "prefetch" not in src.calls


def test_gui_stop_while_waiting_in_pace_ends_stopped_before_moving_on():
    src, arm = StageSource([[cand(1)], []]), StubArm()
    r = runner(src, arm, UsSkills())

    def pace(step, stage):
        if (step, stage) == ("check", "L2"):
            r.stop()  # GUI 정지 = Studio.stop -> task.stop

    r.pace = pace
    res, seen = run(r)
    assert res["end"] == "STOPPED" and len(arm.paths) == 1 and src.probes[0].aborted  # L1만 가고 L2 전에 끊는다
    assert seen[-1].state == "STOPPED"


def _gui_like_pace(task, arm, entered: threading.Event):
    """GUI 고리 흉내(8.9 기다리기): 0.1초마다 task._stop·ctl.state를 다시 읽고, 설명 모드 [다음] 대기(paused)처럼 풀릴 때까지 기다린다."""
    def pace(step, stage):
        if step != "check":
            return
        entered.set()
        t_end = time.monotonic() + 10
        while time.monotonic() < t_end:
            if task._stop or arm.ctl.state in ("stopped", "fault"):
                return
            time.sleep(0.01)
    return pace


@pytest.mark.slow
def test_watchdog_estop_during_pace_returns_quickly_and_ends_fault(tmp_path):
    st, _src, arm = sim_studio(tmp_path)
    entered, done = threading.Event(), {}
    st.task.pace = _gui_like_pace(st.task, arm, entered)

    def go():
        done["res"] = st.start(blocking=True)
        done["t"] = time.monotonic()

    th = threading.Thread(target=go)
    th.start()
    try:
        assert entered.wait(120), "pace(check)에 들어가지 않았다"
        q0 = astuple(arm.driver.read_joints())
        t0 = time.monotonic()
        arm.estop()  # 감시 프로세스 정지(StopListener -> arm.estop())
        th.join(30)
        assert not th.is_alive()
        assert done["res"]["end"] == "FAULT", done["res"]
        assert done["t"] - t0 < 0.2 + 0.3  # pace는 0.2초 안에 돌아오고(여기 고리는 0.01초마다 본다), 실행은 바로 끝난다
        assert astuple(arm.driver.read_joints()) == pytest.approx(q0, abs=1e-6)  # 정지 뒤 더 움직이지 않는다
    finally:
        st.stop()
        th.join(5)
        arm.disconnect()


def test_make_ultrasonic_source_uses_the_arm_clock_and_links(tmp_path):
    arm = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert arm.connect()[0]
    try:
        src = ultrasonic.make_ultrasonic_source(arm, sim=True, boxes=[])
        assert src.clock is arm.clock  # Link의 시계 = 팔 시계(PoseTrail과 같은 축)
        assert src._links_fn == arm.trail.links_at
    finally:
        arm.disconnect()


def test_radar_clear_failure_raises_and_does_not_calibrate(tmp_path, monkeypatch):
    arm = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path))
    assert arm.connect()[0]
    calibrated = []
    monkeypatch.setattr(ultrasonic.SimUltrasonicSource, "calibrate", lambda self: calibrated.append(1))
    monkeypatch.setattr(arm.skills, "radar_clear", lambda: MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0), reason="z_floor"))
    try:
        with pytest.raises(RuntimeError, match="radar_clear"):
            ultrasonic.make_ultrasonic_source(arm, sim=True, boxes=[])
        assert calibrated == []
    finally:
        arm.disconnect()


@pytest.mark.slow
def test_sim_run_without_prefetch_still_erases_the_arm_and_picks_all(tmp_path):
    """prefetch를 끄면 팔 지우기는 _begin_checkable이 넣은 기록뿐이다: 둘째 SCAN부터 팔 링크 자리가 후보가 되면 확인 실패(dropped)가 생긴다."""
    st, src, arm = sim_studio(tmp_path)
    st.task.prefetch_enabled = False
    seen = []
    st.bus.subscribe("task", seen.append)
    try:
        res = st.start(blocking=True)
    finally:
        arm.disconnect()
    assert len(res["picked"]) == 3 and res["failed"] == [], res
    assert src._links_fn is not None
    empties = [e for e in seen if e.state == "CHECK" and e.result not in (None, "ok")]
    assert not empties, [(e.object_id, e.result) for e in empties]
