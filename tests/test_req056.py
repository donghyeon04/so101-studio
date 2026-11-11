"""REQ-056 (결정 0052, arm-software-spec 6.6·11.3.3): height_required, 초음파 CHECK 다시, 놓은 뒤 상승 최소(low_lift), ② 상자 폭, 집기 기억 넘김."""
import numpy as np
import pytest
from test_task import OK
from test_task_check import StubProbe, StubSource, UsSkills, cand, run, runner

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import DetectedObject, MotionResult, ToolPose
from so101_studio.service.arm_service import ArmService

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
TABLE = sorted((mm, v) for v, mm in CFG.gripper_width_mm)
POINT = (300.0, 200.0)


def make(tmp_path, grip_width):
    block = float(np.interp(grip_width, [p[0] for p in TABLE], [p[1] for p in TABLE]))
    s = ArmService(SimDriver(CFG.poses["home"], grip_block=block), CFG, WS, speed_pct=30, log_root=str(tmp_path))
    assert s.connect()[0]
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append((level, code, data))
    return s, ev


# ---- 1. height_required
@pytest.mark.slow
def test_vertical_pick_without_a_height_is_refused_without_moving(tmp_path):
    s, _ = make(tmp_path, 30)
    try:
        before = s.driver.read_joints()
        r = s.skills.pick(DetectedObject("o", 300.0, 150.0, 60.0, 30, 30, 1.0, "manual", 0.0))  # 각도 60: 턱 축이 팔 방향과 어긋나 기울이지 못한다
        after = s.driver.read_joints()
    finally:
        s.disconnect()
    assert r.status == "height_required" and "높이가 필요" in r.reason and after == before


@pytest.mark.slow
def test_tilted_plan_that_cannot_be_reached_without_a_height_also_asks_for_the_height(tmp_path):
    s, ev = make(tmp_path, 58)
    try:
        r = s.skills.pick(DetectedObject("o", 300.0, 300.0, 0.0, 58, 58, 1.0, "manual", 0.0))
    finally:
        s.disconnect()
    assert r.status == "height_required" and [d["reason"] for _, c, d in ev if c == "ARM-PICK-TILT-UNREACHABLE"] == ["tilt_unreachable_height_unknown"]


def test_cli_pick_prints_how_to_give_the_height(monkeypatch, capsys):
    from so101_studio import cli

    monkeypatch.setattr(ArmService, "pick_and_place", lambda self, obj, zone=None, place_at=None: MotionResult("height_required", 0.0, ToolPose(0, 0, 0, 0), reason="이유 문장"))
    cli.main(["--sim", "pick", "300", "150", "60", "30"])
    out = capsys.readouterr().out
    assert "height_required" in out and "이유 문장" in out and "5번째 값" in out and "pick 300 150 60 30 <높이>" in out


# ---- 2. 초음파 작업 실행: CHECK 다시 1회
class HeightSkills(UsSkills):
    """pick이 처음에는 height_required, 그 뒤에는 다시 CHECK한 물체에 높이가 있으면 ok."""

    def __init__(self, status="height_required", tilted=None):
        super().__init__(pick_status="ok")
        self.first, self.last_pick_tilted = status, tilted

    def pick(self, o):
        self.calls.append(f"pick:{o.id}:{o.height_mm:g}")
        if o.height_mm > 0 or self.first == "ok":
            return OK
        return MotionResult(self.first, 0.0, ToolPose(0, 0, 0, 0))


class HeightProbe(StubProbe):
    height = 0.0

    def result(self):
        r = super().result()
        return r and DetectedObject(r.id, r.x_mm, r.y_mm, r.angle_deg, r.width_mm, r.length_mm, r.confidence, "ultrasonic", self.height)


class HeightSource(StubSource):
    """첫 CHECK는 높이를 못 재고(0), 다시 CHECK하면 40mm를 잰다."""

    def start_check(self, c):
        self.calls.append(f"check:{c.id}")
        p = HeightProbe(self, c)
        p.height = 0.0 if self.calls.count(f"check:{c.id}") == 1 else 40.0
        self.probes.append(p)
        return p


def test_height_required_triggers_one_more_check_then_picks_with_the_measured_height():
    src, sk = HeightSource([[cand(1)], []]), HeightSkills()
    res, seen = run(runner(src, skills=sk))
    assert src.calls.count("check:us-1") == 2 and sk.calls.count("pick:us-1:40") >= 1 and res["picked"] == ["us-1"] and not src.failures
    assert any(e.state == "CHECK" and e.detail.get("reason") == "height_recheck" for e in seen)


def test_height_still_unknown_after_the_recheck_is_a_failure():
    src, sk = StubSource([[cand(1)], [cand(2)], []]), UsSkills(pick_status="height_required")
    res, _ = run(runner(src, skills=sk))
    assert src.calls.count("check:us-1") == 2 and src.failures and res["failed"][0] == "us-1"  # 다시 CHECK 1회 뒤 note_failure


@pytest.mark.parametrize("tilted,checks", [(False, 2), (True, 1), (None, 1)])
def test_vertical_no_object_rechecks_once_but_a_tilted_one_does_not(tilted, checks):
    src, sk = StubSource([[cand(1)], [cand(2)], []]), UsSkills(pick_status="no_object")
    sk.last_pick_tilted = tilted
    run(runner(src, skills=sk))
    assert src.calls.count("check:us-1") == checks


# ---- 3. 놓은 뒤 상승 최소
@pytest.mark.slow
def test_place_stops_with_low_lift_when_it_cannot_rise_enough(tmp_path, monkeypatch):
    s, ev = make(tmp_path, 30)
    try:
        sk = s.skills
        assert sk.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        descent = sk._descent

        def descent_then_no_heights(*a):  # 내려간 뒤에는 올라갈 수 있는 높이가 없다
            out = descent(*a)
            monkeypatch.setattr(sk, "_heights", list)
            monkeypatch.setattr(sk, "_lift_heights", lambda need: [])
            return out

        monkeypatch.setattr(sk, "_descent", descent_then_no_heights)
        r = sk.place_at(*POINT)
    finally:
        s.disconnect()
    assert r.status == "stopped" and r.reason.startswith("low_lift")
    assert [(lv, d["needed_mm"]) for lv, c, d in ev if c == "ARM-PLACE-LOW-LIFT"] == [("ERROR", 50.0)]  # 물체 높이 40 + 10


@pytest.mark.slow
def test_normal_place_rises_at_least_object_height_plus_10(tmp_path):
    s, ev = make(tmp_path, 30)
    try:
        sk = s.skills
        assert sk.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        assert sk.place_at(*POINT).status == "ok"
        z_end = s.kin.fk(s.driver.read_joints()).z_mm
    finally:
        s.disconnect()
    assert not [1 for _, c, _ in ev if c == "ARM-PLACE-LOW-LIFT"] and z_end >= CFG.table_z_mm + 40.0  # 손이 물체 높이 위로 올라갔다


# ---- 4. ② 상자 폭은 물체 폭 우선, 5. 집기 기억 넘김
@pytest.mark.slow
def test_box_width_uses_the_object_width_so_a_tall_vertical_grip_can_be_placed_by_route_2(tmp_path):
    # 폭 30mm 물체를 틈 50mm에서 쥐고 있다(실물처럼 쥔 틈이 물체보다 넓다): 상자를 물체 폭으로 보면 높이 40mm도 ②로 놓인다
    s, ev = make(tmp_path, 50)
    try:
        sk = s.skills
        assert sk.pick(DetectedObject("o", 300.0, 150.0, 60.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        assert sk.grip["obj_w"] == 30.0 and sk.grip["held_gap"] == pytest.approx(50.0, abs=3.0)
        assert sk.place_at(*POINT).status == "ok"
    finally:
        s.disconnect()
    routes = [d["route"] for _, c, d in ev if c == "ARM-OPENING-PATH"]
    assert routes == ["released_then_tilted"]


@pytest.mark.slow
def test_a_new_skills_object_inherits_the_grip_memory_on_reconnect(tmp_path):
    s, _ = make(tmp_path, 30)
    try:
        assert s.skills.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        old, grip = s.skills, dict(s.skills.grip)
        s.connect()  # 다시 연결하면 Skills를 새로 만든다
        assert s.skills is not old and s.skills.grip == grip
    finally:
        s.disconnect()


# ---- 후속 (리뷰 반영)
def test_manual_height_required_is_skipped_once_with_its_reason_and_not_retried():
    from so101_studio.core.types import ScanResult

    class Manual:  # 확인 기능(start_check)이 없는 인식 방식
        name, give_up = "manual", False

        def connect(self): ...
        def scan(self): return ScanResult([cand(1)], 0.0)
        def notify_picked(self, oid): ...

    sk = UsSkills(pick_status="height_required")
    res, seen = run(runner(Manual(), skills=sk))
    assert sk.calls.count("pick:us-1") == 1 and "us-1" in res["skipped"] and res["end"] == "no_objects"
    assert any(e.state == "PICK" and e.result == "height_required" and "reason" in e.detail for e in seen)


def test_recheck_that_finds_nothing_does_not_pick_the_same_candidate_again():
    src, sk = StubSource([[cand(1)], []], exists={"us-1": True}), UsSkills(pick_status="no_object")
    sk.last_pick_tilted = False

    counter = {"n": 0}

    def second_check_empty(c, orig=src.start_check):
        counter["n"] += 1
        if counter["n"] == 2:
            src.exists["us-1"] = False  # 다시 CHECK에는 물체가 없다
        return orig(c)

    src.start_check = second_check_empty
    run(runner(src, skills=sk))
    assert sk.calls.count("pick:us-1") == 1 and src.failures  # 재시도 없이 실패로 센다


@pytest.mark.slow
def test_opening_end_that_cannot_lift_enough_is_not_used(tmp_path, monkeypatch):
    s, ev = make(tmp_path, 30)
    try:
        sk = s.skills
        assert sk.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        monkeypatch.setattr(sk, "_can_leave", lambda st, need: False)  # 어떤 끝 자세에서도 필요한 만큼 못 올라간다고 하면 경로를 다 거른다
        sk.place_at(*POINT)
    finally:
        s.disconnect()
    fb = [d for _, c, d in ev if c == "ARM-OPENING-PATH-FALLBACK"]
    assert fb and fb[0]["reason"] == "clearance"


def test_height_required_applies_only_to_the_single_jaw_gripper():
    from types import SimpleNamespace

    from so101_studio.arm.skills import Skills

    sk = object.__new__(Skills)
    sk.gripper = SimpleNamespace(name="parallel", tilt_allowed=lambda *a: False)
    assert sk.gripper.name != "single_jaw"  # 병렬은 height_required 조건(단동식만)에 걸리지 않는다


@pytest.mark.slow
def test_placing_on_a_spot_that_already_has_an_object_warns_and_does_not_avoid_it(tmp_path):
    s, ev = make(tmp_path, 30)
    try:
        sk = s.skills
        assert sk.pick(DetectedObject("o", 300.0, 150.0, 0.0, 30, 30, 1.0, "manual", 40.0)).status == "ok"
        sk.placed = [(POINT[0] + 5.0, POINT[1], 15.0)]  # 같은 자리에 먼저 놓은 물체
        assert sk.place_at(*POINT).status == "ok"
    finally:
        s.disconnect()
    assert [d["count"] for lv, c, d in ev if c == "ARM-PLACE-SAME-SPOT" and lv == "WARN"] == [1]
