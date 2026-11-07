"""자세 즐겨찾기와 프로그램(결정 0044): 확인, 펼치기, 실행 순서·멈춤, 저장. 포트를 쓰지 않는 단위 시험."""

import threading
import time

from so101_studio.core.types import JointState, MotionResult, ToolPose
from so101_studio.gui import programs
from so101_studio.gui.programs import Run, action, describe, flatten, validate

WPS = {"위": {"joints": {"shoulder_pan": 0, "shoulder_lift": -30, "elbow_flex": 40, "wrist_flex": 60, "wrist_roll": 0, "gripper": 50}}}


def _ok(*_a, **_k):
    return MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))


class FakeArm:
    def __init__(self, fail_on=None):
        self.calls, self.fail_on = [], fail_on

    def __getattr__(self, name):
        def call(*args, **kw):
            self.calls.append(name)
            if name == self.fail_on:
                return MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0), reason="workspace")
            return _ok() if name != "point_pose" else ToolPose(0, 0, 0, 0)
        return call


def test_validate_catches_unknown_missing_and_nested_repeat():
    assert validate([{"type": "home"}, {"type": "waypoint", "name": "위"}], WPS) is None
    assert "모르는 종류" in validate([{"type": "fly"}], WPS)
    assert "없습니다" in validate([{"type": "waypoint", "name": "아래"}], WPS)
    assert "값이 없습니다" in validate([{"type": "wait"}], WPS)
    inner = {"type": "repeat", "count": 2, "steps": [{"type": "home"}]}
    assert validate([inner], WPS) is None
    assert "반복 안에 반복" in validate([{"type": "repeat", "count": 2, "steps": [inner]}], WPS)
    assert "1~1000" in validate([{"type": "repeat", "count": 0, "steps": []}], WPS)


def test_flatten_unrolls_repeat_with_indices():
    steps = [{"type": "home"}, {"type": "repeat", "count": 2, "steps": [{"type": "wait", "s": 0}, {"type": "home"}]}]
    assert [(i, j, s["type"]) for i, j, s in flatten(steps)] == [
        (0, None, "home"), (1, 0, "wait"), (1, 1, "home"), (1, 0, "wait"), (1, 1, "home")]
    assert describe(steps[1]) == "반복: 2번, 단계 2개"


def test_action_runs_in_order_and_stops_at_the_first_failure():
    steps = [{"type": "waypoint", "name": "위"}, {"type": "gripper", "value": 0}, {"type": "move", **programs.DEFAULTS["move"]}, {"type": "home"}]
    arm = FakeArm(fail_on="move_tool")
    run = Run()
    r = action(steps, WPS, run, lambda a: True)(arm)
    assert r.status == "out_of_bounds" and arm.calls == ["move_joints", "set_gripper", "point_pose", "move_tool"]
    assert run.index == (2, None) and run.total == 4  # 실패한 단계가 강조된 채로 남는다
    arm = FakeArm()
    assert action(steps, WPS, Run(), lambda a: True)(arm).status == "ok" and arm.calls[-1] == "home"


def test_wait_and_step_mode_obey_stop():
    run = Run(step_mode=True)
    out = []
    t = threading.Thread(target=lambda: out.append(action([{"type": "home"}, {"type": "home"}], WPS, run, lambda a: True)(FakeArm())))
    t.start()
    time.sleep(0.3)
    assert run.done == 0  # 한 단계씩: 첫 단계 뒤 다음 허락을 기다린다
    run.next_step()
    t.join(2)
    assert out[0].status == "ok"
    run = Run()
    t0 = time.monotonic()
    threading.Timer(0.2, run.stop.set).start()
    assert action([{"type": "wait", "s": 30}], WPS, run, lambda a: True)(FakeArm()).status == "stopped"
    assert time.monotonic() - t0 < 2


def test_save_and_load_roundtrip(tmp_path):
    wp = tmp_path / "waypoints.yaml"
    programs.save_waypoints(wp, {"위": programs.pose_entry(JointState(1, 2, 3, 4, 5, 6), ToolPose(300, 200, 40, 90))})
    assert programs.load_waypoints(wp)["위"]["joints"]["elbow_flex"] == 3
    steps = [{"type": "home"}, {"type": "wait", "s": 1.5}]
    programs.save_program(tmp_path / "programs", "시험", steps)
    assert programs.program_names(tmp_path / "programs") == ["시험"]
    assert programs.load_program(tmp_path / "programs", "시험") == steps


def test_program_page_saves_pose_builds_steps_and_runs_in_sim(backend, tmp_path, monkeypatch):
    """화면: 지금 자세 저장 → 자세로 이동·대기 단계 추가 → 저장 → 실행. 시뮬레이션 팔이 저장한 자세로 간다."""
    import asyncio

    from nicegui.testing import user_simulation

    from so101_studio.gui import app as gui_app

    monkeypatch.setattr(backend, "waypoints_path", str(tmp_path / "waypoints.yaml"))
    monkeypatch.setattr(backend, "programs_dir", str(tmp_path / "programs"))
    backend.set_speed(100)

    def one(user, marker):
        return next(iter(user.find(marker=marker).elements))

    async def until(check, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if check():
                return True
            await asyncio.sleep(0.05)
        return False

    async def check(user):
        assert await until(lambda: backend.owner is not None and backend.arm.ctl.state == "idle")
        assert backend.joint_to(4, 25.0) is None  # 홈과 다른 자세(손목 회전 25°)를 저장한다
        start = backend.arm.status().joints
        one(user, "waypoint-name").value = "시작"
        user.find(marker="waypoint-save").click()
        assert await until(lambda: "시작" in backend.waypoints())
        assert backend.home() is None  # 다른 자세로 옮겨 둔다
        one(user, "step-kind").value = "waypoint"
        user.find(marker="step-add").click()
        await user.should_see(marker="edit-name", retries=30)  # 단계 목록·편집 칸이 다시 그려질 때까지
        one(user, "edit-name").value = "시작"
        one(user, "step-kind").value = "wait"
        user.find(marker="step-add").click()
        await user.should_see(marker="edit-s", retries=30)
        one(user, "program-name").value = "시험"
        user.find(marker="program-save").click()
        assert await until(lambda: backend.program_names() == ["시험"])
        assert [s["type"] for s in backend.load_program("시험")] == ["waypoint", "wait"]
        user.find(marker="program-run").click()
        assert await until(lambda: backend.program_run is not None and backend.program_run.done == 2, 20.0)
        assert abs(backend.arm.status().joints.wrist_roll - start.wrist_roll) < 1.0 and abs(start.wrist_roll - 25.0) < 1.0

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/program")
            await check(user)

    asyncio.run(go())
