import time
from dataclasses import astuple

import numpy as np
import pytest

from so101_studio.arm.kinematics import Kinematics
from so101_studio.arm.motion import DEADMAN_S, MotionController
from so101_studio.arm.safety import Safety
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.arm.teach import Teach, smooth
from so101_studio.arm.teleop import keyboard_loop, robot_to_table
from so101_studio.arm.trajectory import move_j
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState

CFG = load_robot("config/robot.yaml")
WS = load_workspace("config/workspace.yaml")
HOME = JointState(*CFG.poses["home"])
LIMITS = CFG.joint_limits_deg


@pytest.fixture(scope="module")
def kin():
    return Kinematics(CFG.urdf, WS.base)


def rig(kin=None, **safety_kw):
    d = SimDriver(CFG.poses["home"])
    return d, MotionController(d, kin, Safety(LIMITS, WS, kin, **safety_kw) if kin else Safety(LIMITS), 100)


# ---------------- T8: jog + 데드맨
def jog_for(ctl, mode, v, seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        ctl.jog(mode, v)
        time.sleep(0.05)


def test_joint_jog_moves_while_input_continues():
    d, ctl = rig()
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.5)
    ctl.close()
    assert d.read_joints().shoulder_pan - HOME.shoulder_pan == pytest.approx(10, abs=3)  # 20°/s x 0.5s


def test_deadman_stops_within_a_third_of_a_second_after_input_ends():
    d, ctl = rig()
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.3)
    time.sleep(DEADMAN_S + 0.1)
    p1 = d.read_joints().shoulder_pan
    time.sleep(0.3)
    p2 = d.read_joints().shoulder_pan
    state = ctl.state
    ctl.close()
    assert p1 == p2 and state == "idle"  # 입력이 끊기자 더 움직이지 않는다


def test_joint_jog_stops_at_the_limit():
    d, ctl = rig()
    jog_for(ctl, "joint", [1000, 0, 0, 0, 0, 0], 0.6)  # 한계 이상으로 밀어도
    ctl.close()
    assert d.read_joints().shoulder_pan <= LIMITS["shoulder_pan"][1] + 1e-6


def test_jog_is_ignored_while_a_trajectory_runs():
    d, ctl = rig()
    h = ctl.execute(move_j(HOME, JointState(20, -90, 93, 0, 0, 3), 20, 60))
    assert ctl.jog("joint", [0, 0, 0, 0, 30, 0]) is False
    h.wait(5)
    ctl.close()
    assert d.read_joints().wrist_roll == pytest.approx(0, abs=1e-6)


def test_cartesian_jog_moves_the_tip_forward(kin):
    d, ctl = rig(kin)
    q0 = d.read_joints()
    p0 = kin.fk(q0)
    f, left, up, yaw = robot_to_table((30.0, 0, 0, 0), WS.base.yaw_deg)  # 로봇 앞쪽 30mm/s
    jog_for(ctl, "cartesian", [f, left, up, yaw], 0.5)
    ctl.close()
    p1 = kin.fk(d.read_joints())
    assert np.hypot(p1.x_mm - p0.x_mm, p1.y_mm - p0.y_mm) > 5  # 움직였다
    assert (p1.x_mm - p0.x_mm, p1.y_mm - p0.y_mm) == pytest.approx((f * 0.5, left * 0.5), abs=8)


def test_keyboard_loop_maps_keys_and_exits():
    keys = iter(["q"] * 8 + ["x"])
    d, ctl = rig()
    keyboard_loop(ctl, "joint", WS.base.yaw_deg, 20.0, lambda: next(keys), out=lambda *_: None, tick_s=0.02)
    ctl.close()
    assert d.read_joints().shoulder_pan > HOME.shoulder_pan + 1  # q 키 = 1번 관절 +


# ---------------- T11: retarget
def test_retarget_replaces_the_move_smoothly_without_a_jump():
    d, ctl = rig()
    log = []
    orig = d.write_joints
    d.write_joints = lambda q: (log.append(astuple(q)), orig(q))[1]
    h1 = ctl.execute(move_j(HOME, JointState(40, -90, 93, 0, 0, 3), 40, 120, profile="ruckig", j_max_deg_s3=400))
    time.sleep(0.4)
    h2 = ctl.retarget(JointState(-30, -90, 93, 0, 0, 3), 40, 120, 400)  # 반대쪽으로 목표를 바꾼다
    assert h1.wait(2).status == "preempted"
    assert h2.wait(8).status == "ok"
    ctl.close()
    q = d.read_joints()
    assert q.shoulder_pan == pytest.approx(-30, abs=0.01)
    steps = np.abs(np.diff(np.array(log), axis=0)).max()
    assert steps < 5.0  # 한 주기 이동량 상한(안전)을 넘지 않는다 = 속도가 이어진다


def test_retarget_when_idle_just_starts_a_move():
    d, ctl = rig()
    r = ctl.retarget(JointState(10, -90, 93, 0, 0, 3), 40, 120, 400).wait(5)
    ctl.close()
    assert r.status == "ok" and d.read_joints().shoulder_pan == pytest.approx(10, abs=0.01)


def test_retarget_outside_limits_is_rejected_and_the_old_move_continues():
    _d, ctl = rig()
    h1 = ctl.execute(move_j(HOME, JointState(30, -90, 93, 0, 0, 3), 30, 100))
    time.sleep(0.2)
    h2 = ctl.retarget(JointState(500, -90, 93, 0, 0, 3), 40, 120, 400)
    assert h2.wait(1).status == "out_of_bounds"
    assert h1.wait(5).status == "ok"
    ctl.close()


# ---------------- T9: 녹화·재생
def test_smooth_keeps_endpoints():
    q = np.array([[0.0, 0], [10, 0], [0, 0], [10, 0], [0, 0], [5, 0]])
    s = smooth(q, 3)
    assert (s[0] == q[0]).all() and (s[-1] == q[-1]).all() and s[2, 0] < 8


def test_record_save_and_replay(tmp_path):
    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    teach.record_start()
    assert d.torque_enabled() is False  # 토크를 끄고 기록한다
    for pan in (0, 8, 16, 24, 30):
        d.write_joints(JointState(pan, -90, 93, 0, 0, 3))  # 사람이 손으로 움직이는 것을 흉내
        time.sleep(0.25)
    path = teach.record_stop("wave")
    assert path.exists() and d.torque_enabled() is True and teach.names() == ["wave"]
    d.write_joints(HOME)  # 다른 자리로 옮겨 놓고 재생
    ctl.sync()
    r = teach.replay("wave", speed=1.0)
    ctl.close()
    assert r.status == "ok" and d.read_joints().shoulder_pan == pytest.approx(30, abs=1.0)


def test_replay_rejects_a_sequence_outside_the_limits(tmp_path):
    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    (tmp_path / "bad.json").write_text('{"hz": 20, "joints": [[0,-90,95,0,0,3],[300,-90,95,0,0,3]]}', encoding="utf-8")
    r = teach.replay("bad")
    ctl.close()
    assert r.status == "out_of_bounds" and d.read_joints().shoulder_pan == pytest.approx(HOME.shoulder_pan, abs=0.01)


def test_cartesian_jog_works_with_the_wrist_rolled(kin):
    d = SimDriver(CFG.poses["home"])
    d.write_joints(JointState(0, -90, 93, 0, 17, 3))  # 손목이 돌아가 있는 자세
    ctl = MotionController(d, kin, Safety(LIMITS, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm), 100)
    y0 = kin.fk(d.read_joints()).y_mm
    f, left, up, yaw = robot_to_table((30.0, 0, 0, 0), WS.base.yaw_deg)
    jog_for(ctl, "cartesian", [f, left, up, yaw], 0.5)
    ctl.close()
    assert kin.fk(d.read_joints()).y_mm - y0 == pytest.approx(0.5 * 30, abs=6)


def test_jog_speed_ramps_up_and_down_without_a_velocity_step():
    d, ctl = rig()
    log = []
    orig = d.write_joints
    d.write_joints = lambda q: (log.append(astuple(q)), orig(q))[1]
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.4)
    ctl.jog_stop()
    time.sleep(0.3)
    ctl.close()
    pan = np.array(log)[:, 0]
    step = np.diff(pan)
    assert abs(step[0]) < 0.2 and np.abs(np.diff(step)).max() < 0.2  # 시작·정지에서 틱당 속도 변화가 가속도 한계(0.16°) 안
    assert step[-1] == 0 or abs(step[-1]) < 0.2
    assert ctl.state == "idle"


def test_lateral_jog_works_from_the_horizontal_home_pose(kin):
    d = SimDriver(CFG.poses["home"])  # 도구가 수평: 방향을 고정하면 옆으로 못 가는 자세
    ctl = MotionController(d, kin, Safety(LIMITS, WS, kin, z_floor_mm=CFG.tip_floor_mm, table_z_mm=CFG.table_z_mm), CFG.control_hz)
    x0 = kin.fk(d.read_joints()).x_mm
    f, left, up, yaw = robot_to_table((0.0, 30.0, 0, 0), WS.base.yaw_deg)
    jog_for(ctl, "cartesian", [f, left, up, yaw], 0.5)
    ctl.close()
    assert abs(kin.fk(d.read_joints()).x_mm - x0) > 3  # 고치기 전에는 풀리지 않아 0이었다


def test_jog_distance_follows_elapsed_time_not_tick_count():
    d, ctl = rig()
    ctl.dt = 0.002  # 틱이 아주 빠르게 돌아도(간격이 들쭉날쭉해도) 이동 거리는 시간에 비례한다
    jog_for(ctl, "joint", [20, 0, 0, 0, 0, 0], 0.6)
    ctl.close()
    assert d.read_joints().shoulder_pan - HOME.shoulder_pan == pytest.approx(10, abs=3)  # 20°/s x 0.6s, 램프 손실 감안


def test_jog_can_leave_a_pose_beyond_the_joint_limit_but_not_go_further_out():
    d = SimDriver([0, -90, 96.5, 0, 0, 3])  # 팔꿈치 한계(95) 밖에서 출발(받침에 기댄 자세 근처)
    ctl = MotionController(d, None, Safety(LIMITS), CFG.control_hz)
    jog_for(ctl, "joint", [0, 0, -20, 0, 0, 0], 0.4)  # 안쪽으로
    inside = d.read_joints().elbow_flex
    jog_for(ctl, "joint", [0, 0, 20, 0, 0, 0], 0.6)  # 다시 바깥쪽으로 밀어도
    ctl.close()
    assert inside < 96.5 - 2 and d.read_joints().elbow_flex <= LIMITS["elbow_flex"][1] + 1e-6


def test_replay_clamps_small_limit_overshoots_but_still_rejects_big_ones(tmp_path):
    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    hi = LIMITS["elbow_flex"][1]
    (tmp_path / "near.json").write_text(f'{{"hz": 20, "joints": [[0,-90,{hi + 1.5},0,0,3],[5,-90,{hi + 1.0},0,0,3]]}}', encoding="utf-8")
    (tmp_path / "far.json").write_text(f'{{"hz": 20, "joints": [[0,-90,{hi + 8},0,0,3],[5,-90,{hi + 8},0,0,3]]}}', encoding="utf-8")
    near, far = teach.replay("near", 2.0), teach.replay("far")
    ctl.close()
    assert near.status == "ok" and d.read_joints().elbow_flex <= hi + 1e-6  # 한계로 잘려 재생됨
    assert far.status == "out_of_bounds"  # 크게 넘으면 거부


def test_replay_until_limit_plays_only_the_part_before_the_first_violation(tmp_path):
    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    lim = LIMITS["shoulder_pan"][1]
    rows = [[float(p), -90, 93, 0, 0, 3] for p in range(int(lim) + 20)]  # 어깨가 1°씩 천천히 한계를 넘어간다
    import json
    (tmp_path / "x.json").write_text(json.dumps({"hz": 20, "joints": rows}), encoding="utf-8")
    full = teach.replay("x", 3.0)
    part = teach.replay("x", 3.0, until_limit=True)
    cut = teach.last_cut
    ctl.close()
    assert full.status == "out_of_bounds"  # 통째로는 거부
    assert part.status == "ok" and cut[0] < cut[1] and "shoulder_pan" in cut[2]  # 한계 앞까지만 재생
    assert d.read_joints().shoulder_pan < lim


def test_check_path_sees_a_short_violation_in_a_long_sequence():
    from so101_studio.arm.safety import Safety
    from so101_studio.arm.trajectory import SequenceTrajectory

    s = Safety({"shoulder_pan": [-100, 100]})
    t = np.arange(0, 60, 0.05)
    q = np.zeros((len(t), 6))
    q[:, 1], q[:, 2] = -90, 93
    q[500:508, 0] = 130  # 0.4초만 한계 밖 (60초 중)
    assert s.check_path(SequenceTrajectory(t, q)) is not None


def test_recording_saves_real_sample_times_and_replay_uses_them(tmp_path):
    import json

    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    teach.record_start()
    for pan in (0, 6, 12, 18):
        d.write_joints(JointState(pan, -90, 93, 0, 0, 3))
        time.sleep(0.2)
    path = teach.record_stop("slow")
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert len(saved["t"]) == len(saved["joints"]) and saved["t"] == sorted(saved["t"])
    dur = saved["t"][-1] - saved["t"][0]
    assert dur == pytest.approx(0.8, abs=0.25)  # 손으로 움직인 시간이 그대로 들어 있다
    _q, t = teach.load("slow")
    assert t[-1] - t[0] == pytest.approx(dur)
    # 시각이 없는 옛 파일은 hz로 일정하다고 본다
    (tmp_path / "old.json").write_text('{"hz": 20, "joints": [[0,-90,93,0,0,3],[1,-90,93,0,0,3],[2,-90,93,0,0,3]]}', encoding="utf-8")
    assert teach.load("old")[1].tolist() == pytest.approx([0.0, 0.05, 0.1])
    ctl.close()


def test_recording_stops_adding_samples_after_the_time_cap(tmp_path):
    d, ctl = rig()
    t = Teach(ctl, CFG, tmp_path)
    t.record_start(max_s=0.3)  # 설정(90초) 대신 이번만 0.3초
    time.sleep(0.8)
    t.record_stop("cap")
    ctl.close()
    q, times = t.load("cap")
    assert t.truncated and times[-1] <= 0.35 and len(q) <= 8
    assert d.torque_enabled() is True  # 끝나면 토크가 다시 켜진다


def _slow_pause_fast():
    """0~2초 느리게(10°/s), 2~3초 멈춤, 3~5초 빠르게(80°/s). 20Hz."""
    t = np.arange(0, 5.0, 0.05)
    v = np.where(t < 2, 10.0, np.where(t < 3, 0.0, 80.0))
    q = np.zeros((len(t), 6))
    q[:, 0], q[:, 1], q[:, 2] = np.cumsum(v * 0.05), -90, 93
    return q, t


def test_analyze_finds_slow_pause_and_fast_segments():
    from so101_studio.arm.teach import analyze

    q, t = _slow_pause_fast()
    info = analyze(q, t, pause=3.0, fast=40.0, min_s=0.5)
    assert [s["label"] for s in info["segments"]] == ["느림", "멈춤", "빠름"]
    seg = info["segments"]
    assert info["duration_s"] == pytest.approx(4.95, abs=0.05) and info["max_deg_s"] > info["mean_deg_s"] > 5
    assert seg[0]["mean_deg_s"] < seg[2]["mean_deg_s"] and seg[1]["start_s"] == pytest.approx(2.0, abs=0.3)


def test_retime_changes_only_the_chosen_segment():
    from so101_studio.arm.teach import analyze, retime

    q, t = _slow_pause_fast()
    segs = analyze(q, t)["segments"]
    new = retime(t, segs, {2: 2.0})  # 빠른 구간만 두 배 빠르게
    assert (new[-1] - new[0]) == pytest.approx((t[-1] - t[0]) - 1.0, abs=0.15)  # 2초짜리 구간이 1초로
    assert retime(t, segs, {})[-1] == pytest.approx(t[-1] - t[0])
    assert retime(t, segs, {}, speed=2.0)[-1] == pytest.approx((t[-1] - t[0]) / 2)


def test_replay_validates_segment_speeds_and_info_works_on_files_without_meta(tmp_path):
    import json

    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    q, t = _slow_pause_fast()
    (tmp_path / "mix.json").write_text(json.dumps({"hz": 20, "t": t.tolist(), "joints": q.tolist()}), encoding="utf-8")
    info = teach.info("mix")  # 요약이 없는 파일도 지금 계산한다
    assert [s["label"] for s in info["segments"]] == ["느림", "멈춤", "빠름"]
    with pytest.raises(ValueError):
        teach.replay("mix", segment_speeds={9: 1.0})  # 없는 구간
    with pytest.raises(ValueError):
        teach.replay("mix", segment_speeds={2: 5.0})  # 빠른 구간 80°/s x 5 = 400°/s라 한계(220°/s)를 넘는다
    ctl.close()


def test_recording_saves_summary_and_segments(tmp_path):
    import json

    d, ctl = rig()
    teach = Teach(ctl, CFG, tmp_path)
    teach.record_start()
    for pan in (0, 3, 6, 9, 12):
        d.write_joints(JointState(pan, -90, 93, 0, 0, 3))
        time.sleep(0.2)
    path = teach.record_stop("sum")
    meta = json.loads(path.read_text(encoding="utf-8"))["meta"]
    ctl.close()
    assert meta["points"] > 5 and meta["duration_s"] > 0.5 and "segments" in meta and meta["recorded_at"]
