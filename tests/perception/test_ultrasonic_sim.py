import math
from pathlib import Path

import pytest

from so101_studio.core.types import DetectedObject
from so101_studio.perception.ultrasonic.benchmark import (
    angle_ok,
    do_check,
    make_source,
    run_cycle,
)
from so101_studio.perception.ultrasonic.benchmark import build as _build
from so101_studio.perception.ultrasonic.model import Box

STRICT_REACH = (100, 240)  # 손끝 도달 반경(mm). 시험 대부분은 도달 제한 없음(reach=None), 이 값은 도달 거부 시험용


def build(boxes, **kw):
    """단위 시험은 이상적인 센서(model=None)와 도달 제한 없는 팔로 돈다. 성능 숫자는 benchmark.py(8.5절 모델)가 낸다."""
    kw.setdefault("model", None)
    kw.setdefault("reach", None)
    return _build(boxes, **kw)


# 센서가 손끝 왼쪽(-x) 66.5mm에 있어(결정 0063) 손끝이 오른쪽으로 66.5mm 나가므로, 바깥 물체가 도달 밖이 되지 않게 옛 배치(x 200/310/400)를 60mm 왼쪽으로 옮겼다
THREE = [Box(140, 190, 0), Box(250, 160, 90), Box(340, 200, 45)]


def test_radar_single_object_nine_spots(tmp_path):
    """R1 가상: 3x3 자리에서 후보가 정답 90mm 안 (합격 기준 95%)."""
    errs = []
    for x in (240, 300, 360):
        for y in (140, 180, 220):
            cfg, clk, esp, arm = build([Box(x, y, 0)], seed=int(x + y))
            src = make_source(cfg, clk, esp, arm, tmp_path)
            objs = src.scan().objects
            assert len(objs) == 1, (x, y, objs)
            errs.append(math.dist((objs[0].x_mm, objs[0].y_mm), (x, y)))
    assert max(errs) < 90, errs
    assert sorted(errs)[len(errs) // 2] < 40  # 중앙값은 가상 센서 비교(20~25mm)와 비슷한 크기


@pytest.mark.parametrize("ang", [0, 45, 90, 135])
def test_check_position_and_angle(tmp_path, ang):
    cfg, clk, esp, arm = build([Box(300, 170, ang)], seed=ang)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    obj, probe, _ = do_check(src, arm, cand)
    assert obj is not None
    assert math.dist((obj.x_mm, obj.y_mm), (300, 170)) < 10
    assert angle_ok(obj.angle_deg, ang)
    assert abs(obj.length_mm - 100) < 15 and abs(obj.width_mm - 30) < 15
    assert probe.info["h_mm"] == pytest.approx(40, abs=5)
    assert obj.height_mm == probe.info["h_mm"]  # REQ-035: 확인 결과가 실측 높이를 담는다


def test_ghost_candidate_is_removed(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    esp.boxes.clear()  # 레이더가 본 것은 오검출
    obj, _, _ = do_check(src, arm, cand)
    assert obj is None
    assert src.scan().objects == []  # 같은 자리는 다시 후보로 안 나온다


def test_events(tmp_path):
    from so101_studio.core.events import EventBus

    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    bus, got = EventBus(), []
    for t in ("us.sweep", "us.candidates", "us.check_path", "us.check_result"):
        bus.subscribe(t, lambda p, t=t: got.append(t))
    src = make_source(cfg, clk, esp, arm, tmp_path)
    src.bus = bus
    do_check(src, arm, src.scan().objects[0])
    assert {"us.sweep", "us.candidates", "us.check_path", "us.check_result"} <= set(got)


def test_square_object_has_no_angle(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0, width=40, length=40)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    obj, _, _ = do_check(src, arm, src.scan().objects[0])
    assert isinstance(obj, DetectedObject) and obj.angle_deg is None
    assert math.dist((obj.x_mm, obj.y_mm), (300, 170)) < 10


def test_unsafe_clearance_rejected(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)], clearance_mm=10)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    with pytest.raises(ValueError):
        src.start_check(src.scan().objects[0])


def _cycles(tmp_path, n=10, **kw):
    oks, times = [], []
    for seed in range(n):
        boxes = [Box(b.x, b.y, b.angle_deg) for b in THREE]
        cfg, clk, esp, arm = build(boxes, seed=seed, **kw)
        src = make_source(cfg, clk, esp, arm, tmp_path)
        ok, secs, _ = run_cycle(src, arm, esp, rescan_home=False)
        oks.append(ok)
        times.append(secs)
    print(f"성공 평균 {sum(oks) / n:.1f}/3, 인식 평균 {sum(times) / n:.1f}초, 최대 {max(times):.1f}초")
    return sum(oks) / n, sum(times) / n


def test_three_objects_end_to_end_success(tmp_path):
    """1절: 10회 평균 2개 이상 성공 (가상 ESP32 + 가상 팔)."""
    ok, _ = _cycles(tmp_path)
    assert ok >= 2.0


def test_three_objects_end_to_end_time(tmp_path):
    """인식 시간 기준(1절 임시): 평균 130초 이하 (팔 100mm/s, 가속 120mm/s², 호출 사이 0.25초). 옛 오프셋은 70초였고, 새 오프셋(센서 높이 155mm)에서 ring_r 60mm가 약 127초다(ring_r 45는 약 69초이나 성공률이 낮다). 실측 A1 뒤 다시 정한다."""
    _, secs = _cycles(tmp_path)
    assert secs <= 130


def test_ring_is_one_follow_path_call(tmp_path):
    """원(R)은 follow_path 한 번(연속 경로)이고, 물체당 4번(L1, L2, 원, L3+L4) 부른다."""
    cfg, clk, esp, arm = build([Box(300, 200, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    do_check(src, arm, src.scan().objects[0])
    assert len(arm.calls) == 4
    lengths = sorted(L for L, _ in arm.calls)
    assert any(abs(L - 2 * math.pi * cfg.check.ring_r_mm) < 3 for L in lengths)  # 원은 한 번
    assert lengths[-1] > 2 * cfg.check.line_mm  # L3+L4는 한 번에 이어 간다(연결 구간 포함)


def test_link_lost_raises(tmp_path):
    from so101_studio.perception.ultrasonic.link import UltrasonicLinkError

    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    esp.disconnect()
    with pytest.raises(UltrasonicLinkError):
        src.scan()


def test_arm_links_are_masked(tmp_path):
    """팔이 레이더 앞에 있어도 팔 지우기가 후보를 만들지 않는다 (5절 4)."""
    cfg, clk, esp, arm = build([])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    arm.move((300, 200), 55)
    assert src.scan().objects == []
    src._links_fn = None  # 지우지 않으면 팔을 물체로 본다
    assert len(src.scan().objects) >= 1


def test_prefetch_makes_next_scan_instant(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    src.prefetch(arm.links_at)
    clk.sleep(6)  # 놓는 동안
    res = src.scan()
    assert res.duration_s < 0.1 and len(res.objects) == 1
    assert res.raw["passes"] == cfg.radar.prefetch_sweeps


def test_unreachable_segment_is_skipped_not_deleted(tmp_path):
    """바깥쪽 물체: 세로선(L2)이 도달 밖이면 그 구간을 건너뛰고(unreachable 기록) 나머지로 확정한다."""
    cfg, clk, esp, arm = build([Box(250, 220, 0)], reach=STRICT_REACH)  # 센서 오프셋 66.5mm 때문에 옛 자리(300, 235)는 전부 도달 밖이 된다
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    obj, probe, _ = do_check(src, arm, cand)
    assert "L2" in probe.info["unreachable"]
    assert obj is not None and probe.status == "ok"
    assert math.dist((obj.x_mm, obj.y_mm), (250, 220)) < 15
    assert obj.confidence < 0.9  # 못 간 구간이 있으면 덜 믿는다


def test_all_unreachable_keeps_candidate(tmp_path):
    """전부 도달 밖이면 못 잰 것이지 물체가 없는 게 아니다: 후보를 지우지 않는다."""
    cfg, clk, esp, arm = build([Box(300, 380, 0)], reach=STRICT_REACH)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    obj, probe, _ = do_check(src, arm, cand)
    assert obj is None and probe.status == "unreachable"
    assert len(src.scan().objects) == 1  # 다시 스캔해도 후보가 남아 있다


@pytest.mark.parametrize("yaw", [0, 45, 90])
def test_yaw_rotates_sensor_offset(tmp_path, yaw):
    """손목 yaw에 따라 센서 오프셋이 돌아가므로 손끝 경로가 달라져도 확인 결과는 같다."""
    cfg, clk, esp, arm = build([Box(300, 230, 0)], yaw_deg=yaw)  # yaw 0은 센서가 손끝 앞 66.5mm라 가까운 자리(y 170)는 손끝이 도달 안쪽 밖이다
    src = make_source(cfg, clk, esp, arm, tmp_path)
    obj, _, _ = do_check(src, arm, src.scan().objects[0])
    assert math.dist((obj.x_mm, obj.y_mm), (300, 230)) < 10


def test_sensor_to_tip_default_yaw_90():
    """기본 yaw 90: 도구 프레임 오프셋 (0.6, 66.5)가 작업대 좌표 (-66.5, 0.6)으로 돌아간다(센서 = 손끝 + 돌린 오프셋)."""
    from so101_studio.perception.ultrasonic.check import sensor_to_tip
    from so101_studio.perception.ultrasonic.model import load_config

    cfg = load_config()
    assert cfg.check.yaw_deg == 90
    (tip,) = sensor_to_tip([(100.0, 100.0)], cfg)
    assert tip == pytest.approx((166.5, 99.4))


def test_pick_yaw_puts_sensor_outward_for_far_and_inward_for_near():
    """check.pick_yaw: 먼 후보는 센서가 손끝보다 바깥(팔→후보 방향), 가까운 후보는 안쪽. pivot이 없으면 yaw_deg 고정."""
    from so101_studio.perception.ultrasonic.check import pick_yaw, sensor_to_tip
    from so101_studio.perception.ultrasonic.model import load_config

    cfg, origin = load_config(), (300.0, 50.0)
    for cand, sign in (((300.0, 300.0), 1), ((300.0, 90.0), -1)):  # 반경 250(먼) / 40(가까운), 방위 90
        y = pick_yaw(cfg, cand, origin)
        (tip,) = sensor_to_tip([cand], cfg, y)
        radial = (cand[1] - tip[1]) * 1.0  # 센서(=cand) - 손끝, 팔→후보가 +y
        assert sign * radial > 0 and abs(radial) == pytest.approx(66.5, abs=1.0)
    y = pick_yaw(cfg, (300.0, 200.0), origin)  # 반경 150 = pivot: 오프셋이 접선 방향
    (tip,) = sensor_to_tip([(300.0, 200.0)], cfg, y)
    assert abs(200.0 - tip[1]) < 1.0
    assert load_config().check.yaw_pivot_mm == 150  # 결정 0065: 기본으로 켠다
    assert pick_yaw(load_config(yaw_pivot_mm=0), (300.0, 300.0), origin) == 90  # 끄면(0) 고정 yaw 90


def test_origin_comes_from_workspace_config(tmp_path):
    from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
    from so101_studio.perception.ultrasonic.link import Link
    from so101_studio.perception.ultrasonic.model import load_config
    from so101_studio.perception.ultrasonic.source import UltrasonicSource

    clk, cfg = SimClock(), load_config()
    src = UltrasonicSource(Link(FakeESP32(clk, [], cfg), clk), cfg, baseline_path=tmp_path / "b.json")
    assert src.arm_origin == (300.0, 50.0)  # config/workspace.yaml robot_base_in_table


def test_unreachable_twice_is_dropped_until_next_cycle(tmp_path):
    """못 가서 못 잰 후보는 두 번째에 뺀다(6절). 사이클 시작(begin_cycle)에 다시 나온다."""
    cfg, clk, esp, arm = build([Box(300, 320, 0)], reach=STRICT_REACH, yaw_pivot_mm=0)  # 레이더 가까이: 팔이 못 간다(yaw 고정, 반경별 yaw는 도달을 넓혀 이 가정을 깬다)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    for n in (1, 2):
        cands = src.scan().objects
        assert len(cands) == 1, n
        obj, probe, _ = do_check(src, arm, cands[0])
        assert obj is None and probe.status == "unreachable"
    assert src.scan().objects == []  # 두 번 못 갔으니 이번 사이클에서는 더 안 고른다
    src.begin_cycle()
    assert len(src.scan().objects) == 1


def test_abort_stops_esp32_and_keeps_candidate(tmp_path):
    """정지·E-STOP: probe.abort()가 ESP32에 stop을 보내고 후보는 지우지 않는다."""
    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    probe = src.start_check(cand)
    assert probe.next_segment() is not None
    clk.sleep(0.2)
    assert esp._stream is not None
    probe.abort()
    assert esp._stream is None and probe.status == "aborted" and probe.next_segment() is None
    assert len(src.scan().objects) == 1  # 지워지지 않았다


def test_check_path_published_at_start_and_end(tmp_path):
    from so101_studio.core.events import EventBus

    cfg, clk, esp, arm = build([Box(300, 235, 0)], reach=STRICT_REACH)
    bus, got = EventBus(), []
    bus.subscribe("us.check_path", got.append)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    src.bus = bus
    do_check(src, arm, src.scan().objects[0])
    stage_phases = [(e["stage"], e["phase"]) for e in got]
    assert stage_phases[0] == ("L1", "start") and got[0]["points"] == []
    assert ("L1", "end") in stage_phases
    assert any(e["unreachable"] for e in got)  # 못 간 구간은 unreachable로 알린다


def _all_segments(src, arm, cand):
    probe = src.start_check(cand)
    segs = []
    while (seg := probe.next_segment()) is not None:
        segs.append(seg)
        probe.feed(arm.follow_path(seg.points_mm, seg.tool_z_mm, seg.speed_mm_s, seg.yaw_deg))
    return probe, probe.result(), segs


def test_clip_to_reach_splits_and_limits_paths(tmp_path):
    """PR #71: 손끝이 tool_reach_mm 밖인 점은 건너뛰고, 끊기면 조각마다 따로 보낸다."""
    cfg, clk, esp, arm = build([Box(300, 170, 0)], yaw_pivot_mm=0)  # yaw 고정 90
    src = make_source(cfg, clk, esp, arm, tmp_path)
    probe = src.start_check(src.scan().objects[0])
    # yaw 90이라 손끝 = 센서 + (66.5, -0.6). 센서 x 250이면 손끝 x 316.5라 원점(300,50) 반경 60 안(손끝 y -7.7~107.7)은 건너뛴다
    pieces = probe._clip([(250, -70), (250, 170)])
    assert len(pieces) == 2
    assert pieces[0][-1][1] == pytest.approx(-7.7, abs=3) and pieces[1][0][1] == pytest.approx(107.7, abs=3)
    reach = cfg.check.tool_reach_mm
    probe, _obj, segs = _all_segments(src, arm, src.scan().objects[0])
    for seg in segs:
        for x, y in seg.points_mm:
            assert reach.min - 1 <= math.hypot(x - 300, y - 50) <= reach.max + 1


def test_inside_path_is_sent_whole(tmp_path):
    """전부 도달 범위 안이면 원래 꼭짓점 그대로 한 구간이다."""
    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    probe = src.start_check(src.scan().objects[0])
    assert probe._clip([(300, 200), (300, 240)]) == [[(300, 200), (300, 240)]]


def test_fake_arm_matches_real_arm_times():
    """#73: 가상 팔은 팔 코드(fillet_path·LookaheadShape)로 움직여 --sim 실측(measurements.md, 속도 100%, 접근 제외)과 시간이 같다."""
    from so101_studio.perception.ultrasonic.benchmark import FakeArm
    from so101_studio.perception.ultrasonic.fake_esp32 import SimClock
    from so101_studio.perception.ultrasonic.model import load_config

    cfg = load_config()

    def secs(path):
        arm = FakeArm(SimClock(), cfg, (300, 50), reach=None)
        arm.follow_path(path, 55, 100)
        return arm.calls[-1][1]

    ring = [(300 + 45 * math.cos(2 * math.pi * k / 36), 150 + 45 * math.sin(2 * math.pi * k / 36)) for k in range(37)]
    c, s = math.cos(math.radians(135)), math.sin(math.radians(135))
    cases = [
        ([(300, 100), (480, 100)], 2.98),  # L1
        (ring, 5.29),  # 원 r45
        ([(300, 100), (390, 100), (390, 190)], 3.94),  # 90° 꺾임
        ([(300, 100), (480, 100), (480 + 127 * c, 100 + 127 * s)], 5.42),  # 135° 꺾임
        ([(300, 100), (480, 100), (390, 190), (390, 10)], 8.39),  # L3+L4 (모서리 2.5mm로 깎아 서지 않고 지난다)
        ([(300, 100), (380, 100), (300, 100)], 3.88),  # 180° 되돌아감: 서서 돈다
    ]
    for path, want in cases:
        assert secs(path) == pytest.approx(want, abs=0.05)


def test_empty_scan_adds_extra_sweeps_and_rescan_runs_four(tmp_path):
    """첫 바퀴에 후보가 없으면 empty_extra_sweeps(3)바퀴를 더 돌려 합친다. 있으면 1바퀴만. RESCAN은 4바퀴."""
    cfg, clk, esp, arm = build([])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    assert src.scan().raw["passes"] == 1 + cfg.radar.empty_extra_sweeps
    esp.boxes.append(Box(300, 170, 0))
    assert src.scan().raw["passes"] == 1
    assert src.rescan().raw["passes"] == cfg.radar.prefetch_sweeps


def test_max_fail_per_run_gives_up(tmp_path):
    """check.max_fail_per_run: 확인 실패가 그만큼이면 give_up. begin_cycle()이 비운다. 0이면 끔."""
    cfg, clk, esp, arm = build([Box(300, 170, 0)], max_fail_per_run=1)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    assert not src.give_up
    esp.boxes.clear()
    do_check(src, arm, cand)  # 물체 없음 = 확인 실패 한 번
    assert src.give_up
    src.begin_cycle()
    assert not src.give_up
    cfg0, clk0, esp0, arm0 = build([Box(300, 170, 0)])  # 기본 0 = 끔
    src0 = make_source(cfg0, clk0, esp0, arm0, tmp_path)
    cand0 = src0.scan().objects[0]
    esp0.boxes.clear()
    do_check(src0, arm0, cand0)
    assert not src0.give_up


def test_connect_retries_hello_while_esp32_boots(tmp_path):
    """포트를 열면 ESP32가 리셋되어 처음 몇 번의 hello가 사라진다. 0.5초마다 다시 보내 약 5초까지 기다린다."""
    from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
    from so101_studio.perception.ultrasonic.link import Link, UltrasonicLinkError
    from so101_studio.perception.ultrasonic.model import load_config
    from so101_studio.perception.ultrasonic.source import UltrasonicSource

    cfg = load_config()
    clk = SimClock()
    src = UltrasonicSource(Link(FakeESP32(clk, [], cfg, ignore_first_writes=3), clk), cfg, arm_origin_mm=(300, 50),
                           baseline_path=tmp_path / "b.json")
    src.connect()  # 처음 3번은 무시되고 4번째에 응답
    assert clk.t_ns >= 1_400_000_000  # 0.5초 x 3번 기다렸다
    clk2 = SimClock()
    dead = UltrasonicSource(Link(FakeESP32(clk2, [], cfg, ignore_first_writes=100), clk2), cfg, arm_origin_mm=(300, 50),
                            baseline_path=tmp_path / "b2.json")
    with pytest.raises(UltrasonicLinkError, match="hello"):
        dead.connect()
    assert 4.5e9 <= clk2.t_ns <= 6e9  # 약 5초 시도


def test_wait_pose_follows_radar_clear_in_robot_yaml(tmp_path):
    """결정 0039: 벤치마크 대기 자세는 robot.yaml의 poses.radar_clear다. 값이 바뀌면 대기 자세도 바뀌고, 키가 없으면 팔 원점 위 150mm."""
    import yaml

    from so101_studio.perception.ultrasonic.benchmark import FakeArm, radar_clear_pose
    from so101_studio.perception.ultrasonic.fake_esp32 import SimClock
    from so101_studio.perception.ultrasonic.model import load_config

    def robot_yaml(name, joints):
        d = tmp_path / name
        d.mkdir()
        cfg = yaml.safe_load(Path("config/robot.yaml").read_text(encoding="utf-8"))
        cfg["poses"].pop("radar_clear", None)  # 실제 robot.yaml에 키가 있어도 복사본은 시험이 정한 값만
        if joints:
            cfg["poses"]["radar_clear"] = joints
        (d / "robot.yaml").write_text(yaml.safe_dump(cfg), encoding="utf-8")
        return str(d / "robot.yaml")

    assert radar_clear_pose(robot_yaml("none", None)) is None
    a = radar_clear_pose(robot_yaml("a", [0, -90, 93, 0, 0, 3]))
    b = radar_clear_pose(robot_yaml("b", [30, -60, 60, 0, 0, 3]))
    assert a != b
    arm = FakeArm(SimClock(), load_config(), (300, 50), wait_pose=b)
    assert arm.wait_pose == b
    default = FakeArm(SimClock(), load_config(), (300, 50))
    assert default.wait_pose == (radar_clear_pose() or ((300, 50), 150.0))


def test_run_cycle_goes_to_wait_pose_only_first_and_after_failed_pick(tmp_path, monkeypatch):
    """결정 0041·기획 #107: radar_clear는 첫 사이클(첫 scan 전), RESCAN, 집기 실패 뒤에만 간다. PLACE 뒤에는 안 간다."""
    from types import SimpleNamespace

    from so101_studio.perception.ultrasonic import benchmark as bm

    wait = ((300.0, 95.0), 320.0)

    def waits(rescan_home, fail_pick):
        cfg, clk, esp, arm = build([Box(300, 170, 0)], arm_kw={"wait_pose": wait})
        src = make_source(cfg, clk, esp, arm, tmp_path)
        moves, move = [], arm.move
        arm.move = lambda to, z, speed=None: (moves.append(tuple(to)), move(to, z, speed))[1]
        if fail_pick:  # 확인 결과가 실제 물체에서 100mm 벗어나 집기 실패(miss)
            real = bm.do_check
            monkeypatch.setattr(bm, "do_check", lambda s, a, c: (SimpleNamespace(x_mm=c.x_mm + 100, y_mm=c.y_mm, id="x", angle_deg=0), *real(s, a, c)[1:]))
        bm.run_cycle(src, arm, esp, rescan_home=rescan_home)
        monkeypatch.undo()
        return moves.count(wait[0]), moves

    n, _ = waits(False, False)
    assert n == 1  # 첫 scan 전만. 집기 성공 뒤 PLACE에서는 안 간다
    n, _ = waits(True, False)
    assert n == 2  # 첫 scan 전 + 후보가 없을 때의 RESCAN
    n, moves = waits(False, True)
    assert n >= 2 and (500.0, 150.0) not in moves  # 첫 scan 전 + 집기 실패마다(같은 자리 두 번째 실패면 3), 놓으러 가지 않는다


def test_kin_reach_hover_band_and_trim_start():
    """--reach kin: 첫 점은 호버(z+40mm)까지 풀려야 한다. 반경 250mm 점은 작업 높이로는 닿아도 호버가 안 닿아 거부, --trim-start면 안쪽으로 당겨 간다. 기본(반경 판정)은 그대로 받는다."""
    from so101_studio.perception.ultrasonic.benchmark import FakeArm, kin_reach
    from so101_studio.perception.ultrasonic.fake_esp32 import SimClock
    from so101_studio.perception.ultrasonic.model import load_config

    far, path = (300, 300), [(300, 300), (300, 150)]  # 팔 원점 (300, 50)에서 반경 250
    k = kin_reach()
    assert k.ok(far, 55) and not k.start_ok(far, 55)
    cfg = load_config()

    def run(**kw):
        arm = FakeArm(SimClock(), cfg, (300, 50), **kw)
        return arm, arm.follow_path(path, 55, 100)

    _, poses = run()
    assert poses  # 반경 판정(50~265)은 받는다
    arm, poses = run(kin=k)
    assert poses == [] and arm.last_path["reason"] == "unreachable"
    arm, poses = run(kin=k, trim_start=True)
    assert poses and math.hypot(poses[0].tool_mm[0] - 300, poses[0].tool_mm[1] - 50) < 250


def test_dxl_servo_reports_actual_angle_and_source_uses_it():
    # dxl 서보는 샘플 r에 읽은 각도 ang를 싣는다(갈 때 -lag). radar는 ang - a를 shift로 받아 후보 방위를 고친다
    import json

    from so101_studio.perception.ultrasonic import radar
    cfg, clk, esp, _ = _build([], model=None, esp_kw={"servo": "dxl", "servo_lag_deg": 3.0, "ang_noise_deg": 0.0})
    esp.write('{"cmd":"hello"}')
    esp.write('{"cmd":"sweep","from":10,"to":30,"step":4,"bounce":true,"sweeps":2,"max_mm":600}')
    clk.sleep(1.0)
    msgs = [json.loads(x) for x in esp.readlines()]
    assert next(m for m in msgs if m.get("t") == "hello")["servo"] == "dxl"
    r = [m for m in msgs if m.get("t") == "r" and m["a"] is not None]
    assert r and r[0]["ang"] == r[0]["a"] - 3.0
    base = radar.Baseline({a: 400.0 for a in range(10, 31, 4)}, set())
    vals = {a: (300.0 if a in (18, 22) else 400.0) for a in range(10, 31, 4)}
    plain, _ = radar.find_candidates(vals, base, cfg)
    shifted, _ = radar.find_candidates(vals, base, cfg, {a: -3.0 for a in vals})
    assert shifted[0].servo_deg == plain[0].servo_deg - 3.0


def test_connect_switches_firmware_servo_driver_to_config():
    # 펌웨어는 켤 때 pwm이고, servo.driver 설정이 dxl이면 연결할 때 `driver` 명령으로 바꾼다(hello가 dxl로 돌아옴)
    from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
    from so101_studio.perception.ultrasonic.link import Link
    from so101_studio.perception.ultrasonic.model import load_config
    from so101_studio.perception.ultrasonic.source import UltrasonicSource
    clk = SimClock()
    cfg = load_config(servo={"driver": "dxl"})
    esp = FakeESP32(clk, [], cfg)
    assert esp.servo == "pwm"
    UltrasonicSource(Link(esp, clk), cfg, arm_origin_mm=(300.0, 50.0)).connect()
    assert esp.servo == "dxl"
    assert load_config().servo.driver == "pwm"  # 기본은 MG90S(pwm), 하위 호환


@pytest.mark.slow
def test_benchmark_seed_timeout_marks_seed_failed():
    """--seed-timeout: 제한 시간을 넘는 칸은 자식 프로세스를 끝내고 실패(timeout)로 적는다."""
    from so101_studio.perception.ultrasonic.benchmark import run_scenario

    (row,) = run_scenario("spread120", "optimistic", 1, kin_spec=(None, "parallel", True), seed_timeout=1)
    assert row["timeout"] and row["ok"] == 0 and row["recog_s"] == 1.0


def test_tool_reach_by_gripper():
    """그리퍼별 도달 범위: 병렬은 70~250, 단동식·없음은 60~255(기본), 명시한 값이 가장 우선."""
    from so101_studio.perception.ultrasonic.model import load_config

    assert load_config().check.tool_reach_mm == {"min": 60, "max": 255}
    assert load_config(gripper="single_jaw").check.tool_reach_mm == {"min": 60, "max": 255}
    assert load_config(gripper="parallel").check.tool_reach_mm == {"min": 70, "max": 250}
    assert load_config(gripper="parallel", tool_reach_mm={"min": 50, "max": 265}).check.tool_reach_mm == {"min": 50, "max": 265}


def test_benchmark_gripper_flows_into_check_reach():
    """벤치마크 build(gripper=...)가 실제 CHECK와 같은 길(load_config)로 그리퍼별 값을 읽는다."""
    from so101_studio.perception.ultrasonic.benchmark import build

    assert build([], gripper="parallel")[0].check.tool_reach_mm.min == 70
    assert build([])[0].check.tool_reach_mm.min == 60


@pytest.mark.slow  # 팔 서비스를 만들어 각 약 11초
@pytest.mark.parametrize(("gripper", "lo", "hi"), [("single_jaw", 60, 255), ("parallel", 70, 250)])
def test_make_ultrasonic_source_reads_reach_from_arm_gripper(gripper, lo, hi, tmp_path):
    """service/ultrasonic.py의 한 줄: 팔 cfg의 그리퍼 종류가 check.tool_reach_mm(그리퍼별)로 이어진다."""
    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.core.config import load_robot, load_workspace
    from so101_studio.service.arm_service import ArmService
    from so101_studio.service.ultrasonic import make_ultrasonic_source

    local = tmp_path / "local.yaml"  # 그리퍼 종류만 바꾼 설정(이 PC의 local.yaml은 읽지 않는다)
    local.write_text(f"gripper: {gripper}\n", encoding="utf-8")
    cfg = load_robot("config/robot.yaml", local_path=local)
    arm = ArmService(SimDriver(cfg.poses["home"]), cfg, load_workspace("config/workspace.yaml"), speed_pct=100, log_root=str(tmp_path / "runs"))
    assert arm.connect()[0]
    src = make_ultrasonic_source(arm, sim=True, boxes=[])
    assert (src.cfg.check.tool_reach_mm.min, src.cfg.check.tool_reach_mm.max) == (lo, hi)
    arm.disconnect()


def test_clip_pieces_drop_collinear_points():
    """_clip: 도달 범위로 잘린 조각은 일직선 중간 점(2mm 판정 점)을 빼고 꺾이는 점·양 끝만 팔에 준다(fillet_path가 2mm마다 서지 않게)."""
    from so101_studio.perception.ultrasonic.check import _straighten

    line = [(float(x), 0.0) for x in range(0, 21, 2)]
    assert _straighten(line) == [(0.0, 0.0), (20.0, 0.0)]
    bent = [*line, *[(20.0, float(y)) for y in range(2, 9, 2)]]
    assert _straighten(bent) == [(0.0, 0.0), (20.0, 0.0), (20.0, 8.0)]
    assert _straighten([(0.0, 0.0), (5.0, 0.0), (0.0, 0.0)]) == [(0.0, 0.0), (5.0, 0.0), (0.0, 0.0)]  # 되돌아가는 점은 그대로
    assert _straighten([(0.0, 0.0), (1.0, 1.0)]) == [(0.0, 0.0), (1.0, 1.0)]


def test_clipped_ring_pieces_are_short(tmp_path):
    """잘린 원 조각은 판정 점(2mm 간격, 수십 개)이 아니라 꼭짓점만 남는다."""
    cfg, clk, esp, arm = build([Box(300, 330, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    probe = src.start_check(src.scan().objects[0])
    ring = [(310 + 60 * math.cos(2 * math.pi * k / 36), 330 + 60 * math.sin(2 * math.pi * k / 36)) for k in range(36)]
    ring.append(ring[0])
    pieces = probe._clip([(x, y - 130) for x, y in ring])  # 팔 원점 쪽으로 내려 도달 범위에 걸치게
    assert pieces and all(len(pc) <= 37 for pc in pieces)
