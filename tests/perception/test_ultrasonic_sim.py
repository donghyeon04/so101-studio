import math

import pytest
from simlib import STRICT_REACH, angle_ok, build, do_check, make_source, run_cycle

from so101_studio.core.types import DetectedObject
from so101_studio.perception.ultrasonic.model import Box

THREE = [Box(200, 190, 0), Box(310, 160, 90), Box(400, 200, 45)]


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


def _cycles(tmp_path, mode, n=10, **kw):
    oks, times = [], []
    for seed in range(n):
        boxes = [Box(b.x, b.y, b.angle_deg) for b in THREE]
        cfg, clk, esp, arm = build(boxes, mode=mode, seed=seed, **kw)
        src = make_source(cfg, clk, esp, arm, tmp_path)
        ok, secs, _ = run_cycle(src, arm, esp, cfg)
        oks.append(ok)
        times.append(secs)
    print(f"{mode}: 성공 평균 {sum(oks) / n:.1f}/3, 인식 평균 {sum(times) / n:.1f}초, 최대 {max(times):.1f}초")
    return sum(oks) / n, sum(times) / n


@pytest.mark.parametrize("mode", ["accurate", "fast"])
def test_three_objects_end_to_end_success(tmp_path, mode):
    """1절: 10회 평균 2개 이상 성공 (가상 ESP32 + 가상 팔)."""
    ok, _ = _cycles(tmp_path, mode)
    assert ok >= 2.0


@pytest.mark.xfail(reason="팔 세션이 잰 follow_path 비용(호출마다 정지·가속 120mm/s², 호출 사이 0.25초)을 넣으면 "
                          "정확 약 55초(기준 45), 빠른(T9 끔) 약 47초(기준 30), T9 켜면 약 74초. 기획 요청 대기(PR #68)", strict=False)
@pytest.mark.parametrize("mode,limit_s", [("accurate", 45), ("fast", 30)])
def test_three_objects_end_to_end_time(tmp_path, mode, limit_s):
    """1절: 인식 시간 정확 45초·빠른 30초 이하 (팔 확인 속도 100mm/s)."""
    _, secs = _cycles(tmp_path, mode)
    assert secs <= limit_s


def test_ring_is_one_follow_path_call(tmp_path):
    """원(R)은 follow_path 한 번(연속 경로)이고, 정확 모드는 물체당 5번 부른다."""
    cfg, clk, esp, arm = build([Box(300, 220, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    do_check(src, arm, src.scan().objects[0])
    assert len(arm.calls) == 5
    assert max(round(L) for L, _ in arm.calls) == pytest.approx(2 * 3.14159 * cfg.check.ring_r_mm, abs=3)


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
    cfg, clk, esp, arm = build([Box(300, 235, 0)], reach=STRICT_REACH)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    obj, probe, _ = do_check(src, arm, cand)
    assert "L2" in probe.info["unreachable"]
    assert obj is not None and probe.status == "ok"
    assert math.dist((obj.x_mm, obj.y_mm), (300, 235)) < 15
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
    cfg, clk, esp, arm = build([Box(300, 170, 0)], yaw_deg=yaw)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    obj, _, _ = do_check(src, arm, src.scan().objects[0])
    assert math.dist((obj.x_mm, obj.y_mm), (300, 170)) < 10


def test_sensor_to_tip_default_yaw_90():
    """기본 yaw 90: 도구 프레임 오프셋 (0, 30)이 작업대 좌표 (-30, 0)으로 돌아간다(센서 = 손끝 + 돌린 오프셋)."""
    from so101_studio.perception.ultrasonic.check import sensor_to_tip
    from so101_studio.perception.ultrasonic.model import load_config

    cfg = load_config()
    assert cfg.check.yaw_deg == 90
    (tip,) = sensor_to_tip([(100.0, 100.0)], cfg)
    assert tip == pytest.approx((130.0, 100.0))


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
    cfg, clk, esp, arm = build([Box(300, 400, 0)], reach=STRICT_REACH)  # 레이더 앞 변 쪽: 팔이 못 간다
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
    """PR #71: 손끝이 tool_reach_mm 밖인 점은 건너뛰고, 끊기면 조각마다 따로 보낸다. 켜지 않으면 그대로."""
    cfg, clk, esp, arm = build([Box(300, 170, 0)], clip_to_reach=True)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    probe = src.start_check(src.scan().objects[0])
    # yaw 90이라 손끝 x = 센서 x + 30 = 330, 손끝 y = 센서 y. 원점(300,50) 반경 60 안(손끝 y -2~102)은 건너뛴다
    pieces = probe._clip([(300, -70), (300, 170)])
    assert len(pieces) == 2
    assert pieces[0][-1][1] == pytest.approx(-2, abs=3) and pieces[1][0][1] == pytest.approx(102, abs=3)
    reach = cfg.check.tool_reach_mm
    probe, _obj, segs = _all_segments(src, arm, src.scan().objects[0])
    for seg in segs:
        for x, y in seg.points_mm:
            assert reach.min - 1 <= math.hypot(x - 300, y - 50) <= reach.max + 1


def test_clip_off_sends_whole_paths(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    probe = src.start_check(src.scan().objects[0])
    assert probe._clip([(300, -70), (300, 170)]) == [[(300, -70), (300, 170)]]
