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


@pytest.mark.parametrize("mode,limit_s", [("accurate", 45), ("fast", 30)])
def test_three_objects_end_to_end(tmp_path, mode, limit_s):
    """1절: 10회 평균 2개 이상 성공, 인식 시간 정확 45초·빠른 30초 이하 (팔 확인 속도 100mm/s)."""
    oks, times = [], []
    for seed in range(10):
        boxes = [Box(b.x, b.y, b.angle_deg) for b in THREE]
        cfg, clk, esp, arm = build(boxes, mode=mode, seed=seed)
        src = make_source(cfg, clk, esp, arm, tmp_path)
        ok, secs, _ = run_cycle(src, arm, esp, cfg)
        oks.append(ok)
        times.append(secs)
    print(f"{mode}: 성공 평균 {sum(oks) / 10:.1f}/3, 인식 평균 {sum(times) / 10:.1f}초, 최대 {max(times):.1f}초")
    assert sum(oks) / 10 >= 2.0
    assert sum(times) / 10 <= limit_s


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


def test_yaw_rotates_sensor_offset(tmp_path):
    """손목 yaw 90도면 센서 오프셋이 돌아가므로 손끝 경로가 달라져도 확인 결과는 같다."""
    cfg, clk, esp, arm = build([Box(300, 170, 0)], yaw_deg=90)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    obj, _, _ = do_check(src, arm, src.scan().objects[0])
    assert math.dist((obj.x_mm, obj.y_mm), (300, 170)) < 10
