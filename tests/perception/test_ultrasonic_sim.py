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
    for x in (140, 195, 250):
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


def test_workspace_view_events(tmp_path):
    """8.9절(REQ-109): us.sweep_progress를 한 바퀴 이으면 us.sweep 각도와 같다(빠짐·겹침 없음), 후보에 search_r_mm·values,
    us.check_estimate가 L1·L2·R·L34 순서이고 마지막 center = us.check_result 위치, Probe가 낸 구간마다 stage가 있다."""
    from so101_studio.core.events import EventBus

    cfg, clk, esp, arm = build([Box(300, 170, 0)])
    bus, ev = EventBus(), []
    for t in ("us.sweep_progress", "us.sweep", "us.candidates", "us.check_estimate", "us.check_result"):
        bus.subscribe(t, lambda p, t=t: ev.append((t, p)))
    src = make_source(cfg, clk, esp, arm, tmp_path)
    src.bus = bus
    cand = src.scan().objects[0]
    _, probe, _ = do_check(src, arm, cand)

    acc, sweeps = [], 0
    for t, p in ev:
        if t == "us.sweep_progress":
            assert {"pass", "total", "dir", "servo_deg", "t_ns", "angles", "d"} <= set(p) and len(p["angles"]) == len(p["d"])
            acc += p["angles"]
        elif t == "us.sweep":
            assert acc == p["angles"] and p["angles"]
            acc, sweeps = [], sweeps + 1
    assert sweeps >= 1
    c = next(p for t, p in ev if t == "us.candidates")
    assert c["objects"][0]["search_r_mm"] == cfg.check.line_mm / 2 and isinstance(c["values"], dict) and c["values"]

    est = [p for t, p in ev if t == "us.check_estimate"]
    assert [e["stage"] for e in est] == ["L1", "L2", "R", "L34"]
    keys = {"id", "stage", "center", "angle_deg", "strength", "size_mm", "spread_mm"}
    assert all(set(e) == keys and e["id"] == cand.id for e in est)
    res = next(p for t, p in ev if t == "us.check_result")["object"]
    assert est[-1]["center"] == pytest.approx((res["x_mm"], res["y_mm"])) and est[-1]["spread_mm"] == 0
    assert est[0]["angle_deg"] is None and est[2]["strength"] is not None
    assert probe.segments and all(s["segment"].stage for s in probe.segments)


def test_square_object_has_no_angle(tmp_path):
    cfg, clk, esp, arm = build([Box(215, 170, 0, width=40, length=40)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    obj, _, _ = do_check(src, arm, src.scan().objects[0])
    assert isinstance(obj, DetectedObject) and obj.angle_deg is None
    assert math.dist((obj.x_mm, obj.y_mm), (215, 170)) < 10


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
    """원(R)은 follow_path 한 번(연속 경로)이다. 물체당 4번(L1, L2, 원, L3+L4) 부르고, 도달 범위(100~255)에 걸려 선이 끊기면 조각마다 더 부른다."""
    cfg, clk, esp, arm = build([Box(215, 190, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    _, probe, _ = do_check(src, arm, src.scan().objects[0])
    assert len(arm.calls) >= 4
    lengths = sorted(L for L, _ in arm.calls)
    r = probe.info["ring_r"]  # 결정 0075: 반지름은 L1·L2 구간 길이로 정한다(ring_auto_mm)
    assert 35 <= r <= 75 and r != cfg.check.ring_r_mm
    assert sum(abs(L - 2 * math.pi * r) < 3 for L in lengths) == 1  # 원은 한 번


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
    cfg, clk, esp, arm = build([Box(215, 170, 0)])
    src = make_source(cfg, clk, esp, arm, tmp_path)
    src.prefetch(arm.links_at)
    clk.sleep(6)  # 놓는 동안
    res = src.scan()
    assert res.duration_s < 0.1 and len(res.objects) == 1
    assert res.raw["passes"] == cfg.radar.prefetch_sweeps


def test_unreachable_segment_is_skipped_not_deleted(tmp_path):
    """바깥쪽 물체: 세로선(L2)이 도달 밖이면 그 구간을 건너뛰고(unreachable 기록) 나머지로 확정한다."""
    cfg, clk, esp, arm = build([Box(250, 200, 0)], reach=STRICT_REACH)  # 새 팔 원점(215.2, 21.4)에서 L2가 반경 240 밖으로 나간다
    src = make_source(cfg, clk, esp, arm, tmp_path)
    cand = src.scan().objects[0]
    obj, probe, _ = do_check(src, arm, cand)
    assert "L2" in probe.info["unreachable"]
    assert obj is not None and probe.status == "ok"
    assert math.dist((obj.x_mm, obj.y_mm), (250, 200)) < 15
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
    """기본 yaw 90: 도구 프레임 오프셋 (0.6, 66.5)가 작업대 좌표 (-66.5, -0.6)으로 간다(센서 = 손끝 + 오프셋, 팔 tool_axes 규약: x축이 거울)."""
    from so101_studio.perception.ultrasonic.check import sensor_to_tip
    from so101_studio.perception.ultrasonic.model import load_config

    cfg = load_config()
    assert cfg.check.yaw_deg == 90
    (tip,) = sensor_to_tip([(100.0, 100.0)], cfg)
    assert tip == pytest.approx((166.5, 100.6))


def test_sensor_offset_matches_arm_tool_axes():
    """check.sensor_offset_xy가 팔 ArmService.sensor_position의 변환(kin.tool_axes(q) @ off)과 같다. off.x를 30으로 키워 거울 차이가 보이게 한다."""
    import numpy as np

    from so101_studio.perception.ultrasonic import benchmark as bm
    from so101_studio.perception.ultrasonic.check import sensor_offset_xy
    from so101_studio.perception.ultrasonic.model import load_config

    kr = bm.kin_reach(None, "parallel")
    cfg = load_config(gripper="parallel")
    cfg.arm_sensor_offset_mm.x = 30.0
    off = cfg.arm_sensor_offset_mm
    for yaw in (0, 45, 90, 135):
        tip = bm.tip_vector(kr.kin, -90.0, yaw, kr.seeds[0].gripper, "tip")
        pose = bm.point_to_pose(212, 200, 75, yaw, "table", kr.ws.base, kr.cfg.table_z_mm, tip)
        q = next(q for sd in kr.seeds if (q := kr.kin.ik(pose, sd)) is not None)
        d = kr.kin.tool_axes(q) @ np.array([off.x, off.y, 0.0])
        assert sensor_offset_xy(cfg, yaw) == pytest.approx((d[0], d[1]), abs=0.5)


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
    assert src.arm_origin == (215.2, 21.4)  # config/workspace.yaml robot_base_in_table


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
    cfg, clk, esp, arm = build([Box(215, 170, 0)], yaw_pivot_mm=0)  # yaw 고정 90
    src = make_source(cfg, clk, esp, arm, tmp_path)
    probe = src.start_check(src.scan().objects[0])
    # yaw 90이라 손끝 = 센서 + (66.5, -0.6). 센서 x 165.2면 손끝 x 231.7이라 원점(215.2, 21.4) 반경 100 안(손끝 y -77.2~120.0)은 건너뛴다
    pieces = probe._clip([(165.2, -100), (165.2, 140)])
    assert len(pieces) == 2
    assert pieces[0][-1][1] == pytest.approx(-77.2, abs=3) and pieces[1][0][1] == pytest.approx(120.0, abs=3)
    reach = cfg.check.tool_reach_mm
    probe, _obj, segs = _all_segments(src, arm, src.scan().objects[0])
    for seg in segs:
        for x, y in seg.points_mm:
            assert reach.min - 1 <= math.hypot(x - 215.2, y - 21.4) <= reach.max + 1


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
        cfg, clk, esp, arm = build([Box(215, 170, 0)], arm_kw={"wait_pose": wait})
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
    assert n >= 2 and (355.0, 80.0) not in moves  # 첫 scan 전 + 집기 실패마다(같은 자리 두 번째 실패면 3), 놓으러 가지 않는다


def test_kin_reach_hover_band_reports_ik_fail():
    """--reach kin: 첫 점은 호버(z+40mm)까지 풀려야 한다. 반경 250mm 점은 작업 높이로는 닿아도 호버가 안 닿아 ik_fail로 거부된다. 기본(반경 판정)은 그대로 받는다."""
    from so101_studio.perception.ultrasonic.benchmark import FakeArm, kin_reach
    from so101_studio.perception.ultrasonic.fake_esp32 import SimClock
    from so101_studio.perception.ultrasonic.model import load_config

    far, path = (215.2, 271.4), [(215.2, 271.4), (215.2, 150)]  # 팔 원점 (215.2, 21.4)에서 반경 250
    k = kin_reach(gripper="single_jaw")  # 기본은 parallel (REQ-098)
    assert k.ok(far, 55) and not k.start_ok(far, 55)
    kp = kin_reach(gripper="parallel")
    assert not kp.ok((215.2, 141.4), 55 + 40) and kp.start_ok((215.2, 141.4), 55)  # 호버 40은 안 풀려도 35→…→10 중 하나가 풀리면 접근 가능(REQ-089)
    cfg = load_config()

    def run(**kw):
        arm = FakeArm(SimClock(), cfg, (215.2, 21.4), **kw)
        return arm, arm.follow_path(path, 55, 100)

    arm, poses = run()
    assert poses and arm.last_status == "ok"  # 반경 판정(50~265)은 받는다
    arm, poses = run(kin=k)
    assert poses == [] and arm.last_path["reason"] == "unreachable" and arm.last_status == "ik_fail"


def _pull_probe(tmp_path, **check):
    check.setdefault("line_yaw_alt_deg", [])  # 첫 점 당기기만 본다(결정 0075의 다른 yaw로 다시는 아래 시험)
    cfg, clk, esp, arm = build([Box(300, 235, 0)], **check)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    return src.start_check(src.scan().objects[0])


def test_ik_fail_tries_other_yaw_before_pull(tmp_path):
    """결정 0075: ik_fail이면 첫 점을 당기기 전에 같은 센서 경로를 다른 yaw로 먼저 다시 낸다(점을 잃지 않는다). 다른 yaw가 다 떨어지면 당긴다."""
    probe = _pull_probe(tmp_path, line_yaw_alt_deg=[-90, -45, 45, 90])
    seg = probe.next_segment()
    name, sensor = probe._cur[0], probe._cur[2]
    alts = list(probe._yaw_alts)
    assert alts  # 이 후보의 첫 선은 다른 yaw로도 도달 범위에 든다
    for y in alts:
        probe.feed([], "ik_fail")
        again = probe.next_segment()
        assert probe._cur[2] == sensor and again.yaw_deg == y != seg.yaw_deg and again.stage == name
    assert "pulled" not in probe.info or all(p["mm"] == 0 for p in probe.info["pulled"])
    probe.feed([], "ik_fail")
    probe.next_segment()
    assert probe.info["pulled"][-1]["mm"] == pytest.approx(2.0)  # 다른 yaw가 다 안 되면 첫 점 당기기


def test_first_point_pull_on_ik_fail(tmp_path):
    """U16: ik_fail이면 같은 단계를 첫 점만 첫 구간을 따라 2mm 당겨 다시 낸다. 다시 부른 경로는 끝 점이 같고 첫 구간만 짧아진다."""
    probe = _pull_probe(tmp_path)
    seg = probe.next_segment()
    first = seg.points_mm
    probe.feed([], "ik_fail")
    again = probe.next_segment()
    assert again is not seg and again.points_mm[1:] == first[1:]
    assert math.dist(first[0], again.points_mm[0]) == pytest.approx(2.0, abs=1e-6)
    assert math.dist(first[0], first[1]) - math.dist(again.points_mm[0], first[1]) == pytest.approx(2.0, abs=1e-6)
    assert "unreachable" not in probe.info


def test_first_point_pull_stops_at_max_and_other_statuses(tmp_path):
    """계속 ik_fail이면 합 20mm(단계마다 10번)까지만 당기고 그 단계를 unreachable로 한다. out_of_bounds 등은 다시 부르지 않는다."""
    probe = _pull_probe(tmp_path, retry_pull_max_mm=20)
    probe.next_segment()
    name, calls = probe._cur[0], 0
    while probe.next_segment() is not None and probe._cur[0] == name and calls < 30:
        calls += 1
        probe.feed([], "ik_fail")
    assert calls == 11 and probe.info["unreachable"][0] == name  # 처음 1번 + 당긴 10번
    assert probe.info["pulled"][0] == {"stage": name, "mm": pytest.approx(2.0)}
    assert len(probe.info["pulled"]) == 10 and probe.info["pulled"][-1]["mm"] == pytest.approx(20.0)

    probe = _pull_probe(tmp_path)
    probe.next_segment()
    name = probe._cur[0]
    probe.feed([], "out_of_bounds")
    assert probe.info["unreachable"] == [name] and "pulled" not in probe.info


def test_first_point_pull_off(tmp_path):
    probe = _pull_probe(tmp_path, retry_pull_max_mm=0)
    probe.next_segment()
    name = probe._cur[0]
    probe.feed([], "ik_fail")
    assert probe.info["unreachable"] == [name] and "pulled" not in probe.info


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

    (row,) = run_scenario("spread120", "optimistic", 1, kin_spec=(None, "parallel"), seed_timeout=1)
    assert row["timeout"] and row["ok"] == 0 and row["recog_s"] == 1.0


def test_tool_reach_by_gripper():
    """그리퍼별 도달 범위(REQ-090, 접근 높이 60 기준): 병렬은 95~230, 단동식·없음은 100~255(기본), 명시한 값이 가장 우선."""
    from so101_studio.perception.ultrasonic.model import load_config

    assert load_config().check.tool_reach_mm == {"min": 100, "max": 255}
    assert load_config(gripper="single_jaw").check.tool_reach_mm == {"min": 100, "max": 255}
    assert load_config(gripper="parallel").check.tool_reach_mm == {"min": 95, "max": 230}
    assert load_config(gripper="parallel", tool_reach_mm={"min": 50, "max": 265}).check.tool_reach_mm == {"min": 50, "max": 265}


def test_benchmark_gripper_flows_into_check_reach():
    """벤치마크 build(gripper=...)가 실제 CHECK와 같은 길(load_config)로 그리퍼별 값을 읽는다."""
    from so101_studio.perception.ultrasonic.benchmark import build

    assert build([], gripper="parallel")[0].check.tool_reach_mm.min == 95
    assert build([])[0].check.tool_reach_mm.min == 100


@pytest.mark.slow  # 팔 서비스를 만들어 각 약 11초
@pytest.mark.parametrize(("gripper", "lo", "hi"), [("single_jaw", 100, 255), ("parallel", 95, 230)])
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


def test_face_offset_polar_and_layout_on_plate():
    """H-4: 거리는 송수신기 앞면에서 재니 위치는 서보 축 + (face_offset + 거리). 시연 배치는 방위 45~135°, 판 안."""
    import random

    from so101_studio.core.config import load_workspace
    from so101_studio.perception.ultrasonic import benchmark as bm
    from so101_studio.perception.ultrasonic import radar
    cfg = bm.load_config()
    x, y = radar.polar(cfg, 90, 100)  # 서보 90° = 정면(heading -90°, 팔 쪽)
    assert (x, y) == pytest.approx((cfg.radar.station_mm.x, cfg.radar.station_mm.y - 100 - cfg.radar.face_offset_mm))
    ws, (bx, by) = load_workspace(), bm.arm_base()
    for seed in range(5):
        for b in bm.random_layout(random.Random(seed), "spread90"):
            assert bm.on_plate(b, ws) and 45 <= math.degrees(math.atan2(b.y - by, b.x - bx)) <= 135


def test_show80_layout():
    """REQ-099: show80은 반경 150~220, 모서리 80mm 이상, 판 안·통 회피(on_plate)를 지킨다."""
    import itertools
    import random

    from so101_studio.core.config import load_workspace
    from so101_studio.perception.ultrasonic import benchmark as bm
    ws, (bx, by) = load_workspace(), bm.arm_base()
    for seed in range(3):  # 배치 하나에 4초쯤 걸린다(edge_gap 다시 뽑기)
        boxes = bm.random_layout(random.Random(seed), "show80")
        assert len(boxes) == 3
        for b in boxes:
            assert bm.on_plate(b, ws) and 150 <= math.hypot(b.x - bx, b.y - by) <= 220
        assert all(bm.edge_gap(a, b) >= 80 for a, b in itertools.combinations(boxes, 2))


def test_layout_rejects_body_touching_bin_wall():
    """설계서 6절: 중심이 통 무시 원 밖이어도 몸체(긴 막대)가 통 벽에 걸리면 거른다."""
    from so101_studio.core.config import load_workspace
    from so101_studio.perception.ultrasonic import benchmark as bm
    z = next(iter(load_workspace().drop_zones.values()))
    assert bm._hits_zone(Box(z["x_mm"] + z["w_mm"] / 2 + 10, z["y_mm"], 0, 30, 100), z)  # 벽 두께 안으로 걸치는 막대
    assert not bm._hits_zone(Box(z["x_mm"], z["y_mm"] + 300, 0, 30, 100), z)


def test_local_yaml_overlays_ultrasonic_yaml(tmp_path):
    """ultrasonic.local.yaml이 같은 폴더에 있으면 키 단위로 겹치고(중첩 키 포함), 명시 인자가 가장 우선이다."""
    import shutil

    from so101_studio.perception.ultrasonic.model import load_config
    cfg_path = tmp_path / "ultrasonic.yaml"
    shutil.copy("config/ultrasonic.yaml", cfg_path)
    assert load_config(cfg_path).servo.driver == "pwm"
    (tmp_path / "ultrasonic.local.yaml").write_text("servo: {driver: dxl}\nport: COM9\nradar: {dip_mm: 77}\n", encoding="utf-8")
    cfg = load_config(cfg_path)
    assert cfg.servo.driver == "dxl" and cfg.port == "COM9" and cfg.radar.dip_mm == 77 and cfg.radar.merge_mm == 110
    assert load_config(cfg_path, servo={"driver": "pwm"}).servo.driver == "pwm"


def test_radar_check_lag_matches_fake_servo_lag(tmp_path):
    """radar_check.lag_deg: 가상 서보 지연 3°를 갈 때·올 때 호 중심 차이/2로 되찾는다(B4-1 도구)."""
    from so101_studio.perception.ultrasonic.radar_check import lag_deg
    cfg, clk, esp, arm = build([Box(207.3, 105.0, 0, width=60, length=60)], esp_kw={"servo_lag_deg": 3.0, "ang_noise_deg": 0.0})  # 정면(서보 축 아래) 약 300mm
    src = make_source(cfg, clk, esp, arm, tmp_path)
    res = lag_deg(src, 6)
    assert res["지연"] == pytest.approx(3.0, abs=1.0)


def _probe(cfg, cand_xy, origin=(215.0, 21.0)):
    from so101_studio.perception.ultrasonic.check import Probe
    from so101_studio.perception.ultrasonic.fake_esp32 import SimClock

    class Src:
        clock = SimClock()

    return Probe(Src(), DetectedObject("c", *cand_xy, None, 0, 0, 0.3, "ultrasonic"), cfg, origin)


def test_plan_splits_radial_line_over_two_yaws():
    """check.line_yaw_alt_deg: 팔 쪽을 향한 180mm 선은 손끝 도달 폭보다 길어 yaw 하나로는 한쪽 끝이 잘린다.
    기본(결정 0075, 켬)은 손목 각도 둘로 나눠 더 많이 덮고, 끄면([]) 후보 yaw 하나로 자른다."""
    from so101_studio.perception.ultrasonic.model import load_config

    line = [(215.0, 80.0), (215.0, 260.0)]  # 팔 원점(215, 21)에서 반경 59~239로 곧게 뻗은 선
    off = _probe(load_config(gripper="parallel", line_yaw_alt_deg=[]), (215.0, 170.0))
    assert {y for _, y in off._plan(line)} == {off.yaw}
    on = _probe(load_config(gripper="parallel"), (215.0, 170.0))  # 결정 0075 기본(켬)
    plan = on._plan(line)
    assert len({y for _, y in plan}) == 2
    def covered(pl):
        return sum(math.dist(pc[0], pc[-1]) for pc, _ in pl)
    assert covered(plan) >= covered(off._plan(line)) + 10  # 152 -> 170mm
    alts = on._alt_yaws(plan[0][0], plan[0][1])  # ik_fail 때 다시 낼 각도: 조각이 도달 범위에 다 드는 것만, 가까운 순
    assert plan[0][1] not in alts and alts == sorted(alts, key=lambda y: abs((y - plan[0][1] + 180) % 360 - 180))


def test_long_echo_run_is_trimmed_to_the_candidate_side():
    """8.10 N2: 이웃이 붙어 긴 구간은 후보 쪽만 쓴다(기본 끔이면 그대로)."""
    from so101_studio.perception.ultrasonic.check import Probe
    from so101_studio.perception.ultrasonic.model import load_config

    def probe(**check):
        p = object.__new__(Probe)
        p.cfg = load_config(**check)
        return p

    line = [(s, 0.0, 154.5, 154.5 - 40) for s in range(30, 160, 2)]  # 직선 위 130mm 내내 높이 40: 물체 둘이 붙었다
    plain, _, _ = probe()._analyze(line, (0.0, 0.0), (1.0, 0.0), [])
    trimmed, _, _ = probe(max_extent_mm=60)._analyze(line, (0.0, 0.0), (1.0, 0.0), [])
    assert not plain["merged"] and trimmed["merged"] and trimmed["extent"] < plain["extent"]
