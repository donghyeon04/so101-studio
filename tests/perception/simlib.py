"""가상 팔과 작업 한 사이클(scan -> CHECK -> pick -> place+prefetch). 팔 세션의 REQ-023·025가 오기 전의 시험용 대역이다."""
import itertools
import math
from bisect import bisect_right

from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
from so101_studio.perception.ultrasonic.link import Link
from so101_studio.perception.ultrasonic.model import (
    PoseSample,
    load_config,
)
from so101_studio.perception.ultrasonic.source import UltrasonicSource

BASE = (300.0, 50.0)  # 팔 원점(작업대 좌표)
DT = 0.02  # 50Hz
TOOL_V_MAX = 100.0  # robot.yaml tool_v_max_mm_s


class FakeArm:
    # 기본은 도달 제한 없음. 실제에 가까운 범위(REQ-023: 반경 100~240mm, 각도 25~155°)는 STRICT_REACH
    # follow_path 시간 모델(팔 세션 측정, dev/arm-req-023 measurements.md): 호출마다 정지에서 시작·정지로 끝, 가속 120mm/s²,
    # 호출 사이 0.25초. motion_scale > 1이면 측정이 모델보다 느린 만큼(180mm 직선 측정 3.63초 / 모델 2.77초) 여유를 준다.
    def __init__(self, clk, cfg, travel_mm_s=400.0, reach=None, accel=120.0, gap_s=0.25, motion_scale=1.0):
        self.accel, self.gap_s, self.motion_scale = accel, gap_s, motion_scale
        self.calls = []  # (경로 길이 mm, 걸린 초)
        self.reach = reach
        self.last_path = None
        self.clk, self.travel = clk, travel_mm_s
        self.off0 = (cfg.arm_sensor_offset_mm.x, cfg.arm_sensor_offset_mm.y, cfg.arm_sensor_offset_mm.z)
        self.yaw = 90.0  # 평소 손목 자세
        self.tip = BASE
        self.z = 150.0
        self.hist = [(0, BASE, self.z)]  # (t_ns, 손끝 xy, 손끝 z)

    def _sensor(self, xy, z):
        a = math.radians(self.yaw)
        x, y, oz = self.off0
        return xy[0] + x * math.cos(a) - y * math.sin(a), xy[1] + x * math.sin(a) + y * math.cos(a), z + oz

    def _tick(self, xy, z):
        self.clk.sleep(DT)
        self.tip, self.z = xy, z
        self.hist.append((self.clk.now_ns(), xy, z))
        return PoseSample(self.clk.now_ns(), (*xy, z), self._sensor(xy, z))

    def reachable(self, xy):
        if self.reach is None:
            return True
        r0, r1, a0, a1 = self.reach
        dx, dy = xy[0] - BASE[0], xy[1] - BASE[1]
        return r0 <= math.hypot(dx, dy) <= r1 and a0 <= math.degrees(math.atan2(dy, dx)) <= a1

    def _at(self, t):
        ts = [h[0] for h in self.hist]
        i = min(max(bisect_right(ts, t) - 1, 0), len(self.hist) - 1)
        return self.hist[i]

    def sensor_at(self, t):
        _, xy, z = self._at(t)
        return self._sensor(xy, z)

    def links_at(self, t):
        _, xy, _ = self._at(t)
        return [BASE, ((BASE[0] + xy[0]) / 2, (BASE[1] + xy[1]) / 2), xy, self._sensor(xy, 0)[:2]]

    def _walk(self, pts, z, speed, record):
        out = []
        for a, b in itertools.pairwise(pts):
            L = math.dist(a, b)
            n = max(1, round(L / speed / DT))
            for k in range(1, n + 1):
                p = self._tick((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n), z)
                if record:
                    out.append(p)
        return out

    def _profile_walk(self, pts, z, vmax):
        """정지에서 출발해 가속(accel)·순항(vmax)·감속으로 폴리라인 전체를 한 번에 간다."""
        seg = [math.dist(a, b) for a, b in itertools.pairwise(pts)]
        L = sum(seg)
        a = self.accel / self.motion_scale**2  # 시간을 motion_scale배로 늘린다
        v = min(vmax / self.motion_scale, math.sqrt(a * L)) if L else 0.0
        ta, tc = v / a, (L - v * v / a) / v if v else 0.0
        T = 2 * ta + tc
        out, t = [], 0.0
        while t < T - 1e-9:
            t = min(t + DT, T)
            s = 0.5 * a * t * t if t < ta else (v * v / (2 * a) + v * (t - ta) if t < ta + tc else L - 0.5 * a * (T - t) ** 2)
            i, rem = 0, s
            while i < len(seg) - 1 and rem > seg[i]:
                rem -= seg[i]
                i += 1
            f = rem / seg[i] if seg[i] else 1.0
            a_, b_ = pts[i], pts[i + 1]
            out.append(self._tick((a_[0] + (b_[0] - a_[0]) * f, a_[1] + (b_[1] - a_[1]) * f), z))
        return out

    def move(self, to, z, speed=None):
        self._walk([self.tip, to], z, speed or self.travel, False)

    def follow_path(self, pts, z, speed, yaw=None):
        bad = [p for p in pts if not self.reachable(p)]
        if bad:
            self.last_path = {"reason": "unreachable", "points": bad}  # REQ-023: 못 가는 경로는 움직이지 않고 []
            return []
        if yaw is not None:
            self.yaw = yaw
        speed = min(speed, TOOL_V_MAX)
        if math.dist(self.tip, pts[0]) > 0.5 or abs(self.z - z) > 0.5:
            self.move(pts[0], z)
        self.clk.sleep(self.gap_s)  # 호출 사이 정지
        t0 = self.clk.now_ns()
        first = PoseSample(self.clk.now_ns(), (*pts[0], z), self._sensor(pts[0], z))
        poses = [first, *self._profile_walk(pts, z, speed)]
        self.calls.append((sum(math.dist(a, b) for a, b in itertools.pairwise(pts)), (self.clk.now_ns() - t0) / 1e9))
        return poses


STRICT_REACH = (100, 240, 25, 155)


def build(boxes, mode="accurate", seed=0, reach=None, arm_kw=None, **check):
    cfg = load_config(mode=mode, **check)
    clk = SimClock()
    arm_holder = {}
    esp = FakeESP32(clk, boxes, cfg, seed=seed, arm_sensor_fn=lambda t: arm_holder["a"].sensor_at(t),
                    arm_links_fn=lambda t: arm_holder["a"].links_at(t))
    arm = arm_holder["a"] = FakeArm(clk, cfg, **({"reach": reach} if reach else {}), **(arm_kw or {}))
    return cfg, clk, esp, arm


def make_source(cfg, clk, esp, arm, tmp_path, calibrate=True):
    src = UltrasonicSource(Link(esp, clk), cfg, arm_origin_mm=BASE, arm_links_fn=arm.links_at,
                           baseline_path=tmp_path / "baseline.json")
    src.connect()
    if calibrate:
        boxes, esp.boxes = esp.boxes, []  # 빈 작업대로 기준값
        src.calibrate()
        esp.boxes = boxes
    return src


def do_check(src, arm, cand):
    probe = src.start_check(cand)
    t0 = arm.clk.now_ns()
    while (seg := probe.next_segment()) is not None:
        probe.feed(arm.follow_path(seg.points_mm, seg.tool_z_mm, seg.speed_mm_s, seg.yaw_deg))
    return probe.result(), probe, (arm.clk.now_ns() - t0) / 1e9


def angle_ok(est, true_deg):
    if est is None:
        return False
    d = abs(est - true_deg) % 180
    return min(d, 180 - d) <= 22.5


def run_cycle(src, arm, esp, cfg, place_s=5.0, drop=(500.0, 150.0), max_iter=8):
    """처음 scan()부터 물체가 다 없어질 때까지. 돌려주는 것: 성공한 물체 수, 인식 시간(초), 기록. 집기·놓기 시간은 뺀다."""
    clk, recog, log, ok = arm.clk, 0.0, [], 0
    for _ in range(max_iter):
        t0 = clk.now_ns()
        scan = src.scan()
        recog += (clk.now_ns() - t0) / 1e9
        near = [o for o in scan.objects if 90 <= math.dist((o.x_mm, o.y_mm), BASE) <= 250]
        if not near:
            break
        cand = min(near, key=lambda o: math.dist((o.x_mm, o.y_mm), BASE))
        obj, _probe, dt = do_check(src, arm, cand)
        recog += dt
        if obj is None:
            log.append(("fail", cand))
            continue
        box = min(esp.boxes, key=lambda b: math.dist((b.x, b.y), (obj.x_mm, obj.y_mm)), default=None)
        err = math.dist((box.x, box.y), (obj.x_mm, obj.y_mm)) if box else 1e9
        square = box is not None and box.width == box.length
        good = box is not None and err <= 15 and (square or angle_ok(obj.angle_deg, box.angle_deg % 180))
        log.append(("ok" if good else "bad", obj, err, box))
        ok += good
        if err <= 25:  # 집기 성공: 물체가 작업대에서 사라진다
            esp.boxes.remove(box)
        src.notify_picked(obj.id)
        arm.move((obj.x_mm, obj.y_mm), 55)
        src.prefetch(arm.links_at)
        arm.move(drop, 55)
        clk.sleep(max(place_s - math.dist((obj.x_mm, obj.y_mm), drop) / arm.travel, 0))
    return ok, recog, log
