"""가상 팔과 작업 한 사이클(scan -> CHECK -> pick -> place+prefetch). 팔 세션의 REQ-023·025가 오기 전의 시험용 대역이다."""
import itertools
import math
from bisect import bisect_right

from so101_studio.perception.ultrasonic.fake_esp32 import FakeESP32, SimClock
from so101_studio.perception.ultrasonic.link import Link
from so101_studio.perception.ultrasonic.model import PoseSample, load_config
from so101_studio.perception.ultrasonic.source import UltrasonicSource

BASE = (300.0, 50.0)  # 팔 원점(작업대 좌표)
DT = 0.02  # 50Hz


class FakeArm:
    def __init__(self, clk, cfg, travel_mm_s=400.0):
        self.clk, self.off, self.travel = clk, cfg.arm_sensor_offset_mm, travel_mm_s
        self.tip = BASE
        self.z = 150.0
        self.hist = [(0, BASE, self.z)]  # (t_ns, 손끝 xy, 손끝 z)

    def _tick(self, xy, z):
        self.clk.sleep(DT)
        self.tip, self.z = xy, z
        self.hist.append((self.clk.now_ns(), xy, z))
        return PoseSample(self.clk.now_ns(), (xy[0] + self.off.x, xy[1] + self.off.y, z + self.off.z))

    def _at(self, t):
        ts = [h[0] for h in self.hist]
        i = min(max(bisect_right(ts, t) - 1, 0), len(self.hist) - 1)
        return self.hist[i]

    def sensor_at(self, t):
        _, xy, z = self._at(t)
        return xy[0] + self.off.x, xy[1] + self.off.y, z + self.off.z

    def links_at(self, t):
        _, xy, _ = self._at(t)
        return [BASE, ((BASE[0] + xy[0]) / 2, (BASE[1] + xy[1]) / 2), xy, (xy[0] + self.off.x, xy[1] + self.off.y)]

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

    def move(self, to, z, speed=None):
        self._walk([self.tip, to], z, speed or self.travel, False)

    def follow_path(self, pts, z, speed):
        if math.dist(self.tip, pts[0]) > 0.5 or abs(self.z - z) > 0.5:
            self.move(pts[0], z)
        poses = [PoseSample(self.clk.now_ns(), (pts[0][0] + self.off.x, pts[0][1] + self.off.y, z + self.off.z))]
        return poses + self._walk(pts, z, speed, True)


def build(boxes, mode="accurate", seed=0, **check):
    cfg = load_config(mode=mode, **check)
    clk = SimClock()
    arm_holder = {}
    esp = FakeESP32(clk, boxes, cfg, seed=seed, arm_sensor_fn=lambda t: arm_holder["a"].sensor_at(t),
                    arm_links_fn=lambda t: arm_holder["a"].links_at(t))
    arm = arm_holder["a"] = FakeArm(clk, cfg)
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
        probe.feed(arm.follow_path(seg.points_mm, seg.tool_z_mm, seg.speed_mm_s))
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
