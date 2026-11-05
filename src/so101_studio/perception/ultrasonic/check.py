"""팔 확인 (ultrasonic-spec 6절): 십자(L1·L2) -> 원(R) -> 긴 축(L3)·짧은 축(L4) 경로와 추정.
경로는 센서 위치 기준으로 짜고, 손끝 경로로 바꾸는 곳은 sensor_to_tip 하나다."""
import itertools
import math
import statistics

from ...core.types import DetectedObject
from .model import PathSegment, beam_radius

UNREACHABLE = object()  # 제너레이터에 보내는 표시: 그 구간을 팔이 못 갔다


def sensor_to_tip(pts, cfg):
    """센서가 지날 선 -> 손끝 xy 경로. 이 변환은 여기 한 곳뿐이다(설계서 8.2절).
    arm_sensor_offset_mm를 경로 동안 고정한 yaw로 돌려 센서 점에서 뺀다."""
    off, a = cfg.arm_sensor_offset_mm, math.radians(cfg.check.yaw_deg)
    ox, oy = off.x * math.cos(a) - off.y * math.sin(a), off.x * math.sin(a) + off.y * math.cos(a)
    return [(x - ox, y - oy) for x, y in pts]


def _near(p, poly, tol) -> bool:
    for (ax, ay), (bx, by) in itertools.pairwise(poly):
        vx, vy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / (vx * vx + vy * vy or 1.0)))
        if math.hypot(p[0] - ax - t * vx, p[1] - ay - t * vy) <= tol:
            return True
    return False


class Probe:
    """CheckProbe 구현. 경로는 제너레이터가 한 구간씩 내고, feed()의 점(x, y, z, d)을 받아 다음 구간을 정한다."""

    def __init__(self, src, cand: DetectedObject, cfg, arm_origin_mm, bus=None):
        c = cfg.check
        if c.clearance_mm < 15:
            raise ValueError("확인 높이는 물체 높이 + 15mm 이상이어야 한다 (6절 안전)")
        self.src, self.cand, self.cfg, self.bus = src, cand, cfg, bus
        self.origin = arm_origin_mm
        self.tool_z = c.max_object_height_mm + c.clearance_mm
        self.info: dict = {}
        self._gen = self._run()
        self._seg = None
        self._cur = None
        self._started = False
        self._res: DetectedObject | None = None
        self.status = "running"  # ok | empty(물체 없음, 후보 삭제) | unreachable(팔이 못 가서 못 재었다, 후보는 지우지 않는다)
        self._t0 = src.clock.now_ns()

    # --- CheckProbe ---
    def next_segment(self) -> PathSegment | None:
        if not self._started:
            self._started = True
            self._step(None)
        return self._seg

    def feed(self, poses) -> None:
        """구간이 끝난 뒤 부른다. poses가 빈 목록이면 follow_path가 그 경로를 못 간 것(움직이지 않고 [] 반환, REQ-023)이라
        데이터 없음으로 보고 다음 경로로 간다. 정지·E-STOP으로 빈 목록이 오는 경우는 작업 실행이 먼저 끊는다는 가정이다."""
        name, seg, sensor_pts = self._cur
        if not poses:
            self.info.setdefault("unreachable", []).append(name)
            self._step(UNREACHABLE)
            return
        # 경로 밖 점(접근 이동 중 측정 등)은 버린다
        pts = [p for p in self.src.arm_points(poses) if _near(p, sensor_pts, 12)]
        if self.bus:
            self.bus.publish("us.check_path", {
                "id": self.cand.id, "stage": name, "points_mm": seg.points_mm, "tool_z_mm": seg.tool_z_mm,
                "points": [(x, y, z - d) for x, y, z, d in pts]})
        self._step(pts)

    def result(self) -> DetectedObject | None:
        self.src.end_check(self.cand, self._res, self.info, self.status)
        return self._res

    def _step(self, pts):
        try:
            self._seg = next(self._gen) if pts is None else self._gen.send(pts)
        except StopIteration as e:
            self._seg, self._res = None, e.value
            self.status = "ok" if self._res else ("unreachable" if self.info.get("unreachable") else "empty")
            self.info["check_s"] = (self.src.clock.now_ns() - self._t0) / 1e9

    # --- 경로 ---
    def _seg_of(self, name, sensor_pts):
        seg = PathSegment(sensor_to_tip(sensor_pts, self.cfg), self.tool_z, self.cfg.check.speed_mm_s,
                          self.cfg.check.yaw_deg)
        self._cur = (name, seg, sensor_pts)
        return seg

    def _runs(self, samples):
        """samples = (s, h, z, x, y). 칸(sample_mm)별로 h가 hit_mm 넘는 칸을 이어(빈 칸 1개까지) 구간으로 만든다."""
        c = self.cfg.check
        above: dict[int, list] = {}
        for sm in samples:
            if sm[1] > c.hit_mm:
                above.setdefault(round(sm[0] / c.sample_mm), []).append(sm)
        runs, cur = [], []
        for i in sorted(above):
            if cur and i - cur[-1] > 2:
                runs.append(cur)
                cur = []
            cur.append(i)
        if cur:
            runs.append(cur)
        out = []
        for r in runs:
            sm = [s for i in r for s in above[i]]
            ss = [s[0] for s in sm]
            out.append({"i0": r[0], "i1": r[-1], "s0": min(ss), "s1": max(ss), "h": statistics.median(s[1] for s in sm),
                            "z": statistics.mean(s[2] for s in sm), "broken": r[-1] - r[0] + 1 != len(r)})
        return out

    def _line(self, center, u, name):
        """center를 지나는 방향 u의 직선(센서 경로). 구간 하나를 돌려준다: 없으면 None."""
        c = self.cfg.check
        half = c.line_mm / 2
        start = (center[0] - u[0] * half, center[1] - u[1] * half)
        chunk = c.line_mm if not c.early_stop_cells else c.sample_mm * 6  # T9: 짧게 끊어 가며 일찍 멈춘다
        samples, s0, runs = [], 0.0, []
        while s0 < c.line_mm - 1e-6:
            s1 = min(c.line_mm, s0 + chunk)
            pts = yield self._seg_of(name, [(start[0] + u[0] * s, start[1] + u[1] * s) for s in (s0, s1)])
            if pts is UNREACHABLE:
                break  # 도달 밖: 여기까지 잰 것으로 판단한다
            samples += [((x - start[0]) * u[0] + (y - start[1]) * u[1], z - d, z, x, y) for x, y, z, d in pts]
            runs = self._runs(samples)
            if c.early_stop_cells and samples:
                last = max(round(s[0] / c.sample_mm) for s in samples)
                if any(r["s1"] >= half and last - r["i1"] >= c.early_stop_cells for r in runs):
                    break
            s0 = s1
        if not runs:
            return None
        r = min(runs, key=lambda r: 0 if r["s0"] <= half <= r["s1"] else min(abs(r["s0"] - half), abs(r["s1"] - half)))
        mid = (r["s0"] + r["s1"]) / 2
        return {"xy": (start[0] + u[0] * mid, start[1] + u[1] * mid), "extent": r["s1"] - r["s0"] + c.sample_mm,
                    "h": r["h"], "z": r["z"], "broken": r["broken"]}

    def _grid(self, center):
        """L1·L2가 다 비었을 때: 후보 둘레 100x100mm를 격자로 한 번 훑는다. 걸린 점들의 평균을 돌려준다."""
        c = self.cfg.check
        n = int(100 / c.fallback_grid_mm) + 1
        pts = []
        for j in range(n):
            y = center[1] - 50 + j * c.fallback_grid_mm
            xs = (center[0] - 50, center[0] + 50)
            pts += [(xs[j % 2], y), (xs[1 - j % 2], y)]
        got = yield self._seg_of("GRID", pts)
        got = [] if got is UNREACHABLE else got
        hit = [(x, y) for x, y, z, d in got if z - d > c.hit_mm]
        if not hit:
            return None
        return (statistics.mean(p[0] for p in hit), statistics.mean(p[1] for p in hit))

    def _run(self):
        c, half = self.cfg.check, self.cfg.beam_half_deg
        cand = (self.cand.x_mm, self.cand.y_mm)
        if c.approach_line:  # T8: 팔 원점에서 후보로 가는 방향이 첫 선
            dx, dy = cand[0] - self.origin[0], cand[1] - self.origin[1]
            u1 = (dx / math.hypot(dx, dy), dy / math.hypot(dx, dy))
        else:
            u1 = (1.0, 0.0)
        h1 = yield from self._line(cand, u1, "L1")
        c1 = h1["xy"] if h1 else cand
        h2 = yield from self._line(c1, (-u1[1], u1[0]), "L2")
        if not h1 and not h2:
            g = yield from self._grid(cand)
            if g is None:
                return None  # 레이더 오검출
            c1 = g
        ctr = h2["xy"] if h2 else c1

        ring = [(ctr[0] + c.ring_r_mm * math.cos(2 * math.pi * k / c.ring_n),
                 ctr[1] + c.ring_r_mm * math.sin(2 * math.pi * k / c.ring_n)) for k in range(c.ring_n + 1)]
        got = yield self._seg_of("R", ring)
        got = [] if got is UNREACHABLE else got
        th = [math.atan2(y - ctr[1], x - ctr[0]) for x, y, z, d in got if z - d > c.hit_mm]
        sx = sum(math.sin(2 * t) for t in th)
        cx = sum(math.cos(2 * t) for t in th)
        R = math.hypot(sx, cx) / len(th) if th else 0.0
        phi = (math.atan2(sx, cx) / 2) % math.pi if R > 0.3 else 0.0
        u3 = (math.cos(phi), math.sin(phi))

        h3 = yield from self._line(ctr, u3, "L3")
        c3 = h3["xy"] if h3 else ctr
        h4 = yield from self._line(c3, (-u3[1], u3[0]), "L4")
        c4 = h4["xy"] if h4 else c3

        def size(h):
            return max(h["extent"] - 2 * beam_radius(h["z"], h["h"], half), 0) if h else 0.0

        hs = [h for h in (h1, h2, h3, h4) if h]
        conf = 0.5 + 0.5 * min(R, 1)
        if any(h["broken"] for h in hs) or math.dist(c3, c4) > 10 or not (h3 and h4):
            conf -= 0.2
        if self.info.get("unreachable"):
            conf -= 0.2  # 못 간 구간이 있어 덜 믿는다
        self.info.update(R=R, h_mm=statistics.median(h["h"] for h in hs) if hs else 0.0, stages=len(hs))
        return DetectedObject(id=self.cand.id, x_mm=c4[0], y_mm=c4[1], angle_deg=math.degrees(phi) if R > 0.3 else None,
                              width_mm=size(h4), length_mm=size(h3), confidence=conf, source="ultrasonic")
