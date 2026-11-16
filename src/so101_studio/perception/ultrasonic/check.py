"""팔 확인 (ultrasonic-spec 6절): 십자(L1·L2) -> 원(R) -> 긴 축(L3)·짧은 축(L4) 경로와 추정.
경로는 센서 위치 기준으로 짜고, 손끝 경로로 바꾸는 곳은 sensor_to_tip 하나다."""
import itertools
import math
import statistics

from ...core.types import DetectedObject
from ..base import PathSegment
from .model import beam_radius

UNREACHABLE = object()  # 제너레이터에 보내는 표시: 그 구간을 팔이 못 갔다


def sensor_to_tip(pts, cfg, yaw_deg=None):
    """센서가 지날 선 -> 손끝 xy 경로. 이 변환은 여기 한 곳뿐이다(설계서 8.2절).
    arm_sensor_offset_mm를 경로 동안 고정한 yaw(기본 check.yaw_deg)로 돌려 센서 점에서 뺀다."""
    off, a = cfg.arm_sensor_offset_mm, math.radians(cfg.check.yaw_deg if yaw_deg is None else yaw_deg)
    ox, oy = off.x * math.cos(a) - off.y * math.sin(a), off.x * math.sin(a) + off.y * math.cos(a)
    return [(x - ox, y - oy) for x, y in pts]


def pick_yaw(cfg, cand_xy, origin):
    """후보 반경에 맞춰 경로 동안 고정할 손목 각도. check.yaw_pivot_mm이 없거나 0이면 고정값(yaw_deg).
    센서가 손끝보다 바깥으로 나가는 양 c = clamp(r - pivot, -L, L): 먼 물체는 센서를 바깥으로(손끝은 안쪽, 최대 도달 안),
    가까운 물체는 안쪽으로(손끝은 바깥, 최소 도달 밖) 둬 손끝 경로가 도달 범위 안에 들게 한다. 오프셋 방향이 팔→후보 방향에서
    acos(c/L)만큼 벗어나게 yaw를 풀고, 두 해 중 yaw_deg에 가까운 쪽을 쓴다(손목 회전이 덜 돈다)."""
    pivot = cfg.check.get("yaw_pivot_mm")
    if not pivot:  # 없음·null·0 = 끔(설계서 표기 0)
        return cfg.check.yaw_deg
    off = cfg.arm_sensor_offset_mm
    L = math.hypot(off.x, off.y)
    dx, dy = cand_xy[0] - origin[0], cand_xy[1] - origin[1]
    c = max(-L, min(L, math.hypot(dx, dy) - pivot))
    delta, th, base = math.acos(c / L), math.atan2(dy, dx), math.atan2(off.y, off.x)
    sols = [math.degrees(th + sg * delta - base) for sg in (1, -1)]
    wrap = lambda a: (a + 180) % 360 - 180
    return min(sols, key=lambda y: abs(wrap(y - cfg.check.yaw_deg)))


def _straighten(run, tol=1e-6):
    """일직선으로 이어지는 중간 점을 뺀다(꺾이는 점·양 끝만 남김). 판정용 2mm 점을 그대로 팔에 주면 fillet_path가 같은 변 위의
    점마다 "서서 도는 꺾임"으로 처리해 2mm마다 선다. 점이 둘 이하면 그대로."""
    out = list(run[:1])
    for i in range(1, len(run) - 1):
        ax, ay, bx, by = run[i][0] - out[-1][0], run[i][1] - out[-1][1], run[i + 1][0] - run[i][0], run[i + 1][1] - run[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-9 or nb < 1e-9 or abs(ax * by - ay * bx) / (na * nb) > tol or ax * bx + ay * by < 0:
            out.append(run[i])
    return out + run[-1:]


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
        self.yaw = pick_yaw(cfg, (cand.x_mm, cand.y_mm), arm_origin_mm)
        self.info: dict = {}
        self._gen = self._run()
        self._seg = None
        self._cur = None
        self._started = False
        self._pos = None  # 마지막으로 보낸 경로 끝(센서 좌표)
        self._retry, self._pull_mm = None, 0.0  # 첫 점 당기기: feed가 만든 다시 낼 경로, 이 조각에서 당긴 거리
        self._res: DetectedObject | None = None
        self.status = "running"  # ok | empty(물체 없음, 후보 삭제) | unreachable(팔이 못 가서 못 재었다, 후보는 지우지 않는다)
        self._t0 = src.clock.now_ns()

    # --- CheckProbe ---
    def next_segment(self) -> PathSegment | None:
        if not self._started:
            self._started = True
            self._step(None)
        return self._seg

    def feed(self, poses, status: str = "ok") -> None:
        """구간이 끝난 뒤 부른다. poses가 빈 목록이면 follow_path가 그 경로를 못 간 것(움직이지 않고 [] 반환, REQ-023)이라
        데이터 없음으로 보고 다음 경로로 간다. 정지·E-STOP으로 빈 목록이 오는 경우는 작업 실행이 먼저 끊는다는 가정이다.
        status = follow_path 결과 상태(last_path.status). "ik_fail"이면 첫 점을 안쪽으로 당겨 같은 단계를 다시 낸다(6절 첫 점 당기기, U16)."""
        name, seg, sensor_pts = self._cur
        if not poses:
            self._retry = self._pulled(sensor_pts) if status == "ik_fail" else None
            if self._retry is None:
                self.info.setdefault("unreachable", []).append(name)
            self._publish_path(name, seg, [], phase="end", unreachable=True)
            self._step(UNREACHABLE)
            return
        # 경로 밖 점(접근 이동 중 측정 등)은 버린다
        pts = [p for p in self.src.arm_points(poses) if _near(p, sensor_pts, 12)]
        self._publish_path(name, seg, [(x, y, z - d) for x, y, z, d in pts], phase="end")
        self._step(pts)

    def abort(self) -> None:
        """정지·E-STOP·fault로 확인을 중간에 끝낼 때 작업 실행이 부른다(follow_path의 last_path가 stopped·fault·locked일 때).
        ESP32에 stop을 보내고 결과 없이 닫는다. 후보는 지우지 않는다(status "aborted")."""
        self._gen.close()
        self._seg, self._res, self.status = None, None, "aborted"
        self.src.end_check(self.cand, None, self.info, self.status)

    def _publish_path(self, name, seg, points, phase, unreachable=False):
        if self.bus:
            self.bus.publish("us.check_path", {
                "id": self.cand.id, "stage": name, "phase": phase, "points_mm": seg.points_mm,
                "tool_z_mm": seg.tool_z_mm, "points": points, "unreachable": unreachable})

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
        else:
            if self._seg is not None:  # 구간을 시작할 때 한 번: 화면이 팔이 움직이는 동안 경로를 그린다
                self._publish_path(*self._cur[:2], [], phase="start")

    # --- 경로 ---
    def _inside(self, sensor_p) -> bool:
        tip = sensor_to_tip([sensor_p], self.cfg, self.yaw)[0]
        r = math.hypot(tip[0] - self.origin[0], tip[1] - self.origin[1])
        reach = self.cfg.check.tool_reach_mm
        return reach.min <= r <= reach.max

    def _clip(self, pts):
        """PR #71: 손끝이 도달 범위(tool_reach_mm) 밖인 점을 건너뛴다. 끊기면 조각마다 따로 follow_path를 부른다.
        전부 범위 안이면 원래 꼭짓점 그대로 한 구간이다."""
        dense = []
        for a, b in itertools.pairwise(pts):
            n = max(1, math.ceil(math.dist(a, b) / 2))  # 2mm 간격으로 판정
            dense += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
        dense.append(pts[-1])
        if all(self._inside(p) for p in dense):
            return [pts]
        pieces, run = [], []
        for p in [*dense, None]:
            if p is not None and self._inside(p):
                run.append(p)
                continue
            if len(run) >= 2:  # 2mm 간격 점이 둘 이상. 닫힌 원은 처음과 끝이 같으니 끝점 거리로 거르지 않는다
                pieces.append([run[0], run[-1]] if len(pts) == 2 else _straighten(run))
            run = []
        return pieces

    def _follow(self, name, sensor_pts):
        """경로를 도달 범위로 자른 조각마다 구간 하나씩 내고, 받은 점을 모두 돌려준다. 하나도 못 가면 UNREACHABLE.
        self._pieces에 마지막으로 보낸 조각들을 남긴다."""
        self._pieces = self._clip(sensor_pts)
        self.info.setdefault("stage_names", []).append(name)  # 벤치마크가 단계당 다시 부른 횟수를 셀 때 쓴다
        if not self._pieces:
            self.info.setdefault("unreachable", []).append(name)  # 전부 도달 밖이라 호출 자체를 안 한다
            return UNREACHABLE
        got, ok = [], False
        for pc in self._pieces:
            self._pull_mm = 0.0
            pts = yield self._seg_of(name, pc)
            while pts is UNREACHABLE and self._retry:  # ik_fail: feed가 첫 점을 당겨 둔 경로로 다시
                self._retry, retry = None, self._retry
                self.info.setdefault("pulled", []).append({"stage": name, "mm": self._pull_mm})
                pts = yield self._seg_of(name, retry)
            self._pos = pc[-1]  # 다음 선은 지금 위치에서 가까운 끝에서 시작한다
            if pts is not UNREACHABLE:
                ok = True
                got += pts
        return got if ok else UNREACHABLE

    def _pulled(self, pts):
        """첫 점을 첫 구간을 따라 retry_pull_mm만큼 당긴 경로. 합이 retry_pull_max_mm를 넘거나 첫 구간이 5mm 아래가 되면 None. 끝 점은 그대로."""
        c = self.cfg.check
        step, first = c.get("retry_pull_mm", 2), math.dist(pts[0], pts[1])
        if self._pull_mm + step > c.get("retry_pull_max_mm", 0) + 1e-9 or first - step < 5:
            return None
        self._pull_mm += step
        k = step / first
        return [(pts[0][0] + (pts[1][0] - pts[0][0]) * k, pts[0][1] + (pts[1][1] - pts[0][1]) * k), *pts[1:]]

    def _order(self, pts):
        """다음 선의 시작점은 지금 위치에서 가까운 끝으로 고른다."""
        if self._pos is not None and math.dist(self._pos, pts[-1]) < math.dist(self._pos, pts[0]):
            return pts[::-1]
        return pts

    def _seg_of(self, name, sensor_pts):
        seg = PathSegment(sensor_to_tip(sensor_pts, self.cfg, self.yaw), self.tool_z, self.cfg.check.speed_mm_s,
                          self.yaw)
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

    def _analyze(self, pts, start, u, cuts):
        """직선(start에서 방향 u)을 지난 점(x, y, z, d)에서 후보 중심을 품은(또는 가장 가까운) 메아리 구간 하나를 고른다."""
        c = self.cfg.check
        half = c.line_mm / 2
        samples = [((x - start[0]) * u[0] + (y - start[1]) * u[1], z - d, z, x, y) for x, y, z, d in pts]
        runs = self._runs(samples)
        if not runs:
            return None, runs, samples
        r = min(runs, key=lambda r: 0 if r["s0"] <= half <= r["s1"] else min(abs(r["s0"] - half), abs(r["s1"] - half)))
        mid = (r["s0"] + r["s1"]) / 2
        hit = {"xy": (start[0] + u[0] * mid, start[1] + u[1] * mid), "extent": r["s1"] - r["s0"] + c.sample_mm,
               "h": r["h"], "z": r["z"], "broken": r["broken"],
               "cut": any(abs(r[k] - cp) <= 1.5 * c.sample_mm for cp in cuts for k in ("s0", "s1"))}
        return hit, runs, samples

    def _cuts(self, pieces, ends, start, u):
        """잘린 경계(호출 끝점이 아닌 조각 끝)의 선 위 위치: 메아리 구간이 거기 닿으면 중심이 치우칠 수 있다."""
        out = []
        for pc in pieces:
            for p in (pc[0], pc[-1]):
                if all(math.dist(p, e) > 1.0 for e in ends):
                    out.append((p[0] - start[0]) * u[0] + (p[1] - start[1]) * u[1])
        return out

    def _line(self, center, u, name):
        """center를 지나는 방향 u의 직선(센서 경로). 구간 하나를 돌려준다: 없으면 None."""
        c = self.cfg.check
        half = c.line_mm / 2
        start = (center[0] - u[0] * half, center[1] - u[1] * half)
        chunk = c.line_mm if not c.early_stop_cells else c.sample_mm * 6  # T9: 짧게 끊어 가며 일찍 멈춘다(지금은 끔)
        pts_all, s0, cuts = [], 0.0, []
        while s0 < c.line_mm - 1e-6:
            s1 = min(c.line_mm, s0 + chunk)
            ends = [(start[0] + u[0] * s, start[1] + u[1] * s) for s in (s0, s1)]
            if not c.early_stop_cells:
                ends = self._order(ends)
            pts = yield from self._follow(name, ends)
            if pts is UNREACHABLE:
                s0 = s1
                continue
            cuts += self._cuts(self._pieces, ends, start, u)
            pts_all += pts
            if c.early_stop_cells and pts_all:
                _, runs, samples = self._analyze(pts_all, start, u, cuts)
                last = max(round(s[0] / c.sample_mm) for s in samples)
                if any(r["s1"] >= half and last - r["i1"] >= c.early_stop_cells for r in runs):
                    break
            s0 = s1
        return self._analyze(pts_all, start, u, cuts)[0] if pts_all else None

    def _cross(self, ctr, u3):
        """L3(긴 축)와 L4(짧은 축)를 같은 중심을 지나게 잡고 한 번의 follow_path로 잇는다(L4 시작은 L3 끝에서 가까운 끝).
        도달 범위로 잘리면 따로 두 번 보낸다. 돌려주는 값은 (L3 구간, L4 구간)."""
        c = self.cfg.check
        u4 = (-u3[1], u3[0])
        half = c.line_mm / 2
        e3 = self._order([(ctr[0] - u3[0] * half, ctr[1] - u3[1] * half), (ctr[0] + u3[0] * half, ctr[1] + u3[1] * half)])
        e4 = [(ctr[0] - u4[0] * half, ctr[1] - u4[1] * half), (ctr[0] + u4[0] * half, ctr[1] + u4[1] * half)]
        if math.dist(e3[-1], e4[-1]) < math.dist(e3[-1], e4[0]):
            e4 = e4[::-1]
        if self._clip([*e3, *e4]) == [[*e3, *e4]] and not c.early_stop_cells:
            pts = yield from self._follow("L34", [*e3, *e4])
            if pts is UNREACHABLE:
                return None, None
            s3 = (ctr[0] - u3[0] * half, ctr[1] - u3[1] * half)
            s4 = (ctr[0] - u4[0] * half, ctr[1] - u4[1] * half)
            h3 = self._analyze([p for p in pts if _near(p, e3, 12)], s3, u3, [])[0]
            h4 = self._analyze([p for p in pts if _near(p, e4, 12)], s4, u4, [])[0]
            return h3, h4
        h3 = yield from self._line(ctr, u3, "L3")
        h4 = yield from self._line(ctr, u4, "L4")
        return h3, h4

    def _grid(self, center):
        """L1·L2가 다 비었을 때: 후보 둘레 100x100mm를 격자로 한 번 훑는다. 걸린 점들의 평균을 돌려준다."""
        c = self.cfg.check
        n = int(100 / c.fallback_grid_mm) + 1
        pts = []
        for j in range(n):
            y = center[1] - 50 + j * c.fallback_grid_mm
            xs = (center[0] - 50, center[0] + 50)
            pts += [(xs[j % 2], y), (xs[1 - j % 2], y)]
        got = yield from self._follow("GRID", pts)
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
                 ctr[1] + c.ring_r_mm * math.sin(2 * math.pi * k / c.ring_n)) for k in range(c.ring_n)]
        if self._pos is not None:  # 지금 위치에서 가장 가까운 점부터 한 바퀴
            k0 = min(range(len(ring)), key=lambda k: math.dist(ring[k], self._pos))
            ring = ring[k0:] + ring[:k0]
        ring.append(ring[0])
        got = yield from self._follow("R", ring)
        got = [] if got is UNREACHABLE else got
        # 칸(sample_mm 호 길이)마다 한 번만 센다: 팔이 멈칫하며 같은 자리를 여러 번 재도 방향 평균이 쏠리지 않게
        cells = {round((math.atan2(y - ctr[1], x - ctr[0]) % (2 * math.pi)) * c.ring_r_mm / c.sample_mm)
                 for x, y, z, d in got if z - d > c.hit_mm}
        th = [i * c.sample_mm / c.ring_r_mm for i in sorted(cells)]
        sx = sum(math.sin(2 * t) for t in th)
        cx = sum(math.cos(2 * t) for t in th)
        R = math.hypot(sx, cx) / len(th) if th else 0.0
        phi = (math.atan2(sx, cx) / 2) % math.pi if R > 0.3 else 0.0
        u3 = (math.cos(phi), math.sin(phi))

        h3, h4 = yield from self._cross(ctr, u3)
        u4 = (-u3[1], u3[0])
        # 최종 중심 = 2차 중심 + L3가 말해 주는 긴 축 쪽 치우침 + L4가 말해 주는 짧은 축 쪽 치우침
        a_off = ((h3["xy"][0] - ctr[0]) * u3[0] + (h3["xy"][1] - ctr[1]) * u3[1]) if h3 else 0.0
        b_off = ((h4["xy"][0] - ctr[0]) * u4[0] + (h4["xy"][1] - ctr[1]) * u4[1]) if h4 else 0.0
        c4 = (ctr[0] + a_off * u3[0] + b_off * u4[0], ctr[1] + a_off * u3[1] + b_off * u4[1])
        # L3·L4 사이 어긋남: 각 선의 중심이 다른 선의 축에서 벗어난 정도
        skew = max(abs((h3["xy"][0] - ctr[0]) * u4[0] + (h3["xy"][1] - ctr[1]) * u4[1]) if h3 else 0.0,
                   abs((h4["xy"][0] - ctr[0]) * u3[0] + (h4["xy"][1] - ctr[1]) * u3[1]) if h4 else 0.0)

        def size(h):
            return max(h["extent"] - 2 * beam_radius(h["z"], h["h"], half), 0) if h else 0.0

        hs = [h for h in (h1, h2, h3, h4) if h]
        conf = 0.5 + 0.5 * min(R, 1)
        if any(h["broken"] for h in hs) or skew > 10 or not (h3 and h4):
            conf -= 0.2
        if any(h["cut"] for h in hs):
            conf -= 0.2  # 메아리 구간이 도달 범위로 잘린 끝에 닿았다(#71)
        if self.info.get("unreachable"):
            conf -= 0.2  # 못 간 구간이 있어 덜 믿는다
        self.info.update(R=R, h_mm=statistics.median(h["h"] for h in hs) if hs else 0.0, stages=len(hs))
        return DetectedObject(id=self.cand.id, x_mm=c4[0], y_mm=c4[1], angle_deg=math.degrees(phi) if R > 0.3 else None,
                              width_mm=size(h4), length_mm=size(h3), confidence=conf, source="ultrasonic",
                              height_mm=self.info["h_mm"])
