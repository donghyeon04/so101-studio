"""초음파 성능 벤치마크 (ultrasonic-spec 8.5절, 결정 0037): 가상 ESP32 + 가상 팔 위에서 실제 radar.py·check.py·source.py를 돌린다.
성능 숫자는 이 도구로만 낸다. 새 아이디어(확인선 길이, 모드, 속도 등)는 옵션으로 붙인다.

python -m so101_studio.perception.ultrasonic.benchmark [--scenario spread90|spread120|demo|demo80|show80|single|all] [--seeds N] [--approach-line]
                                                       [--model optimistic|pessimistic|both] [--reach radius|kin] [--hover-mm N] [--tool-reach MIN,MAX] [--line-mm N] [--speed N] [--rescan-home on|off] [--max-fail N] [--empty-extra N] [--no-note-failure]
"""
import argparse
import itertools
import json
import math
import random
import statistics
import subprocess
import tempfile
import time
from bisect import bisect_right
from dataclasses import replace
from functools import cache
from pathlib import Path

import numpy as np
import yaml

from ...arm.kinematics import Kinematics
from ...arm.skills import SEEDS
from ...arm.trajectory import LookaheadShape, fillet_path
from ...core.config import load_robot, load_workspace
from ...core.scene import BIN_WALL_T_MM
from ...core.types import DetectedObject, JointState
from ...service.targets import point_to_pose, tip_vector
from ...task import policy
from ..base import PoseSample
from .fake_esp32 import FakeESP32, SimClock
from .link import Link
from .model import Box, load_config
from .source import UltrasonicSource

DT = 0.02  # 팔 50Hz
TOOL_V_MAX = 100.0  # robot.yaml tool_v_max_mm_s
REACH = (50.0, 265.0)  # 8.5절 손끝 도달 반경(REQ-023 계산)

# 8.5절 센서 모델. 실측(B1·B2·B4)으로 고친다.
MODELS = {
    "optimistic": {"wrist_beam_half_deg": 7.5, "max_incidence_deg": 12.0, "corner": (0.7, 450.0, 0.9), "servo_lag_deg": 3.0},
    "pessimistic": {"wrist_beam_half_deg": 15.0, "max_incidence_deg": 8.0, "corner": (0.7, 300.0, 0.5), "servo_lag_deg": 3.0},
}
DXL_LAG_DEG = 0.3  # XL330 가정: 4° 이동 약 7ms라 33ms 간격 안에 도착. 남는 지연(실측 전 가정)
OBJECT_SIZES = ((30, 100), (40, 80), (50, 50))  # 폭 x 길이, 높이 40
SCENARIOS = {  # 이름: (물체 수, 반경 mm, 방위 °, 중심 간격 mm 이상)
    "spread90": (3, (120, 220), (45, 135), 90),  # 결정 0067: 판 425x375에서 반경 220의 양 옆은 판 밖이라 방위 45~135°(정면 ±45°)
    "spread120": (3, (120, 220), (45, 135), 120),
    "demo": (3, (120, 220), (45, 135), 120),
    "demo80": (3, (120, 220), (45, 135), 120),  # demo + 물체 모서리 사이 80mm 이상(설계서 6절 권장 배치, edge_gap)
    "show80": (3, (150, 220), (45, 135), 120),  # demo80과 같되 반경 150~220(시연·I1, 사용자 결정 11-20, REQ-099)
    "single": (1, (100, 240), (25, 155), 0),
    "pair": (2, (120, 240), (25, 155), 0),  # 물체 둘, 모서리 사이 간격을 --pair-gap으로 정한다(이웃 물체를 따로 보는 최소 간격 재기)
}
EDGE80 = ("demo80", "show80")  # 모서리 80mm 이상 검사를 켜는 시나리오
SINGLE_RADII = (100, 120, 140, 160, 180, 200, 220, 240)


def arm_base() -> tuple[float, float]:
    b = load_workspace().base
    return b.x_mm, b.y_mm


@cache
def radar_clear_pose(robot_yaml="config/robot.yaml"):
    """대기 자세(결정 0039)의 손끝 ((x, y), 작업대 위 높이 mm). robot.yaml의 poses.radar_clear 관절값을 팔 기구학으로 바꾼다. 키가 없으면 None."""
    cfg = yaml.safe_load(Path(robot_yaml).read_text(encoding="utf-8"))
    joints = cfg.get("poses", {}).get("radar_clear")
    if joints is None:
        return None
    t = Kinematics(cfg["urdf"], load_workspace().base).fk(JointState(*joints))  # fk는 관절 한계를 쓰지 않는다
    return (t.x_mm, t.y_mm), t.z_mm


class KinReach:
    """실제 팔 기구학으로 하는 도달 판정(--reach kin): 점에서 손끝을 수직 아래로 둔 IK가 풀리는지(관절 한계 포함, 시드는 plan_goto와 같이 홈 + SEEDS).
    hover_mm는 접근 호버(첫 점 위 높이, 기본 robot.yaml approach_height_mm). 같은 점은 1mm 격자로 모아 한 번만 푼다."""

    def __init__(self, hover_mm=None, gripper=None):
        if gripper:  # 그리퍼 종류만 바꾼 임시 local.yaml(로컬 설정은 읽지 않는다)
            tmp = Path(tempfile.mkdtemp()) / "local.yaml"
            tmp.write_text(f"gripper: {gripper}\n", encoding="utf-8")
            self.cfg = load_robot(local_path=tmp)
        else:
            self.cfg = load_robot()
        self.ws = load_workspace()
        self.kin = Kinematics(self.cfg.urdf, self.ws.base, self.cfg.joint_limits_deg)
        self.hover = self.cfg.approach_height_mm if hover_mm is None else hover_mm
        home = JointState(*self.cfg.episode_start())  # 가상 팔 시작 자세(REQ-103: 병렬 home이 바뀌어도 숫자 그대로)
        self.seeds = [home, *(replace(sd, gripper=home.gripper) for sd in SEEDS)]
        self.cache = {}

    def ok(self, xy, z, yaw=90.0):
        key = (round(xy[0]), round(xy[1]), round(z), round(yaw))
        if key not in self.cache:
            tip = tip_vector(self.kin, -90.0, yaw, self.seeds[0].gripper, "tip")
            pose = point_to_pose(key[0], key[1], key[2], yaw, "table", self.ws.base, self.cfg.table_z_mm, tip)
            self.cache[key] = any(self.kin.ik(pose, sd) is not None for sd in self.seeds)
        return self.cache[key]

    def start_ok(self, xy, z, yaw=90.0):
        """경로 첫 점: 점 자체와 그 위 호버가 풀려야 plan_goto의 접근이 된다. 호버는 hover_mm부터 5mm씩 10mm까지 내려 처음 풀리는 것을 쓴다(REQ-089, 놓기 _heights()와 같은 규칙)."""
        return self.ok(xy, z, yaw) and any(self.ok(xy, z + h, yaw) for h in (range(int(self.hover), 9, -5) if self.hover >= 10 else [self.hover]))


_KIN = {}


def kin_reach(hover_mm=None, gripper=None):
    if (hover_mm, gripper) not in _KIN:
        _KIN[hover_mm, gripper] = KinReach(hover_mm, gripper)
    return _KIN[hover_mm, gripper]


class FakeArm:
    """가상 팔: 손끝 위치 기록, 센서 위치(손끝 + yaw로 돌린 오프셋), follow_path(REQ-023 모양).
    시간 모델(8.5절, #77): 호출마다 정지에서 출발·정지로 끝, 가속·감속 accel, 호출 사이 gap_s, 한 호출 안에서는 서지 않되
    꺾임·원에서 가속 한도를 넘지 않게 속도를 줄인다(팔 코드의 fillet_path·LookaheadShape를 그대로 쓴다, #73. 원은 v²/r <= accel/√2). 못 가는 경로는 움직이지 않고 []."""

    def __init__(self, clk, cfg, base, travel_mm_s=400.0, reach=REACH, accel=120.0, gap_s=0.25, motion_scale=1.0, wait_pose=None, kin=None):
        self.clk, self.base, self.reach = clk, base, reach
        self.travel, self.accel, self.gap_s, self.motion_scale = travel_mm_s, accel, gap_s, motion_scale
        self.wait_pose = wait_pose or radar_clear_pose() or (base, 150.0)  # 대기 자세 손끝. 설정에 radar_clear가 없으면 팔 원점 위 150mm
        self.kin = kin  # kin: KinReach면 반경 대신 기구학으로 도달 판정(못 가면 ik_fail, 첫 점 당기기는 Probe가 한다)
        self._hover_needed = True  # 직전 동작이 follow_path가 아니면 접근에 호버가 든다(follow_path끼리 이어 부르면 합쳐진 직선 접근)
        self.off0 = (cfg.arm_sensor_offset_mm.x, cfg.arm_sensor_offset_mm.y, cfg.arm_sensor_offset_mm.z)
        self.yaw = 90.0  # 평소 손목 자세
        self.tip = base
        self.z = 150.0
        self.hist = [(0, base, self.z)]  # (t_ns, 손끝 xy, 손끝 z)
        self.last_path = None
        self.last_status = "ok"  # 마지막 follow_path 결과 상태(arm.last_path.status): ok | unreachable(반경 모델) | ik_fail(기구학 모델)
        self.calls = []  # (경로 길이 mm, 걸린 초)

    def _sensor(self, xy, z):
        a = math.radians(self.yaw)
        x, y, oz = self.off0
        # 팔 sensor_position(tool_axes(q) @ off)과 같이: 아래를 보는 손에서 도구 x축은 yaw 회전의 거울(yaw 90에서 (0, -1))
        return xy[0] - x * math.cos(a) - y * math.sin(a), xy[1] - x * math.sin(a) + y * math.cos(a), z + oz

    def _tick(self, xy, z):
        self.clk.sleep(DT)
        self.tip, self.z = xy, z
        self.hist.append((self.clk.now_ns(), xy, z))
        return PoseSample(self.clk.now_ns(), (*xy, z), self._sensor(xy, z))

    def reachable(self, xy):
        if self.reach is None:
            return True
        return self.reach[0] <= math.hypot(xy[0] - self.base[0], xy[1] - self.base[1]) <= self.reach[1]

    def _at(self, t):
        i = bisect_right([h[0] for h in self.hist], t) - 1
        return self.hist[min(max(i, 0), len(self.hist) - 1)]

    def sensor_at(self, t):
        _, xy, z = self._at(t)
        return self._sensor(xy, z)

    def links_at(self, t):
        _, xy, _ = self._at(t)
        mid = ((self.base[0] + xy[0]) / 2, (self.base[1] + xy[1]) / 2)
        return [self.base, mid, xy, self._sensor(xy, 0)[:2]]

    def move(self, to, z, speed=None):
        """접근·복귀 이동(일정 속도). follow_path 바깥."""
        self._hover_needed = True
        L = math.dist(self.tip, to)
        n = max(1, round(L / (speed or self.travel) / DT))
        a = self.tip
        for k in range(1, n + 1):
            self._tick((a[0] + (to[0] - a[0]) * k / n, a[1] + (to[1] - a[1]) * k / n), z)

    def _profile_walk(self, pts, z, vmax):
        """폴리라인 전체를 한 번에 간다. 팔 코드(arm/trajectory.py)의 smooth_l과 같은 규칙: 꼭짓점을 호로 감고(fillet_path, 모서리 2.5mm),
        처음·끝에서만 서고 호에서는 구심 가속이 한도 안이 되게 줄이는(LookaheadShape, 접선·구심 모두 accel/√2) 속도 곡선을 50Hz로 샘플링한다."""
        P = [np.array([x, y, z]) for x, y in pts]
        a = self.accel / self.motion_scale**2 / math.sqrt(2)
        path, s, arcs, _vs, _cusps = fillet_path(P, a, 1.0)
        if s[-1] < 1e-9:
            return []
        shape = LookaheadShape(float(s[-1]), arcs, vmax / self.motion_scale, a)
        n = max(1, math.ceil(shape.duration_s / DT - 1e-9))
        out = []
        for k in range(1, n + 1):
            d = shape.frac(min(k * DT, shape.duration_s))[0] * s[-1]
            out.append(self._tick((float(np.interp(d, s, path[:, 0])), float(np.interp(d, s, path[:, 1]))), z))
        return out

    def follow_path(self, pts, z, speed, yaw=None):
        need_hover, pts = self._hover_needed, list(pts)
        if self.kin is None:
            bad = [p for p in pts if not self.reachable(p)]
        else:  # 기구학 판정: 호버가 필요한 호출은 첫 점에 호버까지, 나머지 점은 작업 높이에서
            yw = self.yaw if yaw is None else yaw
            start_ok = (lambda p: self.kin.start_ok(p, z, yw)) if need_hover else (lambda p: self.kin.ok(p, z, yw))
            bad = [p for p in pts[1:] if not self.kin.ok(p, z, yw)] + ([] if start_ok(pts[0]) else [pts[0]])
        if bad:
            self.last_path = {"reason": "unreachable", "points": bad}
            self.last_status = "unreachable" if self.kin is None else "ik_fail"
            return []
        self.last_status = "ok"
        if yaw is not None:
            self.yaw = yaw
        speed = min(speed, TOOL_V_MAX)
        if math.dist(self.tip, pts[0]) > 0.5 or abs(self.z - z) > 0.5:
            self.move(pts[0], z)
        self.clk.sleep(self.gap_s)
        t0 = self.clk.now_ns()
        first = PoseSample(self.clk.now_ns(), (*pts[0], z), self._sensor(pts[0], z))
        poses = [first, *self._profile_walk(pts, z, speed)]
        self._hover_needed = False
        self.calls.append((sum(math.dist(a, b) for a, b in itertools.pairwise(pts)), (self.clk.now_ns() - t0) / 1e9))
        return poses


def build(boxes, seed=0, model="optimistic", reach=REACH, arm_kw=None, esp_kw=None, radar=None, gripper=None, **check):
    """설정 + 가상 시계 + 가상 ESP32 + 가상 팔. model: MODELS 이름 또는 None(이상적 센서)."""
    clk = SimClock()
    holder = {}
    kw = dict(MODELS[model]) if model else {}
    kw.update(esp_kw or {})
    cfg = load_config(radar=radar, servo={"driver": kw.get("servo", "pwm")}, gripper=gripper, **check)
    esp = FakeESP32(clk, boxes, cfg, seed=seed, arm_sensor_fn=lambda t: holder["a"].sensor_at(t),
                    arm_links_fn=lambda t: holder["a"].links_at(t), **kw)
    arm = holder["a"] = FakeArm(clk, cfg, arm_base(), reach=reach, **(arm_kw or {}))
    return cfg, clk, esp, arm


def make_source(cfg, clk, esp, arm, baseline_dir=None, calibrate=True):
    path = Path(baseline_dir or tempfile.mkdtemp()) / "baseline.json"
    src = UltrasonicSource(Link(esp, clk), cfg, arm_origin_mm=arm.base, arm_links_fn=arm.links_at, baseline_path=path,
                          ignore_zones=load_workspace().ignore_zones())  # 작업 실행과 같이 배출 구역(통)을 후보에서 뺀다
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
        probe.feed(arm.follow_path(seg.points_mm, seg.tool_z_mm, seg.speed_mm_s, seg.yaw_deg), arm.last_status)
    return probe.result(), probe, (arm.clk.now_ns() - t0) / 1e9


def angle_ok(est, true_deg):
    if est is None:
        return False
    d = abs(est - true_deg) % 180
    return min(d, 180 - d) <= 22.5


def on_plate(b: Box, ws) -> bool:
    """물체 테두리가 판(workspace table_mm, 원점은 판 모서리) 안이고 배출 구역 무시 원(반 대각선 + 벽 + safety_margin_mm, 설계서 6절) 밖이다."""
    t = ws.table_mm
    return (all(0 <= x <= t["x"] and 0 <= y <= t["y"] for x, y in _edge_pts(b))
            and all(math.hypot(b.x - zx, b.y - zy) >= zr for zx, zy, zr in ws.ignore_zones())
            and not any(_hits_zone(b, z) for z in ws.drop_zones.values()))


def _hits_zone(b: Box, z: dict) -> bool:
    """물체 몸체(테두리 점)가 배출 구역 사각형에 걸리는지(설계서 6절). 통은 벽 두께만큼 바깥까지 본다. 가운데만 보면 긴 막대가 벽에 걸린다."""
    if "w_mm" not in z:
        return False
    t = BIN_WALL_T_MM if z.get("type") == "bin" else 0.0
    hw, hh = z["w_mm"] / 2 + t, z["h_mm"] / 2 + t
    return any(abs(x - z["x_mm"]) <= hw and abs(y - z["y_mm"]) <= hh for x, y in _edge_pts(b))


def random_layout(rng, name) -> list[Box]:
    """8.5절 시나리오: 팔 원점 기준 반경·방위 범위 안, 중심 간격 이상(demo80·show80은 모서리 간격 80 이상도), 판 안, 통 원 밖.
    물체를 하나씩 뽑고 앞 물체와 조건이 안 맞으면 그 물체만 다시 뽑는다(200번 안에 안 되면 처음부터). 간격이 큰 배치에 치우친다.
    물체는 30x100·40x80·50x50, 방향 0/45/90/135° ±5°."""
    n, (r0, r1), (b0, b1), spacing = SCENARIOS[name]
    bx, by = arm_base()
    ws = load_workspace()
    for _ in range(1000):
        boxes = []
        for _ in range(n):
            for _ in range(200):
                r, th = rng.uniform(r0, r1), math.radians(rng.uniform(b0, b1))
                w, length = rng.choice(OBJECT_SIZES)
                b = Box(bx + r * math.cos(th), by + r * math.sin(th), rng.choice((0, 45, 90, 135)) + rng.uniform(-5, 5), w, length)
                if on_plate(b, ws) and all(math.dist((a.x, a.y), (b.x, b.y)) >= spacing and (name not in EDGE80 or edge_gap(a, b) >= 80) for a in boxes):
                    boxes.append(b)
                    break
            else:
                break
        if len(boxes) == n:
            return boxes
    raise RuntimeError("배치를 못 만들었다")


def _edge_pts(b: Box):
    """사각형 테두리를 1mm 간격으로 찍은 점."""
    a = math.radians(b.angle_deg)
    u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    cs = [(sx * b.length / 2, sy * b.width / 2) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    pts = []
    for (x0, y0), (x1, y1) in itertools.pairwise([*cs, cs[0]]):
        n = max(1, round(math.hypot(x1 - x0, y1 - y0)))
        pts += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]
    return [(b.x + px * u[0] + py * v[0], b.y + px * u[1] + py * v[1]) for px, py in pts]


def edge_gap(a: Box, b: Box) -> float:
    """두 사각형 모서리 사이 최소 거리(테두리 점 기준, 겹치면 0)."""
    pb = np.array(_edge_pts(b))
    return float(min(np.min(np.hypot(pb[:, 0] - x, pb[:, 1] - y)) for x, y in _edge_pts(a)))


def pair_layout(rng, gap_mm) -> list[Box]:
    """물체 둘: 첫 물체는 random_layout처럼, 둘째는 무작위 방향으로 모서리 간격이 gap_mm가 되는 자리(이분법)에 둔다."""
    _n, (r0, r1), (b0, b1), _ = SCENARIOS["pair"]
    bx, by = arm_base()
    zones = load_workspace().ignore_zones()  # 후보에서 빼는 배출 구역: 물체 중심이 그 둘레 50mm 안이면 일부가 가려지니 피한다
    for _ in range(10000):
        boxes = []
        for _ in range(2):
            r, th = rng.uniform(r0, r1), math.radians(rng.uniform(b0, b1))
            w, length = rng.choice(OBJECT_SIZES)
            boxes.append(Box(bx + r * math.cos(th), by + r * math.sin(th), rng.choice((0, 45, 90, 135)) + rng.uniform(-5, 5), w, length))
        a, b = boxes
        phi = rng.uniform(0, 2 * math.pi)
        lo, hi = 0.0, 400.0
        for _ in range(30):
            mid = (lo + hi) / 2
            b.x, b.y = a.x + mid * math.cos(phi), a.y + mid * math.sin(phi)
            lo, hi = (mid, hi) if edge_gap(a, b) < gap_mm else (lo, mid)
        b.x, b.y = a.x + hi * math.cos(phi), a.y + hi * math.sin(phi)
        rb, thb = math.hypot(b.x - bx, b.y - by), math.degrees(math.atan2(b.y - by, b.x - bx))
        near_drop = any(math.hypot(o.x - zx, o.y - zy) < zr + 50 for o in boxes for zx, zy, zr in zones)
        if r0 <= rb <= r1 and b0 <= thb <= b1 and not near_drop and abs(edge_gap(a, b) - gap_mm) < 1.5:
            return boxes
    raise RuntimeError("배치를 못 만들었다")


def run_cycle(src, arm, esp, place_s=5.0, drop=(355.0, 80.0), max_iter=12, rescan_home=None, report_failures=True):
    """처음 scan()부터 물체가 다 없어질 때까지(작업 실행 흉내). 인식 시간 = 레이더 + 확인, 집기·놓기는 뺀다.
    판정(8.5절): 확인 결과 자리에 가장 가까운 실제 물체가 30mm 안이면 그 물체를 집은 것. 위치 오차 15mm 이하이고 방향까지 맞으면 성공(정사각형은 위치만).
    rescan_home(RESCAN, 기획 #77): 후보가 없으면 팔을 대기 자세로 옮겨 src.rescan()(4바퀴)을 한 번 하고, 후보가 있으면 계속, 또 없으면 끝낸다.
    대기 자세는 robot.yaml의 poses.radar_clear(결정 0039)를 기구학으로 바꾼 손끝이고, 키가 없으면 팔 원점 위(arm.base, 높이 150mm)다. None이면 설정 radar.rescan_home을 따른다.
    check.max_fail_per_run: 확인 실패가 그만큼이면 끝낸다(src.give_up).
    PREPARE(결정 0041, 기획 #107): radar_clear는 첫 scan 전과 집기 실패 뒤에만 간다. PLACE 뒤는 거치지 않고 미리 돌린 결과로 scan한다(올라가는 이동은 인식 시간에 안 넣고, 대기 자세에서 첫 확인 경로로 내려오는 이동은 do_check에 들어간다).
    prefetch는 PLACE 시작(집은 뒤 배출 구역으로 옮기기 시작할 때)에 건다(2절·T5)."""
    if rescan_home is None:
        rescan_home = src.cfg.radar.rescan_home
    clk, recog, log, ok, rescanned = arm.clk, 0.0, [], 0, False
    ws, dropped = load_workspace(), []  # DECIDE는 작업 실행과 같은 task/policy를 쓴다(REQ-065). dropped = 확정 위치가 집기 범위 밖이라 뺀 후보 자리
    src.begin_cycle()
    arm.move(*arm.wait_pose)  # PREPARE(결정 0039·0041): 첫 scan 전에 radar_clear로. 올라가는 이동은 인식 시간에 넣지 않는다
    for _ in range(max_iter):
        if src.give_up:
            break
        t0 = clk.now_ns()
        if rescanned:  # 대기 자세로 옮긴 뒤의 RESCAN 결과
            scan = src.rescan()
        else:
            scan = src.scan()
        recog += (clk.now_ns() - t0) / 1e9
        for o in scan.objects:  # 레이더 후보 위치 오차: 남은 실제 물체 중 가장 가까운 것까지(없으면 기록 안 함)
            if esp.boxes:
                log.append({"result": "radar", "err": min(math.dist((b.x, b.y), (o.x_mm, o.y_mm)) for b in esp.boxes)})
        cand = policy.choose([o for o in scan.objects if all(math.dist((o.x_mm, o.y_mm), d) > 60 for d in dropped)], ws, unconfirmed=True)  # 확인 전 후보는 reach_mm 최대 + candidate_reach_margin_mm까지
        if cand is None:
            if rescan_home and not rescanned:
                rescanned = True
                arm.move(*arm.wait_pose)  # 레이더 시야 밖 대기 자세(poses.radar_clear)
                continue
            break
        rescanned = False  # 후보가 나왔으니 다음에 또 없으면 다시 RESCAN할 수 있다(연달아 두 번만 안 함)
        obj, probe, dt = do_check(src, arm, cand)
        recog += dt
        pulled = [p["stage"] for p in probe.info.get("pulled", [])]
        log.append({"result": "pulls", "per_stage": [pulled.count(n) for n in probe.info.get("stage_names", [])]})  # 첫 점 당기기: 단계마다 다시 부른 횟수
        if obj is None:
            log.append({"result": probe.status, "cand": (cand.x_mm, cand.y_mm)})
            continue
        if not policy.reachable(obj, ws):  # 확정 위치가 집기 범위 밖: 집지 않고 이 후보를 뺀다(runner.py와 같은 규칙)
            dropped.append((cand.x_mm, cand.y_mm))
            log.append({"result": "unreachable", "cand": (cand.x_mm, cand.y_mm)})
            continue
        box = min(esp.boxes, key=lambda b: math.dist((b.x, b.y), (obj.x_mm, obj.y_mm)), default=None)
        err = math.dist((box.x, box.y), (obj.x_mm, obj.y_mm)) if box else 1e9
        picked = err <= 30
        square = picked and box.width == box.length
        good = picked and err <= 15 and (square or angle_ok(obj.angle_deg, box.angle_deg % 180))
        log.append({"result": "ok" if good else "bad" if picked else "miss", "err": err,
                    "angle": (None if square else angle_ok(obj.angle_deg, box.angle_deg % 180)) if picked else None,
                    "h_err": abs(obj.height_mm - box.height) if picked else None})
        if not picked and report_failures:  # 확인한 자리에 물체가 없어 집기 실패: 작업 실행이 실패를 알린다(note_failure)
            src.note_failure(cand.x_mm, cand.y_mm)
        ok += good
        if picked:
            esp.boxes.remove(box)
        src.notify_picked(obj.id)
        arm.move((obj.x_mm, obj.y_mm), 55)
        if not picked:  # 집기 실패 뒤 PREPARE: 놓지 않고 radar_clear로(인식 시간 밖)
            arm.move(*arm.wait_pose)
            continue
        src.prefetch(arm.links_at)
        arm.move(drop, 55)
        clk.sleep(max(place_s - math.dist((obj.x_mm, obj.y_mm), drop) / arm.travel, 0))
    return ok, recog, log


def run_manual(boxes, arm, place_s=5.0, drop=(355.0, 80.0)):
    """비교 기준(비교 실험 (가)): run --source manual 흉내. 정답 좌표를 바로 받아(인식 0초) run_cycle과 같은 DECIDE(policy.choose)·이동·놓기 시간으로
    끝까지 간다. 고른 물체는 다 집은 것으로 본다. 돌려주는 값 = (집은 수, 끝까지 걸린 초)."""
    clk, ws, t0 = arm.clk, load_workspace(), arm.clk.now_ns()
    objs = [DetectedObject(f"m{i}", b.x, b.y, b.angle_deg % 180, b.width, b.length, 1.0, "manual", b.height) for i, b in enumerate(boxes)]
    ok = 0
    while (o := policy.choose(objs, ws)) is not None:
        objs.remove(o)
        ok += 1
        arm.move((o.x_mm, o.y_mm), 55)
        arm.move(drop, 55)
        clk.sleep(max(place_s - math.dist((o.x_mm, o.y_mm), drop) / arm.travel, 0))
    return ok, (clk.now_ns() - t0) / 1e9


def _cell(name, model, radius, seed, rescan_home, report_failures, arm_kw, pair_gap, opts):
    """시나리오의 칸 하나(씨앗 하나): 배치를 만들어 사이클을 돌고 행을 돌려준다."""
    rng = random.Random(f"{name}-{radius}-{seed}")
    if radius is not None:
        bx, by = arm_base()
        th = math.radians(rng.uniform(25, 155))
        w, length = rng.choice(OBJECT_SIZES)
        boxes = [Box(bx + radius * math.cos(th), by + radius * math.sin(th), rng.choice((0, 45, 90, 135)) + rng.uniform(-5, 5), w, length)]
    else:
        boxes = pair_layout(rng, pair_gap) if name == "pair" else random_layout(rng, name)
    n = len(boxes)
    truth_boxes = [replace(b) for b in boxes]  # run_cycle이 집은 물체를 뺀다
    cfg, clk, esp, arm = build(boxes, seed=seed, model=model, arm_kw=arm_kw, **opts)
    src = make_source(cfg, clk, esp, arm)
    t0 = clk.now_ns()
    ok, secs, log = run_cycle(src, arm, esp, rescan_home=rescan_home, report_failures=report_failures)
    total = (clk.now_ns() - t0) / 1e9
    *_, marm = build([], seed=seed, model=model, arm_kw=arm_kw, **opts)
    m_ok, m_s = run_manual(truth_boxes, marm)
    return {"radius": radius, "seed": seed, "n": n, "ok": ok, "recog_s": secs, "left": len(esp.boxes), "log": log,
            "total_s": total, "checks": sum(x["result"] == "pulls" for x in log), "manual_ok": m_ok, "manual_s": m_s,
            "radar_errs": [x["err"] for x in log if x["result"] == "radar"],
            "check_errs": [x["err"] for x in log if x["result"] in ("ok", "bad")],
            "angles": [x["angle"] for x in log if x["result"] in ("ok", "bad") and x["angle"] is not None],
            "h_errs": [x["h_err"] for x in log if x["result"] in ("ok", "bad")],
            "pulls": [n for x in log if x["result"] == "pulls" for n in x["per_stage"]]}


def _kin_arm_kw(kin_spec):
    hover, gripper = kin_spec
    return {"kin": kin_reach(hover, gripper)}


def _worker(conn, kin_spec):
    """시드를 하나씩 받아 돌리는 자식 프로세스(--seed-timeout). 한 칸이 멈추거나(예: 팔 궤적 계산이 끝나지 않을 때) 너무 오래 걸리면 부모가 이 프로세스를 끝낸다."""
    arm_kw = _kin_arm_kw(kin_spec)
    while True:
        job = conn.recv()
        if job is None:
            return
        name, model, radius, seed, rescan_home, report_failures, pair_gap, opts = job
        conn.send(_cell(name, model, radius, seed, rescan_home, report_failures, arm_kw, pair_gap, opts))


class _Guard:
    """칸마다 제한 시간을 두고 자식 프로세스에서 돌린다. 넘으면 자식을 끝내고 그 칸을 실패(recog_s = 제한)로 적은 뒤 다음 칸은 새 자식에서 돌린다."""

    def __init__(self, kin_spec, timeout_s):
        self.kin_spec, self.timeout_s, self.proc, self.conn = kin_spec, timeout_s, None, None

    def _start(self):
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_worker, args=(child, self.kin_spec), daemon=True)
        self.proc.start()

    def run(self, name, model, radius, seed, rescan_home, report_failures, pair_gap, opts, n):
        if self.proc is None:
            self._start()
        self.conn.send((name, model, radius, seed, rescan_home, report_failures, pair_gap, opts))
        if self.conn.poll(self.timeout_s):
            return self.conn.recv()
        self.proc.kill()
        self.proc = None
        print(f"  시간 초과: {name} {model} 반경 {radius} 씨앗 {seed} ({self.timeout_s:.0f}초) -> 실패로 적고 다음으로", flush=True)
        return {"radius": radius, "seed": seed, "n": n, "ok": 0, "recog_s": float(self.timeout_s), "left": n, "log": [],
                "radar_errs": [], "check_errs": [], "angles": [], "h_errs": [], "pulls": [], "timeout": True}

    def close(self):
        if self.proc is not None:
            self.proc.kill()


def run_scenario(name, model, seeds, rescan_home=None, report_failures=True, arm_kw=None, pair_gap=None, kin_spec=None, seed_timeout=None, seed_start=0, **opts):
    """시나리오 하나: 씨앗값 seed_start..seed_start+seeds-1마다 배치를 만들어 사이클을 돈다. single은 반경 구간(SINGLE_RADII)마다 seeds번.
    kin_spec=(호버, 그리퍼)와 seed_timeout(초)을 주면 칸마다 자식 프로세스에서 돌려 제한 시간을 넘는 칸은 실패로 적는다."""
    ss = range(seed_start, seed_start + seeds)
    cells = [(r, s) for r in SINGLE_RADII for s in ss] if name == "single" else [(None, s) for s in ss]
    n = {"single": 1, "pair": 2}.get(name, SCENARIOS[name][0])
    guard = _Guard(kin_spec, seed_timeout) if kin_spec and seed_timeout else None
    try:
        if guard:
            return [guard.run(name, model, r, s, rescan_home, report_failures, pair_gap, opts, n) for r, s in cells]
        return [_cell(name, model, r, s, rescan_home, report_failures, arm_kw, pair_gap, opts) for r, s in cells]
    finally:
        if guard:
            guard.close()


def summarize(rows) -> dict:
    n = rows[0]["n"]
    secs = sorted(r["recog_s"] for r in rows)
    out = {"runs": len(rows), "objects": n, "success_mean": statistics.mean(r["ok"] for r in rows),
           "success_pct": 100 * statistics.mean(r["ok"] for r in rows) / n, "left_mean": statistics.mean(r["left"] for r in rows),
           "recog_mean_s": statistics.mean(secs), "recog_median_s": statistics.median(secs),
           "recog_p90_s": secs[math.ceil(0.9 * len(secs)) - 1],  # 가장 가까운 순위(n=20이면 18번째). 평균과 같은 표본
           "recog_max_s": secs[-1]}
    # 1절 윗 3줄용 정확도(같은 표본): 레이더 후보 위치 오차, 확인 위치 오차, 방향 정답률
    radar = sorted(e for r in rows for e in r["radar_errs"])
    check = sorted(e for r in rows for e in r["check_errs"])
    angles = [a for r in rows for a in r["angles"]]
    heights = sorted(e for r in rows for e in r["h_errs"])
    out["radar_n"], out["check_n"], out["angle_n"] = len(radar), len(check), len(angles)
    if radar:
        out.update(radar_err_median_mm=statistics.median(radar), radar_err_p90_mm=radar[math.ceil(0.9 * len(radar)) - 1],
                   radar_within90_pct=100 * sum(e <= 90 for e in radar) / len(radar))
    if check:
        out.update(check_err_median_mm=statistics.median(check), check_err_p90_mm=check[math.ceil(0.9 * len(check)) - 1])
    if angles:
        out["angle_pct"] = 100 * sum(angles) / len(angles)
    if heights:
        out.update(h_err_median_mm=statistics.median(heights), h_err_p90_mm=heights[math.ceil(0.9 * len(heights)) - 1])
    done = [r for r in rows if "total_s" in r]  # 시간 초과 칸은 끝까지 시간·수동 비교가 없다
    if done:
        tot = sorted(r["total_s"] for r in done)
        out.update(checks_mean=statistics.mean(r["checks"] for r in done), total_mean_s=statistics.mean(tot), total_p90_s=tot[math.ceil(0.9 * len(tot)) - 1],
                   manual_pct=100 * statistics.mean(r["manual_ok"] for r in done) / n, manual_mean_s=statistics.mean(r["manual_s"] for r in done))
        # REQ-116: 같은 배치를 정답 위치로 집은 것(팔 도달 한계)과의 차이 = 인식 탓 손실. 이웃 합침과 도달 실패를 나눠 본다
        out.update(recog_loss_pct=out["manual_pct"] - 100 * statistics.mean(r["ok"] for r in done) / n,
                   recog_loss_runs=sum(r["manual_ok"] > r["ok"] for r in done))
    pulls = [n for r in rows for n in r.get("pulls", [])]
    if pulls:
        out.update(pull_mean=statistics.mean(pulls), pull_max=max(pulls), pull_stages=len(pulls))
    if rows[0]["radius"] is not None:
        out["by_radius_pct"] = {r: 100 * statistics.mean(x["ok"] for x in rows if x["radius"] == r) for r in SINGLE_RADII}
    return out


def commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], check=False, capture_output=True, text=True, timeout=5).stdout.strip() or "?"
    except Exception:  # noqa: BLE001  git이 없어도 벤치마크는 돈다
        return "?"


def format_table(results: list[dict]) -> str:
    lines = [f"commit {results[0]['commit']}", f"재현: {results[0].get('command', '?')}",
             "| 시나리오 | 모델 | 수 | 성공 평균 | 성공 % | 남은 물체 | 인식 평균(초) | 중앙값 | 90% | 최대 |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        s = r["summary"]
        lines.append(f"| {r['scenario']} | {r['model']} | {s['runs']} | {s['success_mean']:.2f}/{s['objects']} | {s['success_pct']:.0f}% | "
                     f"{s['left_mean']:.2f} | {s['recog_mean_s']:.0f} | {s['recog_median_s']:.0f} | {s['recog_p90_s']:.0f} | {s['recog_max_s']:.0f} |")
        if "by_radius_pct" in s:
            lines.append("|  | 반경별 성공 % | " + " ".join(f"{k}:{v:.0f}" for k, v in s["by_radius_pct"].items()) + " | | | | | | |")
    lines += ["", "| 시나리오 | 모델 | 레이더 후보 오차 중앙·90%(mm) | 90mm 안 % | 확인 위치 오차 중앙·90%(mm) | 방향 정답률 % | 높이 오차 중앙·90%(mm) |", "|---|---|---|---|---|---|---|"]
    for r in results:
        s = r["summary"]
        lines.append(f"| {r['scenario']} | {r['model']} | {s.get('radar_err_median_mm', float('nan')):.0f}·{s.get('radar_err_p90_mm', float('nan')):.0f} "
                     f"(n={s['radar_n']}) | {s.get('radar_within90_pct', float('nan')):.0f} | "
                     f"{s.get('check_err_median_mm', float('nan')):.1f}·{s.get('check_err_p90_mm', float('nan')):.1f} (n={s['check_n']}) | "
                     f"{s.get('angle_pct', float('nan')):.0f} (n={s['angle_n']}) | "
                     f"{s.get('h_err_median_mm', float('nan')):.1f}·{s.get('h_err_p90_mm', float('nan')):.1f} |")
    lines += ["", "| 시나리오 | 모델 | CHECK 평균 | 끝까지 평균·90%(초) | 수동(정답 좌표) 성공 % | 수동 끝까지 평균(초) |", "|---|---|---|---|---|---|"]
    for r in results:
        s = r["summary"]
        if "total_mean_s" in s:
            lines.append(f"| {r['scenario']} | {r['model']} | {s['checks_mean']:.2f} | {s['total_mean_s']:.0f}·{s['total_p90_s']:.0f} | "
                         f"{s['manual_pct']:.0f}% | {s['manual_mean_s']:.0f} |")
    lines += ["", "| 시나리오 | 모델 | 단계당 다시 부른 횟수 평균 | 최대 | 단계 수 |", "|---|---|---|---|---|"]
    for r in results:
        s = r["summary"]
        lines.append(f"| {r['scenario']} | {r['model']} | {s.get('pull_mean', 0):.3f} | {s.get('pull_max', 0)} | {s.get('pull_stages', 0)} |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="all", help="spread90 | spread120 | demo | demo80(모서리 80mm 이상) | show80(demo80, 반경 150~) | single | pair | all(pair·demo80·show80 뺀 넷)")
    ap.add_argument("--seeds", type=int, default=20, help="시나리오마다 배치 수(8.5절: 20 이상). single은 반경 구간마다")
    ap.add_argument("--seed-start", type=int, default=0, help="첫 씨앗값(앞 측정과 겹치지 않게 비교 실험에서 바꾼다)")
    ap.add_argument("--approach-line", action="store_true", help="T8: 첫 확인선을 팔 원점에서 후보로 가는 방향으로(check.approach_line)")
    ap.add_argument("--model", default="both", help="optimistic | pessimistic | both")
    ap.add_argument("--line-mm", type=int, help="확인 직선 길이(기본 설정값)")
    ap.add_argument("--speed", type=float, help="확인 속도 mm/s(기본 설정값)")
    ap.add_argument("--rescan-home", choices=["on", "off"], help="후보가 없을 때 대기 자세로 옮겨 RESCAN(기본: 설정 radar.rescan_home)")
    ap.add_argument("--no-note-failure", action="store_true", help="집기 실패를 source.note_failure로 알리지 않는다(효과 비교용)")
    ap.add_argument("--max-fail", type=int, help="check.max_fail_per_run(실패가 이만큼이면 끝냄, 기본: 설정값 3)")
    ap.add_argument("--empty-extra", type=int, help="radar.empty_extra_sweeps(기본: 설정값 3)")
    ap.add_argument("--reach", choices=["radius", "kin"], default="radius", help="가상 팔 도달 판정: 반경(기본, REACH) | 실제 기구학 IK(첫 점은 호버 포함)")
    ap.add_argument("--hover-mm", type=float, help="--reach kin의 접근 호버 높이(기본 robot.yaml approach_height_mm). (가)안")
    ap.add_argument("--tool-reach", help="MIN,MAX: check.tool_reach_mm를 이번만 바꾼다(기본 설정값 60,255). (나)안")
    ap.add_argument("--servo", choices=["pwm", "dxl"], default="pwm", help=f"가상 레이더 서보: pwm(기본, 지연 ±3°) | dxl(XL330: 지연 {DXL_LAG_DEG}° + 읽은 각도 ang 사용, 가정값이라 B4로 고친다)")
    ap.add_argument("--gripper", choices=["single_jaw", "parallel"], help="--reach kin의 그리퍼 종류(기본: 설정 파일 그대로 = 병렬). 병렬은 robot.yaml 위에 grippers/parallel.yaml(URDF 포함)을 겹친다")
    ap.add_argument("--seed-timeout", type=float, default=300.0, help="--reach kin에서 씨앗 하나의 제한 시간(초, 기본 300, 0이면 끔). 넘으면 그 씨앗을 실패로 적고 다음으로 간다(멈춘 칸을 끊기 위해 자식 프로세스에서 돌린다)")
    ap.add_argument("--pair-gap", type=float, default=40.0, help="--scenario pair의 두 물체 모서리 간격 mm(기본 40)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL", help="check.<KEY>를 이번만 바꾼다(여러 번 가능, 예: --set yaw_deg=45 --set ring_r_mm=50 --set clearance_mm=40)")
    ap.add_argument("--out", default="runs/bench")
    a = ap.parse_args(argv)
    names = [k for k in SCENARIOS if k not in ("pair", *EDGE80)] if a.scenario == "all" else [a.scenario]
    models = list(MODELS) if a.model == "both" else [a.model]
    opts = {k: v for k, v in (("line_mm", a.line_mm), ("speed_mm_s", a.speed), ("max_fail_per_run", a.max_fail)) if v is not None}
    opts.update({k: json.loads(v) for k, v in (kv.split("=", 1) for kv in a.set)})
    if a.gripper:  # 그리퍼별 도달 범위(check.tool_reach_by_gripper)를 같은 길로 읽는다. --tool-reach가 있으면 그쪽이 우선
        opts["gripper"] = a.gripper
    if a.approach_line:
        opts["approach_line"] = True
    if a.tool_reach:
        lo, hi = (float(v) for v in a.tool_reach.split(","))
        opts["tool_reach_mm"] = {"min": lo, "max": hi}
    kin_spec = (a.hover_mm, a.gripper) if a.reach == "kin" else None
    seed_timeout = a.seed_timeout if kin_spec and a.seed_timeout > 0 else None
    arm_kw = _kin_arm_kw(kin_spec) if kin_spec and not seed_timeout else None
    if a.servo == "dxl":
        opts["esp_kw"] = {"servo": "dxl", "servo_lag_deg": DXL_LAG_DEG}
    if a.empty_extra is not None:
        opts["radar"] = {"empty_extra_sweeps": a.empty_extra}
    rescan = None if a.rescan_home is None else a.rescan_home == "on"
    results, sha = [], commit()
    cmd = "python -m so101_studio.perception.ultrasonic.benchmark " + " ".join(argv if argv is not None else __import__("sys").argv[1:])
    for name, model in itertools.product(names, models):
        rows = run_scenario(name, model, a.seeds, rescan_home=rescan, report_failures=not a.no_note_failure, arm_kw=arm_kw, pair_gap=a.pair_gap, kin_spec=kin_spec, seed_timeout=seed_timeout, seed_start=a.seed_start, **opts)
        results.append({"commit": sha, "command": cmd, "scenario": name, "model": model, "options": {**opts, "reach": a.reach, "hover_mm": a.hover_mm, "rescan_home": rescan, "note_failure": not a.no_note_failure}, "summary": summarize(rows),
                        "rows": [{k: v for k, v in r.items() if k != "log"} for r in rows]})
        s = results[-1]["summary"]
        loss = f" · 정답 위치 {s['manual_pct']:.0f}% · 인식 손실 {s['recog_loss_pct']:.0f}%p" if "recog_loss_pct" in s else ""
        print(f"  {name} {model}: 성공 {s['success_mean']:.2f}/{s['objects']}{loss}", flush=True)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d-%H%M%S')}-{sha}.json"
    path.write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")
    print(format_table(results))
    print("saved", path)


if __name__ == "__main__":
    main()
