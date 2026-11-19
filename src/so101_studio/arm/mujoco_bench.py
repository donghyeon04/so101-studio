"""M5 집기 시험장 (mujoco-spec 13.10.1, 결정 0062): 공통 층(세계 짓기·판정·기록)과 실행기 A(ArmService·Skills를 MuJoCoDriver 위에서 그대로).

실행: `python -m so101_studio.arm.mujoco_bench --suite <이름> [--seeds N] [--gripper parallel|single_jaw] [--jobs N] [--only <에피소드 id>]`.
한 시도 = 한 에피소드이고 에피소드마다 드라이버와 ArmService를 새로 만든다. 지금은 실시간(벽시계) 모드다(시계 주입 13.10.2 전)."""
import argparse
import contextlib
import itertools
import json
import math
import subprocess
import threading
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from ..core.frames import robot_to_table
from ..core.types import DetectedObject

# 모듈 맨 위에서: 장면 STL(load_scene)을 읽은 뒤 torch(lerobot)를 처음 부르면 shm.dll을 못 읽는다(Windows)
from ..service.arm_service import ArmService
from . import mujoco_grid

REPO_ROOT = Path(__file__).resolve().parents[3]
BENCH_DIR = REPO_ROOT / "config" / "sim_scenarios" / "bench"
OVERRIDE = BENCH_DIR / "robot.yaml"
ARM = "so101_follower_01"
SAMPLE_S = 0.02  # 표본 50Hz
SETTLE_S = 1.0  # 끝나고 판정까지 기다리는 시간
PLACE_TOL_MM, TILT_MAX_DEG, PUSHED_MM, PICKED_Z_MM = 10.0, 10.0, 10.0, 5.0
EVENT_CODES = ("ARM-BLOCKED", "ARM-GRIP-OVERLOAD-WARN", "ARM-GRIP-OVERLOAD", "ARM-FAULT", "ARM-CARRY-LOW", "ARM-OPENING-PATH-FALLBACK")
GIVEN_KEYS = ("pos_offset_mm", "width_error_mm", "height_error_mm", "angle_error_deg")
OBJ = "box"  # 한 개짜리 묶음의 물체 이름


# -- 표본·미끄러짐·이벤트 (13.7 (b) 정의를 한 곳에) --
class Sampler:
    """sim_state()를 50Hz로 읽어 모은다. max_load는 관절 부하 % 최대.
    벽시계면 스레드 하나, StepClock(clock.drives_status)이면 시계의 every(SAMPLE_S)로 돈다(13.10.2: 시뮬레이션 시간 50Hz, 따로 스레드 없음)."""

    def __init__(self, drv, clock=None):
        self.drv, self.samples, self.max_load, self._stop = drv, [], 0.0, threading.Event()
        self.clock = clock if getattr(clock, "drives_status", False) else None
        self._t = None if self.clock else threading.Thread(target=self._run, daemon=True)

    def _sample(self) -> bool:
        try:
            self.samples.append(self.drv.sim_state())
            self.max_load = max(self.max_load, *self.drv.read_health()["load_pct"].values())
            return True
        except RuntimeError:  # 물리가 멈췄다(failed_reason): 표본만 그친다
            return False

    def _run(self):
        while not self._stop.is_set() and self._sample():
            time.sleep(SAMPLE_S)

    def _every(self):
        if not self._stop.is_set():
            self._sample()

    def __enter__(self):
        if self.clock:
            self.clock.every(SAMPLE_S, self._every)
        else:
            self._t.start()
        return self

    def __exit__(self, *a):
        self._stop.set()
        if self._t is not None:
            self._t.join()


def slip_series(samples, name: str) -> list[tuple[float, float, float]]:
    """13.7 (b): 물체가 held이고 바닥이 5mm 넘게 뜬 표본마다 (시뮬레이션 시각 s, 바닥 z mm, 처음 든 순간 대비 손 좌표계 안에서 움직인 거리 mm)."""
    out, first = [], None
    for st in samples:
        o = st["objects"][name]
        if st["held"] == name and o["z_mm"] > PICKED_Z_MM:
            rel = np.array(st["tool"]["rot"]).T @ (np.array([o["x_mm"], o["y_mm"], o["z_mm"]]) - np.array(st["tool"]["pos_mm"]))
            first = rel if first is None else first
            out.append((st["t_s"], o["z_mm"], float(np.linalg.norm(rel - first))))
    return out


def read_events(svc) -> list[dict]:
    svc.logger.flush()  # 이벤트는 쓰기 스레드의 큐를 거친다: 끝 이벤트(ARM-BLOCKED 등)까지 파일에 쓴 뒤 읽는다
    return [json.loads(x) for x in (svc.logger.dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]


# -- 묶음과 배치 --
@dataclass
class Episode:
    id: str
    seed: int
    truth: dict  # SimObject 인자(MuJoCo에 놓는 참값)
    inject: dict  # given 어긋남(GIVEN_KEYS)
    place: dict  # {zone: 이름} 또는 {at: [x, y]}


def load_suite(name: str) -> dict:
    path = Path(name) if str(name).endswith(".yaml") else BENCH_DIR / f"{name}.yaml"
    if not path.exists():
        raise ValueError(f"묶음 파일이 없다: {path}")
    suite = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    suite.setdefault("name", path.stem)
    unknown = set(suite.get("given") or {}) - set(GIVEN_KEYS)
    if unknown:
        raise ValueError(f"{path.name}의 given에 모르는 키: {sorted(unknown)}")
    return suite


def _truth(r: float, bearing: float, angle: float, size, base, dx: float = 0.0, dy: float = 0.0) -> dict:
    """반경·방위(팔 원점, 팔 정면 기준)·각도·크기 → SimObject 인자(작업대 좌표)."""
    length, width, height = size
    x, y = robot_to_table(r * math.cos(math.radians(bearing)), r * math.sin(math.radians(bearing)), base)
    return {"name": OBJ, "x_mm": round(x + dx, 3), "y_mm": round(y + dy, 3), "angle_deg": round(float(angle), 3),
            "length_mm": float(length), "width_mm": float(width), "height_mm": float(height)}


def _bin_rects() -> list[tuple]:
    """통 구역의 바깥 사각형(벽 두께 포함) (x0, y0, x1, y1) 목록."""
    from ..core.config import load_workspace
    from ..core.scene import BIN_WALL_T_MM

    t = BIN_WALL_T_MM
    return [(z["x_mm"] - z["w_mm"] / 2 - t, z["y_mm"] - z["h_mm"] / 2 - t, z["x_mm"] + z["w_mm"] / 2 + t, z["y_mm"] + z["h_mm"] / 2 + t)
            for z in load_workspace(REPO_ROOT / "config" / "workspace.yaml").drop_zones.values() if z.get("type") == "bin"]


def _hits_bin(truth: dict, rects) -> bool:
    """물체를 감싸는 원(반 대각선)이 통 바깥 사각형에 닿는가. 회전과 상관없이 보수적으로 본다."""
    rad = math.hypot(truth["length_mm"], truth["width_mm"]) / 2
    return any(math.hypot(max(x0 - truth["x_mm"], 0, truth["x_mm"] - x1), max(y0 - truth["y_mm"], 0, truth["y_mm"] - y1)) < rad for x0, y0, x1, y1 in rects)


def _base():
    from ..core.config import load_workspace

    return load_workspace(REPO_ROOT / "config" / "workspace.yaml").base


def episodes(suite: dict, seeds: int = 1, base=None, seed_start: int = 0) -> list[Episode]:
    """묶음 → 에피소드 목록(씨앗 고정). 격자 순번은 objects 키(YAML 순서, size_mm이 바깥) × given 키(YAML 순서)의 곱집합.
    objects는 radius_mm·bearing_deg(팔 원점에서 반경, 팔 정면 기준 방위) 대신 positions: [[반경, 방위], …]를 받을 수 있다.
    objects 대신 sample: {size_mm, radius_mm: [하한, 상한], bearing_deg: [..], angle_deg: [..]}이면 씨앗마다 고르게 뽑는다(수집용, 13.10.4).
    angle_deg: along이면 상자 한 면이 그 자리 방위(팔에서 상자로 가는 방향)와 나란하다(1차 묶음, 결정 0068)."""
    base = base or _base()
    place = suite.get("place") or {"zone": "right"}
    if "sample" in suite:
        sm = suite["sample"]
        eps = []
        bins = _bin_rects()
        for s in range(seed_start, seed_start + seeds):
            rng = np.random.default_rng(s)
            for _ in range(100):  # 배출 통(벽 포함)에 걸리는 자리는 같은 씨앗의 다음 수로 다시 뽑는다(결정 0067)
                r, b = rng.uniform(*sm["radius_mm"]), rng.uniform(*sm["bearing_deg"])
                a = b + base.yaw_deg if sm["angle_deg"] == "along" else rng.uniform(*sm["angle_deg"])
                truth = _truth(r, b, a, sm["size_mm"], base)
                if not _hits_bin(truth, bins):
                    break
            eps.append(Episode(f"{suite['name']}-0000-s{s}", s, truth, {}, place))
        return eps
    objs, given = dict(suite["objects"]), dict(suite.get("given") or {})
    if "positions" not in objs:  # radius_mm × bearing_deg를 positions 하나로(반경이 바깥)
        objs = {("positions" if k == "radius_mm" else k): ([[r, b] for r in v for b in objs["bearing_deg"]] if k == "radius_mm" else v)
                for k, v in objs.items() if k != "bearing_deg"}
    objs.setdefault("angle_deg", [0])
    unknown = set(objs) - {"size_mm", "positions", "angle_deg"}
    if unknown:
        raise ValueError(f"묶음 objects에 모르는 키: {sorted(unknown)}")
    keys, gkeys = list(objs), list(given)
    spread = suite.get("spread") or {}
    sxy, sang = float(spread.get("xy_mm", 0)), float(spread.get("angle_deg", 0))
    n_seeds = seeds if (sxy or sang) else 1
    eps = []
    for i, combo in enumerate(itertools.product(*[objs[k] for k in keys], *[given[k] for k in gkeys])):
        c = dict(zip(keys + gkeys, combo))
        r, bearing = c["positions"]
        if any(list(c["size_mm"]) == list(k["size_mm"]) and [r, bearing] == [k["radius_mm"], k["bearing_deg"]] and c["angle_deg"] == k["angle_deg"]
               for k in suite.get("skip") or []):  # 묶음의 skip 칸(배출 통과 겹침 등)은 뺀다. 순번 i는 그대로라 다른 칸의 에피소드 id가 바뀌지 않는다
            continue
        for s in range(seed_start, seed_start + n_seeds):
            rng = np.random.default_rng([i, s])
            dx, dy, da = (rng.uniform(-sxy, sxy), rng.uniform(-sxy, sxy), rng.uniform(-sang, sang)) if (sxy or sang) else (0.0, 0.0, 0.0)
            truth = _truth(r, bearing, float(c["angle_deg"]) + da, c["size_mm"], base, dx, dy)
            eps.append(Episode(f"{suite['name']}-{i:04d}-s{s}", s, truth, {k: c[k] for k in gkeys}, place))
    return eps


# -- 공개 규약 (13.10.4): GUI와 PolicyRunner는 Layout·layout·judge만 쓴다 --
@dataclass
class Layout:
    objects: list  # SimObject(MuJoCo에 놓는 참값)
    place: dict  # {zone: 이름} 또는 {at: [x, y]}
    place_xy_mm: tuple  # 놓을 곳 가운데(구역이면 구역 가운데)
    task: str  # 과제 문장(데이터셋 task)


def task_text(suite: dict) -> str:
    place = suite.get("place") or {"zone": "right"}
    return suite.get("task") or (f"pick the box and place it in the {place['zone']} zone" if "zone" in place else "pick the box and place it at the target")


def layout(suite: str, seed: int) -> Layout:
    """묶음 이름과 씨앗 하나 → 배치. sample 묶음은 씨앗으로 뽑고, 격자 묶음은 씨앗을 격자 순번으로 본다(칸 수로 나눈 나머지)."""
    s = load_suite(suite)
    eps = episodes(s, 1, seed_start=seed) if "sample" in s else episodes(s)
    return _layout_of(eps[0] if "sample" in s else eps[seed % len(eps)], s)


def _layout_of(ep: Episode, suite: dict) -> Layout:
    from ..core.config import load_workspace
    from .mujoco_model import SimObject

    if "zone" in ep.place:
        z = load_workspace(REPO_ROOT / "config" / "workspace.yaml").drop_zones[ep.place["zone"]]
        xy = (float(z["x_mm"]), float(z["y_mm"]))
    else:
        xy = tuple(float(v) for v in ep.place["at"])
    return Layout([SimObject(**ep.truth)], ep.place, xy, task_text(suite))


def in_bin(o: dict, zone: dict) -> bool:
    """통 구역(type: bin, 결정 0067) 판정: 물체 중심이 통 안쪽 사각형 안이고 바닥 위(벽 위에 얹히거나 판 밖으로 떨어지지 않음). 기울기·각도는 보지 않는다."""
    return abs(o["x_mm"] - zone["x_mm"]) <= zone["w_mm"] / 2 and abs(o["y_mm"] - zone["y_mm"]) <= zone["h_mm"] / 2 and -1.0 <= o["z_mm"] <= float(zone["wall_mm"]) - 1.0  # z_mm는 물체 바닥 높이: 통 바닥(판) 위, 벽 윗면(wall_mm)에 얹힌 것은 아님


def judge(driver, layout: Layout) -> dict:
    """물리만 보는 판정(보고 없이, 13.10.1 판정 표의 물리 쪽): driver.sim_state()와 layout만 읽는다.
    outcome: placed_ok(구역 직사각형 안 또는 at에서 PLACE_TOL_MM 안, 기울기 < 10°) → toppled → moved(처음 자리에서 10mm 넘게) → untouched.
    표본이 없어 '집었다가 떨굼'과 '밀림'은 moved 하나로 본다."""
    from ..core.config import load_workspace

    o0 = layout.objects[0]
    o = driver.sim_state()["objects"][o0.name]
    err = math.hypot(o["x_mm"] - layout.place_xy_mm[0], o["y_mm"] - layout.place_xy_mm[1])
    tilt = float(o["tilt_deg"])
    z = load_workspace(REPO_ROOT / "config" / "workspace.yaml").drop_zones[layout.place["zone"]] if "zone" in layout.place else {}
    if "w_mm" in z and "h_mm" in z:  # 사각형 구역: 그 안 어디든
        inside = abs(o["x_mm"] - z["x_mm"]) <= z["w_mm"] / 2 and abs(o["y_mm"] - z["y_mm"]) <= z["h_mm"] / 2
    else:  # at 또는 옛 점 구역
        inside = err <= PLACE_TOL_MM
    placed = in_bin(o, z) if z.get("type") == "bin" else inside and tilt < TILT_MAX_DEG
    moved = math.hypot(o["x_mm"] - o0.x_mm, o["y_mm"] - o0.y_mm)
    outcome = "placed_ok" if placed else "toppled" if tilt >= TILT_MAX_DEG else "moved" if moved > PUSHED_MM else "untouched"
    return {"outcome": outcome, "placed": placed, "place_err_mm": round(err, 2), "tilt_end_deg": round(tilt, 2), "moved_mm": round(moved, 2)}


def make_given(truth: dict, inject: dict, skills) -> tuple[DetectedObject, float, float]:
    """참값 + 어긋남 → (팔에게 알려 주는 값, G mm, 벌림 축 yaw °). G·벌림 축은 자리 어긋남 전의 값으로 Skills가 계산한다(13.10.1 실패 주입)."""
    g0 = DetectedObject(id=OBJ, x_mm=truth["x_mm"], y_mm=truth["y_mm"], angle_deg=truth["angle_deg"] + inject.get("angle_error_deg", 0.0),
                        width_mm=truth["width_mm"] + inject.get("width_error_mm", 0.0), length_mm=truth["length_mm"],
                        confidence=1.0, source="manual", height_mm=truth["height_mm"] + inject.get("height_error_mm", 0.0))
    gap = float(skills._needed_gap(g0, tilt_ok=False))
    yaw = float(skills._pick_yaw(g0))
    a, b = inject.get("pos_offset_mm", (0.0, 0.0))
    ux, uy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return replace(g0, x_mm=g0.x_mm + a * ux - b * uy, y_mm=g0.y_mm + a * uy + b * ux), gap, yaw


# -- 설정과 세계 --
def load_cfg(override: Path | None = None):
    """시험장 기준 cfg: robot.yaml + 팔 파일 + 덮어쓰기(PC의 local.yaml 대신)."""
    from ..core.config import load_robot

    return load_robot(REPO_ROOT / "config" / "robot.yaml", arm=ARM, local_path=override or OVERRIDE)


def gripper_override(gripper: str, folder: Path) -> Path:
    """`--gripper`: 그리퍼 종류는 읽을 때 층을 고르므로 cfg를 바꾸지 않고 한 줄짜리 덮어쓰기 파일을 만든다."""
    folder.mkdir(parents=True, exist_ok=True)
    p = folder / "robot_override.yaml"
    p.write_text(f"gripper: {gripper}\n", encoding="utf-8")
    return p


class BenchEnv:
    """Gymnasium 꼴 겉모양(reset만, step은 실행기 B 몫). reset마다 드라이버·ArmService를 새로 짓는다."""

    def __init__(self, cfg, ws=None, realtime: bool = True, viewer: bool = False, clock: str = "wall"):
        """clock: wall(실시간 벽시계) | step(StepClock, 13.10.2: realtime=False 드라이버, 수동 뷰어 안 됨)."""
        from ..core.config import load_workspace
        from ..core.scene import load_scene

        if clock == "step" and viewer:
            raise ValueError("--clock step은 수동 뷰어(--viewer)와 쓸 수 없다(--viewer grid를 쓴다)")
        self.cfg, self.realtime, self.viewer, self.clock_kind = cfg, realtime and clock != "step", viewer, clock
        self.clock = None
        self.ws = ws or load_workspace(REPO_ROOT / "config" / "workspace.yaml")
        self.scene = load_scene(REPO_ROOT / "config" / "scene.yaml", self.ws).for_gripper(cfg.gripper)  # 통 벽(bin_*)까지: 서비스와 같은 장면, 결과 머리의 부품 목록에도 나온다
        self.drv = self.svc = None

    def reset(self, seed=None, options=None):
        """options: {episode: Episode, log_dir: 폴더}. 관측(관절, 참값, 알려 준 값)과 info(G, 벌림 축, 연결 직후 버림 수)."""
        from .mujoco_driver import MuJoCoDriver
        from .mujoco_model import SimObject
        from .sim_driver import _check_overlap

        self.close()
        ep = options["episode"]
        objs = [SimObject(**ep.truth)]
        _check_overlap(objs, Path(f"{ep.id}.yaml"))
        self.drv = MuJoCoDriver(self.cfg, self.ws, scene=self.scene, objects=objs, realtime=self.realtime, viewer=self.viewer, scenario=ep.id.rsplit("-", 2)[0])
        if self.clock_kind == "step":
            from .mujoco_clock import StepClock

            self.clock = StepClock(self.drv)  # 에피소드 스레드에서 에피소드마다 새로(만든 스레드가 참여, every·틱 스레드가 앞 에피소드와 섞이지 않게)
        self.svc = ArmService(self.drv, self.cfg, self.ws, log_root=str(options["log_dir"]), clock=self.clock)
        self.clock = self.svc.clock
        self.svc.incident_after_s = 0  # 실패 주입이 사고 묶음·60초 후속 스레드를 띄우지 않게
        ok, msg = self.svc.connect()
        if not ok:
            raise RuntimeError(f"연결 실패: {msg}")
        given, gap, yaw = make_given(ep.truth, ep.inject, self.svc.skills)
        info = {"gap0_mm": round(gap, 2), "jaw_yaw_deg": round(yaw, 2), "drops_at_connect": self.drv.dropped_count}
        return {"joints": asdict(self.svc.driver.read_joints()), "truth": ep.truth, "given": asdict(given)}, info

    def close(self):
        if self.svc is not None:
            self.svc.disconnect()
        if getattr(self.clock, "drives_status", False):
            self.clock.close()
        self.drv = self.svc = self.clock = None


# -- 판정 --
def judge_episode(result, samples: list, end: dict, truth: dict, target_xy, events: list, failed: str | None, zone: dict | None = None) -> dict:
    """13.10.1 판정 표. result는 MotionResult(물리 실패면 None), end는 끝나고 1.0초 뒤 sim_state. zone이 통(type: bin)이면 놓기는 in_bin으로 본다."""
    reported = {"status": result.status if result else None, "reason": result.reason if result else None}
    o_end = end["objects"][OBJ] if end else None
    picked = any(st["held"] == OBJ and st["objects"][OBJ]["z_mm"] > PICKED_Z_MM for st in samples)
    place_err = float(math.hypot(o_end["x_mm"] - target_xy[0], o_end["y_mm"] - target_xy[1])) if (o_end and target_xy) else None
    tilt_end = float(o_end["tilt_deg"]) if o_end else None
    if zone is not None and zone.get("type") == "bin":
        placed = o_end is not None and in_bin(o_end, zone)
    else:
        placed = place_err is not None and place_err <= PLACE_TOL_MM and tilt_end < TILT_MAX_DEG
    tools = [np.array(st["tool"]["pos_mm"]) for st in samples]
    tool_moved = max((float(np.linalg.norm(p - tools[0])) for p in tools), default=0.0)
    moved = float(math.hypot(o_end["x_mm"] - truth["x_mm"], o_end["y_mm"] - truth["y_mm"])) if o_end else 0.0
    st = reported["status"]
    if failed:
        outcome = "sim_fail"
    elif st in ("out_of_bounds", "height_required", "locked") or (st == "ik_fail" and tool_moved < 1.0):
        outcome = "rejected"
    elif placed:
        outcome = "placed_ok"
    elif picked:
        outcome = "dropped"
    elif tilt_end is not None and tilt_end >= TILT_MAX_DEG:
        outcome = "toppled"
    elif moved > PUSHED_MM:
        outcome = "pushed"
    else:
        outcome = "untouched"
    agree = "false_ok" if st == "ok" and outcome != "placed_ok" else "false_fail" if st != "ok" and outcome == "placed_ok" else "match"
    series = slip_series(samples, OBJ)
    others = [n for n in (end or {}).get("objects", {}) if n != OBJ]  # 한 개짜리 묶음은 비어 0
    counts = {c: sum(1 for e in events if e.get("code") == c) for c in EVENT_CODES}
    return {"reported": reported, "picked": picked, "placed": placed, "outcome": outcome, "agree": agree,
            "slip_mm": round(max(d for _, _, d in series), 2) if series else None,
            "place_err_mm": None if place_err is None else round(place_err, 2), "tilt_end_deg": None if tilt_end is None else round(tilt_end, 2),
            "other_pushed_mm": 0.0 if not others else None, "events": counts, "target_xy": target_xy, "failed_reason": failed}


def run_episode(ep: Episode, cfg, log_dir: Path, env: BenchEnv | None = None, samples_out: list | None = None, recorder=None, viewer: bool = False,
                settle_s: float = SETTLE_S, clock: str = "wall", start_pose=None) -> dict:
    """실행기 A: reset → pick_and_place(그동안 표본) → settle_s 뒤 판정 → disconnect. 결과 한 줄(dict). samples_out에 표본을 붙인다(시험용).
    recorder(EpisodeRecorder)가 있으면 pick_and_place 직전부터 판정까지 기록하고, outcome이 placed_ok이고 realtime_lag가 없을 때만 저장한다(13.10.4, 결정 0068).
    settle_s: 수집 묶음은 suite의 settle_s(0.3초, 결정 0068)를 쓴다."""
    env = env or BenchEnv(cfg, viewer=viewer, clock=clock)
    t0 = time.perf_counter()
    obs, info = env.reset(options={"episode": ep, "log_dir": log_dir})
    drv, svc, clk = env.drv, env.svc, env.clock
    stepped = getattr(clk, "drives_status", False)
    feeder = _feeder(drv, ep.id, clk)
    given = DetectedObject(**obs["given"])
    zone, at = ep.place.get("zone"), ep.place.get("at")
    result, failed = None, None
    drops0 = info["drops_at_connect"]
    try:
        _go_start(svc, start_pose)
        if recorder is not None:
            recorder.svc, recorder.driver = svc, drv  # 에피소드마다 새 드라이버·ArmService
            recorder.start(clock=clk)
            t_first = time.perf_counter()
            while not stepped and recorder.frames < 2 and time.perf_counter() - t_first < 5.0:  # 첫 렌더(렌더러 만들기)가 물리를 한 번 밀 수 있다: 팔이 서 있을 때 끝내고 거기서부터 lag를 센다
                time.sleep(0.01)
            drops0 = drv.dropped_count
        with Sampler(drv, clk) as s:
            try:
                result = svc.pick_and_place(given, zone=zone, place_at=tuple(at) if at else None)
            except RuntimeError as e:  # 물리 실패(MuJoCoDriver._check)
                failed = str(e)
            clk.sleep(settle_s)
        failed = failed or drv.failed_reason
        if samples_out is not None:
            samples_out.extend(s.samples)
        end = None if failed else drv.sim_state()
        target = svc.skills.last_place_xy
        row = judge_episode(result, s.samples, end, ep.truth, list(target) if target else None, read_events(svc), failed, env.ws.drop_zones.get(zone) if zone else None)
        drops = drv.dropped_count - drops0
        row.update({"sim_s": round(end["t_s"], 2) if end else None, "wall_s": round(time.perf_counter() - t0, 2), "dropped_count": drops,
                    "realtime_lag": drv.realtime and drops > 0, "gap0_mm": info["gap0_mm"], "jaw_yaw_deg": info["jaw_yaw_deg"], "max_load_pct": round(s.max_load, 1)})
        if feeder is not None:
            feeder.note(f"{ep.id}: {row['outcome']}")
        if recorder is not None:
            row["recorded_frames"] = recorder.stop(save=row["outcome"] == "placed_ok" and not row["realtime_lag"])
        return {"episode": ep.id, "seed": ep.seed, "gripper": cfg.gripper, "scene_parts": [p.name for p in env.scene.parts],
                "truth": ep.truth, "given": obs["given"], "inject": ep.inject, "place": ep.place, **row}
    finally:
        if recorder is not None:
            recorder.stop(save=False)  # 판정 전에 끝났으면 버린다(이미 멈췄으면 아무 일도 안 한다)
        if feeder is not None:
            feeder.__exit__()
        env.close()


def _feeder(drv, label: str, clock=None):
    """--viewer grid면 이 에피소드의 보기용 시점을 격자 창으로 보낸다(mujoco_grid). 아니면 None. --clock step이면 첫 일꾼만 그린다(13.10.2)."""
    q = mujoco_grid.current()
    if q is None or (getattr(clock, "drives_status", False) and not mujoco_grid.claim()):
        return None
    return mujoco_grid.Feeder(q, drv, label, clock).__enter__()


POLICY_MAX_S = 45.0  # 정책 한 에피소드 상한(전문가 시연이 약 31초라 PolicyRunner 기본 20초로는 모자란다)


def _run_stepped(runner, clock) -> None:
    """--clock step: PolicyRunner 한 걸음을 시계의 every(1/fps)로 돌린다(13.10.2, 스레드 없음, 추론 지연은 시뮬레이션 시간 0이라 too_slow 없음).
    순서는 팔 REQ-086의 공개 메서드대로: begin() 뒤 걸음마다 due_stop(slow=False) or step(), 이유가 생기면 finish(이유)."""
    why = runner.begin(clock="step")
    if why:
        raise RuntimeError(why)
    done, out = threading.Event(), {}

    def tick():
        if done.is_set():
            return
        if (why := runner.due_stop(slow=False) or runner.step()) is not None:
            out["reason"] = why
            done.set()

    clock.every(1.0 / runner.fps, tick)
    clock.wait(done)
    runner.finish(out["reason"])


def _go_start(svc, start_pose) -> None:
    """묶음의 start_pose(관절 6개, 결정 0071)가 있으면 연결 뒤 그 자세로 옮긴 다음 기록·정책을 시작한다(손목 카메라가 상자를 보게)."""
    if start_pose:
        from ..core.types import JointState

        r = svc.move_joints(JointState(*map(float, start_pose)))
        if r.status != "ok":
            raise RuntimeError(f"시작 자세로 못 갔다: {r.status} {r.reason}")


def run_policy_episode(ep: Episode, suite: dict, cfg, log_dir: Path, policy, max_s: float = POLICY_MAX_S, viewer: bool = False, clock: str = "wall") -> dict:
    """정책 평가 한 에피소드(13.10.4 학습과 평가): reset → PolicyRunner(GUI 없이) → 끝나고 judge(driver, layout). 한 줄(dict)."""
    from functools import partial

    from ..service.policy_runner import PolicyRunner

    env = BenchEnv(cfg, viewer=viewer, clock=clock)
    t0 = time.perf_counter()
    env.reset(options={"episode": ep, "log_dir": log_dir})
    feeder = _feeder(env.drv, ep.id, env.clock)
    try:
        _go_start(env.svc, suite.get("start_pose"))
        runner = PolicyRunner(env.svc, env.drv, policy=policy, max_s=max_s, judge=partial(judge, layout=_layout_of(ep, suite)))
        if getattr(env.clock, "drives_status", False):
            _run_stepped(runner, env.clock)
        else:
            why = runner.start()
            if why:
                raise RuntimeError(why)
            while runner.result is None:  # 러너가 판정까지 마치면 result를 채운다
                time.sleep(0.2)
        r = runner.result
        if feeder is not None:
            feeder.note(f"{ep.id}: {r['outcome']}")
        return {"episode": ep.id, "seed": ep.seed, "truth": ep.truth, "outcome": r["outcome"], "policy": r, "wall_s": round(time.perf_counter() - t0, 2)}
    finally:
        if feeder is not None:
            feeder.__exit__()
        env.close()


# -- 실행과 기록 --
def _commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short=7", "HEAD"], check=False, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10).stdout.strip() or "nogit"
    except Exception:  # noqa: BLE001
        return "nogit"


def _worker(job, recorder=None):
    ep, override, out, grip_force, viewer, settle_s, clock, start_pose = job
    import os

    from . import mujoco_hand

    os.chdir(REPO_ROOT)  # ArmService는 config/ 를 현재 폴더 기준으로 읽는다
    force0 = mujoco_hand.GRIP_FORCE_N
    if grip_force is not None:  # U15 민감도: 병렬 손 액추에이터 힘 한계(forcerange)만 바꾼다. kp·Torque_Limit 배율은 그대로
        mujoco_hand.GRIP_FORCE_N = float(grip_force)
    try:
        cfg = load_cfg(override)
        row = run_episode(ep, cfg, Path(out) / "episodes" / ep.id, recorder=recorder, viewer=viewer, settle_s=settle_s, clock=clock, start_pose=start_pose)
    except Exception as e:  # noqa: BLE001 한 에피소드가 깨져도 묶음은 계속 돈다: sim_fail 한 줄로 남긴다
        row = {"episode": ep.id, "seed": ep.seed, "gripper": None, "scene_parts": [], "truth": ep.truth, "given": None, "inject": ep.inject, "place": ep.place,
               "reported": {"status": None, "reason": None}, "picked": False, "placed": False, "outcome": "sim_fail", "agree": "match", "slip_mm": None,
               "failed_reason": f"{type(e).__name__}: {e}"[:300], "wall_s": None, "realtime_lag": False}
    finally:
        row_force, mujoco_hand.GRIP_FORCE_N = mujoco_hand.GRIP_FORCE_N, force0  # --jobs 1이면 같은 프로세스에서 다음 묶음이 돈다
    row["grip_force_n"] = row_force
    return row


def summarize(rows: list[dict]) -> tuple[dict, str]:
    """요약(json)과 표(md): 크기 × 반경 성공률, outcome별 건수, 거짓 성공·거짓 실패 목록, realtime_lag 건수."""
    from collections import Counter

    def size(r):
        t = r["truth"]
        return f"{t['length_mm']:.0f}x{t['width_mm']:.0f}x{t['height_mm']:.0f}"

    base = None
    def radius(r):
        nonlocal base
        if base is None:
            from ..core.config import load_workspace

            base = load_workspace(REPO_ROOT / "config" / "workspace.yaml").base
        return round(math.hypot(r["truth"]["x_mm"] - base.x_mm, r["truth"]["y_mm"] - base.y_mm))

    outcomes = Counter(r["outcome"] for r in rows)
    sizes, radii = sorted({size(r) for r in rows}, key=lambda s: [float(v) for v in s.split("x")]), sorted({radius(r) for r in rows})
    grid = {s: {rr: [r for r in rows if size(r) == s and radius(r) == rr] for rr in radii} for s in sizes}
    false_ok = [r["episode"] for r in rows if r["agree"] == "false_ok"]
    false_fail = [r["episode"] for r in rows if r["agree"] == "false_fail"]
    slips = [r["slip_mm"] for r in rows if r["slip_mm"] is not None]
    summary = {"episodes": len(rows), "placed_ok": outcomes.get("placed_ok", 0), "outcomes": dict(outcomes), "false_ok": false_ok, "false_fail": false_fail,
               "realtime_lag": sum(1 for r in rows if r["realtime_lag"]),
               "slip_mm": {"n": len(slips), "median": round(float(np.median(slips)), 2), "p90": round(float(np.percentile(slips, 90)), 2), "max": round(max(slips), 2)} if slips else None,
               "rate": {s: {str(rr): [sum(1 for r in v if r["outcome"] == "placed_ok"), len(v)] for rr, v in d.items()} for s, d in grid.items()}}
    lines = ["| 크기(길이x폭x높이) | " + " | ".join(f"{rr}mm" for rr in radii) + " |", "|---|" + "---|" * len(radii)]
    for s, d in grid.items():
        lines.append(f"| {s} | " + " | ".join(f"{sum(1 for r in v if r['outcome'] == 'placed_ok')}/{len(v)}" if v else "-" for v in d.values()) + " |")
    lines += ["", "outcome: " + ", ".join(f"{k} {v}" for k, v in sorted(outcomes.items())),
              f"거짓 성공 {len(false_ok)}: {', '.join(false_ok) or '-'}", f"거짓 실패 {len(false_fail)}: {', '.join(false_fail) or '-'}",
              f"realtime_lag {summary['realtime_lag']}건"]
    if slips:
        lines.append(f"미끄러짐(들어 올린 {len(slips)}건) 중앙 {summary['slip_mm']['median']}mm, 90% {summary['slip_mm']['p90']}mm, 최대 {summary['slip_mm']['max']}mm")
    return summary, "\n".join(lines) + "\n"


def _grid(on: bool, n: int, title: str):
    """--viewer grid: 창 하나에 일꾼 n개의 위 카메라를 격자로(mujoco_grid.Grid). 아니면 큐 None."""
    return mujoco_grid.Grid(n, title) if on else contextlib.nullcontext((None, None))


def suite_override(s: dict, gripper: str | None, out: Path) -> Path:
    """덮어쓰기 파일: --gripper > 묶음의 robot(수집 프리셋, config/sim_scenarios/bench/<파일>) > 기본 robot.yaml."""
    if gripper:
        return gripper_override(gripper, out)
    return BENCH_DIR / s["robot"] if s.get("robot") else OVERRIDE


def _record_part(args) -> list[dict]:
    """--record --jobs N의 일꾼 하나: 자기 몫 에피소드를 자기 조각 데이터셋(part-NN)에 쓰고, 결과 줄은 results-part-NN.jsonl에 바로 적는다(일꾼이 죽어도 남게)."""
    jobs_part, part_dir, task = args
    from .mujoco_record import EpisodeRecorder

    rec = EpisodeRecorder(None, None, part_dir, task=task)  # svc·driver는 에피소드마다 run_episode가 넣는다
    rows = []
    out = Path(jobs_part[0][2]) if jobs_part else None
    try:
        for j in jobs_part:
            rows.append(_worker(j, rec))
            r = rows[-1]
            with (out / f"results-{Path(part_dir).name}.jsonl").open("a", encoding="utf-8") as f:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
            print(f"{r['episode']}: {r['outcome']} lag={r['realtime_lag']} {r['wall_s']}s", flush=True)
    finally:
        rec.close()
    return rows


def _write_collect_info(root: Path, s: dict, override: Path, cfg, settle_s: float, commit: str, seed_start: int, seeds: int) -> None:
    """데이터셋 meta/collect.json: 어느 묶음·프리셋·집기 규칙으로 모았는지(결정 0068·0069)."""
    preset = yaml.safe_load(override.read_text(encoding="utf-8")) or {}
    info = {"suite": s["name"], "preset": override.name, "preset_values": preset, "settle_s": settle_s, "commit": commit,
            "seed_start": seed_start, "seeds": seeds, "grasp_approach": getattr(cfg, "grasp_approach", "vertical")}
    (root / "meta" / "collect.json").write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")


def run(suite: str, seeds: int = 1, gripper: str | None = None, jobs: int = 1, only: str | None = None, out_root: Path | None = None,
        grip_force: float | None = None, seed_start: int = 0, record: str | None = None, viewer: bool = False, grid: bool = False, clock: str = "wall") -> Path:
    """묶음을 돌려 결과 폴더를 돌려준다: results.jsonl·summary.json·summary.md·episodes/<id>/(events·status).
    record(데이터셋 이름)를 주면 runs/datasets/<이름>/에 placed_ok이고 realtime_lag 없는 에피소드만 LeRobot 데이터셋으로 남긴다.
    --jobs N이면 일꾼마다 <이름>/part-NN/에 쓰고 끝에 lerobot aggregate_datasets로 하나로 합친다(결정 0068)."""
    s = load_suite(suite)
    settle_s = float(s.get("settle_s", SETTLE_S))
    ds_root = None
    if record:
        from .mujoco_record import dataset_dir

        ds_root = dataset_dir(record, out_root or REPO_ROOT)
        if ds_root.exists():
            raise ValueError(f"데이터셋 폴더가 이미 있다(이어 쓰기는 2차): {ds_root}")
    eps = [e for e in episodes(s, seeds, seed_start=seed_start) if only is None or e.id == only]
    if not eps:
        raise ValueError(f"돌릴 에피소드가 없다(--only {only})")
    commit = _commit()
    tag = f"-f{grip_force:g}" if grip_force is not None else ""
    out = (out_root or REPO_ROOT / "runs" / "bench-mujoco") / f"{datetime.now():%Y%m%d-%H%M%S}-{commit}{tag}"
    out.mkdir(parents=True, exist_ok=True)
    override = suite_override(s, gripper, out)
    jobs_list = [(e, str(override), str(out), grip_force, viewer, settle_s, clock, s.get("start_pose")) for e in eps]
    parted = bool(record) and jobs > 1
    rows = []
    with _grid(grid, jobs, s["name"]) as gq, (out / "results.jsonl").open("w", encoding="utf-8") as f:
        def keep(row):
            row["suite"], row["commit"] = s["name"], commit
            rows.append(row)
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
            f.flush()
            if not parted:  # 조각 일꾼은 자기가 찍는다
                print(f"{row['episode']}: {row['outcome']} ({row['reported']['status']}) {row['wall_s']}s", flush=True)

        if parted:
            from .mujoco_record import merge_parts

            parts = [(jobs_list[i::jobs], ds_root / f"part-{i:02d}", task_text(s)) for i in range(jobs)]
            with ProcessPoolExecutor(jobs, initializer=mujoco_grid.set_queue, initargs=gq) as ex:
                for part_rows in ex.map(_record_part, parts):
                    for row in part_rows:
                        keep(row)
            merge_parts(ds_root)
        elif record:
            from .mujoco_record import EpisodeRecorder

            rec = EpisodeRecorder(None, None, ds_root, task=task_text(s))  # svc·driver는 에피소드마다 run_episode가 넣는다
            try:
                for j in jobs_list:
                    keep(_worker(j, rec))
            finally:
                rec.close()
        elif jobs > 1:
            with ProcessPoolExecutor(jobs, initializer=mujoco_grid.set_queue, initargs=gq) as ex:
                for row in ex.map(_worker, jobs_list):
                    keep(row)
        else:
            for j in jobs_list:
                keep(_worker(j))
    if ds_root is not None:
        _write_collect_info(ds_root, s, override, load_cfg(override), settle_s, commit, seed_start, seeds)
    summary, md = summarize(rows)
    first = next((r for r in rows if r["gripper"]), rows[0])
    walls = [r["wall_s"] for r in rows if r.get("wall_s")]
    summary.update({"suite": s["name"], "commit": commit, "seed_start": seed_start, "record": record, "recorded": sum(1 for r in rows if r.get("recorded_frames")),
                    "recorded_frames": sum(r.get("recorded_frames") or 0 for r in rows), "jobs": jobs, "clock": clock, "preset": override.name, "settle_s": settle_s,
                    "wall_s_median": round(float(np.median(walls)), 2) if walls else None,
                    "gripper": first["gripper"], "scene_parts": first["scene_parts"], "seeds": seeds, "grip_force_n": rows[0]["grip_force_n"]})
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    (out / "summary.md").write_text(f"# {s['name']} (커밋 {commit}, 그리퍼 {summary['gripper']}, 힘 한계 {summary['grip_force_n']:g}N, 부품 {', '.join(summary['scene_parts']) or '-'})\n\n" + md, encoding="utf-8")
    return out


def _policy_part(args) -> list[dict]:
    """정책 평가 일꾼 하나: 체크포인트를 한 번 열고 자기 몫 에피소드를 돈다. 줄마다 results-part-NN.jsonl에 바로 적는다."""
    import os

    from ..service.policy_runner import load_act

    i, eps, suite, checkpoint, out, max_s, viewer, clock = args
    os.chdir(REPO_ROOT)
    s = load_suite(suite)
    if clock == "step":  # 같은 씨앗이면 같은 결과(13.10.2 완료 기준 7): 추론 스레드 수를 고정한다
        import torch

        torch.set_num_threads(1)
    policy = load_act(checkpoint)  # 한 번만 연다(에피소드마다 PolicyRunner.start가 reset)
    _warm_up(policy)
    cfg = load_cfg(suite_override(s, None, Path(out)))
    rows = []
    with (Path(out) / f"results-part-{i:02d}.jsonl").open("w", encoding="utf-8") as f:
        for ep in eps:
            row = run_policy_episode(ep, s, cfg, Path(out) / "episodes" / ep.id, policy, max_s, viewer, clock)
            row.update({"suite": s["name"], "checkpoint": str(checkpoint)})
            rows.append(row)
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
            f.flush()
            print(f"{row['episode']}: {row['outcome']} ({row['policy']['reason']}) {row['wall_s']}s", flush=True)
    return rows


def _warm_up(policy) -> None:
    """첫 추론(CUDA 커널 준비)이 100~270ms라 일꾼마다 첫 에피소드가 too_slow로 끝났다(11-19, --jobs 6 평가마다 5~6개): 빈 관측으로 한 번 미리 돈다."""
    import numpy as np

    obs = {"observation.state": np.zeros(6, np.float32), **{f"observation.images.{c}": np.zeros((240, 320, 3), np.uint8) for c in ("top", "wrist")}}
    policy.reset()
    policy.select_action(obs)
    policy.reset()


def run_policy(suite: str, checkpoint: str, seeds: int = 50, seed_start: int = 10000, out_root: Path | None = None, max_s: float = POLICY_MAX_S,
               viewer: bool = False, jobs: int = 1, grid: bool = False, clock: str = "wall") -> Path:
    """학습한 정책을 씨앗 seed_start부터 seeds개 배치에서 평가한다(실시간, --jobs N이면 일꾼 N개). 체크포인트는 이 placed_ok 비율로 고른다.
    묶음에 robot(수집 프리셋)이 있으면 같은 cfg로 평가한다."""
    from collections import Counter

    s = load_suite(suite)
    out = (out_root or REPO_ROOT / "runs" / "bench-mujoco") / f"{datetime.now():%Y%m%d-%H%M%S}-{_commit()}-policy"
    out.mkdir(parents=True, exist_ok=True)
    eps = episodes(s, seeds, seed_start=seed_start)
    parts = [(i, eps[i::jobs], suite, str(checkpoint), str(out), max_s, viewer, clock) for i in range(jobs)]
    with _grid(grid, jobs, f"{s['name']} 정책") as gq:
        if jobs > 1:
            with ProcessPoolExecutor(jobs, initializer=mujoco_grid.set_queue, initargs=gq) as ex:
                rows = [r for part in ex.map(_policy_part, parts) for r in part]
        else:
            rows = _policy_part(parts[0])
    rows.sort(key=lambda r: r["seed"])
    with (out / "results.jsonl").open("w", encoding="utf-8") as f:
        f.writelines(json.dumps(r, ensure_ascii=False) + "\n" for r in rows)
    ok = sum(1 for r in rows if r["outcome"] == "placed_ok")
    ms = [r["policy"]["infer_ms_mean"] for r in rows if r["policy"]["infer_ms_mean"] is not None]
    summary = {"suite": s["name"], "checkpoint": str(checkpoint), "seed_start": seed_start, "episodes": len(rows), "placed_ok": ok,
               "outcomes": dict(Counter(r["outcome"] for r in rows)), "reasons": dict(Counter(r["policy"]["reason"] for r in rows)),
               "infer_ms_mean": round(sum(ms) / len(ms), 2) if ms else None, "max_s": max_s, "jobs": jobs, "clock": clock,
               "wall_s_sum": round(sum(r["wall_s"] for r in rows), 2)}
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    (out / "summary.md").write_text(f"# {s['name']} 정책 평가 ({checkpoint}, 씨앗 {seed_start}~{seed_start + seeds - 1})\n\n"
                                    f"placed_ok {ok}/{len(rows)}\noutcome: {summary['outcomes']}\n끝난 이유: {summary['reasons']}\n"
                                    f"추론 평균 {summary['infer_ms_mean']}ms\n", encoding="utf-8")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m so101_studio.arm.mujoco_bench", description="M5 집기 시험장(mujoco-spec 13.10.1)")
    ap.add_argument("--suite", required=True)
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--gripper", choices=["parallel", "single_jaw"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only")
    ap.add_argument("--grip-force", type=float, help="병렬 손 힘 한계 N(U15 민감도, 기본 mujoco_hand.GRIP_FORCE_N)")
    ap.add_argument("--seed-start", type=int, default=0, help="첫 씨앗(전문가 수집 0~9999, 평가 10000~, 13.10.4)")
    ap.add_argument("--policy", help="학습한 ACT 체크포인트 폴더(pretrained_model): 수집 대신 정책을 평가한다(PolicyRunner, --jobs N이면 일꾼마다 체크포인트를 연다)")
    ap.add_argument("--clock", choices=["wall", "step"], default="wall", help="wall: 실시간(벽시계). step: 시뮬레이션 시계(StepClock, 13.10.2)로 실시간보다 빠르게, realtime_lag 없음. 수동 뷰어(--viewer)와는 안 되고 --viewer grid는 된다")
    ap.add_argument("--viewer", nargs="?", const="window", choices=["window", "grid"],
                    help="보며 돌린다(실시간). --viewer: MuJoCo 뷰어 창(에피소드마다 새로, --jobs N이면 창 N개). --viewer grid: 창 하나에 일꾼마다 위 카메라 칸")
    ap.add_argument("--record", help="데이터셋 이름: placed_ok 에피소드를 runs/datasets/<이름>/에 LeRobot 데이터셋으로(realtime_lag 에피소드는 빼고, --jobs N이면 part-NN 조각을 쓰고 끝에 합친다)")
    a = ap.parse_args(argv)
    if a.clock == "step" and a.viewer == "window":
        ap.error("--clock step은 수동 뷰어(--viewer)와 쓸 수 없다. --viewer grid를 쓴다")
    try:
        if a.policy:
            out = run_policy(a.suite, a.policy, a.seeds, a.seed_start, viewer=a.viewer == "window", jobs=a.jobs, grid=a.viewer == "grid", clock=a.clock)
            print((out / "summary.md").read_text(encoding="utf-8"))
            print(f"결과: {out}")
            return
        out = run(a.suite, a.seeds, a.gripper, a.jobs, a.only, grip_force=a.grip_force, seed_start=a.seed_start, record=a.record,
                  viewer=a.viewer == "window", grid=a.viewer == "grid", clock=a.clock)
    except (RuntimeError, ValueError) as e:  # lerobot 없음·데이터셋 폴더 있음·이름 규칙 등: 사람이 읽을 한 줄로 끝낸다
        raise SystemExit(str(e)) from e
    print((out / "summary.md").read_text(encoding="utf-8"))
    print(f"결과: {out}")


if __name__ == "__main__":
    main()
