"""M5 집기 시험장 (mujoco-spec 13.10.1, 결정 0062): 공통 층(세계 짓기·판정·기록)과 실행기 A(ArmService·Skills를 MuJoCoDriver 위에서 그대로).

실행: `python -m so101_studio.arm.mujoco_bench --suite <이름> [--seeds N] [--gripper parallel|single_jaw] [--jobs N] [--only <에피소드 id>]`.
한 시도 = 한 에피소드이고 에피소드마다 드라이버와 ArmService를 새로 만든다. 지금은 실시간(벽시계) 모드다(시계 주입 13.10.2 전)."""
import argparse
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
    """sim_state()를 50Hz로 읽어 모은다(벽시계 스레드). max_load는 관절 부하 % 최대."""

    def __init__(self, drv):
        self.drv, self.samples, self.max_load, self._stop = drv, [], 0.0, threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            try:
                self.samples.append(self.drv.sim_state())
                self.max_load = max(self.max_load, *self.drv.read_health()["load_pct"].values())
            except RuntimeError:  # 물리가 멈췄다(failed_reason): 표본만 그친다
                return
            time.sleep(SAMPLE_S)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *a):
        self._stop.set()
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


def layout(suite: dict, seeds: int = 1, base=None) -> list[Episode]:
    """묶음 → 에피소드 목록(씨앗 고정). 격자 순번은 objects 키(YAML 순서, size_mm이 바깥) × given 키(YAML 순서)의 곱집합.
    objects는 radius_mm·bearing_deg(팔 원점에서 반경, 팔 정면 기준 방위) 대신 positions: [[반경, 방위], …]를 받을 수 있다."""
    if base is None:
        from ..core.config import load_workspace

        base = load_workspace(REPO_ROOT / "config" / "workspace.yaml").base
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
    place = suite.get("place") or {"zone": "right"}
    eps = []
    for i, combo in enumerate(itertools.product(*[objs[k] for k in keys], *[given[k] for k in gkeys])):
        c = dict(zip(keys + gkeys, combo))
        (length, width, height), (r, bearing) = c["size_mm"], c["positions"]
        for s in range(n_seeds):
            rng = np.random.default_rng([i, s])
            dx, dy, da = (rng.uniform(-sxy, sxy), rng.uniform(-sxy, sxy), rng.uniform(-sang, sang)) if (sxy or sang) else (0.0, 0.0, 0.0)
            x, y = robot_to_table(r * math.cos(math.radians(bearing)), r * math.sin(math.radians(bearing)), base)
            truth = {"name": OBJ, "x_mm": round(x + dx, 3), "y_mm": round(y + dy, 3), "angle_deg": round(float(c["angle_deg"]) + da, 3),
                     "length_mm": float(length), "width_mm": float(width), "height_mm": float(height)}
            eps.append(Episode(f"{suite['name']}-{i:04d}-s{s}", s, truth, {k: c[k] for k in gkeys}, place))
    return eps


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

    def __init__(self, cfg, ws=None, realtime: bool = True):
        from ..core.config import load_workspace
        from ..core.scene import load_scene

        self.cfg, self.realtime = cfg, realtime
        self.ws = ws or load_workspace(REPO_ROOT / "config" / "workspace.yaml")
        self.scene = load_scene(REPO_ROOT / "config" / "scene.yaml").for_gripper(cfg.gripper)
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
        # shortcut: 실시간만. 시계 주입(13.10.2) 뒤에는 여기 한 곳에서 realtime=False + StepClock으로 바꾼다
        self.drv = MuJoCoDriver(self.cfg, self.ws, scene=self.scene, objects=objs, realtime=self.realtime, scenario=ep.id.rsplit("-", 2)[0])
        self.svc = ArmService(self.drv, self.cfg, self.ws, log_root=str(options["log_dir"]))
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
        self.drv = self.svc = None


# -- 판정 --
def judge(result, samples: list, end: dict, truth: dict, target_xy, events: list, failed: str | None) -> dict:
    """13.10.1 판정 표. result는 MotionResult(물리 실패면 None), end는 끝나고 1.0초 뒤 sim_state."""
    reported = {"status": result.status if result else None, "reason": result.reason if result else None}
    o_end = end["objects"][OBJ] if end else None
    picked = any(st["held"] == OBJ and st["objects"][OBJ]["z_mm"] > PICKED_Z_MM for st in samples)
    place_err = float(math.hypot(o_end["x_mm"] - target_xy[0], o_end["y_mm"] - target_xy[1])) if (o_end and target_xy) else None
    tilt_end = float(o_end["tilt_deg"]) if o_end else None
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


def run_episode(ep: Episode, cfg, log_dir: Path, env: BenchEnv | None = None, samples_out: list | None = None) -> dict:
    """실행기 A: reset → pick_and_place(그동안 표본) → 1.0초 뒤 판정 → disconnect. 결과 한 줄(dict). samples_out에 표본을 붙인다(시험용)."""
    env = env or BenchEnv(cfg)
    t0 = time.perf_counter()
    obs, info = env.reset(options={"episode": ep, "log_dir": log_dir})
    drv, svc = env.drv, env.svc
    given = DetectedObject(**obs["given"])
    zone, at = ep.place.get("zone"), ep.place.get("at")
    result, failed = None, None
    try:
        with Sampler(drv) as s:
            try:
                result = svc.pick_and_place(given, zone=zone, place_at=tuple(at) if at else None)
            except RuntimeError as e:  # 물리 실패(MuJoCoDriver._check)
                failed = str(e)
            time.sleep(SETTLE_S)
        failed = failed or drv.failed_reason
        if samples_out is not None:
            samples_out.extend(s.samples)
        end = None if failed else drv.sim_state()
        target = svc.skills.last_place_xy
        row = judge(result, s.samples, end, ep.truth, list(target) if target else None, read_events(svc), failed)
        drops = drv.dropped_count - info["drops_at_connect"]
        row.update({"sim_s": round(end["t_s"], 2) if end else None, "wall_s": round(time.perf_counter() - t0, 2), "dropped_count": drops,
                    "realtime_lag": drv.realtime and drops > 0, "gap0_mm": info["gap0_mm"], "jaw_yaw_deg": info["jaw_yaw_deg"], "max_load_pct": round(s.max_load, 1)})
        return {"episode": ep.id, "seed": ep.seed, "gripper": cfg.gripper, "scene_parts": [p.name for p in env.scene.parts],
                "truth": ep.truth, "given": obs["given"], "inject": ep.inject, "place": ep.place, **row}
    finally:
        env.close()


# -- 실행과 기록 --
def _commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short=7", "HEAD"], check=False, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10).stdout.strip() or "nogit"
    except Exception:  # noqa: BLE001
        return "nogit"


def _worker(job):
    ep, override, out, grip_force = job
    import os

    from . import mujoco_hand

    os.chdir(REPO_ROOT)  # ArmService는 config/ 를 현재 폴더 기준으로 읽는다
    if grip_force is not None:  # U15 민감도: 병렬 손 액추에이터 힘 한계(forcerange)만 바꾼다. kp·Torque_Limit 배율은 그대로
        mujoco_hand.GRIP_FORCE_N = float(grip_force)
    row = run_episode(ep, load_cfg(override), Path(out) / "episodes" / ep.id)
    row["grip_force_n"] = mujoco_hand.GRIP_FORCE_N
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


def run(suite: str, seeds: int = 1, gripper: str | None = None, jobs: int = 1, only: str | None = None, out_root: Path | None = None,
        grip_force: float | None = None) -> Path:
    """묶음을 돌려 결과 폴더를 돌려준다: results.jsonl·summary.json·summary.md·episodes/<id>/(events·status)."""
    s = load_suite(suite)
    eps = [e for e in layout(s, seeds) if only is None or e.id == only]
    if not eps:
        raise ValueError(f"돌릴 에피소드가 없다(--only {only})")
    commit = _commit()
    tag = f"-f{grip_force:g}" if grip_force is not None else ""
    out = (out_root or REPO_ROOT / "runs" / "bench-mujoco") / f"{datetime.now():%Y%m%d-%H%M%S}-{commit}{tag}"
    out.mkdir(parents=True, exist_ok=True)
    override = gripper_override(gripper, out) if gripper else OVERRIDE
    jobs_list = [(e, str(override), str(out), grip_force) for e in eps]
    rows = []
    with (out / "results.jsonl").open("w", encoding="utf-8") as f:
        def keep(row):
            row["suite"], row["commit"] = s["name"], commit
            rows.append(row)
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
            f.flush()
            print(f"{row['episode']}: {row['outcome']} ({row['reported']['status']}) {row['wall_s']}s", flush=True)

        if jobs > 1:
            with ProcessPoolExecutor(jobs) as ex:
                for row in ex.map(_worker, jobs_list):
                    keep(row)
        else:
            for j in jobs_list:
                keep(_worker(j))
    summary, md = summarize(rows)
    summary.update({"suite": s["name"], "commit": commit, "gripper": rows[0]["gripper"], "scene_parts": rows[0]["scene_parts"], "seeds": seeds, "grip_force_n": rows[0]["grip_force_n"]})
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    (out / "summary.md").write_text(f"# {s['name']} (커밋 {commit}, 그리퍼 {summary['gripper']}, 힘 한계 {summary['grip_force_n']:g}N, 부품 {', '.join(summary['scene_parts']) or '-'})\n\n" + md, encoding="utf-8")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m so101_studio.arm.mujoco_bench", description="M5 집기 시험장(mujoco-spec 13.10.1)")
    ap.add_argument("--suite", required=True)
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--gripper", choices=["parallel", "single_jaw"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only")
    ap.add_argument("--grip-force", type=float, help="병렬 손 힘 한계 N(U15 민감도, 기본 mujoco_hand.GRIP_FORCE_N)")
    a = ap.parse_args(argv)
    out = run(a.suite, a.seeds, a.gripper, a.jobs, a.only, grip_force=a.grip_force)
    print((out / "summary.md").read_text(encoding="utf-8"))
    print(f"결과: {out}")


if __name__ == "__main__":
    main()
