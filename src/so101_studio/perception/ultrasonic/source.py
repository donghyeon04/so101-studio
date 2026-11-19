"""UltrasonicSource: PerceptionSource + 확인 기능(CheckableSource). 레이더로 후보를 찾고, 팔 확인(check.py)으로 확정한다.
이벤트(Studio.bus): us.sweep(한 방향 회전마다), us.candidates, us.check_path(구간·점), us.check_result."""
import dataclasses
import json
import math
from bisect import bisect_right
from pathlib import Path

from ...core.config import load_workspace
from ...core.types import DetectedObject, ScanResult
from ..base import PoseSample
from . import radar
from .check import Probe
from .link import UltrasonicLinkError


def _arm_origin_from_workspace() -> tuple[float, float]:
    b = load_workspace().base  # 다른 설정처럼 config/ 아래를 현재 폴더 기준으로 읽는다(so101 런처가 레포 루트로 옮겨 실행)
    return b.x_mm, b.y_mm


class UltrasonicSource:
    name = "ultrasonic"

    def __init__(self, link, cfg, *, bus=None, arm_origin_mm=None, arm_links_fn=None,
                 baseline_path="config/ultrasonic_baseline.json", ignore_zones=(), record_dir=None):
        self.link, self.cfg, self.bus = link, cfg, bus
        self.clock = link.clock
        self.arm_origin = arm_origin_mm or _arm_origin_from_workspace()
        self._links_fn = arm_links_fn  # t_ns -> [(x, y)] 팔 링크 점 (작업 실행이 넘긴다)
        self.baseline_path = Path(baseline_path)
        self.baseline: radar.Baseline | None = None
        self.ignore_zones = list(ignore_zones)  # (x, y, 반경): 배출 구역처럼 후보에서 뺄 곳
        self._passes: list[radar.Pass] = []
        self._cur: list = []
        self._cur_shift: dict = {}
        self._arm_buf: list = []  # (t_ns, d)
        self._last_pose: PoseSample | None = None
        self._prefetching = False
        self._checking = False
        self._rejected: list[tuple[float, float]] = []  # 이번 사이클에 다시 고르지 않을 자리
        self._fails: list[list] = []  # [x, y, 못 간 횟수]: 못 가서 못 잰 후보는 두 번째에 뺀다(6절)
        self._fail_count = 0  # 이번 작업 실행의 확인 실패(물체 없음·못 감) 횟수
        self._n = 0
        self.record_dir = record_dir  # 있으면 scans/NNN.json·checks/NNN.json을 남긴다(8.4, 정답 쌍은 truth.py)
        self.saved: list[str] = []  # record_dir 기준 상대 경로, 남긴 순서
        self._check_raw: list = []  # 이번 확인의 팔 센서 측정 (t_ns, d)
        self._probe: Probe | None = None

    # --- PerceptionSource ---
    def connect(self) -> None:
        """hello를 0.5초마다 다시 보내 최대 5초 기다린다: 포트를 열면 DTR로 ESP32가 리셋되어(보드마다 다름) 부팅 중 보낸 줄이 사라질 수 있다."""
        for _ in range(10):
            self.link.send(cmd="hello")
            try:
                hello = self.link.wait_for("hello", 0.5)
                break
            except UltrasonicLinkError:
                continue
        else:
            raise UltrasonicLinkError("hello 응답 없음(5초): 포트·케이블·펌웨어를 확인한다")
        want = self.cfg.servo.driver
        if hello.get("servo") != want:  # 서보 구동을 설정(servo.driver)에 맞춘다. 펌웨어는 켤 때마다 pwm이다
            self.link.send(cmd="driver", name=want)
            hello = self.link.wait_for("hello", 1.0)
            if hello.get("servo") != want:
                raise UltrasonicLinkError(f"펌웨어가 서보 구동 {want!r}를 지원하지 않는다(hello servo={hello.get('servo')!r}): 펌웨어를 올린다")
        self.link.sync()

    def calibrate(self) -> dict:
        """빈 작업대 3바퀴(정방향·역방향·정방향) 기준값."""
        self._run_passes(3)
        self.baseline = radar.make_baseline(self._passes)
        self.baseline.save(self.baseline_path)
        self.begin_cycle()
        return {"angles": len(self.baseline.d), "unstable": sorted(self.baseline.unstable)}

    def scan(self) -> ScanResult:
        t0 = self.clock.now_ns()
        if self.baseline is None:
            if not self.baseline_path.exists():
                raise RuntimeError("기준값이 없다: calibrate()를 먼저 부른다")
            self.baseline = radar.Baseline.load(self.baseline_path)
        if self._prefetching:  # T5: 놓는 동안 돌려 둔 바퀴를 합친다
            self._wait(lambda: len(self._passes) >= self.cfg.radar.prefetch_sweeps, self.cfg.radar.prefetch_sweeps * 2 + 2)
            self._prefetching = False
            self.link.expect(False)
        else:
            self._run_passes(1 if self.cfg.radar.early_start else 2)  # T3
        objs, raw = self._candidates()
        if not objs and len(self._passes) <= 2 and self.cfg.radar.empty_extra_sweeps > 0:
            # 첫 1~2바퀴에 아무것도 안 보여도 바로 "없음"으로 끝내지 않는다: empty_extra_sweeps바퀴를 더 돌려 합친다(응답 없음을 빼고
            # 각도별 중앙값이라 약한 반사가 한 번이라도 잡히면 호가 이어진다). 물체가 하나라도 보이면 이 시간은 들지 않는다.
            first = self._passes
            self._run_passes(self.cfg.radar.empty_extra_sweeps)
            self._passes = first + self._passes
            objs, raw = self._candidates()
        if self.bus:
            self.bus.publish("us.candidates", {"objects": [dataclasses.asdict(o) for o in objs], "arcs": raw["arcs"],
                                               "masked": raw["masked"]})
        self._record_scan("scan", objs, raw)
        return ScanResult(objs, (self.clock.now_ns() - t0) / 1e9, raw)

    def rescan(self) -> ScanResult:
        """RESCAN(설계서 2절): 후보가 없을 때 작업 실행이 팔을 레이더 시야 밖 대기 자세로 옮긴 뒤 부른다.
        prefetch_sweeps바퀴를 새로 돌려 합친다. 후보가 있으면 DECIDE, 또 없으면 DONE이고 연달아 두 번은 부르지 않는다."""
        t0 = self.clock.now_ns()
        if self.baseline is None:
            self.baseline = radar.Baseline.load(self.baseline_path)
        self._prefetching = False
        self._run_passes(self.cfg.radar.prefetch_sweeps)
        objs, raw = self._candidates()
        if self.bus:
            self.bus.publish("us.candidates", {"objects": [dataclasses.asdict(o) for o in objs], "arcs": raw["arcs"],
                                               "masked": raw["masked"], "rescan": True})
        self._record_scan("rescan", objs, raw)
        return ScanResult(objs, (self.clock.now_ns() - t0) / 1e9, raw)

    @property
    def give_up(self) -> bool:
        """check.max_fail_per_run이 0이 아니고 이번 작업 실행의 확인 실패가 그만큼 쌓였으면 True: 작업 실행은 끝낸다."""
        n = self.cfg.check.max_fail_per_run
        return n > 0 and self._fail_count >= n

    def _candidates(self):
        vals, raw = radar.combine(self._passes, self._links_fn, self.cfg)
        cands, arcs = radar.find_candidates(vals, self.baseline, self.cfg, raw.get("shift"))
        objs = []
        for c in cands:
            if any(math.hypot(c.x - x, c.y - y) <= self.cfg.radar.merge_mm / 2 for x, y in self._rejected):
                continue
            if any(math.hypot(c.x - x, c.y - y) <= r for x, y, r in self.ignore_zones):
                continue
            self._n += 1
            objs.append(DetectedObject(f"us-{self._n}", c.x, c.y, None, 0.0, 0.0,
                                       min(0.3, 0.1 + 0.05 * c.arc_len), "ultrasonic"))
        raw.update(arcs=arcs, passes=len(self._passes), values=vals)
        return objs, raw

    def notify_picked(self, object_id: str) -> None:
        pass  # 레이더가 매번 새로 보므로 따로 뺄 것이 없다

    def close(self) -> None:
        self.link.send(cmd="stop")
        if hasattr(self.link.tx, "close"):
            self.link.tx.close()

    # --- CheckableSource ---
    def start_check(self, candidate: DetectedObject) -> Probe:
        self.link.send(cmd="stop")
        self._prefetching = False
        hz = round(1000 / self.cfg.ping_gap_ms)  # 30Hz
        self._service()
        self._arm_buf.clear()
        self._check_raw = []
        self._last_pose = None
        self.link.send(cmd="stream", hz=hz, max_mm=300)
        self.link.expect(True)
        self._checking = True
        self._probe = Probe(self, candidate, self.cfg, self.arm_origin, self.bus)  # end_check가 기록할 구간을 읽는다
        return self._probe

    def end_check(self, cand=None, result=None, info=None, status="ok") -> None:
        if self._checking:
            self._checking = False
            self.link.send(cmd="stop")
            self.link.expect(False)
        if cand is not None:
            if status == "empty":
                self._fail_count += 1
                self._rejected.append((cand.x_mm, cand.y_mm))  # 레이더 오검출: 후보 삭제
            elif status == "unreachable":
                self.note_failure(cand.x_mm, cand.y_mm)
            if self.bus:
                self.bus.publish("us.check_result", {"id": cand.id, "object": dataclasses.asdict(result) if result else None,
                                                     "info": info or {}, "status": status})
            if self.record_dir:
                segs = [{**s, "segment": dataclasses.asdict(s["segment"]), "poses": [dataclasses.asdict(p) for p in s["poses"]]}
                        for s in (self._probe.segments if self._probe else [])]
                self._record("checks", {"t_ns": self.clock.now_ns(), "candidate": dataclasses.asdict(cand), "status": status,
                                        "result": dataclasses.asdict(result) if result else None, "info": info or {},
                                        "segments": segs, "arm_raw": self._check_raw})

    def note_failure(self, x_mm: float, y_mm: float) -> None:
        """확인 실패(못 가서 못 잼)나 집기 실패를 작업 실행이 알린다. check.max_fail_per_run에 센다.
        같은 자리가 두 번째면 이번 사이클에서 다시 고르지 않는다(DECIDE→CHECK 무한 반복 방지)."""
        self._fail_count += 1
        near = next((f for f in self._fails if math.hypot(f[0] - x_mm, f[1] - y_mm) <= self.cfg.radar.merge_mm / 2), None)
        if near is None:
            near = [x_mm, y_mm, 0]
            self._fails.append(near)
        near[2] += 1
        if near[2] >= 2:
            self._rejected.append((x_mm, y_mm))

    def begin_cycle(self) -> None:
        """작업 사이클 시작 때 작업 실행(REQ-025)이 부른다: 지운 자리와 실패 횟수를 비운다. calibrate()도 부른다."""
        self._rejected.clear()
        self._fails.clear()
        self._fail_count = 0

    def prefetch(self, arm_links_fn) -> None:
        """T5: 놓는 동안 레이더를 돌린다. 다음 scan()이 그 바퀴들을 합쳐 쓴다."""
        self._links_fn = arm_links_fn
        self._start_sweep(self.cfg.radar.prefetch_sweeps)
        self._prefetching = True

    def arm_points(self, poses: list[PoseSample]) -> list[tuple[float, float, float, int]]:
        """구간의 팔 센서 측정을 PoseSample 시각으로 보간해 (x, y, z, d)로 바꾼다. 앞 구간 끝 자세와 이어 본다."""
        if not poses:
            return []
        if self._last_pose and 0 < poses[0].t_ns - self._last_pose.t_ns <= 60_000_000:
            poses = [self._last_pose, *poses]  # 이어지는 구간: 사이 측정도 보간
        self._last_pose = poses[-1]
        ts = [p.t_ns for p in poses]
        period = 1e9 / max(1, round(1000 / self.cfg.ping_gap_ms))
        self._wait(lambda: bool(self._arm_buf) and self._arm_buf[-1][0] >= ts[-1] - 1.5 * period, 0.5)
        out, keep = [], []
        for t, d in self._arm_buf:
            if t > ts[-1]:
                keep.append((t, d))  # 다음 구간에서 쓴다
            elif t >= ts[0] and d is not None:
                i = min(max(bisect_right(ts, t) - 1, 0), max(len(ts) - 2, 0))
                p, q = poses[i], poses[min(i + 1, len(poses) - 1)]
                f = 0.0 if q.t_ns == p.t_ns else (t - p.t_ns) / (q.t_ns - p.t_ns)
                out.append((*(a + (b - a) * f for a, b in zip(p.sensor_mm, q.sensor_mm)), d))
        self._arm_buf = keep
        return out

    # --- 내부 ---
    def _record_scan(self, kind, objs, raw) -> None:
        if not self.record_dir:
            return
        b = self.baseline
        self._record("scans", {
            "t_ns": self.clock.now_ns(), "kind": kind,
            "passes": [{"dir": p.dir, "meas": p.meas, "shift": p.shift} for p in self._passes],
            "baseline": {"d": b.d, "unstable": sorted(b.unstable)} if b else None,
            "values": raw["values"], "arcs": raw["arcs"], "masked": raw["masked"], "shift": raw["shift"],
            "candidates": [dataclasses.asdict(o) for o in objs]})

    def _record(self, kind: str, data: dict) -> None:
        if not self.record_dir:
            return
        n = sum(f.startswith(kind + "/") for f in self.saved) + 1
        rel = f"{kind}/{n:03d}.json"
        path = Path(self.record_dir) / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        self.saved.append(rel)

    def _service(self) -> None:
        for m in self.link.poll():
            t = m.get("t")
            if t == "r":
                self._cur.append((m["t_ns"], m["a"], m["d"]))
                if "ang" in m:
                    self._cur_shift.setdefault(m["a"], []).append(m["ang"] - m["a"])
            elif t == "end":
                self._passes.append(radar.Pass(m["dir"], self._cur, {a: sum(v) / len(v) for a, v in self._cur_shift.items()} or None))
                if self.bus:
                    self.bus.publish("us.sweep", {"pass": len(self._passes), "dir": m["dir"], "t_ns": m["t_ns"],
                                                  "angles": [a for _, a, _ in self._cur],
                                                  "d": [d for _, _, d in self._cur]})
                self._cur, self._cur_shift = [], {}
            elif t == "a":
                self._arm_buf.append((m["t_ns"], m["d"]))
                if self.record_dir and self._checking:
                    self._check_raw.append((m["t_ns"], m["d"]))
            elif t == "err":
                raise UltrasonicLinkError(f"ESP32 오류: {m.get('msg')}")

    def _wait(self, pred, timeout_s: float) -> None:
        end = self.clock.now_ns() + int(timeout_s * 1e9)
        while True:
            self._service()
            if pred():
                return
            if self.clock.now_ns() > end:
                raise UltrasonicLinkError("ESP32 데이터가 시간 안에 오지 않았다")
            self.clock.sleep(0.005)

    def _start_sweep(self, n: int) -> None:
        self._service()
        self._passes, self._cur, self._cur_shift = [], [], {}
        s = self.cfg.radar.sweep
        self.link.send(cmd="sweep", sweeps=n, max_mm=self.cfg.radar.max_mm, bounce=s.bounce,
                       **{"from": s.from_deg, "to": s.to_deg, "step": s.step_deg})
        self.link.expect(True)

    def _run_passes(self, n: int) -> None:
        self._start_sweep(n)
        self._wait(lambda: len(self._passes) >= n, n * 2 + 2)
        self.link.expect(False)
