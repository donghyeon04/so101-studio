"""작업 실행: 상태 머신 (system-architecture.md 5절).
IDLE -> PREPARE -> SCAN -> DECIDE -> PICK -> PLACE -> (물체 하나마다 다시 PREPARE) ... -> DONE
확인 기능이 있는 인식 방식(CheckableSource, 초음파)이면 DECIDE 뒤에 CHECK, 후보가 없을 때 RESCAN이 더해진다 (ultrasonic-spec.md 2절, REQ-025)."""
import contextlib
import time
from collections.abc import Callable

from ..core.events import EventBus
from ..core.types import TaskEvent
from .policy import choose, explain, reachable
from .trail import PoseTrail

MAX_CYCLES = 20  # 사이클 상한
MAX_CONSECUTIVE_FAILS = 3  # 같은 물체를 연속으로 이만큼 실패하면 끝낸다


class TaskRunner:
    def __init__(self, skills, source, ws, bus: EventBus | None = None, zone: str = "right", accept_no_object: bool = False, arm=None):
        self.skills, self.source, self.ws, self.zone = skills, source, ws, zone
        self.arm = arm  # CHECK가 follow_path를 부를 ArmService. 확인 기능이 없는 인식 방식은 필요 없다
        self.accept_no_object = accept_no_object  # 물체 없이 동작만 보일 때(dry-run): 접촉이 없어도(no_object) 성공으로 보고 이어간다
        self.bus = bus or EventBus()
        self.state = "IDLE"
        self.skipped: dict[str, str] = {}  # 다시 집어도 소용없어 건너뛴 물체 -> 이유 (height_required)
        self._stop = False
        self._step_mode = False
        self._trail: PoseTrail | None = None
        self._auto_zones: list[tuple[float, float, float]] = []  # 전에 소스에 넘긴 배출 구역(소스 고유 영역과 구별한다)
        self._warned_rescan_cfg = False
        self._cand_ready = False  # check_candidate가 연결·확인 준비를 했는지
        self.wait_step: Callable[[], None] = lambda: None  # 단계별 모드에서 다음 단계를 기다리는 함수(CLI는 Enter, GUI는 step())

    def stop(self) -> None:
        self._stop = True
        self.skills.ctl.stop()

    def _emit(self, state: str, object_id: str | None = None, result: str | None = None, **detail) -> None:
        self.state = state
        self.bus.publish("task", TaskEvent(time.time(), state, object_id, result, detail))
        if self._step_mode and state not in ("DONE", "STOPPED", "FAULT"):
            self.wait_step()  # 상태가 바뀔 때마다 멈추고 다음 단계를 기다린다 (D3)

    def _tell_known(self, objs) -> None:
        """놓을 때 벌림이 아직 남은 물체에 부딪히지 않게 스캔한 물체를 팔에 알린다(`Skills.known_objects`, (x, y, 반경 mm=max(폭, 길이)/2), GUI와 같은 규칙).
        집은 물체는 `Skills.pick`이 장애물에서 뺀다. 매 스캔마다 다시 알려 옮겨 놓은 자리가 남지 않는다."""
        if hasattr(self.skills, "known_objects"):
            self.skills.known_objects = [(o.x_mm, o.y_mm, max(o.width_mm or 0.0, o.length_mm or 0.0) / 2.0) for o in objs]
            self.skills.known_height = {(o.x_mm, o.y_mm): o.height_mm for o in objs}  # 집은 뒤 옮길 때 이보다 높이 든다

    def _placed(self) -> dict:
        """방금 놓은 자리(x_mm, y_mm)와 손 방향(place_yaw_deg), 놓인 물체 각도(placed_angle_deg). 모르는 값은 뺀다: 놓인 물체를 다시 집을 때 각도로 쓴다."""
        sk = self.skills
        out = {"x_mm": getattr(sk, "last_place_xy", None) and sk.last_place_xy[0], "y_mm": getattr(sk, "last_place_xy", None) and sk.last_place_xy[1],
               "place_yaw_deg": getattr(sk, "last_place_yaw_deg", None), "placed_angle_deg": getattr(sk, "last_place_angle_deg", None)}
        return {k: round(v, 1) for k, v in out.items() if v is not None}

    def _stopped(self) -> bool:
        if self._stop:
            self._emit("STOPPED")
        return self._stop

    def _ok(self, r) -> bool:
        return r.status == "ok" or (self.accept_no_object and r.status == "no_object")

    def _end(self, end: str) -> dict:
        return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": end, "skipped": dict(self.skipped)}

    def _fault(self, where: str) -> dict:
        self._emit("FAULT", result=where, detail_state=self.skills.ctl.state)
        return {"picked": self.picked, "failed": self.failed, "cycles": self.cycles, "end": "FAULT", "skipped": dict(self.skipped)}

    def _wait_pose(self, chk: bool):
        """대기 자세로 이동: 초음파면 레이더를 가리지 않는 radar_clear, 그 밖의 인식 방식은 park (결정 0039)."""
        return self.skills.radar_clear() if chk else self.skills.park()

    def _rescan_enabled(self) -> bool:
        try:
            return bool(self.source.cfg.radar.rescan_home)
        except AttributeError:  # 설정이 없는 인식 방식: 기본값(켬)으로 가되 한 번은 알린다(설정 오타가 조용히 묻히지 않게)
            if not self._warned_rescan_cfg:
                self._warned_rescan_cfg = True
                emit = getattr(getattr(self.skills, "ctl", None), "_emit", None)  # 이벤트 기록(없는 시험용 스킬은 건너뜀)
                if emit:
                    emit("task_config_warn", "WARN", "TASK-RESCAN-CFG-MISSING", {"detail": "rescan_home 설정이 없어 기본값(켬)으로 진행"})
            return True

    def refresh_ignore_zones(self) -> None:
        """배출 구역을 인식 소스의 ignore_zones로 넘긴다(소스가 따로 가진 영역은 남기고 전에 넘긴 배출 구역만 바꾼다). 실행 시작과 작업대 설정을 다시 읽을 때 부른다."""
        if not hasattr(self.source, "ignore_zones"):
            return
        self.source.ignore_zones = [z for z in self.source.ignore_zones if z not in self._auto_zones] + (auto := self.ws.ignore_zones())
        self._auto_zones = auto

    def _begin_checkable(self) -> None:
        if self.arm is None:
            raise ValueError("확인 기능이 있는 인식 방식은 TaskRunner(arm=...)이 필요하다")
        if getattr(self.source, "bus", 1) is None:
            self.source.bus = self.bus  # us.* 이벤트가 Studio.bus로 나간다 (GUI가 구독)
        self.arm.set_sensor_offset(dict(self.source.cfg.arm_sensor_offset_mm))
        self.refresh_ignore_zones()
        self.source.begin_cycle()
        self._trail = getattr(self.arm, "trail", None) or PoseTrail(self.arm)  # 가상 ESP32가 쓰는 기록이 있으면 그것을 같이 쓴다
        self._trail.attach()

    def _check(self, cand):
        """(확정한 물체 또는 None, 상태). 상태: ok | empty | unreachable | aborted. aborted면 정지·fault라 작업을 끝내야 한다."""
        probe = self.source.start_check(cand)
        try:
            while (seg := probe.next_segment()) is not None:
                poses = self.arm.follow_path(seg.points_mm, seg.tool_z_mm, seg.speed_mm_s, seg.yaw_deg)
                if self.arm.last_path.status not in ("ok", "ik_fail", "out_of_bounds"):  # stopped·fault·locked 등: ESP32에 stop을 보내고 끊는다
                    probe.abort()
                    return None, "aborted"
                probe.feed(poses, self.arm.last_path.status)  # 빈 목록 = 그 구간은 못 감(ik_fail·out_of_bounds)
        except BaseException:
            with contextlib.suppress(Exception):
                probe.abort()
            raise
        res = probe.result()
        return res, "ok" if res else getattr(probe, "status", "empty")

    def check_candidate(self, cand):
        """작업 실행 밖에서 CHECK 한 번(A1, ultrasonic-spec 8.6). (확정한 물체 또는 None, ok|empty|unreachable|aborted).
        처음 부를 때 한 번 run()과 같은 순서로 connect()·_begin_checkable(). reachable() 거르기는 하지 않는다(집지 않고 확인 정확도만 잰다).
        다 쓰면 부르는 쪽이 self._trail.detach()."""
        if not self._cand_ready:
            self.source.connect()
            self._begin_checkable()
            self._cand_ready = True
        return self._check(cand)

    def _skip(self, obj, r) -> None:
        """다시 집어도 소용없는 이유(height_required)로 이 물체를 건너뛴다: 이유를 PICK 이벤트에 남기고 이번 작업에서는 다시 고르지 않는다 (GUI가 보인다)."""
        self.skipped[obj.id] = getattr(r, "reason", None) or r.status
        self._emit("PICK", obj.id, result=r.status, duration_s=r.duration_s, reason=self.skipped[obj.id])

    def _recheck_height(self, obj):
        """높이를 모르거나(height_required) 수직 집기가 no_object였던 후보를 한 번 더 CHECK한다. (새 물체 또는 None, 작업을 끝낼 결과 dict 또는 None)."""
        self._emit("CHECK", obj.id, reason="height_recheck")
        if self._stopped():
            return None, self._end("STOPPED")
        t0 = time.time()
        res, status = self._check(obj)
        if status == "aborted":
            return None, (self._end("STOPPED") if self._stopped() else self._fault("check"))
        self._emit("CHECK", obj.id, result=status, duration_s=time.time() - t0, reason="height_recheck")
        return res, None

    def run(self, step_mode: bool = False) -> dict:
        self._step_mode, self._stop = step_mode, False
        self.picked, self.failed, self.cycles, self.skipped = [], [], 0, {}
        self.source.connect()
        chk = hasattr(self.source, "start_check")
        if chk:
            self._begin_checkable()
        try:
            return self._loop(chk)
        finally:
            if self._trail is not None:
                self._trail.detach()

    def _loop(self, chk: bool) -> dict:
        fails = {}
        prepare = True  # 초음파는 radar_clear로 가는 PREPARE를 첫 사이클·RESCAN·집기 실패 뒤에만 한다. PLACE 뒤에는 바로 SCAN (system-architecture 5절)
        while True:
            if self.cycles >= MAX_CYCLES:
                self._emit("DONE", result="cycle_limit")
                return self._end("cycle_limit")
            if prepare or not chk:
                self._emit("PREPARE")
                if self._stopped():
                    return self._end("STOPPED")
                if self._wait_pose(chk).status != "ok":
                    return self._fault("radar_clear" if chk else "park")
                prepare = False
            self._emit("SCAN")
            scan = self.source.scan()
            self._tell_known(scan.objects)
            dropped, rescanned = set(), False  # dropped: 이번 스캔에서 확인에 실패해 뺀 후보
            while True:
                if chk and self.source.give_up:
                    self._emit("DONE", result="give_up")
                    return self._end("give_up")
                objs = [o for o in scan.objects if o.id not in dropped and o.id not in self.skipped]
                # 확인 전 레이더 후보는 가까운 면 쪽으로 치우쳐 실제보다 멀게 보인다: 최대 반경에 여유를 두고 고르고, 집는 범위는 CHECK 뒤 확정 위치로 다시 본다 (REQ-065)
                obj = choose(objs, self.ws, unconfirmed=chk)
                self._emit("DECIDE", detail_n=len(objs), scan_s=scan.duration_s, **explain(objs, self.ws, obj, unconfirmed=chk))
                if obj is None:
                    if chk and not rescanned and self._rescan_enabled():  # 연달아 두 번은 안 한다
                        rescanned = True
                        self._emit("RESCAN")
                        if self._stopped():
                            return self._end("STOPPED")
                        if self._wait_pose(chk).status != "ok":  # 확인하러 갔던 팔이 레이더 정면을 가리지 않게 PREPARE와 같은 자세로
                            return self._fault("radar_clear")
                        scan, dropped = self.source.rescan(), set()
                        continue
                    self._emit("DONE", result="no_objects")
                    return self._end("no_objects")
                rescanned = False
                if not chk:
                    break
                self._emit("CHECK", obj.id)
                if self._stopped():
                    return self._end("STOPPED")
                t0 = time.time()
                res, status = self._check(obj)
                if status == "aborted":
                    return self._end("STOPPED") if self._stopped() else self._fault("check")
                if res is not None and not reachable(res, self.ws):  # 확정 위치가 집기 범위 밖: 집지 않는다
                    res, status = None, "unreachable"
                self._emit("CHECK", obj.id, result=status, duration_s=time.time() - t0)
                if res is not None:
                    obj = res
                    break
                dropped.add(obj.id)  # 물체가 없거나 못 갔다: 이 후보를 빼고 남은 후보로 다시 DECIDE
            self.cycles += 1
            self._emit("PICK", obj.id)
            if self._stopped():
                return self._end("STOPPED")
            r = self.skills.pick(obj, self.skills.place_targets(zone=self.zone))
            retry = True
            if not self._ok(r):  # 재시도 1회
                if self._stop:
                    self._stopped()
                    return self._end("STOPPED")
                if self.skills.ctl.state == "fault":
                    return self._fault("pick")
                if chk and (r.status == "height_required" or (r.status == "no_object" and getattr(self.skills, "last_pick_tilted", None) is False)):
                    obj2, ended = self._recheck_height(obj)  # 수직으로만 집는 후보는 한 번 더 CHECK해 높이를 다시 잰다 (결정 0052)
                    if ended is not None:
                        return ended
                    if obj2 is None:  # 다시 CHECK에서 물체가 없다/못 갔다: 같은 후보로 다시 집지 않고 실패로 센다
                        retry = False
                    else:
                        obj = obj2
                elif r.status == "height_required":  # 수동 입력은 다시 집어도 같은 결과다(높이는 사람이 준다): 이유와 함께 건너뛴다
                    retry = False
                    self._skip(obj, r)
                    continue
                if retry:
                    r = self.skills.pick(obj, self.skills.place_targets(zone=self.zone))
            if not self._ok(r):
                fails[obj.id] = fails.get(obj.id, 0) + 1
                self.failed.append(obj.id)
                if chk:
                    self.source.note_failure(obj.x_mm, obj.y_mm)
                    prepare = True  # 집기 실패 뒤에는 대기 자세로 돌아가 다시 본다
                self._emit("PICK", obj.id, result=r.status, duration_s=r.duration_s)
                if fails[obj.id] >= MAX_CONSECUTIVE_FAILS:
                    self._emit("DONE", obj.id, result="repeated_failure")
                    return self._end("repeated_failure")
                continue
            if self._stopped():
                return self._end("STOPPED")
            self._emit("PLACE", obj.id)
            if chk:
                self.source.prefetch(self._trail.links_at)  # 놓는 동안 레이더가 다음 후보를 미리 찾는다
            r = self.skills.place(self.zone)
            if r.status != "ok":
                return self._fault("place")
            fails.pop(obj.id, None)
            self.picked.append(obj.id)
            self.source.notify_picked(obj.id)
            self._emit("PLACE", obj.id, result="ok", duration_s=r.duration_s, **self._placed())
