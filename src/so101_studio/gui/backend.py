"""ArmService·Studio를 감싸는 유일한 창구 (gui-spec 7·8절). 실행 방식, 조작권, REQ 임시 우회를 여기에 둔다."""

import contextlib
import csv
import json
import logging
import re
import threading
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

from ..arm.kinematics import Kinematics
from ..arm.sim_driver import SimDriver
from ..core.config import arm_files, load_robot_ex, load_workspace
from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from ..doctor import doctor
from ..perception.manual import ManualSource
from ..service.arm_service import ArmService
from ..service.diagnostics import BIG_MOVES
from ..service.stop_listener import StopListener
from ..service.studio import Studio
from ..service.targets import reason_text, tip_vector
from ..task.policy import reachable
from .scene3d import Scene3D
from .state import EMPTY, SAFETY_TEXT, LogBook, LogRecord, State, hottest_text
from .watchdog_link import WatchdogLink

JOG_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")  # JointState 순서
JOINT_TEXT = {
    "shoulder_pan": "어깨 회전", "shoulder_lift": "어깨 들기", "elbow_flex": "팔꿈치", "wrist_flex": "손목 굽힘", "wrist_roll": "손목 회전",
    "gripper": "그리퍼",
}  # JointState 순서
CART_AXES = ("x", "y", "z", "yaw")  # 작업대 좌표계
DIAGNOSTICS = {"servoinfo": "서보 설정 읽기", "linkwatch": "연결 끊김 찾기", "powertest": "전원 시험", "shaketest": "떨림 시험"}
RUN_POINTS = 600  # 실행 기록 그래프로 보내는 점의 상한
SEQUENCE_POINTS = 300  # 화면에 보내는 기록 표본 수의 상한
JOG_TOOL_MM_S = 100.0  # 속도 100%일 때. 컨트롤러가 60mm/s로 한 번 더 자른다
JOG_YAW_DEG_S = 60.0
RUN_MODES = {"sim": "시뮬레이션만", "real": "실제만", "both": "동시", "sim_first": "시뮬레이션 먼저"}
REASON_EXTRA = {"z_ceiling": "허용 상자보다 높습니다"}  # service.targets에 아직 없는 이유
BUSY_TEXT = "팔이 아직 움직이고 있습니다"
MOTION_TEXT = {
    "ik_fail": reason_text("ik_fail"),
    "out_of_bounds": "허용 범위 밖이라 움직이지 않았습니다",
    "stopped": "정지 상태입니다. 정지 해제를 눌러 주세요",
    "preempted": "다른 명령으로 바뀌었습니다",
    "fault": "오류로 멈췄습니다",
    "timeout": "시간 안에 끝나지 않았습니다",
    "locked": "실제 팔이 잠겨 있어 움직이지 않았습니다. 장치 > 팔에서 이유를 확인해 주세요",
}


def _num(value: float) -> str:
    return f"{value:g}"


def update_workspace_text(text: str, values: dict, zone: str) -> str:
    """workspace.yaml의 값만 바꾼다. yaml로 다시 쓰면 파일의 주석이 사라진다."""
    rules = (
        (r"^table_mm: \{.*?\}", f"table_mm: {{x: {_num(values['table_x'])}, y: {_num(values['table_y'])}}}"),
        (r"^robot_base_in_table: \{.*?\}",
         f"robot_base_in_table: {{x_mm: {_num(values['base_x'])}, y_mm: {_num(values['base_y'])}, yaw_deg: {_num(values['base_yaw'])}}}"),
        (rf"^  {re.escape(zone)}: \{{.*?\}}", f"  {zone}: {{x_mm: {_num(values['zone_x'])}, y_mm: {_num(values['zone_y'])}}}"),
        (r"^safety_margin_mm: \S+", f"safety_margin_mm: {_num(values['margin'])}"),
    )
    for pattern, replacement in rules:
        text, count = re.subn(pattern, replacement, text, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"workspace 파일에서 찾지 못한 항목: {pattern}")
    return text


@dataclass
class Plan:
    ok: bool
    text: str


class Backend:
    workspace_path = "config/workspace.yaml"
    arms_path = "config/arms"
    local_path = "config/local.yaml"

    def __init__(self, speed: float | None = None, log_root: str | None = "runs", link: WatchdogLink | None = None,
                 viser_port: int | None = None, real_driver=None, arm_name: str | None = None, serials=(), port: str | None = None):
        cfg, self.arm_info = load_robot_ex(arm=arm_name, serials=serials)
        self._arm_option = arm_name
        self.state = State()
        self.link = link
        self.scene: Scene3D | None = None
        self.reach = EMPTY
        self.runs_root = log_root or "runs"
        self.logbook = LogBook()
        logging.getLogger().addHandler(self.logbook)
        self._event_log: deque = deque(maxlen=2000)
        self._log_offset = 0
        self._next_object = 0
        self._path_kin: Kinematics | None = None
        self._listener = None
        self.owner: str | None = None  # 조작권을 가진 브라우저. NiceGUI 이벤트 루프에서만 바꾼다
        ws = load_workspace()
        # 실행 기록은 주 팔 하나만 남긴다. 같은 초에 만들면 폴더 이름이 겹친다
        self.real = ArmService(real_driver, cfg, ws, speed, log_root) if real_driver else None
        self.sim = ArmService(SimDriver(cfg.poses.get("home")), cfg, ws, speed, None if real_driver else log_root)
        self.arm = self.real or self.sim  # 상태줄과 3D의 실제 팔이 읽는 쪽
        self.mode = "sim"
        self.pending: tuple | None = None  # 시뮬레이션 먼저: 실제 팔로 실행할지 묻는 동작 (이름, action)
        self._planning = False
        self._record_name: str | None = None
        for arm in self._arms():
            ok, msg = arm.connect(port if arm is self.real else None)
            if not ok:
                raise RuntimeError(msg)
        self.arm.status_stream(self.state.update)
        self.source = ManualSource()
        self.studio = Studio(self.arm, self.source, dry_run=self.real is None)  # 시뮬레이션에는 잡히는 물체가 없어 접촉 없음도 성공으로 본다
        self.studio.bus.subscribe("task", self._on_task)
        self.paused = False
        if viser_port is not None:
            self.scene = Scene3D(cfg, self.arm.ws, self.arm.kin, viser_port)
            self.scene.set_objects(self.objects())
            self.arm.status_stream(self.scene.update_status)
            if self.real:
                self.sim.status_stream(self._on_twin_status)
        if link:  # 감시 프로세스의 정지 명령을 받는다
            self._listener = StopListener(self.arm, link.stop_port).start()

    def claim(self, client: str) -> bool:
        """비어 있으면 조작권을 가져간다. 이 브라우저가 조작권을 가졌는지 돌려준다."""
        if self.owner is None:
            self.owner = client
            if self.link:
                self.link.want_armed = True
        return self.owner == client

    def real_allowed(self) -> bool:
        """감시 프로세스가 응답할 때만 실제 팔을 움직인다 (decisions/0026). 시뮬레이션은 상관없다."""
        return bool(self.link and self.link.alive)

    def resume(self) -> None:
        """비상 정지를 풀고 감시 프로세스에 다시 등록한다. 팔은 움직이지 않는다."""
        for arm in self._arms():
            arm.resume()
        if self.link:
            self.link.rearm()

    # ---- 실행 방식 (gui-spec 8절). 실제 장치용과 SimDriver용 ArmService를 고른다
    def set_run_mode(self, mode: str) -> str | None:
        """실행 방식을 바꾼다. 못 바꾸면 이유를 돌려주고 그대로 둔다."""
        if mode not in RUN_MODES:
            return f"알 수 없는 실행 방식: {mode}"
        if mode != "sim":
            reason = self._real_blocked()
            if reason:
                return reason
        self.mode, self.pending = mode, None
        return None

    def move_lock(self) -> str | None:
        """실제 팔이 잠겼으면 이유 (decisions/0027). 새 팔이거나 캘리브레이션이 다를 때다."""
        return self.real.locked_reason if self.real else None

    def _real_blocked(self) -> str | None:
        if self.real is None:
            return "실제 팔이 연결되어 있지 않습니다. 시뮬레이션만 쓸 수 있습니다"
        if self.move_lock():
            return self.move_lock()
        if not self.real_allowed():
            return "감시 프로세스가 응답하지 않아 실제 팔을 움직일 수 없습니다"
        return None

    def _targets(self) -> list[ArmService]:
        """지금 방식에서 바로 움직일 팔. 시뮬레이션 먼저는 시뮬레이션만 움직이고 확인을 기다린다."""
        if self.mode == "real":
            return [self.real]
        if self.mode == "both":
            return [self.real, self.sim]
        return [self.sim]

    def _model_arm(self) -> ArmService:
        """미리보기를 계산할 팔. 다음에 움직일 팔을 기준으로 한다."""
        return self.real if self.mode == "real" else self.sim

    @staticmethod
    def _run_on(arm: ArmService, action) -> str | None:
        try:
            result = action(arm)
        except RuntimeError:  # 컨트롤러가 이미 다른 동작을 실행 중이다
            return BUSY_TEXT
        except ValueError as e:  # 배율 때문에 속도 한계를 넘는 경우 등
            return str(e)
        return None if result.status == "ok" else MOTION_TEXT.get(result.status, result.status)

    def _dispatch(self, action, label: str) -> str | None:
        """action(ArmService) -> MotionResult를 실행 방식대로 부른다. 실패 이유 또는 None. 스레드에서 부른다."""
        if self.mode != "sim":
            reason = self._real_blocked()
            if reason:
                return reason
        targets = self._targets()
        if len(targets) == 1:
            reason = self._run_on(targets[0], action)
        else:
            reasons: list = [None, None]

            def run(i: int, arm: ArmService) -> None:
                reasons[i] = self._run_on(arm, action)

            threads = [threading.Thread(target=run, args=(i, arm), daemon=True) for i, arm in enumerate(targets)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            reason = next((r for r in reasons if r), None)
        if reason is None and self.mode == "sim_first":
            self.pending = (label, action)
        return reason

    def confirm_pending(self) -> str | None:
        """시뮬레이션에서 본 동작을 실제 팔로 실행한다."""
        pending, self.pending = self.pending, None
        if pending is None:
            return None
        reason = self._real_blocked()
        return reason or self._run_on(self.real, pending[1])

    def cancel_pending(self) -> None:
        self.pending = None

    def stop_all(self) -> None:
        for arm in self._arms():
            arm.stop()

    def _arms(self) -> list[ArmService]:
        return [self.sim] if self.real is None else [self.real, self.sim]

    # ---- 직접 조작. 실패하면 이유 문장을, 성공하면 None을 돌려준다. 스레드에서 부른다
    def home(self) -> str | None:
        return self._dispatch(lambda arm: arm.home(), "홈으로 이동")

    def park(self) -> str | None:
        return self._dispatch(lambda arm: arm.park(), "파크로 이동")

    def gripper(self, value: float) -> str | None:
        return self._dispatch(lambda arm: arm.set_gripper(value), f"그리퍼 {value:.0f}")

    def gripper_width(self, mm: float) -> str | None:
        return self._dispatch(lambda arm: arm.set_gripper(mm, as_width=True), f"그리퍼 폭 {mm:.0f} mm")

    def set_speed(self, pct: float) -> None:
        for arm in self._arms():
            arm.set_speed(pct)

    def plan(self, x: float, y: float, h: float, yaw: float) -> Plan:
        """움직이지 않고 (x, y, 높이)에 닿는지 본다. 3D에 점, 계산 자세, 손끝 경로를 그린다."""
        arm = self._model_arm()
        pose = arm.point_pose(x, y, h, yaw)
        reason, q = arm.preview_tool(pose)
        path = None
        duration = 0.0
        if reason is None:
            preview = arm.preview_path(pose)
            reason, path, duration = preview["reason"], preview["tool"], preview["duration_s"]
        ok = reason is None
        text = f"닿습니다 · 약 {duration:.1f}초" if ok else REASON_EXTRA.get(reason) or reason_text(reason)
        self.reach = "닿음" if ok else "안 닿음"
        self._planning = True
        if self.scene:
            self.scene.set_preview(q if ok else None)
            self.scene.show_target(x, y, h, ok, path)
        return Plan(ok, text)

    def clear_plan(self) -> None:
        self._planning = False
        if self.scene:
            self.scene.clear_target()

    def _on_twin_status(self, status) -> None:
        """실제 팔이 있으면 계산 자세 팔은 시뮬레이션 쪽 자세를 보여 준다."""
        if self.scene and not self._planning:
            self.scene.set_preview(status.joints)

    def move_to(self, x: float, y: float, h: float, yaw: float) -> str | None:
        results = []

        def run(arm: ArmService):
            result = arm.move_tool(arm.point_pose(x, y, h, yaw))
            results.append(result)
            return result

        reason = self._dispatch(run, f"x {x:.0f}, y {y:.0f}, 높이 {h:.0f}으로 이동")
        self.clear_plan()
        if results and results[0].position_error_mm is not None:
            self.reach = f"오차 {results[0].position_error_mm:.1f} mm"
        return reason

    def jog(self, mode: str, index: int, sign: int) -> bool:
        """누르는 동안 조그. 입력이 0.3초 끊기면 컨트롤러가 스스로 멈춘다. 시뮬레이션 먼저 방식은 시뮬레이션 팔만 움직인다."""
        if self.mode != "sim" and self._real_blocked():
            return False
        speed = self.arm.speed / 100
        if mode == "joint":
            velocity = [0.0] * 6
            velocity[index] = sign * self.arm.cfg.joint_v_max_deg_s * speed
        else:
            velocity = [0.0] * 4
            velocity[index] = sign * (JOG_YAW_DEG_S if CART_AXES[index] == "yaw" else JOG_TOOL_MM_S) * speed
        moved = False
        for arm in self._targets():
            moved = arm.jog(mode, velocity) or moved
        return moved

    def jog_release(self, mode: str) -> None:
        """손을 떼면 속도를 0으로 내려 부드럽게 멈춘다."""
        for arm in self._targets():
            arm.jog(mode, [0.0] * (6 if mode == "joint" else 4))

    def jog_step(self, mode: str, index: int, amount: float) -> str | None:
        """정해진 간격만큼 움직인다. 관절은 도, 직교는 mm(yaw는 도)."""

        def run(arm: ArmService):
            q = arm.status().joints
            if mode == "joint":
                name = JOG_JOINTS[index]
                return arm.move_joints(replace(q, **{name: getattr(q, name) + amount}))
            delta = [0.0] * 4
            delta[index] = amount
            goal = arm.kin.step(q, *delta)
            return arm.move_joints(goal) if goal else MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))

        return self._dispatch(run, "조그 간격 이동")

    # ---- 작업
    def objects(self) -> list[DetectedObject]:
        return self.source.scan().objects

    def object_reachable(self, obj: DetectedObject) -> bool:
        return reachable(obj, self.arm.ws)

    def _on_task(self, event) -> None:
        self.state.on_task(event)
        if self.scene and event.state in ("DECIDE", "PLACE", "DONE", "STOPPED"):
            self.scene.set_objects(self.objects(), self.state.chosen)

    def task_running(self) -> bool:
        thread = self.studio._thread
        return thread is not None and thread.is_alive()

    def scan(self) -> list[DetectedObject]:
        objects = self.studio.perception.scan().objects
        if self.scene:
            self.scene.set_objects(objects, self.state.chosen)
        return objects

    def start_task(self, step_mode: bool = False) -> str | None:
        """작업을 시작한다. 시작하지 못하면 이유를 돌려준다."""
        if self.task_running():
            return "이미 실행 중입니다"
        if self.real and self.mode != "real":  # 작업은 주 팔(실제)에서 돈다. 다른 방식은 설계서가 나중으로 미뤘다
            return "자동 작업은 실행 방식이 '실제만'일 때 씁니다"
        if self.real and self._real_blocked():
            return self._real_blocked()
        if self.arm.ctl.state != "idle":  # 그대로 시작하면 동작 충돌로 곧 FAULT로 끝난다
            return MOTION_TEXT["stopped"] if self.arm.ctl.state in ("stopped", "fault") else BUSY_TEXT
        self.state.placed, self.paused = 0, step_mode
        self.studio.start(step_mode=step_mode)
        return None

    def step_task(self) -> str | None:
        """멈춘 작업을 한 단계 진행한다. 작업이 없으면 단계별 모드로 시작한다."""
        if self.task_running():
            self.studio.step()
            return None
        return self.start_task(step_mode=True)

    def toggle_pause(self) -> None:
        """다음 상태 변화에서 멈추거나 다시 이어 간다."""
        if not self.task_running():
            return
        self.paused = not self.paused
        if self.paused:
            self.studio.pause()
        else:
            self.studio.resume()

    def stop_task(self) -> None:
        self.paused = False
        self.studio.stop()
        self.stop_all()

    def _changed_objects(self) -> None:
        if self.scene:
            self.scene.set_objects(self.objects(), self.state.chosen)

    def add_object(self, x: float, y: float, angle: float = 0.0, width: float = 30.0, length: float = 100.0) -> str:
        """클릭한 자리에 물체를 놓고 id를 돌려준다."""
        self._next_object += 1
        object_id = f"added-{self._next_object}"
        self.source.add(DetectedObject(object_id, x, y, angle, width, length, 1.0, "manual"))
        self._changed_objects()
        return object_id

    def move_object(self, object_id: str, x: float, y: float) -> None:
        old = next(o for o in self.objects() if o.id == object_id)
        self.source.remove(object_id)
        self.source.add(replace(old, x_mm=x, y_mm=y))
        self._changed_objects()

    def remove_object(self, object_id: str) -> None:
        self.source.remove(object_id)
        self._changed_objects()

    def reset_objects(self) -> None:
        """파일에 적힌 처음 물체로 되돌린다(옮긴 표시도 지운다). 지금 있는 물체는 지운다."""
        for obj in self.objects():
            self.source.remove(obj.id)
        for obj in ManualSource().scan().objects:
            with contextlib.suppress(KeyError):
                self.source.remove(obj.id)  # 이미 옮겨 scan에 안 보이는 물체
            self.source.add(obj)
        self.state.placed = 0
        self._changed_objects()

    # ---- 설정
    def get_safety(self) -> dict:
        return self.arm.get_safety()

    def set_safety(self, mode: str, margin: float, box: dict | None) -> str | None:
        """안전 설정을 바꾼다. 잘못된 값이면 이유를 돌려주고 아무것도 바꾸지 않는다."""
        try:
            for arm in self._arms():
                arm.set_safety(mode, margin, box)
        except ValueError as e:
            return str(e)
        self.show_safety_box()
        return None

    def show_safety_box(self) -> None:
        if not self.scene:
            return
        now = self.arm.get_safety()
        box = now["box_mm_effective"]
        if now["mode"] == "off":
            box = None
        elif now["mode"] == "free" and box:  # 자유 모드는 x, y를 검사하지 않는다
            table = self.arm.ws.table_mm
            box = {**box, "x": [0, table["x"]], "y": [0, table["y"]]}
        self.scene.show_allowed(box)

    def set_params(self, changes: dict) -> tuple[str | None, dict]:
        """(실패 이유, {이름: 적용 시점}). 하나라도 틀리면 아무것도 바뀌지 않는다."""
        try:
            timings = self.arm.set_params(changes)
            if self.real:  # 설정 객체는 같이 쓰고 속도만 따로 있다
                self.sim.set_speed(self.arm.speed)
            return None, timings
        except ValueError as e:
            return str(e), {}

    def save_preset(self, name: str) -> str | None:
        try:
            self.arm.save_preset(name)
        except ValueError as e:
            return str(e)
        return None

    def load_preset(self, name: str) -> str | None:
        try:
            self.arm.load_preset(name)
            if self.real:
                self.sim.set_speed(self.arm.speed)
        except (ValueError, FileNotFoundError) as e:
            return str(e)
        return None

    def save_workspace(self, values: dict, zone: str) -> str | None:
        """작업대·좌표를 파일에 저장한다. 주석은 그대로 둔다. 다시 시작해야 적용된다."""
        if min(values["table_x"], values["table_y"]) <= 0:
            return "작업대 크기는 0보다 커야 합니다"
        if not (0 <= values["base_x"] <= values["table_x"] and 0 <= values["base_y"] <= values["table_y"]):
            return "팔 받침이 작업대 밖입니다"
        if not (0 <= values["zone_x"] <= values["table_x"] and 0 <= values["zone_y"] <= values["table_y"]):
            return "배출 구역이 작업대 밖입니다"
        if not 0 <= values["margin"] < min(values["table_x"], values["table_y"]) / 2:
            return "가장자리 여유가 작업대보다 큽니다"
        path = Path(self.workspace_path)
        path.write_text(update_workspace_text(path.read_text(encoding="utf-8"), values, zone), encoding="utf-8")
        return None

    # ---- 장치·시스템
    def run_doctor(self) -> dict:
        """읽기 전용 점검. 블로킹이므로 스레드에서 부른다."""
        # shortcut: 포트를 열지 않는 점검만 한다(COM3은 알린 뒤에만 연다, decisions/0024), G10에서 실제 팔 연결 전 탐침을 붙인다
        try:
            return doctor(probe=False, arm=self._arm_option)
        except Exception as e:  # noqa: BLE001  점검이 실패해도 페이지는 살아 있어야 한다
            return {"ok": False, "checks": [{"name": "점검", "state": "fail", "text": f"점검하지 못했습니다: {e}", "fix": ""}], "ports": [], "arm": None, "esp32": None}

    def arm_names(self) -> list[str]:
        return list(arm_files(Path(self.arms_path)))

    def choose_arm(self, name: str) -> str | None:
        """다음 실행부터 쓸 팔을 local.yaml에 적는다. 실패 이유 또는 None. 설정은 시작할 때 읽어 지금 팔은 그대로다."""
        if name not in self.arm_names():
            return f"팔 파일이 없습니다: {name}"
        path = Path(self.local_path)
        text = path.read_text(encoding="utf-8") if path.exists() else ""
        line = f"arm: {name}"
        text, count = re.subn(r"^arm:.*$", line, text, flags=re.MULTILINE)  # 주석을 지키려고 그 줄만 바꾼다
        path.write_text(text if count else f"{text.rstrip()}\n{line}\n".lstrip(), encoding="utf-8")
        return None

    def diagnostic_available(self, name: str) -> bool:
        return name != "linkwatch"  # shortcut: 포트를 직접 열어 시뮬레이션에서는 못 쓴다, G10에서 실제 팔 연결을 끊고 부르게 한다

    def run_diagnostic(self, name: str, progress=None, stop=None) -> tuple[dict | None, str | None]:
        """(결과, 실패 이유). 블로킹이므로 스레드에서 부른다."""
        kw = {"repeats": 1} if name == "powertest" else {}
        if self.real and (name in BIG_MOVES or not self.move_lock()) and self._real_blocked():  # 잠겨도 읽기는 된다
            return None, self._real_blocked()
        try:
            return self.arm.run_diagnostic(name, progress, stop, **kw), None
        except ValueError as e:
            return None, str(e)
        except RuntimeError:
            return None, BUSY_TEXT

    def run_names(self) -> list[str]:
        """실행 기록 폴더 이름. 최근 것이 먼저다."""
        root = Path(self.runs_root)
        if not root.is_dir():
            return []
        return sorted((d.name for d in root.iterdir() if d.is_dir() and (d / "events.jsonl").exists()), reverse=True)

    def load_run(self, name: str) -> dict:
        """실행 하나의 이벤트와 전압·온도. 화면에 보내는 점은 RUN_POINTS 이하로 줄인다."""
        if name not in self.run_names():  # 화면에서 온 이름으로 다른 경로를 읽지 않는다
            raise ValueError(f"없는 실행: {name}")
        folder = Path(self.runs_root) / name
        events = []
        for line in (folder / "events.jsonl").read_text(encoding="utf-8").splitlines():
            with contextlib.suppress(ValueError):  # 강제 종료로 마지막 줄이 잘렸을 수 있다
                row = json.loads(line)
                events.append({"t": row["t"], "kind": row["kind"], "text": json.dumps(row["data"], ensure_ascii=False)[:200]})
        rows = []
        path = folder / "status.csv"
        if path.exists():
            with path.open(encoding="utf-8", newline="") as f:
                for row in csv.DictReader(f):
                    with contextlib.suppress(ValueError, TypeError):
                        rows.append((float(row["t"]), float(row["voltage_v"]), float(row["max_temp_c"])))
        stride = max(1, len(rows) // RUN_POINTS)
        t0 = rows[0][0] if rows else 0.0
        return {
            "name": name, "events": events, "rows": len(rows), "duration_s": rows[-1][0] - t0 if rows else 0.0,
            "voltage": [[round(t - t0, 2), v] for t, v, _ in rows[::stride]],
            "temperature": [[round(t - t0, 2), c] for t, _, c in rows[::stride]],
        }

    def logs(self) -> list[LogRecord]:
        """이번 실행의 이벤트와 파이썬 로그를 시간순으로."""
        logger = self.arm.logger
        if logger:
            path = logger.dir / "events.jsonl"
            with path.open(encoding="utf-8") as f:
                f.seek(self._log_offset)
                while line := f.readline():
                    if not line.endswith("\n"):
                        break
                    self._log_offset = f.tell()
                    with contextlib.suppress(ValueError):
                        row = json.loads(line)
                        level = "ERROR" if row["kind"] == "fault" else "WARNING" if row["kind"] in ("estop", "safety") else "INFO"
                        self._event_log.append(LogRecord(row["t"], level, f"{row['kind']}: {json.dumps(row['data'], ensure_ascii=False)[:300]}"))
        return sorted([*self._event_log, *self.logbook.records], key=lambda r: r.t)

    # ---- 교시
    def sequences(self) -> list[str]:
        return self.arm.teach.names()

    def sequence(self, name: str) -> dict:
        """기록의 요약, 화면용 표본, 3D 궤적. 궤적을 그리고 돌려준다. 스레드에서 부른다."""
        info = self.arm.record_info(name)
        q, t = self.arm.teach.load(name)
        stride = max(1, len(q) // SEQUENCE_POINTS)
        q, t = q[::stride], t[::stride] - t[0]
        if self._path_kin is None:  # 팔의 Kinematics는 다른 스레드도 쓰므로 궤적 계산에는 따로 둔다
            self._path_kin = Kinematics(self.arm.cfg.urdf, self.arm.ws.base, self.arm.cfg.joint_limits_deg)
        tips = [self._path_kin.fk(JointState(*row)) for row in q.tolist()]
        segments = [
            (s["label"], [(p.x_mm, p.y_mm, p.z_mm) for p, at in zip(tips, t, strict=True) if s["start_s"] <= at <= s["end_s"]])
            for s in info["segments"]
        ]
        if self.scene:
            self.scene.show_trajectory(segments)
        return {"info": info, "t": t.round(3).tolist(), "q": q.round(2).tolist()}

    # ---- 새 기록 (실제 팔에서만). 순서는 CLI record와 같다: 홈 -> 팔 잡기 -> 카운트다운 -> 토크 끄고 기록
    def recording(self) -> bool:
        return bool(self.real and self.real.teach.recording)

    def record_name_problem(self, name: str) -> str | None:
        """파일 이름이 되므로 경로 문자를 막고, 다른 기록을 덮어쓰지 않는다."""
        if not re.fullmatch(r"[\w-]{1,40}", name or ""):
            return "기록 이름은 글자, 숫자, -, _ 로 40자까지 쓸 수 있습니다"
        if name in self.sequences():
            return f"같은 이름의 기록이 있습니다: {name}"
        return None

    def record_prepare(self) -> str | None:
        """홈으로 옮긴다. 토크를 끄면 팔이 처지므로 받침 근처에서 시작한다. 실패 이유 또는 None. 스레드에서 부른다."""
        return self._real_blocked() or self._run_on(self.real, lambda arm: arm.home())

    def record_begin(self, name: str) -> str | None:
        """토크를 끄고 기록을 시작한다. 사람이 팔을 잡고 있어야 한다. 실패 이유 또는 None."""
        reason = self._real_blocked() or self.record_name_problem(name)
        if reason:
            return reason
        if self.task_running():
            return "작업 실행 중입니다"
        self.real.record_start()
        self._record_name = name
        return None

    def record_finish(self) -> str | None:
        """기록을 저장하고 토크를 켜서 지금 자세를 잡는다. 저장한 이름 또는 None."""
        if not self.recording():
            return None
        name, self._record_name = self._record_name, None
        self.real.record_stop(name)
        return name

    def record_truncated(self) -> float | None:
        """최대 시간에 걸려 앞부분만 남았으면 그 시간(초)."""
        teach = self.real.teach
        return teach.max_s if teach.truncated else None

    def replay(self, name: str, speed: float, until_limit: bool = False, segment_speeds: dict[int, float] | None = None) -> str | None:
        reason = self._dispatch(lambda arm: arm.replay(name, speed, until_limit, segment_speeds), f"{name} 재생")
        if reason == MOTION_TEXT["out_of_bounds"] and not until_limit:
            return "기록에 허용 범위 밖 자세가 있어 재생하지 않았습니다(첫 자세까지는 이동합니다). 한계 앞까지만 재생을 켜 보세요"
        return reason

    def replay_cut(self) -> str | None:
        """한계 앞까지만 재생했을 때 어디서 잘렸는지."""
        cut = self._model_arm().teach.last_cut
        if cut is None:
            return None
        at, total, why = cut
        return f"표본 {total}개 중 {at}개까지 재생했습니다: {reason_text(why)}"

    def disconnected(self, client: str) -> None:
        """조작하던 브라우저의 연결이 끊기면 팔을 세운다(토크는 유지)."""
        if self.owner == client:
            self.owner = None
            self.record_finish()  # 토크가 꺼진 채 조작하던 화면이 사라지면 지금 자세를 잡는다
            self.stop_task()

    def estop(self) -> None:
        self.record_finish()  # 기록 중에는 토크가 꺼져 있어 정지 명령만으로는 팔이 서지 않는다
        self.paused = False
        self.studio.stop()
        self.pending = None
        for arm in self._arms():
            arm.estop()  # Studio.estop은 실행 기록에 남기지 않는다

    def close(self) -> None:
        logging.getLogger().removeHandler(self.logbook)
        if self._listener:
            self._listener.close()
        self.record_finish()
        self.stop_task()  # 제어 루프를 먼저 끄면 동작을 기다리던 스레드가 풀리지 않아 프로세스가 안 꺼진다
        for arm in self._arms():
            arm.ctl.wait_idle()
            arm.disconnect()
        if self.scene:
            self.scene.close()
        if self.link:
            self.link.stop()

    def readout(self) -> dict[str, str]:
        """상태줄에 보일 값. 키는 상태줄 칸 이름이다."""
        arm = self.arm
        values = {
            "팔": "실제" if self.real else "시뮬레이션",
            "실행 방식": RUN_MODES[self.mode],
            "감시": self.link.text() if self.link else "없음",
            "속도": f"{arm.speed:.0f} %",
            "안전 모드": SAFETY_TEXT.get(arm.safety.settings.mode, arm.safety.settings.mode),
            "기록 폴더": arm.logger.dir.name if arm.logger else EMPTY,
            "닿는지·오차": self.reach,
        }
        st = self.state.status
        if st is None:
            return values
        t = st.tool
        tip = tip_vector(arm.kin, t.pitch_deg, t.yaw_deg, st.joints.gripper, "tip")
        x, y, h = t.x_mm + tip[0], t.y_mm + tip[1], t.z_mm + tip[2] - arm.cfg.table_z_mm
        return values | {
            "전압": f"{st.voltage_v:.1f} V",
            "최고 온도": hottest_text(st),
            "손끝 좌표": f"x {x:4.0f}  y {y:4.0f}  높이 {h:4.0f} mm",  # 폭을 고정해 값이 바뀔 때 옆 칸이 흔들리지 않게 한다
            "토크": "켜짐" if st.torque_enabled else "꺼짐",
        }
