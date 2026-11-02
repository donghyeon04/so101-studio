"""ArmService·Studio를 감싸는 유일한 창구 (gui-spec 7·8절). 실행 방식, 조작권, REQ 임시 우회를 여기에 둔다."""

import contextlib
import re
from dataclasses import dataclass, replace
from pathlib import Path

from ..arm.kinematics import Kinematics
from ..arm.sim_driver import SimDriver
from ..core.config import load_robot, load_workspace
from ..core.types import DetectedObject, JointState
from ..perception.manual import ManualSource
from ..service.arm_service import ArmService
from ..service.stop_listener import StopListener
from ..service.studio import Studio
from ..service.targets import reason_text, tip_vector
from ..task.policy import reachable
from .scene3d import Scene3D
from .state import EMPTY, SAFETY_TEXT, State, hottest_text
from .watchdog_link import WatchdogLink

JOG_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")  # JointState 순서
JOINT_TEXT = {
    "shoulder_pan": "어깨 회전", "shoulder_lift": "어깨 들기", "elbow_flex": "팔꿈치", "wrist_flex": "손목 굽힘", "wrist_roll": "손목 회전",
    "gripper": "그리퍼",
}  # JointState 순서
CART_AXES = ("x", "y", "z", "yaw")  # 작업대 좌표계
SEQUENCE_POINTS = 300  # 화면에 보내는 기록 표본 수의 상한
JOG_TOOL_MM_S = 100.0  # 속도 100%일 때. 컨트롤러가 60mm/s로 한 번 더 자른다
JOG_YAW_DEG_S = 60.0
REASON_EXTRA = {"z_ceiling": "허용 상자보다 높습니다"}  # service.targets에 아직 없는 이유
BUSY_TEXT = "팔이 아직 움직이고 있습니다"
MOTION_TEXT = {
    "ik_fail": reason_text("ik_fail"),
    "out_of_bounds": "허용 범위 밖이라 움직이지 않았습니다",
    "stopped": "정지 상태입니다. 정지 해제를 눌러 주세요",
    "preempted": "다른 명령으로 바뀌었습니다",
    "fault": "오류로 멈췄습니다",
    "timeout": "시간 안에 끝나지 않았습니다",
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

    def __init__(self, speed: float | None = None, log_root: str | None = "runs", link: WatchdogLink | None = None,
                 viser_port: int | None = None):
        cfg = load_robot()
        self.state = State()
        self.link = link
        self.scene: Scene3D | None = None
        self.reach = EMPTY
        self._next_object = 0
        self._path_kin: Kinematics | None = None
        self._listener = None
        self.owner: str | None = None  # 조작권을 가진 브라우저. NiceGUI 이벤트 루프에서만 바꾼다
        self.arm = ArmService(SimDriver(cfg.poses.get("home")), cfg, load_workspace(), speed, log_root)
        ok, msg = self.arm.connect()
        if not ok:
            raise RuntimeError(msg)
        self.arm.status_stream(self.state.update)
        self.source = ManualSource()
        self.studio = Studio(self.arm, self.source, dry_run=True)  # 시뮬레이션에는 잡히는 물체가 없다. 접촉 없음도 성공으로 본다
        self.studio.bus.subscribe("task", self._on_task)
        self.paused = False
        if viser_port is not None:
            self.scene = Scene3D(cfg, self.arm.ws, self.arm.kin, viser_port)
            self.scene.set_objects(self.objects())
            self.arm.status_stream(self.scene.update_status)
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
        self.arm.resume()
        if self.link:
            self.link.rearm()

    # ---- 직접 조작. 실패하면 이유 문장을, 성공하면 None을 돌려준다. 스레드에서 부른다
    def _motion(self, fn, *args, **kwargs) -> str | None:
        try:
            result = fn(*args, **kwargs)
        except RuntimeError:  # 컨트롤러가 이미 다른 동작을 실행 중이다
            return BUSY_TEXT
        return None if result.status == "ok" else MOTION_TEXT.get(result.status, result.status)

    def home(self) -> str | None:
        return self._motion(self.arm.home)

    def park(self) -> str | None:
        return self._motion(self.arm.park)

    def gripper(self, value: float) -> str | None:
        return self._motion(self.arm.set_gripper, value)

    def gripper_width(self, mm: float) -> str | None:
        return self._motion(self.arm.set_gripper, mm, as_width=True)

    def set_speed(self, pct: float) -> None:
        self.arm.set_speed(pct)

    def plan(self, x: float, y: float, h: float, yaw: float) -> Plan:
        """움직이지 않고 (x, y, 높이)에 닿는지 본다. 3D에 점, 계산 자세, 손끝 경로를 그린다."""
        pose = self.arm.point_pose(x, y, h, yaw)
        reason, q = self.arm.preview_tool(pose)
        path = None
        duration = 0.0
        if reason is None:
            preview = self.arm.preview_path(pose)
            reason, path, duration = preview["reason"], preview["tool"], preview["duration_s"]
        ok = reason is None
        text = f"닿습니다 · 약 {duration:.1f}초" if ok else REASON_EXTRA.get(reason) or reason_text(reason)
        self.reach = "닿음" if ok else "안 닿음"
        if self.scene:
            self.scene.set_preview(q if ok else None)
            self.scene.show_target(x, y, h, ok, path)
        return Plan(ok, text)

    def move_to(self, x: float, y: float, h: float, yaw: float) -> str | None:
        results = []

        def run():
            results.append(self.arm.move_tool(self.arm.point_pose(x, y, h, yaw)))
            return results[0]

        reason = self._motion(run)
        if self.scene:
            self.scene.clear_target()
        if results and results[0].position_error_mm is not None:
            self.reach = f"오차 {results[0].position_error_mm:.1f} mm"
        return reason

    def jog(self, mode: str, index: int, sign: int) -> bool:
        """누르는 동안 조그. 입력이 0.3초 끊기면 컨트롤러가 스스로 멈춘다."""
        if not self.state.status or not self.state.status.torque_enabled:
            self.arm.driver.set_torque(True)  # REQ-012 임시: ArmService.jog가 토크를 켜지 않는다
        speed = self.arm.speed / 100
        if mode == "joint":
            velocity = [0.0] * 6
            velocity[index] = sign * self.arm.cfg.joint_v_max_deg_s * speed
        else:
            velocity = [0.0] * 4
            velocity[index] = sign * (JOG_YAW_DEG_S if CART_AXES[index] == "yaw" else JOG_TOOL_MM_S) * speed
        return self.arm.jog(mode, velocity)

    def jog_release(self, mode: str) -> None:
        """손을 떼면 속도를 0으로 내려 부드럽게 멈춘다."""
        self.arm.jog(mode, [0.0] * (6 if mode == "joint" else 4))

    def jog_step(self, mode: str, index: int, amount: float) -> str | None:
        """정해진 간격만큼 움직인다. 관절은 도, 직교는 mm(yaw는 도)."""
        q = self.arm.status().joints
        if mode == "joint":
            name = JOG_JOINTS[index]
            return self._motion(self.arm.move_joints, replace(q, **{name: getattr(q, name) + amount}))
        delta = [0.0] * 4
        delta[index] = amount
        goal = self.arm.kin.step(q, *delta)
        if goal is None:
            return reason_text("ik_fail")
        return self._motion(self.arm.move_joints, goal)

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
        if self.arm.ctl.state != "idle":  # 작업 스레드에서 동작 충돌이 나면 조용히 끝난다
            return MOTION_TEXT["stopped"] if self.arm.ctl.state in ("stopped", "fault") else BUSY_TEXT
        self.state.placed, self.paused = 0, step_mode
        self.studio._step.clear()  # REQ-013 임시: 앞선 stop()이 남긴 신호로 첫 단계를 건너뛴다
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
        self.studio.task._step_mode = self.paused  # REQ-013 임시: Studio에 일시정지가 없다
        if not self.paused:
            self.studio.step()

    def stop_task(self) -> None:
        self.paused = False
        self.studio.task._step_mode = False  # REQ-013 임시: 단계별 모드에서 stop()은 다음 단계에서 다시 기다린다
        self.studio.stop()
        self.arm.stop()

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
            self.arm.set_safety(mode, margin, box)
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
            return None, self.arm.set_params(changes)
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

    def replay(self, name: str, speed: float, until_limit: bool = False, segment_speeds: dict[int, float] | None = None) -> str | None:
        try:
            result = self.arm.replay(name, speed, until_limit, segment_speeds)
        except ValueError as e:  # 배율 때문에 속도 한계를 넘는 경우
            return str(e)
        except RuntimeError:
            return BUSY_TEXT
        if result.status == "out_of_bounds" and not until_limit:
            return "기록에 허용 범위 밖 자세가 있어 재생하지 않았습니다(첫 자세까지는 이동합니다). 한계 앞까지만 재생을 켜 보세요"
        return self._motion(lambda: result)

    def replay_cut(self) -> str | None:
        """한계 앞까지만 재생했을 때 어디서 잘렸는지."""
        cut = self.arm.teach.last_cut
        if cut is None:
            return None
        at, total, why = cut
        return f"표본 {total}개 중 {at}개까지 재생했습니다: {reason_text(why)}"

    def disconnected(self, client: str) -> None:
        """조작하던 브라우저의 연결이 끊기면 팔을 세운다(토크는 유지)."""
        if self.owner == client:
            self.owner = None
            self.stop_task()

    def estop(self) -> None:
        self.paused = False
        self.studio.task._step_mode = False  # REQ-013 임시
        self.studio.estop()

    def close(self) -> None:
        if self._listener:
            self._listener.close()
        self.arm.disconnect()
        if self.scene:
            self.scene.close()
        if self.link:
            self.link.stop()

    def readout(self) -> dict[str, str]:
        """상태줄에 보일 값. 키는 상태줄 칸 이름이다."""
        arm = self.arm
        values = {
            "실행 방식": "시뮬레이션만",
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
