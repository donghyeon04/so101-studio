"""ArmService: GUI·CLI가 팔을 부르는 유일한 입구 (arm-software-spec.md 6.9). 이름과 입출력은 GUI 기술이 정해져도 바뀌지 않는다."""
import contextlib
import threading
import time
from dataclasses import asdict, replace

import numpy as np

from ..arm.kinematics import Kinematics
from ..arm.motion import MotionController
from ..arm.safety import MODES, Safety, SafetySettings
from ..arm.skills import Skills
from ..arm.teach import Teach
from ..arm.trajectory import cfg_move_j, cfg_move_l
from ..core.config import RobotConfig, WorkspaceConfig, load_workspace
from ..core.logging import RunLogger
from ..core.types import ArmStatus, DetectedObject, JointState, MotionResult, ToolPose
from .targets import point_to_pose, tip_vector

STATUS_HZ = 10
HEALTH_HZ = 2


def plan_goto(kin, cfg, q0: JointState, goal: ToolPose, speed_pct: float):
    """직선(MoveL)으로 못 가면 목표 위 approach_height_mm 지점으로 MoveJ한 뒤 수직으로 내려간다(pick과 같은 순서)."""
    traj = cfg_move_l(cfg, kin, kin.fk(q0), goal, q0, speed_pct)
    if traj is not None:
        return [traj]
    hover = replace(goal, z_mm=goal.z_mm + cfg.approach_height_mm)
    q_hover = kin.ik(hover, q0)
    if q_hover is None:
        return None
    down = cfg_move_l(cfg, kin, hover, goal, q_hover, speed_pct)
    if down is None:
        return None
    return [cfg_move_j(cfg, q0, q_hover, speed_pct), down]


class ArmService:
    def __init__(self, driver, cfg: RobotConfig, ws: WorkspaceConfig | None = None, speed_pct: float | None = None,
                 log_root: str | None = "runs"):
        self.driver, self.cfg = driver, cfg
        self.ws = ws or load_workspace()
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default
        self.log_root = log_root
        self.logger: RunLogger | None = None
        self.kin = self.safety = self.ctl = self.skills = self.teach = None
        self._callbacks: list = []
        self._status_thread: threading.Thread | None = None
        self._stop_status = threading.Event()
        self._health = {"temperature_c": {}, "load_pct": {}, "voltage_v": 0.0}
        self._connected = False

    # ---- 연결
    def connect(self, port: str | None = None) -> tuple[bool, str]:
        """연결하고 컨트롤러·기능을 만든다. 실패하면 (False, 오류 메시지)."""
        try:
            self.driver.connect(port or self.cfg.port, self.cfg.calibration_id)
        except Exception as e:  # noqa: BLE001 포트 없음, 응답 없는 모터 등 원인을 메시지로 돌려준다
            return False, str(e)
        self.kin = Kinematics(self.cfg.urdf, self.ws.base, self.cfg.joint_limits_deg)
        self.safety = Safety(self.cfg.joint_limits_deg, self.ws, self.kin, z_floor_mm=self.cfg.tip_floor_mm,
                             table_z_mm=self.cfg.table_z_mm, physical_limits=self.cfg.physical_limits_deg)
        self.ctl = MotionController(self.driver, self.kin, self.safety, self.cfg.control_hz)
        self.skills = Skills(self.ctl, self.kin, self.cfg, self.ws, self.speed)
        self.teach = Teach(self.ctl, self.cfg)
        self.logger = RunLogger(self.log_root) if self.log_root else None
        self._health = self.driver.read_health()
        self._connected = True
        self._stop_status.clear()
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()
        self._log("connect", {"port": port or self.cfg.port})
        return True, ""

    def disconnect(self) -> None:
        self._stop_status.set()
        if self._status_thread:
            self._status_thread.join(timeout=1.0)
        if self.ctl:
            self.ctl.close()
        if self.driver:
            self.driver.disconnect()
        if self.logger:
            self.logger.close()
            self.logger = None
        self._connected = False

    # ---- 상태
    def status(self) -> ArmStatus:
        q = self.driver.read_joints()
        return ArmStatus(
            t=time.time(), joints=q, tool=self.kin.fk(q),
            temperature_c=self._health["temperature_c"], load_pct=self._health["load_pct"], voltage_v=self._health["voltage_v"],
            torque_enabled=self.driver.torque_enabled(),
            state="teach" if self.teach.recording else self.ctl.state, fault=self.ctl.fault,
        )

    def status_stream(self, callback) -> None:
        """ArmStatus를 10Hz로 받는다."""
        self._callbacks.append(callback)

    def _status_loop(self) -> None:
        nxt, n = time.perf_counter(), 0
        while not self._stop_status.is_set():
            nxt += 1.0 / STATUS_HZ
            with contextlib.suppress(Exception):  # 통신 오류는 컨트롤러가 fault로 처리한다
                if n % (STATUS_HZ // HEALTH_HZ) == 0:
                    self._health = self.driver.read_health()  # 온도·부하는 2Hz로만 읽는다
                st = self.status()
                if self.logger:
                    self.logger.status(st)
                for cb in list(self._callbacks):
                    cb(st)
            n += 1
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _log(self, kind: str, data) -> None:
        if self.logger:
            self.logger.event(kind, data)

    def _done(self, kind: str, r: MotionResult, **info) -> MotionResult:
        self._log(kind, {**info, "status": r.status, "duration_s": r.duration_s})
        return r

    # ---- 동작
    def set_speed(self, pct: float) -> None:
        self.speed = pct
        self.skills.speed = pct

    def _run(self, trajs) -> MotionResult:
        return self.skills._run(trajs)

    def home(self) -> MotionResult:
        return self._done("home", self.skills.home())

    def park(self) -> MotionResult:
        return self._done("park", self.skills.park())

    def move_joints(self, goal: JointState, speed_pct: float | None = None) -> MotionResult:
        trajs = [cfg_move_j(self.cfg, self.driver.read_joints(), goal, speed_pct or self.speed)]
        self.driver.set_torque(True)
        return self._done("movej", self._run(trajs), goal=asdict(goal))

    def move_tool(self, pose: ToolPose, speed_pct: float | None = None) -> MotionResult:
        trajs = plan_goto(self.kin, self.cfg, self.driver.read_joints(), pose, speed_pct or self.speed)
        if trajs is None:
            return self._done("goto", MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0)), goal=asdict(pose))
        self.driver.set_torque(True)
        return self._done("goto", self._run(trajs), goal=asdict(pose))

    def point_pose(self, x: float, y: float, h: float, yaw: float = 90.0, frame: str = "table", tcp: str = "tip") -> ToolPose:
        """frame 기준 (x, y, 작업대 면 위 높이 h)인 끝점(tcp)을 수직 하강으로 가져갈 ToolPose. GUI에서 점을 찍을 때 쓰고, preview_tool·move_tool에 넘긴다."""
        tip = tip_vector(self.kin, -90.0, yaw, self.driver.read_joints().gripper, tcp)
        return point_to_pose(x, y, h, yaw, frame, self.ws.base, self.cfg.table_z_mm, tip)

    def get_safety(self) -> dict:
        """지금 안전 설정과 실제로 적용되는 값. GUI가 설정 화면을 그릴 때 쓴다."""
        s = self.safety
        return {**asdict(s.settings), "effective_limits_deg": s.limits, "physical_limits_deg": s.physical,
                "box_mm_effective": s.box(), "max_step_deg": s.max_step_deg}

    def set_safety(self, mode: str | None = None, joint_margin_deg: float | None = None, box_mm: dict | None = "keep") -> dict:
        """안전 설정을 바꾼다. 검증에 실패하면 ValueError(바뀌지 않음). box_mm=None이면 기본 범위로 되돌린다."""
        cur = self.safety.settings
        mode = cur.mode if mode is None else mode
        margin = cur.joint_margin_deg if joint_margin_deg is None else float(joint_margin_deg)
        box = cur.box_mm if box_mm == "keep" else box_mm
        if mode not in MODES:
            raise ValueError(f"mode는 {MODES} 중 하나")
        if not -15.0 <= margin <= 30.0:
            raise ValueError("joint_margin_deg는 -15~30")
        if box is not None:
            if not set(box) <= {"x", "y", "z"} or any(len(v) != 2 for v in box.values()):
                raise ValueError("box_mm는 x, y, z 각각 [최소, 최대]")
            for k, (lo, hi) in box.items():
                if lo is not None and hi is not None and lo >= hi:
                    raise ValueError(f"box_mm.{k}: 최소가 최대보다 작아야 한다")
            if box.get("z") and box["z"][0] is not None and box["z"][0] < 0:
                raise ValueError("box_mm.z 최소는 0(작업대 면) 이상")
        self.safety.settings = SafetySettings(mode, margin, box)
        self._log("safety", asdict(self.safety.settings))
        return self.get_safety()

    def preview_tool(self, pose: ToolPose):
        """움직이지 않고 목표가 가능한지만 본다. (이유 또는 None, 풀린 관절값). GUI가 점 위에 마우스를 올릴 때 미리보기로 쓴다."""
        q = self.skills._ik(pose)
        if q is None:
            return "ik_fail", None
        return self.safety.check_goal(q), q

    def pick_and_place(self, obj: DetectedObject, zone: str | None = None) -> MotionResult:
        self.driver.set_torque(True)
        r = self.skills.pick(obj)
        self._log("pick", {"object": asdict(obj), "status": r.status})
        if r.status == "ok" and zone:
            r = self.skills.place(zone)
            self._log("place", {"zone": zone, "status": r.status})
        return r

    def place(self, zone: str) -> MotionResult:
        self.driver.set_torque(True)
        return self._done("place", self.skills.place(zone), zone=zone)

    def set_gripper(self, value: float, as_width: bool = False) -> MotionResult:
        """그리퍼만 움직인다. as_width면 벌림 폭(mm)으로 해석한다(예: 카드 폭 54mm)."""
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        v = float(np.interp(value, [p[0] for p in pts], [p[1] for p in pts])) if as_width else value
        lo, hi = self.cfg.joint_limits_deg["gripper"]
        q = self.driver.read_joints()
        self.driver.set_torque(True)
        return self._done("gripper", self._run([cfg_move_j(self.cfg, q, replace(q, gripper=min(max(v, lo), hi)), self.speed)]), value=v)

    def jog(self, mode: str, velocity) -> bool:
        return self.ctl.jog(mode, velocity)

    def record_start(self, follow: bool = False, stiffness: float = 1.0) -> None:
        self._log("record_start", {"follow": follow, "stiffness": stiffness})
        self.teach.record_start(follow, stiffness)

    def record_stop(self, name: str):
        path = self.teach.record_stop(name)
        self._log("record_stop", {"name": name, "path": str(path)})
        return path

    def replay(self, name: str, speed: float = 1.0, until_limit: bool = False) -> MotionResult:
        self.driver.set_torque(True)
        return self._done("replay", self.teach.replay(name, speed, until_limit), name=name, speed=speed, until_limit=until_limit)

    def run_sequence(self, name: str) -> MotionResult:
        return self.replay(name, 1.0)

    def retarget(self, goal: JointState):
        """진행 중인 동작을 멈추지 않고 새 목표로 바꾼다. 돌려받은 핸들의 wait()가 결과(ok)를 준다."""
        return self.ctl.retarget(goal, self.cfg.joint_v_max_deg_s * self.speed / 100, self.cfg.a_max_deg_s2, self.cfg.joint_jerk_max_deg_s3)

    def stop(self) -> None:
        self.ctl.stop()

    def estop(self) -> None:
        self.ctl.estop()
        self._log("estop", {})
