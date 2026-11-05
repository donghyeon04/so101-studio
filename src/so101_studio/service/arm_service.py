"""ArmService: GUI·CLI가 팔을 부르는 유일한 입구 (arm-software-spec.md 6.9). 이름과 입출력은 GUI 기술이 정해져도 바뀌지 않는다."""
import threading
import time
import traceback
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from ..arm.kinematics import Kinematics
from ..arm.motion import MotionController
from ..arm.safety import MODES, Safety, SafetySettings
from ..arm.skills import Skills
from ..arm.teach import Teach
from ..arm.trajectory import cfg_move_j, cfg_move_l, move_l
from ..core.blackbox import install_excepthooks
from ..core.config import RobotConfig, WorkspaceConfig, load_workspace
from ..core.incident import Incident, create_incident
from ..core.logging import RunLogger
from ..core.types import ArmStatus, DetectedObject, JointState, MotionResult, ToolPose
from ..perception.base import PoseSample
from . import diagnostics as _diag
from . import params as _params
from .targets import point_to_pose, tip_vector

STATUS_HZ = 10
HEALTH_HZ = 2
LOAD_PROBE_PCT = 50.0  # 어느 관절이든 부하가 이 값 이상이거나 상태 루프가 오류를 내면 그 순간 토크·상태 비트·부하를 바로 한 번 더 읽어 남긴다 (결정 0030 보강)
PROBE_COOLDOWN_S = 1.0
INCIDENT_COOLDOWN_S = 10.0  # 같은 이유의 사고 묶음은 이 시간 안에 다시 만들지 않는다(연달아 나는 fault·E-STOP이 묶음을 쏟아내지 않게)
# 이벤트 종류별 기본 수준과 코드(기록 형식은 decisions/0030). 나머지는 INFO, 코드는 종류 이름 대문자
EVENT_LEVEL = {"estop": ("WARN", "ARM-ESTOP"), "locked": ("WARN", "ARM-LOCKED"), "connect_failed": ("ERROR", "ARM-CONNECT-FAILED")}


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
                 log_root: str | None = "runs", proc: str | None = None):
        self.driver, self.cfg = driver, cfg
        self.ws = ws or load_workspace()
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default
        self.log_root = log_root
        self.preset_dir = "config/presets"
        self.proc = proc  # 기록 머리의 proc(cli | gui). 없으면 환경변수 SO101_PROC, 기본 cli
        self.config_dir = Path("config")  # 사고 묶음에 설정 사본을 담는 곳
        self.incident_after_s = 60.0  # 사고 묶음에서 뒤 60초를 모으는 시간(시험은 0)
        self.install_hooks = not getattr(driver, "simulated", False)  # 처리 안 된 예외 감시. 시뮬레이션은 쓰지 않는다
        self.incidents: list[Incident] = []
        self._incident_last: dict[str, float] = {}
        self._incident_thread: threading.Thread | None = None
        self._status_errors: dict[tuple, int] = {}
        self._torque_lost = False
        self._probe_t = -1e9
        self._restore_hooks = None
        self.logger: RunLogger | None = None
        self.kin = self.safety = self.ctl = self.skills = self.teach = None
        self.port: str | None = None  # 마지막으로 연결한 포트
        self.last_path: MotionResult | None = None  # 마지막 follow_path의 결과(status, reason)
        self._callbacks: list = []
        self._status_thread: threading.Thread | None = None
        self._stop_status = threading.Event()
        self._health = {"temperature_c": {}, "load_pct": {}, "voltage_v": 0.0, "current_ma": 0.0}
        self._connected = False

    # ---- 연결
    def connect(self, port: str | None = None) -> tuple[bool, str]:
        """연결하고 컨트롤러·기능을 만든다. 실패하면 (False, 오류 메시지). 이미 연결돼 있으면(통신 끊김 fault 뒤 다시 연결 등) 먼저 정리한다.
        다시 연결해도 토크는 끄지 않고, 새 컨트롤러가 현재 위치를 명령값으로 삼는다 (decisions/0031)."""
        if self._connected:
            self.disconnect()
        self.port = port or self.cfg.port  # 연결한 포트(진단 linkwatch가 같은 포트를 쓴다)
        self.logger = RunLogger(self.log_root, proc=self.proc) if self.log_root else None  # 연결 때 쓰는 레지스터도 기록하려고 먼저 연다
        self.driver.event_sink = self._driver_event
        try:
            self.driver.connect(port or self.cfg.port, self.cfg.calibration_id)
        except Exception as e:  # noqa: BLE001 포트 없음, 응답 없는 모터 등 원인을 메시지로 돌려준다
            self._log("connect_failed", {"port": port or self.cfg.port, "error": repr(e)})
            if self.logger:
                self.logger.close()
                self.logger = None
            return False, str(e)
        if self.kin is None:  # 다시 연결할 때 URDF를 또 읽으면 파이썬이 1.7초 멈춘다(감시 한계 1초): 한 번 만든 것을 쓴다
            self.kin = Kinematics(self.cfg.urdf, self.ws.base, self.cfg.joint_limits_deg)
        self.safety = Safety(self.cfg.joint_limits_deg, self.ws, self.kin, z_floor_mm=self.cfg.tip_floor_mm,
                             table_z_mm=self.cfg.table_z_mm, physical_limits=self.cfg.physical_limits_deg)
        self.ctl = MotionController(self.driver, self.kin, self.safety, self.cfg.control_hz)
        self.skills = Skills(self.ctl, self.kin, self.cfg, self.ws, self.speed)
        self.teach = Teach(self.ctl, self.cfg)
        self.ctl.on_event = self._ctl_event
        self._health = self.driver.read_health()
        self._connected = True
        self._torque_lost = False
        if self.install_hooks:
            self._restore_hooks = install_excepthooks(self._unhandled)
        self.ctl.locked_reason = self._lock_reason()  # 캘리브레이션 불일치나 팔 파일 없음이면 움직이는 명령을 막고 읽기만 허용한다 (decisions/0027)
        if self.ctl.locked_reason:
            self._log("locked", {"reason": self.ctl.locked_reason})
        self._torque_on = False  # jog가 토크를 켠 뒤 true. 교시로 토크를 끄면 다시 false (REQ-012)
        self._stop_status.clear()
        self._log("connect", {"port": port or self.cfg.port, **getattr(self.driver, "connect_info", {})})
        self._write_status(self.status())  # 연결하자마자 첫 줄: 연결 뒤 상태가 비지 않게
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()
        return True, ""

    def _lock_reason(self) -> str | None:
        if getattr(self.driver, "simulated", False):
            return None
        if getattr(self.driver, "locked_reason", None):
            return self.driver.locked_reason
        if not self.cfg.arm_name:
            return "팔 파일이 없다: config/arms/_template.yaml을 복사해 채우기 전에는 움직이지 않는다"
        return None

    @property
    def locked_reason(self) -> str | None:
        """None이 아니면 움직이는 명령이 막혀 있다(읽기만 가능). 이유 문장이다."""
        return self.ctl.locked_reason if self.ctl else None

    def disconnect(self) -> None:
        self._stop_status.set()
        if self._status_thread:
            self._status_thread.join(timeout=1.0)
        if self._connected:
            self._log("disconnect", {})
        if self.ctl:
            self.ctl.close()
        if self.driver:
            self.driver.disconnect()
        if self._incident_thread:
            self._incident_thread.join(timeout=3.0)
        for inc in self.incidents:  # 뒤 60초를 기다리지 않고 지금까지의 기록으로 마무리한다
            inc.finalize(2.0)
        if self._restore_hooks:
            self._restore_hooks()
            self._restore_hooks = None
        if self.logger:
            self.logger.close()
            self.logger = None
        self._connected = False

    # ---- 상태
    def status(self) -> ArmStatus:
        q = self.driver.read_joints()
        return ArmStatus(
            t=time.time(), joints=q, tool=self.kin.fk(q),
            temperature_c=self._health["temperature_c"], load_pct=self._health["load_pct"], voltage_v=self._health["voltage_v"], current_ma=self._health.get("current_ma", 0.0),
            torque_enabled=self.driver.torque_enabled(),
            state="teach" if self.teach.recording else self.ctl.state, fault=self.ctl.fault,
        )

    def status_stream(self, callback) -> None:
        """ArmStatus를 10Hz로 받는다."""
        self._callbacks.append(callback)

    def _status_data(self, st) -> dict:
        """status.jsonl 한 줄의 data (decisions/0030). 온도·부하·전류·토크·Status는 2Hz로 읽은 값을 이어 쓴다."""
        h, ctl = self._health, self.ctl
        torque = {m: bool(v) for m, v in h.get("torque", {}).items()}
        return {"joints": asdict(st.joints), "cmd": asdict(ctl._last), "tool": {"x_mm": st.tool.x_mm, "y_mm": st.tool.y_mm, "z_mm": st.tool.z_mm},
                "torque": torque, "torque_enabled": st.torque_enabled, "temperature_c": st.temperature_c, "load_pct": st.load_pct, "current_ma": st.current_ma,
                "current_ma_by_joint": h.get("current_ma_by_joint", {}), "voltage_v": st.voltage_v, "status_bits": h.get("status_bits", {}),
                "state": st.state, "fault": st.fault, "locked": ctl.locked_reason, "time_lost_s": round(ctl.time_lost_s, 4),
                "tick_dt_max_ms": round(ctl.tick_dt_max_1s() * 1000, 2)}

    def _write_status(self, st) -> None:
        self.ctl.actual_hint = st.joints  # 링 버퍼가 쓰는 가장 최근 실제 값
        if self.logger:
            self.logger.status(self._status_data(st))

    def _status_loop(self) -> None:
        nxt, n = time.perf_counter(), 0
        while not self._stop_status.is_set():
            nxt += 1.0 / STATUS_HZ
            health_tick = n % (STATUS_HZ // HEALTH_HZ) == 0
            try:  # 통신 오류는 컨트롤러가 fault로 처리한다. 여기서는 조용히 버리지 않고 기록한 뒤 계속 돈다
                if health_tick:
                    expect = getattr(self.driver, "expect_torque", None)  # 읽기 전에 정한다: 읽는 사이에 켠 토크를 풀림으로 오해하지 않게
                    self._health = self.driver.read_health()  # 온도·부하·토크·Status는 2Hz로만 읽는다
                    self._check_torque(expect)
                    if max(self._health.get("load_pct", {}).values(), default=0.0) >= LOAD_PROBE_PCT:
                        self._probe("high_load")
                st = self.status()
                self.ctl.note_comm(True)
                self._write_status(st)
                for cb in list(self._callbacks):
                    cb(st)
                if health_tick:  # 오류가 나는 읽기는 2Hz뿐이라, 그 읽기가 다시 성공해야 회복으로 본다
                    self._status_ok()
            except Exception as e:  # noqa: BLE001
                self.ctl.note_comm(False)  # 정지 중에는 50Hz 쓰기가 없어 이 읽기가 끊김을 가장 먼저 본다
                self._status_error(e)
                self._probe("status_loop_error")
            n += 1
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _probe(self, why: str) -> None:
        """과부하·읽기 오류 순간에 2Hz 주기를 기다리지 않고 토크·상태 비트·부하를 바로 한 번 더 읽어 ARM-LOAD-PROBE로 남긴다.
        (서보 보호로 출력이 끊겨도 Torque_Enable 값만으로는 안 보일 수 있어, 그 순간의 Status 비트가 필요하다.) PROBE_COOLDOWN_S에 한 번."""
        now = time.monotonic()
        if now - self._probe_t < PROBE_COOLDOWN_S:
            return
        self._probe_t = now
        try:
            h = self.driver.read_health()
        except Exception as e:  # noqa: BLE001 읽지 못하는 것도 정보다
            self._log("load_probe", {"why": why, "error": repr(e)[:120]}, "WARN", "ARM-LOAD-PROBE")
            return
        self._log("load_probe", {"why": why, "torque": h.get("torque"), "status_bits": h.get("status_bits"), "load_pct": h.get("load_pct"),
                                 "current_ma_by_joint": h.get("current_ma_by_joint"), "state": self.ctl.state if self.ctl else None}, "WARN", "ARM-LOAD-PROBE")

    def _status_error(self, e: Exception) -> None:
        """첫 발생은 전체 traceback, 반복은 횟수(10, 100, 1000번째)로 남긴다."""
        key = (type(e).__name__, str(e)[:80])
        n = self._status_errors[key] = self._status_errors.get(key, 0) + 1
        if n == 1:
            self._log("status_loop_error", {"error": repr(e), "traceback": traceback.format_exc()}, "ERROR", "ARM-STATUS-LOOP-ERROR")
        elif n in (10, 100, 1000, 10000):
            self._log("status_loop_error", {"error": repr(e), "count": n}, "WARN", "ARM-STATUS-LOOP-ERROR")

    def _status_ok(self) -> None:
        if self._status_errors:
            self._log("status_loop_recovered", {"errors": {f"{k[0]}: {k[1]}": v for k, v in self._status_errors.items()}}, "INFO", "ARM-STATUS-LOOP-RECOVERED")
            self._status_errors = {}

    def _check_torque(self, expect) -> None:
        """2Hz로 읽은 Torque_Enable이 컨트롤러가 기대하는 값(켜져 있어야 함)과 다르면 ARM-TORQUE-LOST."""
        torque = self._health.get("torque")
        lost = bool(expect) and bool(torque) and any(not v for v in torque.values())
        if lost and not self._torque_lost:
            self._log("torque_lost", {"expected": True, "read": torque, "status_bits": self._health.get("status_bits"), "state": self.ctl.state, "fault": self.ctl.fault,
                                      "load_pct": self._health.get("load_pct")}, "ERROR", "ARM-TORQUE-LOST")
            self._trigger_incident("ARM-TORQUE-LOST", "켜져 있어야 할 토크가 꺼져 읽혔다")
        self._torque_lost = lost

    def _event(self, kind: str, level: str, code: str, data) -> None:
        if self.logger:
            self.logger.event(kind, data, level, code)

    def _driver_event(self, kind: str, level: str, code: str, data) -> None:
        self._event(kind, level, code, data)

    def _ctl_event(self, kind: str, level: str, code: str, data) -> None:
        self._event(kind, level, code, data)
        if kind == "fault" and data.get("reason") != "comm_lost":  # 통신 끊김은 아래 ARM-COMM-LOST 묶음 하나로
            self._trigger_incident("ARM-FAULT", f"fault: {data.get('reason')}")
        elif kind == "comm_lost":
            self._trigger_incident("ARM-COMM-LOST", "통신 끊김(0.5초 이상 응답 없음)")
        elif kind == "estop":
            self._trigger_incident("ARM-ESTOP", "E-STOP")

    def on_watchdog_stop(self, reason: str | None = None) -> None:
        """감시 프로세스가 보낸 정지(StopListener가 부른다): 기록하고 사고 묶음을 만든다."""
        self._log("watchdog_trip", {"reason": reason}, "ERROR", "WATCHDOG-TRIP")
        self._trigger_incident("WATCHDOG-TRIP", reason or "감시 프로세스 정지")

    def _unhandled(self, exc_type, exc, tb, where: str) -> None:
        self._log("unhandled_exception", {"where": where, "error": repr(exc), "traceback": "".join(traceback.format_exception(exc_type, exc, tb))}, "ERROR", "ARM-UNHANDLED-EXCEPTION")
        self._trigger_incident("ARM-UNHANDLED-EXCEPTION", f"{exc_type.__name__}: {exc}")

    def log_event(self, kind: str, data=None, level: str = "INFO", code: str | None = None) -> None:
        """GUI가 조작권·실행 방식·안전 설정 변경 같은 이벤트를 기록하는 공개 API (decisions/0030)."""
        self._log(kind, data, level, code)

    def _trigger_incident(self, code: str, reason: str) -> None:
        """사고 묶음을 짧은 스레드에서 만든다(제어·상태 루프가 파일과 git에 막히지 않게). 같은 이유는 INCIDENT_COOLDOWN_S 안에 다시 만들지 않는다."""
        now = time.monotonic()
        if not self.log_root or now - self._incident_last.get(code, -1e9) < INCIDENT_COOLDOWN_S:
            return
        self._incident_last[code] = now
        ring = self.ctl.ring.dump() if self.ctl else []

        def run() -> None:
            try:
                self.incidents.append(create_incident(code, self.log_root, reason=reason, ring=ring, proc=self.logger.proc if self.logger else "cli",
                                                      config_dir=self.config_dir, cfg=self.cfg, after_s=self.incident_after_s))
            except Exception as e:  # noqa: BLE001 사고 묶음을 못 만들어도 동작은 계속하고, 못 만든 사실을 남긴다
                self._log("incident_failed", {"code": code, "error": repr(e)}, "ERROR", "ARM-INCIDENT-FAILED")

        self._incident_thread = threading.Thread(target=run, name="Incident", daemon=True)
        self._incident_thread.start()

    def create_incident(self, reason: str = "수동", last_s: float = 60.0, code: str = "MANUAL") -> str | None:
        """손으로 사고 묶음을 만든다(GUI 버튼). 만든 폴더 경로. 링 버퍼는 살아 있는 이 서비스의 것을 쓴다."""
        if not self.log_root:
            return None
        inc = create_incident(code, self.log_root, reason=reason, ring=self.ctl.ring.dump() if self.ctl else [], proc=self.logger.proc if self.logger else "cli",
                              config_dir=self.config_dir, cfg=self.cfg, window_s=last_s, after_s=0)
        return str(inc.path)

    def _log(self, kind: str, data, level: str | None = None, code: str | None = None) -> None:
        if self.logger:
            lv, cd = EVENT_LEVEL.get(kind, ("INFO", kind.upper()))
            self.logger.event(kind, data, level or lv, code or cd)

    def _done(self, kind: str, r: MotionResult, **info) -> MotionResult:
        self._log(kind, {**info, "status": r.status, "duration_s": r.duration_s})
        return r

    # ---- 동작
    def _comm_gate(self) -> MotionResult | None:
        """통신 끊김 fault에서는 버스를 읽거나 쓰는 일도 하지 않고 fault를 돌려준다(다시 연결하기 전까지 아무 명령도 안 보낸다, decisions/0031)."""
        if self.ctl.state == "fault" and self.ctl.fault == "comm_lost":
            return MotionResult("fault", 0.0, ToolPose(0, 0, 0, 0))
        return None

    def set_speed(self, pct: float) -> None:
        self.speed = pct
        self.skills.speed = pct

    def _run(self, trajs) -> MotionResult:
        return self.skills._run(trajs)

    def home(self) -> MotionResult:
        return self._comm_gate() or self._done("home", self.skills.home())

    def park(self) -> MotionResult:
        return self._comm_gate() or self._done("park", self.skills.park())

    def move_joints(self, goal: JointState, speed_pct: float | None = None) -> MotionResult:
        if (gate := self._comm_gate()) is not None:
            return gate
        trajs = [cfg_move_j(self.cfg, self.driver.read_joints(), goal, speed_pct or self.speed)]
        self.driver.set_torque(True)
        return self._done("movej", self._run(trajs), goal=asdict(goal))

    def move_tool(self, pose: ToolPose, speed_pct: float | None = None) -> MotionResult:
        if (gate := self._comm_gate()) is not None:
            return gate
        trajs = plan_goto(self.kin, self.cfg, self.driver.read_joints(), pose, speed_pct or self.speed)
        if trajs is None:
            return self._done("goto", MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0)), goal=asdict(pose))
        self.driver.set_torque(True)
        return self._done("goto", self._run(trajs), goal=asdict(pose))

    def sensor_position(self, q: JointState, offset_mm: dict) -> tuple[float, float, float]:
        """손목 센서의 작업대 좌표(x, y, 작업대 면 위 높이). 센서는 고정 턱 끝에서 도구 프레임 수평축으로 (x, y), 위로 z 떨어져 있다 (ultrasonic-spec 3.3)."""
        off = offset_mm
        ref = self.kin.fk(q)
        tip = np.array([ref.x_mm, ref.y_mm, ref.z_mm]) + self.kin.jaw_tip_offsets(ref.pitch_deg, ref.yaw_deg, q.gripper)[0]
        p = tip + self.kin.tool_axes(q) @ np.array([off["x"], off["y"], 0.0])
        return float(p[0]), float(p[1]), float(tip[2] - self.cfg.table_z_mm + off["z"])

    def _path_pose(self, x: float, y: float, z_mm: float, yaw: float) -> ToolPose:
        """경로 점(작업대 xy, 작업대 면 위 높이)을 팔에 보낼 ToolPose로. 점의 기준은 임시로 손끝(고정 턱 끝)이다: 센서 기준으로 바꾸려면 여기서 센서 오프셋만큼 빼면 된다."""
        return self.point_pose(x, y, z_mm, yaw)

    def arm_links(self, q: JointState | None = None) -> list[tuple[float, float]]:
        """어깨·팔꿈치·손목·손끝의 작업대 xy(mm). 레이더가 팔 자리를 지울 때 쓰는 arm_links_fn (ultrasonic-spec 8.2절)."""
        return self.kin.link_points(q or self.driver.read_joints())

    def follow_path(self, points_mm, tool_z_mm: float, speed_mm_s: float, sensor_offset_mm: dict) -> list[PoseSample]:
        """손끝(고정 턱 끝)을 수직 아래로 둔 채 작업대 좌표 꺾은선(points_mm)을 tool_z_mm 높이로 따라간다 (REQ-023, 블로킹).
        움직이기 전에 접근과 모든 구간을 계획하고 안전 검사까지 마쳐서, 하나라도 못 가면 아무것도 움직이지 않고 빈 목록을 돌려준다.
        sensor_offset_mm({x, y, z})은 config/ultrasonic.yaml의 arm_sensor_offset_mm를 호출하는 쪽이 넘긴다(값을 여기 두지 않는다). points_mm는 임시로 손끝 xy다(_path_pose 한 곳에서 처리, 기획 답에 따라 바뀔 수 있다).
        반환: 구간을 따라가는 동안의 50Hz 제어 틱마다 PoseSample(PC 단조 시계 perf_counter_ns, 센서 위치). 결과(ok, ik_fail, out_of_bounds+reason, stopped…)는 self.last_path.
        shortcut: 센서 위치는 명령 관절값에서 계산한다(서보 추종 지연 무시, 읽기는 10Hz뿐). 실제 팔에서 지연이 문제면 틱에서 읽는다.
        shortcut: 꺾이는 점에서 한 번 멈춘다(구간마다 따로 실행). 속도는 min(요청, tool_v_max × 속도%)."""
        pts = [(float(x), float(y)) for x, y in points_mm]
        if len(pts) < 2:
            raise ValueError("점이 2개 이상 있어야 합니다")
        if speed_mm_s <= 0:
            raise ValueError("속도는 0보다 커야 합니다")
        if (gate := self._comm_gate()) is not None:
            self.last_path = gate
            return []
        v = min(speed_mm_s, self.cfg.tool_v_max_mm_s * self.speed / 100)
        q0 = self.driver.read_joints()
        yaw = self.kin.fk(q0).yaw_deg  # 손목 회전은 지금 그대로
        poses = [self._path_pose(x, y, tool_z_mm, yaw) for x, y in pts]
        approach = plan_goto(self.kin, self.cfg, q0, poses[0], self.speed)
        fail = None if approach is not None else "ik_fail"
        segs, q = [], approach[-1].sample(approach[-1].duration_s) if approach else None
        for a, b in zip(poses, poses[1:]) if approach else []:
            tr = move_l(self.kin, a, b, q, v, self.cfg.tool_a_max_mm_s2, profile=self.cfg.trajectory_profile, j_mm_s3=self.cfg.tool_jerk_max_mm_s3)
            if tr is None:
                fail = "ik_fail"
                break
            segs.append(tr)
            q = tr.sample(tr.duration_s)
        reason = None
        for tr in (approach or []) + segs:
            if fail is None and (reason := self.safety.check_path(tr)):
                fail = "out_of_bounds"
        info = {"points": pts, "tool_z_mm": tool_z_mm, "speed_mm_s": v}
        if fail:
            self.last_path = MotionResult(fail, 0.0, ToolPose(0, 0, 0, 0), reason=reason)
            self._log("follow_path", {**info, "status": fail, "reason": reason})
            return []
        self.driver.set_torque(True)
        r = self._run(approach)
        raw: list[tuple[int, JointState]] = []
        if r.status == "ok":
            self.ctl.tap = lambda t, qq: raw.append((t, qq))
            try:
                r = self._run(segs)
            finally:
                self.ctl.tap = None
        self.last_path = r
        samples = [PoseSample(t, self.sensor_position(qq, sensor_offset_mm)) for t, qq in raw]
        self._log("follow_path", {**info, "status": r.status, "samples": len(samples)})
        return samples

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

    def get_params(self) -> dict:
        """조정할 수 있는 파라미터: {이름: {value, default, timing}}. timing = now | next_motion | reconnect (REQ-004)."""
        return _params.get_params(self)

    def set_params(self, changes: dict) -> dict:
        """{이름: 값}을 한꺼번에 검증해 적용한다. 틀린 값이 있으면 ValueError(아무것도 안 바뀜). {이름: 적용 시점}을 돌려준다."""
        return _params.set_params(self, changes)

    def save_preset(self, name: str):
        return _params.save_preset(self, name)

    def load_preset(self, name: str) -> dict:
        return _params.load_preset(self, name)

    def list_presets(self) -> list[str]:
        return _params.list_presets(self)

    def run_diagnostic(self, name: str, progress=None, stop=None, **kw) -> dict:
        """servoinfo | linkwatch | powertest | shaketest를 결과 dict, 진행 콜백, 중단과 함께 실행한다 (블로킹, REQ-006). 자세한 인자는 diagnostics.py."""
        return _diag.run_diagnostic(self, name, progress, stop, **kw)

    def preview_tool(self, pose: ToolPose):
        """움직이지 않고 목표가 가능한지만 본다. (이유 또는 None, 풀린 관절값). GUI가 점 위에 마우스를 올릴 때 미리보기로 쓴다."""
        q = self.skills._ik(pose)
        if q is None:
            return "ik_fail", None
        return self.safety.check_goal(q), q

    def preview_path(self, pose: ToolPose, speed_pct: float | None = None, dt: float = 0.05) -> dict:
        """움직이지 않고 move_tool(pose)이 따라갈 경로를 계산한다 (REQ-008). 3D 경로 선용.
        {"ok", "reason"(안전 검사에 걸린 이유 또는 ik_fail 또는 None), "duration_s", "joints": [{"t", "q"(JointState)}], "tool": [{"t", "x_mm", "y_mm", "z_mm"}]}.
        t는 전체 경로의 시작부터 초. tool은 작업대 좌표 mm(z는 move_tool과 같은 모델 z). 직선으로 못 가면 move_tool처럼 위에서 내려오는 두 구간이다."""
        trajs = plan_goto(self.kin, self.cfg, self.driver.read_joints(), pose, speed_pct or self.speed)
        if trajs is None:
            return {"ok": False, "reason": "ik_fail", "duration_s": 0.0, "joints": [], "tool": []}
        reason, joints, tool, t0 = None, [], [], 0.0
        for tr in trajs:
            reason = reason or self.safety.check_path(tr)
            n = max(1, round(tr.duration_s / dt))
            for i in range(n + 1):
                t = tr.duration_s * i / n
                q = tr.sample(t)
                f = self.kin.fk(q)
                joints.append({"t": t0 + t, "q": q})
                tool.append({"t": t0 + t, "x_mm": f.x_mm, "y_mm": f.y_mm, "z_mm": f.z_mm})
            t0 += tr.duration_s
        return {"ok": reason is None, "reason": reason, "duration_s": t0, "joints": joints, "tool": tool}

    def pick_and_place(self, obj: DetectedObject, zone: str | None = None) -> MotionResult:
        if (gate := self._comm_gate()) is not None:
            return gate
        self.driver.set_torque(True)
        r = self.skills.pick(obj)
        self._log("pick", {"object": asdict(obj), "status": r.status})
        if r.status == "ok" and zone:
            r = self.skills.place(zone)
            self._log("place", {"zone": zone, "status": r.status})
        return r

    def place(self, zone: str) -> MotionResult:
        if (gate := self._comm_gate()) is not None:
            return gate
        self.driver.set_torque(True)
        return self._done("place", self.skills.place(zone), zone=zone)

    def set_gripper(self, value: float, as_width: bool = False) -> MotionResult:
        """그리퍼만 움직인다. as_width면 벌림 폭(mm)으로 해석한다(예: 카드 폭 54mm)."""
        if (gate := self._comm_gate()) is not None:
            return gate
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        v = float(np.interp(value, [p[0] for p in pts], [p[1] for p in pts])) if as_width else value
        lo, hi = self.cfg.joint_limits_deg["gripper"]
        q = self.driver.read_joints()
        if min(max(v, lo), hi) > q.gripper + 0.5:  # 지금보다 여는 명령: 예압 고정을 풀고 조임 한계를 원래 값으로
            self.skills.release_grip_hold()
        self.driver.set_torque(True)
        return self._done("gripper", self._run([cfg_move_j(self.cfg, q, replace(q, gripper=min(max(v, lo), hi)), self.speed)]), value=v)

    def jog(self, mode: str, velocity) -> bool:
        """조그 한 걸음. 토크가 꺼져 있으면 먼저 켠다. 이미 켠 뒤에는 버스를 읽지 않으므로 10Hz로 불러도 된다 (REQ-012).
        밖에서 토크가 꺼진 경우(전원 끊김 등)는 알 수 없다: 그때는 connect를 다시 하거나 move_*를 한 번 부른다."""
        if not self._torque_on:
            if not self.driver.torque_enabled():
                self.driver.set_torque(True)
            self._torque_on = True
        return self.ctl.jog(mode, velocity)

    def record_start(self, hz: int | None = None, max_s: float | None = None) -> None:
        """hz: 기록 주기, max_s: 최대 기록 시간. 안 주면 설정값(record_hz, record_max_s)."""
        self._log("record_start", {"hz": hz, "max_s": max_s})
        self._torque_on = False  # 기록 중 토크가 꺼진다
        self.teach.record_start(hz, max_s)

    def record_info(self, name: str) -> dict:
        """기록의 길이, 평균·최대 속도, 속도 구간(멈춤/느림/빠름) 목록. GUI가 구간별 속도 조절 화면을 그릴 때 쓴다."""
        return self.teach.info(name)

    def record_stop(self, name: str):
        path = self.teach.record_stop(name)
        self._log("record_stop", {"name": name, "path": str(path)})
        return path

    def replay(self, name: str, speed: float = 1.0, until_limit: bool = False, segment_speeds: dict[int, float] | None = None) -> MotionResult:
        self.driver.set_torque(True)
        return self._done("replay", self.teach.replay(name, speed, until_limit, segment_speeds), name=name, speed=speed,
                          until_limit=until_limit, segment_speeds=segment_speeds)

    def run_sequence(self, name: str) -> MotionResult:
        return self.replay(name, 1.0)

    def retarget(self, goal: JointState):
        """진행 중인 동작을 멈추지 않고 새 목표로 바꾼다. 돌려받은 핸들의 wait()가 결과(ok)를 준다."""
        return self.ctl.retarget(goal, self.cfg.joint_v_max_deg_s * self.speed / 100, self.cfg.a_max_deg_s2, self.cfg.joint_jerk_max_deg_s3)

    def stop(self) -> None:
        self.ctl.stop()

    def estop(self) -> None:
        self.ctl.estop()  # 이벤트(ARM-ESTOP)와 사고 묶음은 컨트롤러가 알린다

    def resume(self) -> bool:
        """stop·estop(상태 stopped)이나 fault 뒤에 다시 움직일 수 있게 idle로 되돌린다. 팔은 움직이지 않는다 (REQ-009). 통신 끊김 fault는 풀리지 않고 False(다시 연결해야 한다, decisions/0031)."""
        ok = self.ctl.resume()
        self._log("resume" if ok else "resume_refused", {} if ok else {"reason": "comm_lost: 다시 연결해야 한다"}, None if ok else "WARN", None if ok else "ARM-RESUME-REFUSED")
        return ok
