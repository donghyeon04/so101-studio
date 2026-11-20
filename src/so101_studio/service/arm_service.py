"""ArmService: GUI·CLI가 팔을 부르는 유일한 입구 (arm-software-spec.md 6.9). 이름과 입출력은 GUI 기술이 정해져도 바뀌지 않는다."""
import math
import threading
import time
import traceback
import warnings
from collections import deque
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from ..arm.collision import SceneCollision
from ..arm.kinematics import Kinematics
from ..arm.motion import MotionController
from ..arm.reset_probe import classify as classify_torque_loss
from ..arm.reset_probe import decode_status
from ..arm.safety import MODES, Safety, SafetySettings
from ..arm.skills import SEEDS, Skills
from ..arm.teach import Teach
from ..arm.trajectory import cfg_move_j, cfg_move_l, smooth_l
from ..core import frames
from ..core.blackbox import install_excepthooks
from ..core.calibration import zero_check
from ..core.clock import WallClock
from ..core.config import GRIPPER_TYPES, RobotConfig, WorkspaceConfig, load_workspace
from ..core.incident import Incident, create_incident
from ..core.logging import RunLogger
from ..core.scene import Scene, load_scene
from ..core.types import ArmStatus, DetectedObject, JointState, MotionResult, ToolPose
from ..perception.base import PoseSample
from . import diagnostics as _diag
from . import params as _params
from .targets import point_to_pose, tip_vector

STATUS_HZ = 10
HEALTH_HZ = 2
LOAD_PROBE_PCT = 50.0  # 어느 관절이든 부하가 이 값 이상이거나 상태 루프가 오류를 내면 그 순간 토크·상태 비트·부하를 바로 한 번 더 읽어 남긴다 (결정 0030 보강)
PROBE_COOLDOWN_S = 1.0
PLAN_SLOW_BASE_S, PLAN_SLOW_FACTOR = 30.0, 20.0  # follow_path 계획 시간 상한 = 30초 + 20 × 길이/속도. 넘으면 계획이 터진 것으로 보고 움직이지 않는다 (REQ-066)
INCIDENT_COOLDOWN_S = 10.0  # 같은 이유의 사고 묶음은 이 시간 안에 다시 만들지 않는다(연달아 나는 fault·E-STOP이 묶음을 쏟아내지 않게)
# 이벤트 종류별 기본 수준과 코드(기록 형식은 decisions/0030). 나머지는 INFO, 코드는 종류 이름 대문자
EVENT_LEVEL = {"estop": ("WARN", "ARM-ESTOP"), "locked": ("WARN", "ARM-LOCKED"), "connect_failed": ("ERROR", "ARM-CONNECT-FAILED")}


def plan_goto(kin, cfg, q0: JointState, goal: ToolPose, speed_pct: float):
    """직선(MoveL)으로 못 가면 목표 위 호버 지점으로 MoveJ한 뒤 수직으로 내려간다(pick과 같은 순서).
    호버 높이는 놓기(Skills._heights)처럼 approach_height_mm부터 5mm씩 10mm까지 낮춰 처음 풀리는 것(REQ-089: 새 받침 table_z -2.9에서 높이 55mm 목표는 호버 40이 안 닿는다)."""
    traj = cfg_move_l(cfg, kin, kin.fk(q0), goal, q0, speed_pct)
    if traj is not None:
        return [traj]
    for h in [float(h) for h in range(int(cfg.approach_height_mm), 9, -5)] or [float(cfg.approach_height_mm)]:
        hover = replace(goal, z_mm=goal.z_mm + h)
        # 현재 관절로 풀리면 그 답을 쓰고(기존 동작), 못 풀면 Skills._ik처럼 대표 자세들을 씨앗으로 쓴다: 손목을 접은 자세(radar_clear)에서 출발하면 현재 관절만으로는 대부분 실패한다 (REQ-030)
        for seed in [q0, *(replace(sd, gripper=q0.gripper) for sd in SEEDS)]:
            q_hover = kin.ik(hover, seed)
            if q_hover is None:
                continue
            down = cfg_move_l(cfg, kin, hover, goal, q_hover, speed_pct)
            if down is not None:
                return [cfg_move_j(cfg, q0, q_hover, speed_pct), down]
    return None


SERVO_OPEN_UNITS = 3.0  # servo_joints: 접촉 위치보다 이만큼(그리퍼 눈금) 넘게 열어야 여는 목표로 본다
SERVO_OPEN_N = 3  # 여는 목표가 이만큼 이어져야(25Hz로 0.12초) 예압 고정을 푼다


class _SimBlocked:
    """막힘 판정이 읽는 설정: sim_blocked에 있는 키는 그 값, 나머지는 살아 있는 cfg 값."""

    def __init__(self, cfg):
        self._cfg = cfg

    def __getattr__(self, name):
        return self._cfg.sim_blocked.get(name, getattr(self._cfg, name))


class ArmService:
    def __init__(self, driver, cfg: RobotConfig, ws: WorkspaceConfig | None = None, speed_pct: float | None = None,
                 log_root: str | None = "runs", proc: str | None = None, clock=None):
        if bad := [k for k in cfg.sim_blocked if not k.startswith("blocked_") or not hasattr(cfg, k)]:
            raise ValueError(f"sim_blocked에 모르는 키: {bad}")
        self.driver, self.cfg = driver, cfg
        self.ws = ws or load_workspace()
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default
        self.saved_params = {**_params._scalars(cfg), _params.SPEED: cfg.speed_pct_default}  # 설정 파일 값(합친 cfg, set_params로 바꾸기 전): get_params의 saved, GUI의 [기본값으로]
        self.log_root = log_root
        self.collision = None
        self.scene_error: str | None = None  # config/scene.yaml을 못 읽었을 때의 이유(빈 장면으로 시작한다)
        try:
            self.scene = load_scene(ws=self.ws).for_gripper(cfg.gripper)  # 장면 부품 (config/scene.yaml, 없으면 빈 목록). reload_workspace가 다시 읽는다
        except Exception as e:  # noqa: BLE001 장면 파일 하나가 앱 시작을 막지 않게
            self.scene, self.scene_error = Scene(), str(e)
            warnings.warn(f"장면 부품 목록을 못 읽어 빈 장면으로 시작한다: {e}", stacklevel=2)
        self.preset_dir = "config/presets"
        self.proc = proc  # 기록 머리의 proc(cli | gui). 없으면 환경변수 SO101_PROC, 기본 cli
        self.config_dir = Path("config")  # 캘리브레이션 사본·팔 파일을 찾고 사고 묶음에 설정 사본을 담는 곳
        self.incident_after_s = 60.0  # 사고 묶음에서 뒤 60초를 모으는 시간(시험은 0)
        self.install_hooks = not getattr(driver, "simulated", False)  # 처리 안 된 예외 감시. 시뮬레이션은 쓰지 않는다
        self.incidents: list[Incident] = []
        self._incident_last: dict[str, float] = {}
        self._incident_thread: threading.Thread | None = None
        self._status_errors: dict[tuple, int] = {}
        self._torque_lost = False
        self._volt_hist: deque = deque(maxlen=120)  # (시각, 전압 V): 풀림 직전 전압 진단용(2Hz 표본 1분)
        self._probe_t = -1e9
        self._restore_hooks = None
        self._hard_lock = self._zero_lock = None
        self.logger: RunLogger | None = None
        self.kin = self.safety = self.ctl = self.skills = self.teach = None
        self.port: str | None = None  # 마지막으로 연결한 포트
        self.sensor_offset_mm: dict | None = None  # 손목 센서 오프셋. 초음파 설정(arm_sensor_offset_mm)을 호출하는 쪽이 set_sensor_offset으로 한 번 넘긴다
        self.last_path: MotionResult | None = None  # 마지막 follow_path의 결과(status, reason)
        self._callbacks: list = []
        self._status_thread: threading.Thread | None = None
        self._stop_status = threading.Event()
        self._clock = clock or WallClock()  # 제어 시계(mujoco-spec 13.10.2). MotionController·Skills가 쓴다
        self._status_n = 0  # 상태 한 바퀴 수(2Hz 건강 읽기 셈)
        self._health = {"temperature_c": {}, "load_pct": {}, "voltage_v": 0.0, "current_ma": 0.0}
        self._connected = False

    # ---- 연결
    def connect(self, port: str | None = None, fresh: bool = False) -> tuple[bool, str]:
        """연결하고 컨트롤러·기능을 만든다. 실패하면 (False, 오류 메시지). 이미 연결돼 있으면(통신 끊김 fault 뒤 다시 연결 등) 먼저 정리한다.
        다시 연결해도 토크는 끄지 않고, 새 컨트롤러가 현재 위치를 명령값으로 삼는다 (decisions/0031).
        fresh=True면 집기 기억·알려진 물체·놓은 물체 기억을 넘기지 않는다(세계를 다시 지을 때, mujoco-spec 13.10.4, REQ-093)."""
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
        if self.collision is None:  # URDF 메쉬는 한 번만 읽는다(재연결 때 또 읽지 않는다)
            self.collision = SceneCollision(self.kin, self.cfg.urdf, self.ws, self.cfg.table_z_mm, self.cfg.tip_floor_mm)
        self.collision.set_scene(self.scene, self.ws)
        self.safety.collision = self.collision
        self.ctl = MotionController(self.driver, self.kin, self.safety, self.cfg.control_hz, clock=self._clock)
        old = self.skills
        self.skills = Skills(self.ctl, self.kin, self.cfg, self.ws, self.speed)
        if getattr(self.driver, "physical_objects", False) and self.cfg.sim_blocked:  # MuJoCo는 추종 오차가 작아 판정할 때만 막힘 문턱을 시뮬레이션 값(robot.yaml sim_blocked)으로 본다. cfg·프리셋 저장값은 실제 설정 그대로
            self.ctl.blocked_cfg = _SimBlocked(self.cfg)
        if old is not None and not fresh:  # 다시 연결하며 Skills를 새로 만들어도 집기 기억과 알려진 물체는 넘긴다 (11.3.3)
            self.skills.grip, self.skills.placed, self.skills.known_objects = old.grip, old.placed, old.known_objects
            self.skills.known_height = old.known_height
            self.skills.placed_size = old.placed_size
            self.skills.placed_height, self.skills.placed_angle = old.placed_height, old.placed_angle  # 놓은 물체의 높이·각도도(GUI [다시 연결] 뒤 다시 집을 때 각도 0으로 집지 않게)
            self.skills.last_place_yaw_deg, self.skills.last_place_angle_deg = old.last_place_yaw_deg, old.last_place_angle_deg
        self.teach = Teach(self.ctl, self.cfg)
        self.ctl.on_event = self._ctl_event
        self._health = self.driver.read_health()
        self._connected = True
        self._torque_lost = False
        if self.install_hooks:
            self._restore_hooks = install_excepthooks(self._unhandled)
        self._hard_lock = self._lock_reason()  # 캘리브레이션 불일치나 팔 파일 없음 (decisions/0027)
        self._zero_lock = None if self._hard_lock else self._zero_lock_reason()  # 0° 자세 확인 전 (decisions/0029)
        self.ctl.locked_reason = self._hard_lock or self._zero_lock  # 움직이는 명령을 막고 읽기만 허용한다
        if self.ctl.locked_reason:
            self._log("locked", {"reason": self.ctl.locked_reason})
        self._servo_open_n = 0  # servo_joints가 여는 목표를 이어 받은 수
        self._torque_on = False  # jog가 토크를 켠 뒤 true. 교시로 토크를 끄면 다시 false (REQ-012)
        self._stop_status.clear()
        self._log("connect", {"port": "sim" if getattr(self.driver, "simulated", False) else port or self.cfg.port, **getattr(self.driver, "connect_info", {})})  # 시뮬레이션이면 실제 포트처럼 보이지 않게
        self._write_status(self.status())  # 연결하자마자 첫 줄: 연결 뒤 상태가 비지 않게
        self._status_n = 0
        if self._clock.drives_status:  # 시뮬레이션 시계(StepClock)는 상태 한 바퀴를 시뮬레이션 시간 10Hz로 돌린다(스레드 없음)
            self._clock.every(1.0 / STATUS_HZ, self._status_once)
        else:
            self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
            self._status_thread.start()
        return True, ""

    @property
    def clock(self):
        """제어 시계(읽기 전용). 기본 WallClock."""
        return self._clock

    def _lock_reason(self) -> str | None:
        if getattr(self.driver, "simulated", False):
            return None
        if getattr(self.driver, "locked_reason", None):
            return self.driver.locked_reason
        if not self.cfg.arm_name:
            return "팔 파일이 없다: config/arms/_template.yaml을 복사해 채우기 전에는 움직이지 않는다"
        return None

    def _zero_lock_reason(self) -> str | None:
        if getattr(self.driver, "simulated", False):
            return None
        z = zero_check(self.cfg, self.config_dir)
        return None if z["state"] == "checked" else f"0° 자세 확인 필요: {z['reason']}. so101 calibrate --verify-zero로 확인하기 전에는 움직이지 않는다"

    def unlock_for_verification(self) -> bool:
        """so101 calibrate --verify-zero만 쓴다: 0° 자세 확인 때문에 걸린 잠금만 푼다. 캘리브레이션 불일치·팔 파일 없음은 풀지 않는다. 풀렸으면 True."""
        self.ctl.locked_reason = self._hard_lock
        return self._hard_lock is None

    def relock_zero(self) -> None:
        self.ctl.locked_reason = self._hard_lock or self._zero_lock

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
            state="teach" if self.teach.recording else self.ctl.state, fault=self.ctl.fault, fault_text=self.ctl.fault_text, cmd=self.ctl._last,
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
                "tick_dt_max_ms": round(ctl.tick_dt_max_1s() * 1000, 2),
                "log_write_errors": getattr(self.logger, "write_errors", 0), "log_dropped": getattr(self.logger, "dropped_total", 0)}  # 기록이 빠지고 있는지 GUI가 보여 준다

    def _write_status(self, st) -> None:
        self.ctl.actual_hint = st.joints  # 링 버퍼가 쓰는 가장 최근 실제 값
        self.ctl.check_blocked(st.joints, self.speed / 100)  # 추종 오차로 막힘·접촉 감지 (결정 0059)
        if self.logger:
            self.logger.status(self._status_data(st))

    def _status_loop(self) -> None:
        nxt = time.perf_counter()
        while not self._stop_status.is_set():
            nxt += 1.0 / STATUS_HZ
            self._status_once()
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _status_once(self) -> None:
        """상태 한 바퀴(10Hz): 2Hz로 건강 읽기, 상태 쓰기·콜백, 통신 성공·실패 알림."""
        if self._stop_status.is_set():
            return
        health_tick = self._status_n % (STATUS_HZ // HEALTH_HZ) == 0
        self._status_n += 1
        try:  # 통신 오류는 컨트롤러가 fault로 처리한다. 여기서는 조용히 버리지 않고 기록한 뒤 계속 돈다
            if health_tick:
                expect = getattr(self.driver, "expect_torque", None)  # 읽기 전에 정한다: 읽는 사이에 켠 토크를 풀림으로 오해하지 않게
                self._health = self.driver.read_health()  # 온도·부하·토크·Status는 2Hz로만 읽는다
                self._volt_hist.append((time.monotonic(), self._health.get("voltage_v", 0.0)))
                self._check_torque(expect)
                if max(self._health.get("load_pct", {}).values(), default=0.0) >= LOAD_PROBE_PCT:
                    self._probe("high_load")
            st = self.status()
            self.ctl.note_comm(True)
            self._write_status(st)
            for cb in list(self._callbacks):
                try:
                    cb(st)
                except Exception as e:  # noqa: BLE001 GUI 콜백의 코드 오류는 통신 끊김이 아니다: 따로 기록하고(첫 번째는 traceback) 통신 상태는 건드리지 않는다
                    self._status_error(e, "status_callback_error")
            if health_tick:  # 오류가 나는 읽기는 2Hz뿐이라, 그 읽기가 다시 성공해야 회복으로 본다
                self._status_ok()
        except Exception as e:  # noqa: BLE001
            self.ctl.note_comm(False)  # 정지 중에는 50Hz 쓰기가 없어 이 읽기가 끊김을 가장 먼저 본다
            self._status_error(e)
            self._probe("status_loop_error")

    def _probe(self, why: str) -> None:
        """과부하·읽기 오류 순간에 2Hz 주기를 기다리지 않고 토크·상태 비트·부하를 바로 한 번 더 읽어 ARM-LOAD-PROBE로 남긴다.
        (서보 보호로 출력이 끊겨도 Torque_Enable 값만으로는 안 보일 수 있어, 그 순간의 Status 비트가 필요하다.) PROBE_COOLDOWN_S에 한 번."""
        if self.ctl is not None and self.ctl.state == "fault" and self.ctl.fault == "comm_lost":
            return  # 통신이 끊긴 동안은 읽어 볼 수 없다: 끊김은 ARM-COMM-LOST 하나로 이미 남았다 (1초마다 경고가 쌓이던 것)
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

    def _status_error(self, e: Exception, kind: str = "status_loop_error") -> None:
        """첫 발생은 전체 traceback, 반복은 횟수(10, 100, 1000번째)로 남긴다. kind = status_loop_error(상태 읽기) | status_callback_error(GUI 콜백)."""
        key = (kind, type(e).__name__, str(e)[:80])
        code = "ARM-STATUS-CALLBACK-ERROR" if kind == "status_callback_error" else "ARM-STATUS-LOOP-ERROR"
        n = self._status_errors[key] = self._status_errors.get(key, 0) + 1
        if n == 1:
            self._log(kind, {"error": repr(e), "traceback": traceback.format_exc()}, "ERROR", code)
        elif n in (10, 100, 1000, 10000):
            self._log(kind, {"error": repr(e), "count": n}, "WARN", code)

    def _status_ok(self) -> None:
        errs = {k: v for k, v in self._status_errors.items() if k[0] == "status_loop_error"}  # 콜백 오류는 상태 읽기가 회복돼도 계속되므로 지우지 않는다(매번 traceback을 쏟지 않게)
        if errs:
            self._log("status_loop_recovered", {"errors": {f"{k[1]}: {k[2]}": v for k, v in errs.items()}}, "INFO", "ARM-STATUS-LOOP-RECOVERED")
            for k in errs:
                del self._status_errors[k]

    def _check_torque(self, expect) -> None:
        """2Hz로 읽은 Torque_Enable이 컨트롤러가 기대하는 값(켜져 있어야 함)과 다르면 ARM-TORQUE-LOST."""
        torque = self._health.get("torque")
        lost = bool(expect) and bool(torque) and any(not v for v in torque.values())
        if lost and not self._torque_lost:
            self._log("torque_lost", {"expected": True, "read": torque, "status_bits": self._health.get("status_bits"), "state": self.ctl.state, "fault": self.ctl.fault,
                                      "load_pct": self._health.get("load_pct")}, "ERROR", "ARM-TORQUE-LOST")
            self._diagnose_torque_loss(torque)
            self._trigger_incident("ARM-TORQUE-LOST", "켜져 있어야 할 토크가 꺼져 읽혔다")
        self._torque_lost = lost

    def _diagnose_torque_loss(self, torque: dict) -> None:
        """토크가 풀린 것을 읽은 직후 한 번: 서보 응답·ID·Torque_Enable·리셋 흔적(Acceleration)·Status를 읽고 '리셋(전원)인지 개별 보호인지' 한 줄 이벤트(ARM-TORQUE-LOSS-DIAG)로 남긴다 (REQ-048 D6).
        직전 전압은 2Hz 건강 읽기 표본이라 ms 단위 강하는 못 잡는다(그건 powertest의 160Hz 표본이 맡는다)."""
        probe = getattr(self.driver, "probe_reset", None)
        now = time.monotonic()
        recent = [v for t, v in self._volt_hist if now - t <= 10.0]
        base = {"voltage_min_10s_v": min(recent) if recent else None, "voltage_last_v": recent[-1] if recent else None, "voltage_samples_10s": len(recent),
                "n_torque_off": sum(1 for v in torque.values() if not v)}
        if probe is None:
            self._log("torque_loss_diag", {**base, "verdict": "unknown", "reasons": ["드라이버가 probe_reset을 지원하지 않음"]}, "ERROR", "ARM-TORQUE-LOSS-DIAG")
            return
        try:
            info = probe()
            verdict, reasons = classify_torque_loss(info, base["n_torque_off"])
        except Exception as e:  # noqa: BLE001 진단 실패도 기록하되 상태 루프는 계속
            self._log("torque_loss_diag", {**base, "verdict": "unknown", "reasons": [f"진단을 읽지 못함: {e!r}"[:160]]}, "ERROR", "ARM-TORQUE-LOSS-DIAG")
            return
        responded = sum(1 for ok in info["ping"].values() if ok)
        self._log("torque_loss_diag", {**base, "verdict": verdict, "reasons": reasons, "responded": responded, "ids": info["ids"], "ping": info["ping"], "torque": info["torque"],
                                       "acceleration": info["acceleration"], "expected_acceleration": info["expected_acceleration"], "status": info["status"],
                                       "status_names": {n: decode_status(v) for n, v in info["status"].items()}, "voltage_dv": info["voltage_dv"]}, "ERROR", "ARM-TORQUE-LOSS-DIAG")

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
            return MotionResult("fault", 0.0, ToolPose(0, 0, 0, 0), reason=self.ctl.fault_text)
        return None

    def enable_torque(self) -> bool:
        """움직이는 명령이 토크를 켜는 공통 입구. 잠긴 팔(캘리브레이션 불일치·팔 파일 없음, CLAUDE.md: 잠김이면 읽기만 된다)에는 켜지 않고 False를 돌려준다(REQ-027).
        드라이버의 set_torque 잠금은 캘리브레이션 불일치만 알고 팔 파일 없음 잠금(ctl.locked_reason)은 모르므로 서비스에서 막는다. 직접 driver.set_torque(True)를 부르지 않는다."""
        if self.ctl is None or self.ctl.locked_reason:
            return False
        if getattr(self.driver, "expect_torque", None) is True and not self._torque_lost:
            return True  # 이미 켜져 있다: 조그 keepalive마다 다시 쓰지 않는다(ARM-TORQUE-ON이 0.2초마다 쌓였다). 풀린 것을 읽었을 때(_torque_lost)만 다시 켠다
        self.driver.set_torque(True)
        return True

    def set_speed(self, pct: float) -> None:
        self.speed = pct
        self.skills.speed = pct

    def _run(self, trajs) -> MotionResult:
        return self.skills._run(trajs)

    def home(self) -> MotionResult:
        return self._comm_gate() or self._done("home", self._with_zero_lock_open(self.skills.home))

    def _with_zero_lock_open(self, move):
        """0° 자세 확인 잠금(decisions/0029)만 걸려 있으면 home은 허용한다: --verify-zero가 팔을 작업대 밖으로 뻗은 채 끝났을 때 12V를 끄는 것 말고 내릴 길이 없어지지 않게.
        캘리브레이션 불일치·팔 파일 없음 잠금은 그대로 막는다. 정지·E-STOP 상태는 resume() 뒤에만 움직인다."""
        if self._hard_lock is None and self._zero_lock is not None and self.ctl.locked_reason == self._zero_lock:
            self.ctl.locked_reason = None
            try:
                return move()
            finally:
                self.ctl.locked_reason = self._zero_lock
        return move()

    def park(self) -> MotionResult:
        return self._comm_gate() or self._done("park", self.skills.park())

    def move_joints(self, goal: JointState, speed_pct: float | None = None) -> MotionResult:
        if (gate := self._comm_gate()) is not None:
            return gate
        trajs = [cfg_move_j(self.cfg, self.driver.read_joints(), goal, speed_pct or self.speed)]
        self.enable_torque()
        return self._done("movej", self._run(trajs), goal=asdict(goal))

    def move_tool(self, pose: ToolPose, speed_pct: float | None = None) -> MotionResult:
        if (gate := self._comm_gate()) is not None:
            return gate
        trajs = plan_goto(self.kin, self.cfg, self.driver.read_joints(), pose, speed_pct or self.speed)
        if trajs is None:
            return self._done("goto", MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0)), goal=asdict(pose))
        self.enable_torque()
        return self._done("goto", self._run(trajs), goal=asdict(pose))

    def reload_workspace(self, path: str | Path = "config/workspace.yaml") -> WorkspaceConfig:
        """작업대 설정(workspace.yaml: 작업대 크기, 팔 원점, 배출 구역, 도달 반경)을 다시 읽어 Skills·안전 검사·기구학이 새 값을 쓰게 한다 (REQ-034).
        동작이나 조그가 진행 중이면 거부한다(RuntimeError). 파일을 못 읽으면 아무것도 바뀌지 않는다. 작업 실행(Studio)의 스레드는 Studio.reload_workspace가 막는다."""
        if self.ctl is None:
            raise RuntimeError("연결되지 않음")
        if self.ctl.busy:
            raise RuntimeError("동작 중에는 작업대 설정을 다시 읽을 수 없다")
        if getattr(self.driver, "physical_objects", False):
            raise RuntimeError("MuJoCo 시뮬레이션은 작업대 설정을 바꾸면 다시 시작해야 합니다")
        ws = load_workspace(path)
        try:  # 장면을 먼저 읽는다: 실패하면 아무것도 바꾸지 않는다
            scene = load_scene(Path(path).parent / "scene.yaml", ws).for_gripper(self.cfg.gripper)
        except Exception as e:
            raise RuntimeError(f"장면 부품 목록을 읽지 못했다: {e}") from e
        self.ws = self.skills.ws = self.safety.ws = ws
        self.kin.base = ws.base
        self.scene, self.scene_error = scene, None
        self.collision.set_scene(scene, ws)
        self._log("workspace_reloaded", {"base": asdict(ws.base), "table_mm": ws.table_mm, "drop_zones": list(ws.drop_zones)})
        return ws

    def set_sensor_offset(self, offset_mm: dict) -> None:
        """손목 초음파 센서가 고정 턱 끝에서 떨어진 위치({x, y, z}, config/ultrasonic.yaml의 arm_sensor_offset_mm). 값은 이 서비스에 두지 않고 호출하는 쪽이 넘긴다.
        x·y는 도구 프레임 수평축(x = 턱이 벌어지는 축), z는 턱 끝 위 높이 (ultrasonic-spec 3.3)."""
        self.sensor_offset_mm = {"x": float(offset_mm["x"]), "y": float(offset_mm["y"]), "z": float(offset_mm["z"])}

    def tool_position(self, q: JointState) -> tuple[float, float, float]:
        """손끝의 작업대 좌표(x, y, 작업대 면 위 높이). 손끝 = 단동식은 고정 턱 끝, 병렬은 두 턱 끝 가운데(tip_vector와 같은 기준, follow_path의 점도 이것)."""
        ref = self.kin.fk(q)
        tip = np.array([ref.x_mm, ref.y_mm, ref.z_mm]) + tip_vector(self.kin, ref.pitch_deg, ref.yaw_deg, q.gripper, "tip")
        return float(tip[0]), float(tip[1]), float(tip[2] - self.cfg.table_z_mm)

    def sensor_position(self, q: JointState) -> tuple[float, float, float]:
        """손목 센서의 작업대 좌표 = 기준점 + 손목 yaw로 돌린 센서 오프셋(set_sensor_offset).
        기준점은 tool_position: 단동식은 고정 턱 끝, 병렬은 두 턱 끝 가운데(센서는 손에 고정이라 벌림에 따라 움직이면 안 된다: 실제 CAD 오프셋이 두 턱 끝 가운데 기준)."""
        if self.sensor_offset_mm is None:
            raise ValueError("set_sensor_offset으로 센서 오프셋을 먼저 넘겨야 합니다")
        off = self.sensor_offset_mm
        x, y, h = self.tool_position(q)
        d = self.kin.tool_axes(q) @ np.array([off["x"], off["y"], 0.0])
        return float(x + d[0]), float(y + d[1]), float(h + off["z"])

    def arm_links(self, q: JointState | None = None) -> list[tuple[float, float]]:
        """어깨·팔꿈치·손목·손끝의 작업대 xy(mm). 레이더가 팔 자리를 지울 때 쓰는 arm_links_fn (ultrasonic-spec 8.2절)."""
        return self.kin.link_points(q or self.driver.read_joints())

    def follow_path(self, points_mm, tool_z_mm: float, speed_mm_s: float, yaw_deg: float | None = None) -> list[PoseSample]:
        """손끝(단동식 고정 턱 끝, 병렬 두 턱 끝 가운데)을 수직 아래로 둔 채 작업대 좌표 꺾은선(points_mm)을 tool_z_mm 높이로 따라간다 (REQ-023, 블로킹).
        points_mm는 손끝 xy다(센서 xy가 아니다). yaw_deg는 경로 동안 고정할 손목 각도(None이면 지금 값). 센서 오프셋은 set_sensor_offset으로 먼저 받아 둬야 한다.
        한 번의 호출은 지금 손끝 위치에서 첫 점까지 가는 것까지 이어서 서지 않고 지난다: 처음과 끝에서만 가속·감속하고, 꺾이는 점과 원은 호(모서리를 2.5mm 안으로 깎는 반지름 R)로 감아
        구심 가속이 한도(tool_a_max_mm_s2) 안이 되게 속도를 줄여 지난다(v ≤ sqrt(a/√2·R); 반대로 되돌아가는 180° 꺾임은 서서 돈다). 경로는 호 위 1mm 간격 경유점을 관절 공간 스플라인으로 잇는다(smooth_l). 속도는 min(요청, tool_v_max × 속도%).
        지금 위치에서 첫 점까지 직선으로 갈 수 없으면(IK 실패·관절 도약) 예전처럼 plan_goto의 안전 경로(관절 이동 후 위에서 수직 하강)로 먼저 가고 경로는 그다음에 따로 지난다(이 경우 접근과 경로 사이에서 선다).
        움직이기 전에 접근과 모든 구간을 계획해 안전 검사까지 마치고, 하나라도 못 가면 아무것도 움직이지 않고 빈 목록을 돌려준다.
        반환: 따라가는 동안 50Hz 제어 틱마다 PoseSample(PC 단조 시계 perf_counter_ns, 손끝 위치, 센서 위치). 접근을 합쳤을 때는 접근 구간의 샘플도 들어 있다(tool_mm로 가를 수 있다).
        센서 위치는 그 틱에 읽은 실제 관절값으로 계산한다(그 때문에 이 동안만 틱마다 버스를 한 번 읽는다; 읽는 시간은 follow_path 기록의 read_ms_max·mean, 읽기가 실패하면 0.5초 안에 comm_lost fault).
        결과는 self.last_path(MotionResult)에 남고, 빈 목록이 와도 이것으로 원인을 구분한다:
          "ok": 끝까지 따라감(목록이 비지 않음). "ik_fail": 접근이나 경로 어딘가가 닿지 않아 움직이지 않음(빈 목록). "out_of_bounds": 안전 검사(관절 한계·작업 영역·바닥)에 걸려 움직이지 않음, last_path.reason에 이유(빈 목록).
          "stopped": stop()으로 도중에 멈춤(그때까지의 샘플, 아직 접근 중이었다면 빈 목록). "fault": 통신 끊김 등 fault로 끝남(마찬가지). E-STOP도 진행 중이면 "stopped", 이미 멈춘 뒤 부르면 바로 "stopped"(빈 목록)다.
          "locked": 캘리브레이션 불일치로 잠김(빈 목록). 호출하는 쪽은 빈 목록이면 last_path.status로 "그 구간 못 감"(ik_fail·out_of_bounds)과 "정지·사고"(stopped·fault·locked)를 가른다."""
        pts = [(float(x), float(y)) for x, y in points_mm]
        if len(pts) < 2:
            raise ValueError("점이 2개 이상 있어야 합니다")
        if speed_mm_s <= 0:
            raise ValueError("속도는 0보다 커야 합니다")
        if self.sensor_offset_mm is None:
            raise ValueError("set_sensor_offset으로 센서 오프셋을 먼저 넘겨야 합니다")
        if self.ctl.locked_reason:  # 잠긴 팔(캘리브레이션 불일치·팔 파일 없음)에는 토크를 켜지도 계획하지도 않는다
            self.last_path = MotionResult("locked", 0.0, ToolPose(0, 0, 0, 0))
            self._log("follow_path", {"status": "locked", "reason": self.ctl.locked_reason})
            return []
        if (gate := self._comm_gate()) is not None:
            self.last_path = gate
            return []
        v = min(speed_mm_s, self.cfg.tool_v_max_mm_s * self.speed / 100)
        pre, runs, fail, reason = self._plan_path(pts, tool_z_mm, v, yaw_deg)
        info = {"points": pts, "tool_z_mm": tool_z_mm, "speed_mm_s": v}
        if fail:
            self.last_path = MotionResult(fail, 0.0, ToolPose(0, 0, 0, 0), reason=reason)
            self._log("follow_path", {**info, "status": fail, "reason": reason})
            return []
        self.enable_torque()
        raw: list[tuple[int, JointState, int]] = []
        r = self._run(pre) if pre else MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))
        if r.status == "ok" and runs:
            self.ctl.tap = lambda t, qq, ns: raw.append((t, qq, ns))
            try:
                r = self._run(runs)
            finally:
                self.ctl.tap = None
        self.last_path = r
        samples = [PoseSample(t, self.tool_position(qq), self.sensor_position(qq)) for t, qq, _ in raw]
        reads = [ns / 1e6 for _, _, ns in raw]
        self._log("follow_path", {**info, "status": r.status, "samples": len(samples), "pre": len(pre), "runs": len(runs),
                                  "read_ms_max": round(max(reads, default=0.0), 3), "read_ms_mean": round(sum(reads) / len(reads), 3) if reads else 0.0})
        return samples

    def _plan_path(self, pts, tool_z_mm: float, v: float, yaw_deg: float | None):
        """(접근 궤적 목록, 경로 궤적 목록, 실패 이유 또는 None, 안전 검사 이유). 움직이지 않는다. 지금 위치에서 첫 점까지 직선으로 갈 수 있으면 접근 없이 한 궤적으로 합친다."""
        q0 = self.driver.read_joints()
        cur = self.kin.fk(q0)
        yaw = cur.yaw_deg if yaw_deg is None else float(yaw_deg)  # None이면 손목 회전은 지금 그대로
        tip = tip_vector(self.kin, -90.0, yaw, q0.gripper, "tip")  # 점마다 버스를 읽지 않게 시작에 읽은 그리퍼 값으로 한 번만
        poses = [self._path_pose(x, y, tool_z_mm, yaw, tip) for x, y in pts]
        a = self.cfg.tool_a_max_mm_s2

        def dedupe(ps):  # 같은 자리가 이어지면 하나로(길이 0인 구간은 꺾임 계산을 망친다)
            out = []
            for p in ps:
                if not out or abs(p.x_mm - out[-1].x_mm) + abs(p.y_mm - out[-1].y_mm) + abs(p.z_mm - out[-1].z_mm) > 0.01:
                    out.append(p)
            return out

        path = dedupe(poses)
        if len(path) < 2:
            raise ValueError("경로 점이 모두 같습니다(서로 다른 점이 2개 이상 있어야 합니다)")
        full = dedupe([cur, *poses])
        merged = smooth_l(self.kin, full, q0, v, a) if len(full) > 1 else None
        if merged is not None:
            pre, runs = [], [merged]
        else:
            pre = plan_goto(self.kin, self.cfg, q0, poses[0], self.speed)  # 직선으로 못 가면 위에서 수직으로 내려가는 안전 경로
            if pre is None:
                return [], [], "ik_fail", None
            tr = smooth_l(self.kin, path, pre[-1].sample(pre[-1].duration_s), v, a)
            if tr is None:
                return [], [], "ik_fail", None
            runs = [tr]
        for tr in runs:  # 안전망(REQ-066): 계획 시간이 말이 안 되게 길면(속도 곡선 계산이 터진 경우) 움직이지 않는다. 정상 경로는 길이/속도의 몇 배 안이다. 안전 검사(경로 전체 샘플링)보다 먼저 본다
            if tr.duration_s > PLAN_SLOW_BASE_S + PLAN_SLOW_FACTOR * tr.length / v:
                return [], [], "out_of_bounds", f"plan_too_slow: 계획 시간 {tr.duration_s:.3g}s (길이 {tr.length:.1f}mm, 속도 {v:.0f}mm/s)"
        for tr in pre + runs:
            if why := self.safety.check_path(tr):
                return [], [], "out_of_bounds", why
        return pre, runs, None, None

    def _path_pose(self, x: float, y: float, z_mm: float, yaw: float, tip: np.ndarray) -> ToolPose:
        """경로 점(손끝 xy, 작업대 면 위 높이)을 팔에 보낼 ToolPose로. 센서 경로 → 손끝 경로 변환은 여기서 하지 않는다(check.py 한 곳, ultrasonic-spec 8.2절)."""
        return point_to_pose(x, y, z_mm, yaw, "table", self.ws.base, self.cfg.table_z_mm, tip)

    def point_pose(self, x: float, y: float, h: float, yaw: float = 90.0, frame: str = "table", tcp: str = "tip", pitch: float = -90.0) -> ToolPose:
        """frame 기준 (x, y, 작업대 면 위 높이 h)인 끝점(tcp)을 pitch 자세로 가져갈 ToolPose. pitch: -90 수직 아래(기본), 0 수평(손끝이 팔이 뻗은 쪽을 향함).
        GUI에서 점을 찍을 때 쓰고, preview_tool·move_tool에 넘긴다. 바닥·작업대·관절 검사는 그 자세의 실제 두 턱(메쉬)으로 한다."""
        tip = tip_vector(self.kin, pitch, yaw, self.driver.read_joints().gripper, tcp)
        return point_to_pose(x, y, h, yaw, frame, self.ws.base, self.cfg.table_z_mm, tip, pitch)

    def reach_ranges(self, pitch_deg: float = -90.0, heights_mm=(0, 40, 80, 120, 160, 200, 260), step_mm: float = 10.0, max_r_mm: float = 420.0) -> dict:
        """두 범위를 한 계산(--sim 기구학 IK + 안전 검사)으로 낸다 (REQ-037).
        pick: 집기 범위 = workspace.yaml reach_mm (작업 실행·물체 판정에 쓰는 값, 수직 아래 집기).
        motion: 움직임 범위 = 손끝을 pitch 자세(-90 아래 ~ 0 수평)로 높이별로 보낼 수 있는 팔 앞쪽 반경 구간. 직접 조작·조그·안전 상자 표시에 쓴다.
        {"pitch_deg", "pick": {"min", "max"} (= ws.pick_reach_mm, 키 이름은 min/max), "motion": [{"height_mm", "segments": [[r0, r1], ...], "min_mm", "max_mm"}]}. 반경은 로봇 원점 기준 앞쪽(mm).
        한 번에 수백 번 IK를 풀어 1~3초 걸리고 같은 인자는 캐시한다. 동작 중에는 풀이기를 같이 쓸 수 없어 캐시가 없으면 RuntimeError."""
        key = (round(pitch_deg, 1), tuple(heights_mm), step_mm, max_r_mm, self.cfg.table_z_mm, tuple(sorted(self.ws.base.__dict__.items())))
        cache = self.__dict__.setdefault("_reach_cache", {})
        if key not in cache:
            if self.ctl.busy:
                raise RuntimeError("동작 중에는 움직임 범위를 새로 계산할 수 없다")
            rows = []
            for h in heights_mm:
                segs, start = [], None
                n = int(max_r_mm // step_mm) + 1
                for i in range(n + 1):
                    r = i * step_mm
                    ok = False
                    if i < n:
                        x, y = frames.robot_to_table(r, 0.0, self.ws.base)
                        pose = self.point_pose(x, y, float(h), self.ws.base.yaw_deg, "table", "tip", pitch_deg)
                        q = self.skills._ik(pose)
                        ok = q is not None and self.safety.check_goal(q) is None
                    if ok and start is None:
                        start = r
                    elif not ok and start is not None:
                        segs.append([start, r - step_mm])
                        start = None
                rows.append({"height_mm": h, "segments": segs, "min_mm": segs[0][0] if segs else None, "max_mm": segs[-1][1] if segs else None})
            cache[key] = {"pitch_deg": pitch_deg, "motion": rows}
        return {**cache[key], "pick": dict(self.ws.pick_reach_mm)}

    def gripper_info(self) -> dict:
        """그리퍼 선택 화면용: 지금 종류와 고를 수 있는 종류(결정 0043-1). 종류는 config/local.yaml의 `gripper:`로 고르고 다시 연결(재시작)해야 적용된다."""
        g = self.skills.gripper if self.skills else None
        return {"type": self.cfg.gripper, "available": list(GRIPPER_TYPES), "needs_centering": getattr(g, "needs_centering", None),
                "width_mm": [list(p) for p in self.cfg.gripper_width_mm], "max_open_mm": self.cfg.grasp_max_open_mm}

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

    def _reach_reason(self, label: str, x_mm: float, y_mm: float) -> str | None:
        """작업대 점이 집기 범위(workspace.yaml reach_mm, 팔 원점 기준 반경) 안인지. 밖이면 이유 문장, 안이면 None."""
        xr, yr = frames.table_to_robot(x_mm, y_mm, self.ws.base)
        r, lo, hi = math.hypot(xr, yr), self.ws.pick_reach_mm["min"], self.ws.pick_reach_mm["max"]
        return None if lo <= r <= hi else f"{label}_out_of_reach: 반경 {r:.0f}mm (허용 {lo:.0f}~{hi:.0f}mm)"

    def _rejected(self, kind: str, reason: str, **info) -> MotionResult:
        """움직이지 않고 거부한다. 이유는 MotionResult.reason."""
        return self._done(kind, MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0), reason=reason), **info)

    @property
    def held(self) -> dict | None:
        """쥔 물체(REQ-106): Skills.held. 집기가 닫아 쥘 때 채우고 놓기가 벌리기 시작할 때 비운다. 기구학 엔진 GUI가 손끝에 붙여 그린다(MuJoCo는 물리)."""
        return self.skills.held if self.skills else None

    def pick_and_place(self, obj: DetectedObject, zone: str | None = None, place_at: tuple[float, float] | None = None) -> MotionResult:
        """집은 물체를 배출 구역(zone) 또는 작업대 점(place_at = (x_mm, y_mm))에 놓는다. 둘 다 없으면 집기만.
        집기 점과 놓을 점이 집기 범위 밖이면 아무것도 움직이지 않고 out_of_bounds + reason으로 돌려준다 (REQ-031)."""
        if zone and place_at:
            raise ValueError("zone과 place_at 중 하나만 준다")
        if (gate := self._comm_gate()) is not None:
            return gate
        target = place_at or (None if zone is None else (self.ws.drop_zones[zone]["x_mm"], self.ws.drop_zones[zone]["y_mm"]))
        why = self._reach_reason("pick", obj.x_mm, obj.y_mm) or (target and place_at and self._reach_reason("place", *target))
        if why:
            return self._rejected("pick_and_place", why, object=asdict(obj), place_at=place_at)
        self.enable_torque()
        r = self.skills.pick(obj, self.skills.place_targets(zone, place_at))
        self._log("pick", {"object": asdict(obj), "status": r.status, "grasp": self.skills.last_pick_info})
        if r.status == "ok" and target:
            r = self.skills.place_at(*target) if place_at else self.skills.place(zone)
            self._log("place", {"zone": zone, "at": place_at, "status": r.status, "correction_mm": self._place_correction(), **self._place_angle()})
        return r

    def place(self, zone: str | None = None, at: tuple[float, float] | None = None) -> MotionResult:
        """배출 구역 이름(zone) 또는 작업대 점(at = (x_mm, y_mm))에 놓는다. 점이 집기 범위 밖이면 움직이지 않고 out_of_bounds + reason (REQ-031)."""
        if (zone is None) == (at is None):
            raise ValueError("zone과 at 중 하나만 준다")
        if (gate := self._comm_gate()) is not None:
            return gate
        if at is not None and (why := self._reach_reason("place", *at)):
            return self._rejected("place", why, at=at)
        self.enable_torque()
        r = self.skills.place_at(*at) if at is not None else self.skills.place(zone)
        return self._done("place", r, zone=zone, at=at, correction_mm=self._place_correction(), **self._place_angle())

    def _place_correction(self) -> list[float] | None:
        """마지막 place가 두 턱 끝 중간점을 구역 점에 맞추려고 손 위치를 옮긴 양(mm, x·y). 기록용."""
        c = getattr(self.skills, "last_place_correction_mm", None)
        return None if c is None else [round(c[0], 1), round(c[1], 1)]

    @property
    def last_placed_angle_deg(self) -> float | None:
        """마지막 place가 놓은 물체의 작업대 기준 각도(0~180°, 정사각형은 90° 주기). 모르면 None. 놓은 물체를 다시 집을 때 이 각도를 DetectedObject.angle_deg로 준다
        (놓는 손은 반경 방향 턱 축이라 집을 때와 달라 물체가 돌아 놓인다: 실물 2-2 11-13, 각도 0으로 알려 정사각형 상자의 모서리를 집음)."""
        return getattr(self.skills, "last_place_angle_deg", None)

    def placed_angle_at(self, x_mm: float, y_mm: float, tol_mm: float = 40.0) -> float | None:
        """(x, y) 근처(tol_mm 안, 가장 가까운 것)에 이 세션이 놓은 물체의 작업대 기준 각도(0~180°). 없거나 모르면 None. 여러 개를 놓은 자동 작업·GUI가 앞서 놓은 물체를 다시 집을 때 쓴다."""
        near = min(self.skills.placed_angle.items(), key=lambda kv: math.hypot(kv[0][0] - x_mm, kv[0][1] - y_mm), default=None)
        return near[1] if near is not None and math.hypot(near[0][0] - x_mm, near[0][1] - y_mm) <= tol_mm else None

    def _place_angle(self) -> dict:
        y, a = getattr(self.skills, "last_place_yaw_deg", None), self.last_placed_angle_deg
        return {**({} if y is None else {"place_yaw_deg": round(y, 1)}), **({} if a is None else {"placed_angle_deg": round(a, 1)})}

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
        self.enable_torque()
        return self._done("gripper", self._run([cfg_move_j(self.cfg, q, replace(q, gripper=min(max(v, lo), hi)), self.speed * self.skills._gripper_speed())]), value=v)

    def jog(self, mode: str, velocity) -> bool:
        """조그 한 걸음. 토크가 꺼져 있으면 먼저 켠다. 이미 켠 뒤에는 버스를 읽지 않으므로 10Hz로 불러도 된다 (REQ-012).
        밖에서 토크가 꺼진 경우(전원 끊김 등)는 알 수 없다: 그때는 connect를 다시 하거나 move_*를 한 번 부른다."""
        self._torque_once()
        return self.ctl.jog(mode, velocity)

    def _torque_once(self) -> None:
        if not self._torque_on and not self.ctl.locked_reason:
            if not self.driver.torque_enabled():
                self.enable_torque()
            self._torque_on = True

    def servo_joints(self, goal: JointState) -> bool:
        """정책이 낸 절대 관절 목표 하나 (arm-software-spec 6.10). 25Hz로 불러도 버스를 읽지 않는다. False면 이유는 ctl.servo_reason.
        12/4 전에는 시뮬레이션에서만 쓴다: 실제 팔 드라이버면 거부한다(결정 0066의 4항)."""
        if not getattr(self.driver, "simulated", False):
            self.ctl.servo_reason = "real_arm"
            return False
        hold = self.ctl.grip_hold
        # 정책 출력은 주기마다 1~2 흔들린다: 접촉 위치(고정 + 예압)보다 SERVO_OPEN_UNITS 넘게 여는 목표가 SERVO_OPEN_N번 이어질 때만 예압 고정을 푼다(set_gripper와 같은 풀기)
        opening = hold is not None and goal.gripper > hold + self.cfg.grip_preload_units + SERVO_OPEN_UNITS
        self._servo_open_n = self._servo_open_n + 1 if opening else 0
        if self._servo_open_n >= SERVO_OPEN_N:
            self._servo_open_n = 0
            self.skills.release_grip_hold()
        self._torque_once()
        return self.ctl.servo(goal, self.cfg.joint_v_max_deg_s * self.speed / 100)

    def record_start(self, hz: int | None = None, max_s: float | None = None) -> None:
        """hz: 기록 주기, max_s: 최대 기록 시간. 안 주면 설정값(record_hz, record_max_s)."""
        self._log("record_start", {"hz": hz, "max_s": max_s})
        self._torque_on = False  # 기록 중 토크가 꺼진다
        self.teach.record_start(hz, max_s)

    def record_info(self, name: str, speed: float = 1.0, segment_speeds: dict[int, float] | None = None) -> dict:
        """기록의 길이, 평균·최대 속도, 속도 구간(멈춤/느림/빠름) 목록. GUI가 구간별 속도 조절 화면을 그릴 때 쓴다.
        info["fit"] = {fits_1x, slowed_segments, slowed_samples, added_s}: 이 배율로 그대로(original) 재생해도 한도 안인지와 변환 재생(fitted)이 늦추는 구간 수·늘어나는 시간."""
        return self.teach.info(name, speed, segment_speeds)

    def record_stop(self, name: str):
        path = self.teach.record_stop(name)
        self._log("record_stop", {"name": name, "path": str(path)})
        return path

    def replay(self, name: str, speed: float = 1.0, until_limit: bool = False, segment_speeds: dict[int, float] | None = None,
               mode: str = "original", any_gripper: bool = False) -> MotionResult:
        """교시 재생. 기록 그리퍼가 지금과 다르면 토크도 켜지 않고 stopped(gripper_mismatch)로 거부한다(any_gripper=True면 재생, REQ-113). GUI는 확인 창 뒤 any_gripper=True로 부른다."""
        if (refused := self.teach.gripper_refusal(name, any_gripper)) is not None:
            return self._done("replay", refused, name=name, any_gripper=any_gripper)
        self.enable_torque()
        return self._done("replay", self.teach.replay(name, speed, until_limit, segment_speeds, mode, any_gripper=True), name=name, speed=speed,
                          until_limit=until_limit, segment_speeds=segment_speeds, mode=mode, any_gripper=any_gripper)

    def run_sequence(self, name: str) -> MotionResult:
        return self.replay(name, 1.0)

    def retarget(self, goal: JointState):
        """진행 중인 동작을 멈추지 않고 새 목표로 바꾼다. 돌려받은 핸들의 wait()가 결과(ok)를 준다."""
        return self.ctl.retarget(goal, self.cfg.joint_v_max_deg_s * self.speed / 100, self.cfg.a_max_deg_s2, self.cfg.joint_jerk_max_deg_s3)

    def stop(self) -> None:
        self.ctl.stop()

    def estop(self) -> str | None:
        """즉시 정지. 위치 고정을 서보에 쓰지 못했으면 이유(통신 오류), 아니면 None."""
        return self.ctl.estop()  # 이벤트(ARM-ESTOP)와 사고 묶음은 컨트롤러가 알린다

    def resume(self) -> bool:
        """stop·estop(상태 stopped)이나 fault 뒤에 다시 움직일 수 있게 idle로 되돌린다. 팔은 움직이지 않는다 (REQ-009). 통신 끊김 fault는 풀리지 않고 False(다시 연결해야 한다, decisions/0031)."""
        ok = self.ctl.resume()
        self._log("resume" if ok else "resume_refused", {} if ok else {"reason": "comm_lost: 다시 연결해야 한다"}, None if ok else "WARN", None if ok else "ARM-RESUME-REFUSED")
        return ok
