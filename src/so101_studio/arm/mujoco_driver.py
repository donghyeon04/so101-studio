"""MuJoCo 물리 드라이버 (arm-software-spec 13.2·13.3, 결정 0053, M2).

`SimDriver`와 같은 메서드로 하드웨어 자리에 끼운다. 물리는 백그라운드 스레드가 벽시계에 맞춰 돌린다(제어 주기와 무관).
`realtime=False`면 스레드가 없고 `advance(seconds)`만 시간을 흘린다(시험용)."""
import threading
import time

import numpy as np

from ..core.types import JointState
from .driver import JOINTS
from .kinematics import VALUE_TO_Q
from .mujoco_model import JOINT_MAP, load_model

CATCHUP_MAX_S = 0.1  # 이 이상 밀리면 넘는 만큼 버린다


class MuJoCoDriver:
    def __init__(self, cfg, ws, *, scene=None, objects=(), realtime=True, viewer=False, scenario=None):
        self.cfg, self.ws, self.scene, self.objects, self.realtime = cfg, ws, scene, list(objects), realtime
        self.viewer = viewer  # M4
        self.connected = False
        self.acceleration = 10
        self.simulated = True
        self.locked_reason: str | None = None
        self.event_sink = None
        self.expect_torque: bool | None = None
        self.torque_limit = 1000
        self.physical_objects = True
        self.connect_info = {"engine": "mujoco", "scenario": scenario}
        self.dropped_count, self.dropped_s, self.step_ms_avg = 0, 0.0, 0.0
        self.model = self.data = None
        self._lock = threading.RLock()
        self._pending: list[tuple] = []
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._failed: str | None = None
        self._torque = False
        self._target = np.zeros(6)  # 우리 관절 순서, MuJoCo 각도(rad)

    # -- 연결 --
    def connect(self, port: str = "sim", calibration_id: str = "sim") -> None:
        import mujoco

        self.disconnect()  # 잠금 밖에서: 물리 스레드가 잠금을 기다리며 join과 맞물리지 않게
        with self._lock:
            self.model, warnings = load_model(self.scene, self.objects, base=self.ws.base, table_z_mm=self.cfg.table_z_mm, joint_limits=self.cfg.physical_limits_deg or None)
            self.data = mujoco.MjData(self.model)
            self._mj = mujoco
            self._qadr = [self.model.joint(JOINT_MAP[j][0]).qposadr[0] for j in JOINTS]
            self._aid = [self.model.actuator(JOINT_MAP[j][0]).id for j in JOINTS]
            self._force0 = self.model.actuator_forcerange.copy()  # 부하 %의 분모(그리퍼 한계를 줄이기 전 값)
            self._failed = None
            self.torque_limit = 1000
            home = self._deg_to_rad(self.cfg.poses["home"])
            self.data.qpos[self._qadr] = home
            self._target = home
            self.data.ctrl[self._aid] = home
            mujoco.mj_forward(self.model, self.data)
            self._warn_base = self._warn_count()
            self._torque = True
            self.model.opt.disableactuator = 0
            self.connected = True
            for code in warnings:
                self._emit("sim", "WARN", code, {})
            self._wall0, self._sim0 = time.perf_counter(), 0.0
            self._stop.clear()
            if self.realtime:
                self._thread = threading.Thread(target=self._run, name="mujoco-physics", daemon=True)
                self._thread.start()

    def disconnect(self) -> None:
        self._stop.set()
        t, self._thread = self._thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2.0)
        self.connected = False
        self.acceleration = 10

    # -- 변환 --
    def _deg_to_rad(self, deg) -> np.ndarray:
        """우리 관절값(도, 그리퍼는 0~100 값) 6개 -> MuJoCo 관절각(rad)."""
        out = []
        for j, v in zip(JOINTS, deg):
            _, sign, off = JOINT_MAP[j]
            if j == "gripper":
                v = VALUE_TO_Q[0] + VALUE_TO_Q[1] * v
            out.append(np.radians(sign * v + off))
        return np.array(out)

    def _rad_to_state(self, rad) -> JointState:
        vals = []
        for j, r in zip(JOINTS, rad):
            _, sign, off = JOINT_MAP[j]
            v = (np.degrees(r) - off) / sign
            vals.append(float((v - VALUE_TO_Q[0]) / VALUE_TO_Q[1]) if j == "gripper" else float(v))
        return JointState(*vals)

    # -- 물리 --
    def _warn_count(self) -> int:
        return sum(int(w.number) for w in self.data.warning)

    def _fail(self, reason: str) -> None:
        if self._failed is None:
            self._failed = reason
            self._emit("sim", "ERROR", "SIM-PHYSICS-FAIL", {"reason": reason})

    def _step_n(self, n: int) -> None:
        """잠금을 쥐고 n 스텝. 예외·경고 증가·NaN이면 물리 실패."""
        t0 = time.perf_counter()
        try:
            for _ in range(n):
                self._mj.mj_step(self.model, self.data)
        except Exception as e:  # noqa: BLE001
            self._fail(f"스텝 예외 {e!r}"[:120])
            return
        if n:
            self.step_ms_avg += ((time.perf_counter() - t0) * 1000.0 / n - self.step_ms_avg) * min(1.0, n / 500.0)
        if not np.isfinite(self.data.qpos).all():
            self._fail("관절값이 NaN")
        elif self._warn_count() > self._warn_base:
            self._fail("MuJoCo 경고 카운터 증가")

    def advance(self, seconds: float) -> None:
        """realtime=False에서 시간을 흘린다."""
        with self._lock:
            if self._failed is None:
                self._step_n(round(seconds / self.model.opt.timestep))

    def _run(self) -> None:
        dt = self.model.opt.timestep
        while not self._stop.is_set() and self._failed is None:
            with self._lock:
                lag = (time.perf_counter() - self._wall0) - (self.data.time - self._sim0)
                if lag > CATCHUP_MAX_S:
                    self._wall0 += lag - CATCHUP_MAX_S
                    self.dropped_count += 1
                    self.dropped_s += lag - CATCHUP_MAX_S
                    self._emit("sim", "WARN", "SIM-PHYSICS-DROP", {"dropped_s": round(lag - CATCHUP_MAX_S, 3)})
                    lag = CATCHUP_MAX_S
                if lag >= dt:
                    self._step_n(int(lag / dt))
            time.sleep(0.001)

    def _emit(self, kind: str, level: str, code: str, data: dict) -> None:
        """event_sink가 아직 없으면 모아 두었다가 처음 보낼 수 있을 때 함께 보낸다."""
        self._pending.append((kind, level, code, data))
        sink = self.event_sink
        if sink is None:
            return
        pending, self._pending = self._pending, []
        for ev in pending:
            try:
                sink(*ev)
            except Exception:  # noqa: BLE001, S110 기록 실패가 물리를 막지 않는다
                pass

    def _check(self) -> None:
        if self._failed is not None:
            raise RuntimeError(f"물리 시뮬레이션이 멈췄습니다: {self._failed}")

    # -- 드라이버 메서드 --
    def read_joints(self) -> JointState:
        with self._lock:
            self._check()
            return self._rad_to_state(self.data.qpos[self._qadr])

    def write_joints(self, q: JointState) -> None:
        with self._lock:
            self._check()
            self._target = self._deg_to_rad([getattr(q, j) for j in JOINTS])
            if self._torque:
                self.data.ctrl[self._aid] = self._target

    def torque_enabled(self) -> bool:
        return self._torque

    def set_torque(self, on: bool) -> None:
        """모든 관절에 한 번에. 끄면 액추에이터 힘이 0(중력으로 처짐), 켜면 그 순간의 관절값을 목표로 잡는다."""
        with self._lock:
            if on:
                self._target = self.data.qpos[self._qadr].copy()
                self.data.ctrl[self._aid] = self._target
            self.model.opt.disableactuator = 0 if on else 1  # 액추에이터 그룹 0 전체
            self._torque = on
            self.expect_torque = on

    def set_acceleration(self, value: int) -> None:
        self.acceleration = value

    def set_gripper_torque_limit(self, value: int) -> None:
        with self._lock:
            self.torque_limit = int(value)
            a = self._aid[JOINTS.index("gripper")]
            self.model.actuator_forcerange[a] = self._force0[a] * (self.torque_limit / 1000.0)

    def gripper_torque_limit(self) -> int:
        return self.torque_limit

    def read_servo_info(self) -> dict:
        return {m: {"Acceleration": self.acceleration} for m in JOINTS}

    def _load_pct(self) -> dict:
        with self._lock:
            return {m: float(abs(self.data.actuator_force[a]) / self._force0[a][1] * 100.0) if self._torque else 0.0 for m, a in zip(JOINTS, self._aid)}

    def read_health(self) -> dict:
        self._check()
        load = self._load_pct()
        ma = {m: v * 10.0 for m, v in load.items()}
        return {
            "temperature_c": {m: 25.0 for m in JOINTS},
            "load_pct": load,
            "voltage_v": 12.0,
            "current_ma": float(sum(ma.values())),
            "current_ma_by_joint": ma,
            "torque": {m: int(self._torque) for m in JOINTS},
            "status_bits": {m: 0 for m in JOINTS},
        }

    def read_power(self) -> tuple[float, dict, dict]:
        self._check()
        load = self._load_pct()
        return 12.0, {m: v * 10.0 for m, v in load.items()}, load

    def probe_reset(self) -> dict:
        exp = {m: self.acceleration for m in JOINTS}
        return {"ping": {m: True for m in JOINTS}, "torque": {m: int(self._torque) for m in JOINTS}, "acceleration": dict(exp),
                "torque_limit": {m: 1000 for m in JOINTS}, "status": {m: 0 for m in JOINTS}, "voltage_dv": {m: 120 for m in JOINTS},
                "ids": {m: i + 1 for i, m in enumerate(JOINTS)}, "expected_acceleration": exp}

    def sim_state(self) -> dict:
        """M2: 물체·접촉은 없다(M3에서 채운다)."""
        with self._lock:
            return {"t_s": float(self.data.time), "objects": {}, "contacts": [], "held": None}
