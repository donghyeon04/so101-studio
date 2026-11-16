"""MuJoCo 물리 드라이버 (arm-software-spec 13.2·13.3, 결정 0053, M2).

`SimDriver`와 같은 메서드로 하드웨어 자리에 끼운다. 물리는 백그라운드 스레드가 벽시계에 맞춰 돌린다(제어 주기와 무관).
`realtime=False`면 스레드가 없고 `advance(seconds)`만 시간을 흘린다(시험용)."""
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from ..core.types import JointState
from .driver import JOINTS
from .kinematics import VALUE_TO_Q
from .mujoco_hand import slide_m, value_of_slide
from .mujoco_model import (
    DEFAULT_OBJECT_HEIGHT_MM,
    JOINT_MAP,
    geom_roles,
    load_model,
    model_to_table_mm,
)

JAW_ROLES = ("jaw_fixed", "jaw_moving", "clamp_right", "clamp_left")  # held 판정에 쓰는 턱 역할(단동식 둘, 병렬 클램프 둘)
VIEWER_SYNC_S = 1 / 15  # 뷰어에 복사본을 보내는 간격(30Hz는 렌더가 CPU를 먹어 60초에 버림 45~64번, 15Hz는 1~4번: measurements.md)
CATCHUP_MAX_S = 0.1  # 이 이상 밀리면 넘는 만큼 버린다


class MuJoCoDriver:
    def __init__(self, cfg, ws, *, scene=None, objects=(), realtime=True, viewer=False, scenario=None):
        if viewer and not realtime:
            raise ValueError("뷰어는 실시간 모드에서만")
        self.cfg, self.ws, self.objects, self.realtime = cfg, ws, list(objects), realtime
        self.scene = scene.for_gripper(cfg.gripper) if scene is not None else None  # REQ-069: cfg 그리퍼와 다른 `gripper:` 부품은 건너뛴다
        self.viewer = viewer
        self._parallel = cfg.gripper == "parallel"  # 로보나인 병렬 손(M4b): 그리퍼 관절이 slide(m)
        self._handle = None  # mujoco.viewer 핸들(뷰어를 켠 경우)
        self._next_sync = 0.0
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
        self._rdata = self._renderer = None  # render(): 물리 데이터 복사본과 화면 밖 렌더러. connect·disconnect에서 버린다(모델이 새로 만들어진다)
        self._rpool: ThreadPoolExecutor | None = None  # 렌더 전용 스레드(처음 render 때 만든다)
        self._from, self._t_write = np.zeros(6), 0.0  # 목표가 바뀐 시각(시뮬레이션 시간): 액추에이터 목표를 다음 쓰기 간격(1/control_hz) 동안 선형으로 옮긴다(13.2)

    # -- 연결 --
    def connect(self, port: str = "sim", calibration_id: str = "sim") -> None:
        import mujoco

        self.disconnect()  # 잠금 밖에서: 물리 스레드가 잠금을 기다리며 join과 맞물리지 않게
        with self._lock:
            self.model, warnings = load_model(self.scene, self.objects, base=self.ws.base, table_z_mm=self.cfg.table_z_mm, joint_limits=self.cfg.physical_limits_deg or None,
                                              hand=self.cfg.gripper, gripper_width_mm=self.cfg.gripper_width_mm, table_mm=self.ws.table_mm)
            self.data = mujoco.MjData(self.model)
            self._mj = mujoco
            self._qadr = [self.model.joint(JOINT_MAP[j][0]).qposadr[0] for j in JOINTS]
            self._aid = [self.model.actuator(JOINT_MAP[j][0]).id for j in JOINTS]
            self._force0 = self.model.actuator_forcerange.copy()  # 부하 %의 분모(그리퍼 한계를 줄이기 전 값)
            self._failed = None
            self.torque_limit = 1000
            home = self._deg_to_rad(self.cfg.poses["home"])
            self.data.qpos[self._qadr] = home
            self._target = self._from = home
            self.data.ctrl[self._aid] = home
            mujoco.mj_forward(self.model, self.data)
            self._warn_base = self._warn_count()
            self._torque = True
            self.model.opt.disableactuator = 0
            self.connected = True
            self._roles = geom_roles(self.model)
            self._site = self.model.site("gripper_frame").id
            self._objs = {o.name: self.model.body(f"obj:{o.name}").id for o in self.objects}
            self._half_h = {o.name: (o.height_mm if o.height_mm > 0 else DEFAULT_OBJECT_HEIGHT_MM) / 2000.0 for o in self.objects}
            self._size = {o.name: [o.length_mm, o.width_mm, o.height_mm if o.height_mm > 0 else DEFAULT_OBJECT_HEIGHT_MM] for o in self.objects}
            for code, data in warnings:
                self._emit("sim", "WARN", code, data)
            self._wall0, self._sim0 = time.perf_counter(), 0.0
            if self.viewer:
                import mujoco.viewer

                self._vdata = mujoco.MjData(self.model)  # 뷰어는 물리 데이터의 복사본을 본다: sync가 화면 그리기를 기다리는 동안 물리 스레드가 멈추지 않게(M4b 뒤 측정: 60초에 버림 184번)
                self._handle = mujoco.viewer.launch_passive(self.model, self._vdata, show_left_ui=False, show_right_ui=False)  # 메인 스레드가 아니어도 열린다(13.9)
                self._sync_evt = threading.Event()
            self._stop.clear()
            if self.realtime:
                self._thread = threading.Thread(target=self._run, name="mujoco-physics", daemon=True)
                self._thread.start()
            if self._handle is not None:  # _stop을 지운 뒤에 시작해야 바로 끝나지 않는다
                threading.Thread(target=self._viewer_loop, name="mujoco-viewer-sync", daemon=True).start()

    def disconnect(self) -> None:
        self._stop.set()
        t, self._thread = self._thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2.0)
        h, self._handle = self._handle, None
        if h is not None:
            h.close()
        self.connected = False
        self.acceleration = 10
        self._drop_renderer()

    def _drop_renderer(self) -> None:
        if self._rpool is not None:
            self._rpool.submit(self._drop_in_render_thread).result()

    def _drop_in_render_thread(self) -> None:
        r, self._renderer, self._rdata = self._renderer, None, None
        if r is not None:
            r.close()

    def render(self, camera: str, width: int = 320, height: int = 240) -> np.ndarray:
        """카메라 `camera`(top·wrist)의 화면 밖 그림 (높이, 폭, 3) uint8(13.10.4). 연결 전이면 RuntimeError, 모르는 카메라면 ValueError. 화면 밖 버퍼는 640×480까지.
        GL 문맥은 만든 스레드에서만 쓸 수 있어(Windows: 다른 스레드에서 부르면 WGL "리소스 사용 중") 그리기는 드라이버의 렌더 전용 스레드 하나가 한다. 어느 스레드에서 불러도 된다."""
        if self._rpool is None:
            with self._lock:
                if self._rpool is None:
                    self._rpool = ThreadPoolExecutor(1, thread_name_prefix="mujoco-render")
        return self._rpool.submit(self._render_in_thread, camera, width, height).result()

    def _render_in_thread(self, camera: str, width: int, height: int) -> np.ndarray:
        with self._lock:  # 잠금 안에서는 복사만: 그리기가 물리 스레드를 막지 않게
            if not self.connected or self.model is None:
                raise RuntimeError("연결 전에는 render할 수 없다")
            m = self.model
            if self._mj.mj_name2id(m, self._mj.mjtObj.mjOBJ_CAMERA, camera) < 0:
                raise ValueError(f"모르는 카메라: {camera!r}")
            if self._rdata is None:
                self._rdata = self._mj.MjData(m)
            self._mj.mj_copyData(self._rdata, m, self.data)
        r = self._renderer
        if r is None or (r.width, r.height) != (width, height):
            if r is not None:
                r.close()
            r = self._renderer = self._mj.Renderer(m, height, width)
        r.update_scene(self._rdata, camera=camera)
        return r.render()

    @property
    def failed_reason(self) -> str | None:
        """물리가 멈췄으면 그 이유, 아니면 None(다시 connect하면 None). 상태 루프가 예외로 멈추므로 GUI는 이것으로 감지한다(13.2)."""
        return self._failed

    # -- 변환 --
    def _deg_to_rad(self, deg) -> np.ndarray:
        """우리 관절값(도, 그리퍼는 0~100 값) 6개 -> MuJoCo 관절각(rad)."""
        out = []
        for j, v in zip(JOINTS, deg):
            _, sign, off = JOINT_MAP[j]
            if j == "gripper" and self._parallel:  # 병렬 손: 클램프 한 쪽 이동(m)
                out.append(slide_m(v, self.cfg.gripper_width_mm))
                continue
            if j == "gripper":
                v = VALUE_TO_Q[0] + VALUE_TO_Q[1] * v
            out.append(np.radians(sign * v + off))
        return np.array(out)

    def _rad_to_state(self, rad) -> JointState:
        vals = []
        for j, r in zip(JOINTS, rad):
            _, sign, off = JOINT_MAP[j]
            if j == "gripper" and self._parallel:
                vals.append(value_of_slide(float(r), self.cfg.gripper_width_mm))
                continue
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
        if not np.isfinite(self.data.qpos).all():  # 스텝 전에 본다: NaN을 먹은 MuJoCo는 데이터를 되돌리고 경고만 남긴다
            self._fail("관절값이 NaN")
            return
        t0 = time.perf_counter()
        try:
            for _ in range(n):
                if self._torque:  # kp가 커서 목표가 계단으로 바뀌면 힘이 한계까지 튄다: 쓰기 간격 동안 선형으로 옮긴다(실물 서보의 내부 궤적 근사)
                    frac = min(1.0, (self.data.time - self._t_write) / (1.0 / self.cfg.control_hz))
                    self.data.ctrl[self._aid] = self._from + (self._target - self._from) * frac
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
                h = self._handle
                if h is not None and time.perf_counter() >= self._next_sync and h.is_running():  # 창을 닫으면 복사만 멈추고 물리는 계속한다
                    self._next_sync = time.perf_counter() + VIEWER_SYNC_S
                    self._mj.mj_copyData(self._vdata, self.model, self.data)  # 복사는 빠르다. 뷰어 쪽 마우스 힘은 복사본에만 있어 물리에 안 들어간다
                    self._sync_evt.set()
            time.sleep(0.001)

    def _viewer_loop(self) -> None:
        """복사본을 뷰어에 보낸다(sync가 그리기를 기다릴 수 있어 물리 잠금 밖의 별도 스레드). 한 장면이 늦어도 다음 복사가 덮어쓴다."""
        while not self._stop.is_set():
            if self._sync_evt.wait(0.1):
                self._sync_evt.clear()
                h = self._handle
                if h is not None and h.is_running():
                    h.sync()

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
            if self._torque:
                self._from = self.data.ctrl[self._aid].copy()
                self._t_write = self.data.time
            self._target = self._deg_to_rad([getattr(q, j) for j in JOINTS])

    def torque_enabled(self) -> bool:
        return self._torque

    def set_torque(self, on: bool) -> None:
        """모든 관절에 한 번에. 끄면 액추에이터 힘이 0(중력으로 처짐), 켜면 그 순간의 관절값을 목표로 잡는다."""
        with self._lock:
            if on:
                self._target = self._from = self.data.qpos[self._qadr].copy()
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
        """13.2: 부하 % = |액추에이터 힘| / 원래 forcerange."""
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

    def _table(self, p_m) -> np.ndarray:
        """모델 좌표 점(m) -> 작업대 좌표 (x, y, 작업 면 위 높이) mm."""
        return np.array(model_to_table_mm(*(np.asarray(p_m) * 1000.0), self.ws.base, self.cfg.table_z_mm))

    def _table_dir(self, p_m, v) -> np.ndarray:
        """모델 좌표의 방향 v -> 작업대 좌표의 단위 방향."""
        d = self._table(np.asarray(p_m) + 0.1 * np.asarray(v)) - self._table(p_m)
        return d / np.linalg.norm(d)

    def sim_state(self) -> dict:
        """13.6: 시뮬레이션 전용 읽기. 좌표는 작업대 좌표 mm, z는 작업 면 위 높이."""
        with self._lock:
            d, m = self.data, self.model
            objects = {}
            for name, b in self._objs.items():
                R = d.xmat[b].reshape(3, 3)
                x, y, z = self._table(d.xpos[b] - R[:, 2] * self._half_h[name])  # 상자 바닥 가운데
                ax = self._table_dir(d.xpos[b], R[:, 0])
                objects[name] = {"x_mm": float(x), "y_mm": float(y), "z_mm": float(z), "yaw_deg": math.degrees(math.atan2(ax[1], ax[0])),
                                 "tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2])))), "size_mm": self._size[name]}
            site = d.site_xpos[self._site]
            R = d.site_xmat[self._site].reshape(3, 3)
            tool = {"pos_mm": self._table(site).tolist(), "rot": np.stack([self._table_dir(site, R[:, i]) for i in range(3)], axis=1).tolist()}
            contacts, jaw_force = [], {}
            f6 = np.zeros(6)
            for i in range(d.ncon):
                c = d.contact[i]
                self._mj.mj_contactForce(m, d, i, f6)
                a, b = self._roles[c.geom1], self._roles[c.geom2]
                contacts.append({"a": a, "b": b, "force_n": float(f6[0]), "pos_mm": self._table(c.pos).tolist()})
                if f6[0] > 0:
                    for jaw, other in ((a, b), (b, a)):
                        if jaw in JAW_ROLES and other.startswith("obj:"):
                            jaw_force.setdefault(other[4:], {}).setdefault(jaw, 0.0)
                            jaw_force[other[4:]][jaw] += float(f6[0])
            both = {o: sum(f.values()) for o, f in jaw_force.items() if len(f) == 2}
            return {"t_s": float(d.time), "objects": objects, "tool": tool, "contacts": contacts, "held": max(both, key=both.get) if both else None}
