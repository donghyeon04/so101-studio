"""기능: home, park, pick, place (arm-software-spec.md 6.6). 동작 계획은 MoveJ(관절 보간)와 MoveL(직선)의 조합이다."""
import math
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .gripper import SingleJawGripper
from .trajectory import cfg_move_j, cfg_move_l

SEEDS = [JointState(0, -30, 30, 60, 0, 0), JointState(0, -60, 60, 30, 0, 0), JointState(0, 0, 60, 30, 0, 0)]


class Skills:
    def __init__(self, ctl, kin, cfg, ws, speed_pct: float | None = None):
        self.ctl, self.kin, self.cfg, self.ws = ctl, kin, cfg, ws
        self.gripper = kin.gripper = SingleJawGripper(kin, cfg)  # 그리퍼 종류별 기하·보정·경로 지식은 arm/gripper.py (work-queue 5-3)
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default
        self.opening_fallback_reason: str | None = None  # 마지막 _opening_path가 None이었던 이유
        self.last_place_correction_mm: tuple[float, float] | None = None  # 마지막 place가 손 xy를 옮긴 양
        self._grip_orig: int | None = None  # 조임 한계를 바꾸기 전 값(놓은 뒤 되돌린다)
        ctl.grip_warn_pct, ctl.grip_load_s, ctl.grip_open = cfg.gripper_load_warn_pct, cfg.gripper_load_s, cfg.gripper_open  # 그리퍼 부하 감시 (결정 0033)

    def _set_grip_torque(self, value: int) -> None:
        if self._grip_orig is None:
            self._grip_orig = self.ctl.driver.gripper_torque_limit()
        self.ctl.driver.set_gripper_torque_limit(value)

    def release_grip_hold(self) -> None:
        """그리퍼를 여는 명령 앞에서: 예압 고정을 풀고 조임 한계를 원래 값으로 되돌린다. 정지·E-STOP은 이것을 부르지 않는다(결정 0033)."""
        self._restore_grip_torque()

    def _restore_grip_torque(self) -> None:
        """조임 한계를 원래 값으로 되돌리고 그리퍼 목표 고정을 푼다. 아무것도 안 바꿨으면 아무것도 쓰지 않는다."""
        self.ctl.hold_gripper(None)
        if self._grip_orig is not None:
            orig, self._grip_orig = self._grip_orig, None
            self.ctl.driver.set_gripper_torque_limit(orig)

    def _q(self) -> JointState:
        return self.ctl.driver.read_joints()

    def _run(self, trajs) -> MotionResult:
        """궤적을 차례로 실행한다. 하나라도 ok가 아니면 거기서 멈추고 그 결과를 돌려준다."""
        r = None
        for t in trajs:
            r = self.ctl.execute(t).wait()
            if r.status != "ok":
                break
        return r

    def _joint_move(self, goal: JointState):
        return cfg_move_j(self.cfg, self._q(), goal, self.speed)

    def _gripper_speed(self) -> float:
        """그리퍼만 움직이는 동작의 속도 배율(0.01~1). 설정 gripper_speed_pct를 그때그때 읽어 바로 적용한다."""
        return min(max(self.cfg.gripper_speed_pct, 1.0), 100.0) / 100.0

    def _gripper(self, value: float):
        return cfg_move_j(self.cfg, self._q(), replace(self._q(), gripper=value), self.speed * self._gripper_speed())

    def _ik(self, pose: ToolPose) -> JointState | None:
        cur = self._q()
        for seed in [cur] + [replace(s, gripper=cur.gripper) for s in SEEDS]:
            q = self.kin.ik(pose, seed)
            if q is not None:
                return q
        return None

    def _line(self, start_q: JointState, goal: ToolPose):
        return cfg_move_l(self.cfg, self.kin, self.kin.fk(start_q), goal, start_q, self.speed)

    def gripper_for_width(self, width_mm: float) -> float:
        """벌어진 폭(mm) -> 그리퍼 값. 대응표는 robot.yaml의 gripper_width_mm (T7에서 측정)."""
        return self.gripper.width_to_value(width_mm)

    def home(self) -> MotionResult:
        return self._run([self._joint_move(JointState(*self.cfg.poses["home"]))])

    def park(self) -> MotionResult:
        return self._run([self._joint_move(JointState(*self.cfg.poses["park"]))])

    def radar_clear(self) -> MotionResult:
        """초음파의 대기 자세: 팔이 레이더 정면을 가리지 않게 접어 올린다 (결정 0039)."""
        return self._run([self._joint_move(JointState(*self.cfg.poses["radar_clear"]))])

    def _jaw_yaw(self, yaw_t: float) -> float:
        """턱 축은 180° 돌려도 같은 파지다(고정·가동 턱 쪽만 바뀐다). 손목이 덜 돌도록 로봇 기준 ±90° 안의 쪽을 고른다."""
        base = self.ws.base.yaw_deg
        r = (yaw_t - base + 180.0) % 360.0 - 180.0
        if abs(r) > 90.0:
            r -= math.copysign(180.0, r)
        return r + base

    def _tilt_ok(self, obj: DetectedObject, yaw_t: float) -> bool:
        """턱 축이 팔이 뻗은 방향(방사 방향)과 거의 나란할 때만 기울일 수 있다."""
        return self.gripper.tilt_allowed(obj.x_mm, obj.y_mm, yaw_t)

    def _tilt(self, v: float) -> float:
        """그리퍼 값 v에서의 손 기울기(°). 구현은 그리퍼에 있다."""
        return self.gripper.tilt(v)

    def _hand_pose(self, ox: float, oy: float, yaw_t: float, v: float, tilt_ok: bool, tip_z: float, tilt_scale: float = 1.0) -> ToolPose:
        """두 턱 끝의 중간점이 (ox, oy)에 오고 낮은 턱 끝이 tip_z에 오는 손 자세(기준점 gripper_frame의 위치)."""
        return self.gripper.hand_pose(ox, oy, yaw_t, v, tilt_ok, tip_z, tilt_scale)

    def _close_until_contact(self, traj) -> tuple[bool, MotionResult]:
        """경로를 실행하다가 그리퍼가 명령보다 앞서 멈추면(물체에 닿음) 그 자리에서 정지한다."""
        lo = self.cfg.joint_limits_deg["gripper"][0]
        h, count, contact = self.ctl.execute(traj), 0, False
        while h.result is None:
            time.sleep(0.04)
            lag = self._q().gripper - self.ctl._last.gripper  # 실제 - 명령. 닫는 중에 양수로 커지면 물체가 막고 있다
            count = count + 1 if lag > self.cfg.contact_lag_units and self.ctl._last.gripper > lo + 1 else 0
            if count >= 4:
                contact = True
                self.ctl.stop()
                break
        r = h.wait()
        self.ctl.wait_idle()
        return contact, r

    def _closing_path(self, q_start: JointState, obj, yaw_t: float, tilt_ok: bool, v0: float, v_end: float, gap0: float, gap_end: float, tip_z: float | None = None):
        """닫으면서 두 턱 끝의 중간점이 물체 중심에 머물게 손을 같이 보내는 경로(열 때도 같은 경로). 구현은 그리퍼에 있다."""
        tip_z = (self.cfg.table_z_mm + self.cfg.grasp_height_mm) if tip_z is None else tip_z
        return self.gripper.closing_path(q_start, obj, yaw_t, tilt_ok, v0, v_end, gap0, gap_end, tip_z, self._gripper_speed())

    def _grasp_height(self, obj: DetectedObject, gap0: float, v0: float, tilt_ok: bool) -> tuple[float, str | None]:
        """집을 때 낮은 턱 끝의 작업대 면 위 높이 t와 거부 이유. 물체 높이를 모르면(height_mm 0)나 폭을 모르면 설정값.
        벌어진 턱 사이 간격은 턱 끝에서 dz 위에서 G - dz*s (s = tan(기울기) + tan(벌림각 - 기울기))로 좁아진다. 물체 위 모서리를 안 치려면 W <= G - (H - t)*s - 4 이고,
        턱이 물체에 닿는 높이 H - t는 grasp_min_contact_mm 이상이어야 한다 (결정 0042)."""
        t, H, W = self.cfg.grasp_height_mm, obj.height_mm, obj.width_mm
        if H <= 0 or W <= 0:
            return t, None
        s = self.gripper.narrowing(v0, tilt_ok)
        t = min(t, H - self.cfg.grasp_min_contact_mm)  # 낮은 물체는 턱 끝을 그만큼 내려 물체에 닿게
        if t < self.cfg.tip_floor_mm:
            return t, f"object_too_low: 높이 {H:.0f}mm 물체는 턱이 {self.cfg.grasp_min_contact_mm:.0f}mm 이상 닿게 집을 수 없다(턱 끝 최소 높이 {self.cfg.tip_floor_mm:.0f}mm)"
        room = gap0 - W - 4.0  # 위 모서리 높이에서 남겨야 하는 틈
        if room < 0 or (H - t) * s > room:
            t = H - room / s if room >= 0 else H
            if room < 0 or H - t < self.cfg.grasp_min_contact_mm:
                return t, (f"grasp_blocked: 폭 {W:.0f}mm·높이 {H:.0f}mm 물체는 최대 벌림 {gap0:.0f}mm에서 턱이 위 모서리를 친다 "
                           f"(집는 높이를 올려도 닿는 높이가 {self.cfg.grasp_min_contact_mm:.0f}mm 아래로 줄어든다). 벌림을 키우거나 물체를 바꾼다")
        return t, None

    def _needed_gap(self, obj: DetectedObject, tilt_ok: bool) -> float:
        """폭 W와 높이 H를 알 때 설정 집는 높이로 위 모서리를 안 치는 가장 작은 벌림 G(W + gripper_margin_mm부터 2mm씩). 그 안에 없으면 최대 벌림 (결정 0042와 같은 식)."""
        hi = self.cfg.grasp_max_open_mm
        g = min(obj.width_mm + self.cfg.gripper_margin_mm, hi)
        t0 = min(self.cfg.grasp_height_mm, obj.height_mm - self.cfg.grasp_min_contact_mm)  # 낮은 물체는 설정 높이보다 내려 집는다
        while g < hi:
            t, why = self._grasp_height(obj, g, self.gripper_for_width(g), tilt_ok)
            if why is None and t <= t0 + 1e-9:  # 집는 높이를 올리지 않고도 지난다
                return g
            g += 2.0
        return hi

    def pick(self, obj: DetectedObject) -> MotionResult:
        """고정 턱 1개 + 가동 턱 1개. URDF 기하(결정 0009, 0010, 0011): 기준점(gripper_frame)은 고정 턱 안쪽 면 위에 있고 가동 턱은 +m 쪽이다.
        (1) 틈을 최대로 벌리고(기본), 두 턱 끝의 중간점이 물체에 오도록 손 위치를 정해 내려간다(NVIDIA 포럼의 grasp_center 방식),
        (2) 턱 축이 팔이 뻗은 방향이면 손을 기울여 V자를 대칭에 가깝게 만들고, 닫을수록 기울기를 푼다,
        (3) 닫을 때 중간점이 물체 중심에 머물도록 손을 같이 보낸다(F1 Hand '닫으며 중심 쪽으로 이동'),
        (4) 그리퍼가 명령보다 앞서 멈추면(접촉) 그 자리에서 정지한다. 물체 크기를 몰라도 된다."""
        yaw = self._jaw_yaw((obj.angle_deg or 0.0) + 90.0)
        lo_g = self.cfg.joint_limits_deg["gripper"][0]  # 턱 축이 물체 긴 변에 수직이 되도록 (spec 6.6: 물체 각도 + 90°)
        m = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        known = obj.width_mm > 0
        gap_min = min(mm for _, mm in self.cfg.gripper_width_mm)
        tilt_ok = self._tilt_ok(obj, yaw)
        gap0 = self.cfg.grasp_max_open_mm if self.cfg.grasp_open_policy == "max" or not known else obj.width_mm + self.cfg.gripper_margin_mm
        if known and obj.height_mm > 0 and self.cfg.grasp_open_policy == "max":
            gap0 = self._needed_gap(obj, tilt_ok)  # 폭과 높이를 알면 필요한 만큼만 벌린다(최대 벌림은 턱 끝이 책상에 가장 가깝게 내려간다, REQ-036)
        gap_end = max(obj.width_mm - 10.0, gap_min) if known else gap_min  # 알면 폭보다 10mm 더 조이는 지점까지(접촉 감지에 필요한 명령 차이), 모르면 끝까지
        v0, v_end = self.gripper_for_width(gap0), self.gripper_for_width(gap_end)
        ox, oy = obj.x_mm + self.cfg.grasp_bias_mm * m[0], obj.y_mm + self.cfg.grasp_bias_mm * m[1]
        t, why = self._grasp_height(obj, gap0, v0, tilt_ok)
        if why:
            return MotionResult("out_of_bounds", 0.0, ToolPose(0, 0, 0, 0), reason=why)
        tip_z = self.cfg.table_z_mm + t
        grasp = self._hand_pose(ox, oy, yaw, v0, tilt_ok, tip_z)
        # 접근 높이에서는 기울기를 절반만 준다(전부 기울이면 손목 한계로 닿지 않는다). 내려가며 직선 경로가 pitch를 보간해 전부 기울인다.
        hover = self._hand_pose(ox, oy, yaw, v0, tilt_ok, tip_z + self.cfg.approach_height_mm, 0.5)
        fail = MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        q_hover = self._ik(hover)
        if q_hover is None:
            return fail
        q_open = replace(q_hover, gripper=v0)
        down = self._line(q_open, grasp) if self.kin.ik(grasp, q_open) is not None else None
        if down is None:
            return fail
        closing = self._closing_path(JointState(*down.pts[-1]), replace(obj, x_mm=ox, y_mm=oy), yaw, tilt_ok, v0, v_end, gap0, gap_end, tip_z)
        if closing is None:
            return fail
        r = self._run([self._joint_move(q_hover)])  # 1. 접근 위치로
        if r.status != "ok":
            return r
        r = self._run([self._gripper(v0), down])  # 2. 열기 3. 하강
        if r.status != "ok":
            return r
        self._set_grip_torque(self.cfg.gripper_squeeze_torque)  # 조임 힘을 낮춰 얇은 물체가 미끄러져 빠지지 않게(RAM Torque_Limit, 결정 0033)
        try:
            contact, r = self._close_until_contact(closing)  # 4. 닫으며 손을 같이 보낸다
        except BaseException:
            self._restore_grip_torque()
            raise
        if r.status not in ("ok", "stopped"):
            self._restore_grip_torque()
            return r
        if contact:  # 접촉: 목표를 지금 위치에서 예압만큼만 더 닫힌 쪽으로 두고(계속 미는 것을 없앤다), 들고 가는 동안 조임 한계를 낮춘다. 되돌리는 것은 place가 끝난 뒤
            self.ctl.hold_gripper(max(lo_g, self._q().gripper - self.cfg.grip_preload_units))
            self._set_grip_torque(self.cfg.gripper_hold_torque)
        else:
            self._restore_grip_torque()
        here = self.kin.fk(self._q())
        up = self._line(self._q(), replace(here, z_mm=here.z_mm + self.cfg.approach_height_mm))  # 5. 자세는 그대로 수직으로 상승
        if up is None:
            return fail
        r = self._run([up])
        if contact or r.status != "ok":
            return r
        self._run([self._gripper(self.cfg.gripper_open)])  # 접근 높이까지 올라온 뒤에 연다(바닥에서는 열지 않는다): 턱 사이에 끼인 것을 푼다
        return MotionResult("no_object", r.duration_s, r.final_tool)

    def _opening_path(self):
        """놓을 때: 집을 때의 닫는 경로를 반대로(지금 값 -> gripper_open) 써서 두 턱 끝의 중간점이 제자리에 머물게 손을 같이 보낸다. 못 풀면 None."""
        q = self._q()
        v0, v1 = q.gripper, self.cfg.gripper_open
        self.opening_fallback_reason = "gripper_already_open"  # None을 돌려주는 이유(아래에서 IK가 실패하면 바뀐다)
        if abs(v1 - v0) < 1.0:
            return None
        p = self.kin.fk(q)
        f, m = self.kin.jaw_tip_offsets(-90.0, p.yaw_deg, v0)
        mid = (f[:2] + m[:2]) / 2
        center = SimpleNamespace(x_mm=p.x_mm + float(mid[0]), y_mm=p.y_mm + float(mid[1]))
        tip_z = p.z_mm + float(min(f[2], m[2]))
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        gap = lambda v: float(np.interp(v, [p_[1] for p_ in pts], [p_[0] for p_ in pts]))
        path = self._closing_path(q, center, p.yaw_deg, False, v0, v1, gap(v0), gap(v1), tip_z)
        if path is None:
            self.opening_fallback_reason = "ik_failed_along_opening_path"
        return path

    def place(self, zone: str) -> MotionResult:
        z = self.ws.drop_zones[zone]
        return self.place_at(z["x_mm"], z["y_mm"])

    def place_at(self, x_mm: float, y_mm: float) -> MotionResult:
        """작업대 점(x_mm, y_mm)에 놓는다. 배출 구역(place)도 이것을 거친다."""
        z = {"x_mm": x_mm, "y_mm": y_mm}
        # 한쪽만 열리는 그리퍼라 손끝(기준점)을 구역 점에 내리면 쥔 물체가 한쪽으로 치우쳐 놓인다: 놓는 순간(지금 쥔 그리퍼 값)의 두 턱 끝 중간점이 구역 점에 오도록 손 xy를 보정한다 (pick과 같은 계산, 결정 0033)
        hand = self._hand_pose(z["x_mm"], z["y_mm"], 0.0, self._q().gripper, False, 0.0)
        self.last_place_correction_mm = (float(hand.x_mm - z["x_mm"]), float(hand.y_mm - z["y_mm"]))
        release = ToolPose(hand.x_mm, hand.y_mm, self.cfg.table_z_mm + self.cfg.place_height_mm, 0.0)
        hover = replace(release, z_mm=release.z_mm + self.cfg.approach_height_mm)
        q_hover = self._ik(hover)
        if q_hover is None:
            return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        q_down = self.kin.ik(release, q_hover)
        down = self._line(q_hover, release) if q_down is not None else None
        if down is None:
            return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        r = self._run([self._joint_move(q_hover), down])
        if r.status != "ok":
            return r
        try:
            self._restore_grip_torque()  # 놓는다: 예압 고정을 풀고 조임 한계를 원래 값으로(이후 닫는 일이 없다)
            path = self._opening_path()  # 열리는 동안 손을 같이 보내 물체 위치를 지킨다
            if path is None:  # 못 풀면 제자리에서 그냥 연다: 조용히 넘어가지 않고 이유를 남긴다 (REQ-036)
                self.ctl._emit("opening_path_fallback", "WARN", "ARM-OPENING-PATH-FALLBACK", {"reason": self.opening_fallback_reason, "gripper": float(self._q().gripper)})
            r = self._run([path or self._gripper(self.cfg.gripper_open)])
            if r.status != "ok":
                return r
            # 열면서 손이 옆으로 옮겨졌으니 원래 하강 경로를 거꾸로 타면 낮은 높이에서 release 자리로 되돌아가며 물체를 민다: 지금 자리에서 수직으로 올린다.
            # 그리퍼는 연 채로 둔다(닫기는 호출한 쪽이 호버 높이 이상 올라온 뒤에 한다)
            here = self.kin.fk(self._q())
            up = self._line(self._q(), replace(here, z_mm=here.z_mm + self.cfg.approach_height_mm))
            return self._run([up]) if up is not None else MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        finally:
            self._restore_grip_torque()  # 예외가 나도 조임 한계를 원래 값으로
