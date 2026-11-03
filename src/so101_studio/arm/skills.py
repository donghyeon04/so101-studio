"""기능: home, park, pick, place (arm-software-spec.md 6.6). 동작 계획은 MoveJ(관절 보간)와 MoveL(직선)의 조합이다."""
import math
import time
from dataclasses import astuple, replace
from types import SimpleNamespace

import numpy as np

from ..core import frames
from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .kinematics import FIXED_TIP, JAW_Q_DEG, JAW_TIP_A, JAW_TIP_B, VALUE_TO_Q
from .trajectory import LineTrajectory, QuinticShape, cfg_move_j, cfg_move_l

SEEDS = [JointState(0, -30, 30, 60, 0, 0), JointState(0, -60, 60, 30, 0, 0), JointState(0, 0, 60, 30, 0, 0)]


class Skills:
    def __init__(self, ctl, kin, cfg, ws, speed_pct: float | None = None):
        self.ctl, self.kin, self.cfg, self.ws = ctl, kin, cfg, ws
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default
        self._grip_orig: int | None = None  # 조임 한계를 바꾸기 전 값(놓은 뒤 되돌린다)
        ctl.grip_warn_pct, ctl.grip_load_s, ctl.grip_open = cfg.gripper_load_warn_pct, cfg.gripper_load_s, cfg.gripper_open  # 그리퍼 부하 감시 (결정 0033)

    def _set_grip_torque(self, value: int) -> None:
        if self._grip_orig is None:
            self._grip_orig = self.ctl.driver.gripper_torque_limit()
        self.ctl.driver.set_gripper_torque_limit(value)

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

    def _gripper(self, value: float):
        return self._joint_move(replace(self._q(), gripper=value))

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
        """벌어진 폭(mm) -> 그리퍼 값. 대응표는 robot.yaml의 gripper_width_mm (T7에서 측정, 지금은 자리표시)."""
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        lo, hi = self.cfg.joint_limits_deg["gripper"]
        return float(np.clip(np.interp(width_mm, [p[0] for p in pts], [p[1] for p in pts]), lo, hi))

    def home(self) -> MotionResult:
        return self._run([self._joint_move(JointState(*self.cfg.poses["home"]))])

    def park(self) -> MotionResult:
        return self._run([self._joint_move(JointState(*self.cfg.poses["park"]))])

    def _jaw_yaw(self, yaw_t: float) -> float:
        """턱 축은 180° 돌려도 같은 파지다(고정·가동 턱 쪽만 바뀐다). 손목이 덜 돌도록 로봇 기준 ±90° 안의 쪽을 고른다."""
        base = self.ws.base.yaw_deg
        r = (yaw_t - base + 180.0) % 360.0 - 180.0
        if abs(r) > 90.0:
            r -= math.copysign(180.0, r)
        return r + base

    def _tilt_ok(self, obj: DetectedObject, yaw_t: float) -> bool:
        """턱 축이 팔이 뻗은 방향(방사 방향)과 거의 나란할 때만 기울일 수 있다."""
        if self.cfg.grasp_tilt_deg <= 0:
            return False
        xr, yr = frames.table_to_robot(obj.x_mm, obj.y_mm, self.ws.base)
        radial = math.degrees(math.atan2(yr, xr))
        diff = (yaw_t - self.ws.base.yaw_deg - radial + 90.0) % 180.0 - 90.0  # 턱 축은 180° 대칭
        return abs(diff) <= self.cfg.grasp_tilt_tol_deg

    def _tilt(self, v: float) -> float:
        """그리퍼 값 v에서의 손 기울기(°).
        symmetric(기본): 가동 턱이 벌어진 각도 θ의 절반 -> 두 턱이 수직에서 ±θ/2로 벌어져 사용자가 보기에 대칭인 V자(닫을수록 같이 서서 닫힌다).
          두 턱 끝 높이가 달라지고(θ=80°면 약 30mm), 손목 한계 때문에 grasp_tilt_deg까지만 기울인다.
        equal_tips: 두 턱 끝 높이가 같아지는 각도(약 23°), 거의 닫히면 0으로 줄인다."""
        theta = VALUE_TO_Q[1] * v
        if self.cfg.grasp_tilt_mode == "symmetric":
            return min(self.cfg.grasp_tilt_deg, theta / 2)
        q = VALUE_TO_Q[0] + VALUE_TO_Q[1] * v
        a_m = float(np.interp(q, JAW_Q_DEG, JAW_TIP_A))
        b_m = float(np.interp(q, JAW_Q_DEG, JAW_TIP_B))
        eq = math.degrees(math.atan2(FIXED_TIP[1] - b_m, a_m - FIXED_TIP[0])) if a_m - FIXED_TIP[0] > 1.0 else 0.0
        return min(self.cfg.grasp_tilt_deg, max(0.0, eq)) * min(1.0, v / 25.0)

    def _hand_pose(self, ox: float, oy: float, yaw_t: float, v: float, tilt_ok: bool, tip_z: float, tilt_scale: float = 1.0) -> ToolPose:
        """두 턱 끝의 중간점이 (ox, oy)에 오고 낮은 턱 끝이 tip_z에 오는 손 자세(기준점 gripper_frame의 위치)."""
        pitch = -90.0 - (self._tilt(v) * tilt_scale if tilt_ok else 0.0)
        f, m = self.kin.jaw_tip_offsets(pitch, yaw_t, v)
        mid = (f[:2] + m[:2]) / 2
        return ToolPose(ox - mid[0], oy - mid[1], tip_z - min(f[2], m[2]), yaw_t, pitch)

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
        """닫으면서 손의 자세와 위치를 매 단계 다시 계산한 한 개의 경로: 두 턱 끝의 중간점이 물체 중심에 계속 머물고,
        기울기는 닫히는 정도에 맞춰 풀린다(고정 턱은 +m 쪽으로, 가동 턱은 -m 쪽으로 같이 다가온다)."""
        n = max(2, math.ceil(abs(v0 - v_end) / 1.5))
        q, pts = q_start, [np.array(astuple(replace(q_start, gripper=v0)), float)]
        for i in range(1, n + 1):
            v = v0 + (v_end - v0) * i / n
            q = self.kin.ik(self._hand_pose(obj.x_mm, obj.y_mm, yaw_t, v, tilt_ok, (self.cfg.table_z_mm + self.cfg.grasp_height_mm) if tip_z is None else tip_z), q, tol_mm=3.0)
            if q is None:
                return None
            pts.append(np.array(astuple(replace(q, gripper=v)), float))
        return LineTrajectory(np.array(pts), QuinticShape(max(1.0, abs(gap0 - gap_end) / 16.0)))  # 틈 16mm/s로 천천히

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
        gap0 = self.cfg.grasp_max_open_mm if self.cfg.grasp_open_policy == "max" or not known else obj.width_mm + self.cfg.gripper_margin_mm
        gap_end = max(obj.width_mm - 10.0, gap_min) if known else gap_min  # 알면 폭보다 10mm 더 조이는 지점까지(접촉 감지에 필요한 명령 차이), 모르면 끝까지
        v0, v_end = self.gripper_for_width(gap0), self.gripper_for_width(gap_end)
        tilt_ok = self._tilt_ok(obj, yaw)
        ox, oy = obj.x_mm + self.cfg.grasp_bias_mm * m[0], obj.y_mm + self.cfg.grasp_bias_mm * m[1]
        grasp = self._hand_pose(ox, oy, yaw, v0, tilt_ok, (self.cfg.table_z_mm + self.cfg.grasp_height_mm))
        # 접근 높이에서는 기울기를 절반만 준다(전부 기울이면 손목 한계로 닿지 않는다). 내려가며 직선 경로가 pitch를 보간해 전부 기울인다.
        hover = self._hand_pose(ox, oy, yaw, v0, tilt_ok, (self.cfg.table_z_mm + self.cfg.grasp_height_mm) + self.cfg.approach_height_mm, 0.5)
        fail = MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        q_hover = self._ik(hover)
        if q_hover is None:
            return fail
        q_open = replace(q_hover, gripper=v0)
        down = self._line(q_open, grasp) if self.kin.ik(grasp, q_open) is not None else None
        if down is None:
            return fail
        closing = self._closing_path(JointState(*down.pts[-1]), replace(obj, x_mm=ox, y_mm=oy), yaw, tilt_ok, v0, v_end, gap0, gap_end)
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
        if abs(v1 - v0) < 1.0:
            return None
        p = self.kin.fk(q)
        f, m = self.kin.jaw_tip_offsets(-90.0, p.yaw_deg, v0)
        mid = (f[:2] + m[:2]) / 2
        center = SimpleNamespace(x_mm=p.x_mm + float(mid[0]), y_mm=p.y_mm + float(mid[1]))
        tip_z = p.z_mm + float(min(f[2], m[2]))
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        gap = lambda v: float(np.interp(v, [p_[1] for p_ in pts], [p_[0] for p_ in pts]))
        return self._closing_path(q, center, p.yaw_deg, False, v0, v1, gap(v0), gap(v1), tip_z)

    def place(self, zone: str) -> MotionResult:
        z = self.ws.drop_zones[zone]
        release = ToolPose(z["x_mm"], z["y_mm"], self.cfg.table_z_mm + self.cfg.place_height_mm, 0.0)
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
            self.ctl.hold_gripper(None)  # 놓는다: 예압 고정을 풀고
            r = self._run([self._opening_path() or self._gripper(self.cfg.gripper_open)])  # 열리는 동안 손을 같이 보내 물체 위치를 지킨다
            if r.status != "ok":
                return r
            return self._run([down.reversed(self._q().gripper)])
        finally:
            self._restore_grip_torque()  # 예외가 나도 조임 한계를 원래 값으로
