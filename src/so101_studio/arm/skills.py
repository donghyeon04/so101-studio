"""기능: home, park, pick, place (arm-software-spec.md 6.6). 동작 계획은 MoveJ(관절 보간)와 MoveL(직선)의 조합이다."""
import itertools
import math
import time
from dataclasses import astuple, replace
from types import SimpleNamespace

import numpy as np

from ..core import frames
from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .gripper import make_gripper
from .kinematics import VALUE_TO_Q
from .trajectory import cfg_move_j, cfg_move_l

MIN_CLEARANCE_MM = 10.0  # 부딪힘 때문에 줄여도 이보다 작게는 안 한다 (arm-software-spec 11.3)
CLEARANCE_STEP_MM = 5.0
JAW_HALF_MM = 6.0  # 알려진 물체와의 겹침 판정에서 턱 끝 두께 여유
BOX_GAP_MM = 2.0  # released_then_tilted: 쥔 물체 상자와 두 턱 선분 사이에 남길 간격 (11.3.3)
MIN_SYM_ROOM_MM = 20.0  # 대칭 상한 틈 - 쥔 틈이 이보다 작으면(큰 물체) 대칭 경로를 건너뛴다
HEIGHT_REQUIRED_REASON = "턱 축이 팔 방향과 어긋나 수직으로 집어야 해서 물체 높이가 필요합니다"  # 결정 0052
HOVER_NEIGHBOUR_R_MM = 100.0  # 놓는 점 둘레 이 반경 안의 알려진 물체 높이를 hover 높이 규칙에 쓴다 (11.3.3)
SWING_STEP_DEG = 2.0  # 기울기 풀기 보간에서 가장 크게 도는 관절이 이만큼 돌 때마다 한 자세씩 검사한다
SAME_SPOT_MM = 30.0  # 놓을 자리에서 이 안에 있는 놓은 물체는 같은 자리(배출 구역에 쌓는 것)로 보고 피하지 않는다
LIFT_MARGIN_MM = 10.0  # 놓은 뒤 상승의 최소 = 물체 높이 + 이 값 (11.3.3)
LIFT_UNKNOWN_EXTRA_MM = 30.0  # 물체 높이를 모르면 t_g + 이 값
NEIGHBOUR_H_MM = 40.0  # 이웃 물체 높이를 모를 때 가정하는 높이: 이보다 낮게 지나는 두 턱 끝은 알려진 물체와의 거리를 본다
ROUTES = ("tilted", "released_then_tilted", "fixed_tilt")

SEEDS = [JointState(0, -30, 30, 60, 0, 0), JointState(0, -60, 60, 30, 0, 0), JointState(0, 0, 60, 30, 0, 0)]


class Skills:
    def __init__(self, ctl, kin, cfg, ws, speed_pct: float | None = None):
        self.ctl, self.kin, self.cfg, self.ws = ctl, kin, cfg, ws
        self.gripper = kin.gripper = make_gripper(kin, cfg)  # 그리퍼 종류별 기하·보정·경로 지식은 arm/gripper.py (work-queue 5-3)
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default
        self.opening_fallback_reason: str | None = None  # 마지막 _opening_path가 None이었던 이유
        self.last_place_correction_mm: tuple[float, float] | None = None  # 마지막 place가 손 xy를 옮긴 양
        self._grip_orig: int | None = None  # 조임 한계를 바꾸기 전 값(놓은 뒤 되돌린다)
        self.placed: list[tuple[float, float, float]] = []  # 이 세션에서 놓은 물체 (x, y, 반경 mm): 다음 놓기의 벌림이 부딪히는지 볼 때 알려진 물체로 쓴다 (11.3)
        self.known_objects: list[tuple[float, float, float]] = []  # 호출하는 쪽이 알려 주는 물체(수동 입력 등)
        self.opening_info: dict | None = None  # 마지막 놓기 벌림의 계획 값(clearance 등)
        self.opening_blocked_by: str | None = None
        self.placed_height: dict[tuple[float, float], float] = {}  # 놓은 물체 (x, y) -> 높이(mm). 모르면 NEIGHBOUR_H_MM
        self.hover_need_mm = 0.0  # 마지막 _descent의 hover 높이 하한
        self.untilt_rejected: str | None = None  # 마지막 _untilt_goal이 거절한 이유(ik | box:<mm> | object:<..> | check_goal:<..> | already_vertical)
        self.descent_blocked_by: str | None = None  # 마지막 _descent가 알려진 물체 때문에 거른 이유(object@(x,y))
        self.opening_skipped: list[dict] = []  # 건너뛰었거나 못 쓴 여는 경로와 이유 (11.3.3)
        self.last_pick_tilted: bool | None = None  # 마지막 pick이 기울여 집으려 했는지(수직 집기 no_object를 작업 실행이 알아본다, 결정 0052)
        self.grip: dict | None = None  # 집기 기억(11.3.2): tilted(집기의 tilt_ok), tip_h(낮은 턱 끝 높이 t_g, mm), obj_h(물체 높이, 0=모름). 놓기가 끝나면·집기가 실패하면·새 집기를 시작하면 지운다
        self._place_ctx: dict | None = None  # 지금 놓기의 자세 정보(_opening_path가 읽는다)
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
        try:
            self.ctl.hold_gripper(None)
        finally:  # 고정을 푸는 데 실패해도 조임 한계는 반드시 되돌린다(낮춘 채 남으면 다음 물체를 못 쥔다)
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

    def _closing_path(self, q_start: JointState, obj, yaw_t: float, tilt_ok: bool, v0: float, v_end: float, gap0: float, gap_end: float, tip_z: float | None = None,
                      duration_s: float | None = None, tilt_scale=(1.0, 1.0)):
        """닫으면서 두 턱 끝의 중간점이 물체 중심에 머물게 손을 같이 보내는 경로(열 때도 같은 경로). 구현은 그리퍼에 있다."""
        tip_z = (self.cfg.table_z_mm + self.cfg.grasp_height_mm) if tip_z is None else tip_z
        return self.gripper.closing_path(q_start, obj, yaw_t, tilt_ok, v0, v_end, gap0, gap_end, tip_z, 1.0 if duration_s is not None else self._gripper_speed(), duration_s, tilt_scale)

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

    def _plan_pick(self, obj: DetectedObject, yaw: float, m: tuple[float, float], tilt_ok: bool):
        """집기 계획(틈·집는 높이·접근·하강·닫는 경로). 풀리면 SimpleNamespace, 못 풀면 MotionResult(out_of_bounds | ik_fail)."""
        known = obj.width_mm > 0
        gap_min = min(mm for _, mm in self.cfg.gripper_width_mm)
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
        return SimpleNamespace(gap0=gap0, gap_end=gap_end, v0=v0, v_end=v_end, t=t, q_hover=q_hover, down=down, closing=closing)

    def pick(self, obj: DetectedObject) -> MotionResult:
        """고정 턱 1개 + 가동 턱 1개. URDF 기하(결정 0009, 0010, 0011): 기준점(gripper_frame)은 고정 턱 안쪽 면 위에 있고 가동 턱은 +m 쪽이다.
        (1) 틈을 최대로 벌리고(기본), 두 턱 끝의 중간점이 물체에 오도록 손 위치를 정해 내려간다(NVIDIA 포럼의 grasp_center 방식),
        (2) 턱 축이 팔이 뻗은 방향이면 손을 기울여 V자를 대칭에 가깝게 만들고, 닫을수록 기울기를 푼다,
        기울인 계획이 손목 한계로 안 풀리면(ik_fail) 높이를 알 때만 수직으로 틈·높이를 다시 계산해 한 번 더 한다 (REQ-055),
        (3) 닫을 때 중간점이 물체 중심에 머물도록 손을 같이 보낸다(F1 Hand '닫으며 중심 쪽으로 이동'),
        (4) 그리퍼가 명령보다 앞서 멈추면(접촉) 그 자리에서 정지한다. 물체 크기를 몰라도 된다."""
        self.placed = [o for o in self.placed if math.hypot(o[0] - obj.x_mm, o[1] - obj.y_mm) > 40.0]  # 집어 간 물체는 더 이상 장애물이 아니다
        self.grip = None
        yaw = self._jaw_yaw((obj.angle_deg or 0.0) + 90.0)
        lo_g = self.cfg.joint_limits_deg["gripper"][0]  # 턱 축이 물체 긴 변에 수직이 되도록 (spec 6.6: 물체 각도 + 90°)
        m = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        tilt_ok = self._tilt_ok(obj, yaw)
        self.last_pick_tilted = tilt_ok
        height_required = MotionResult("height_required", 0.0, ToolPose(0, 0, 0, 0), reason=HEIGHT_REQUIRED_REASON)
        if self.gripper.name == "single_jaw" and not tilt_ok and obj.height_mm <= 0:  # (단동식만: 위 모서리 문제는 V자 턱이 위로 좁아지는 데서 나온다. 병렬은 해당 없음) 수직으로만 집는데 높이를 모르면 위 모서리를 친다: 움직이지 않고 높이를 요구한다 (결정 0052, 6.6)
            return height_required
        plan = self._plan_pick(obj, yaw, m, tilt_ok)
        if tilt_ok and isinstance(plan, MotionResult) and plan.status == "ik_fail":
            info = {"x_mm": obj.x_mm, "y_mm": obj.y_mm, "width_mm": obj.width_mm, "height_mm": obj.height_mm}
            if obj.height_mm > 0:  # 기울인 계획이 안 풀리면(멀리서는 손목 한계) 수직으로 틈·높이를 다시 계산해 한 번 더 (REQ-055)
                self.ctl._emit("pick_tilt_unreachable", "WARN", "ARM-PICK-TILT-UNREACHABLE", {"reason": "tilt_unreachable", **info})
                tilt_ok = False
                self.last_pick_tilted = False
                plan = self._plan_pick(obj, yaw, m, False)
            else:  # 높이를 모르면 수직으로 바꾸지 않고 높이를 요구한다 (결정 0052)
                self.ctl._emit("pick_tilt_unreachable", "WARN", "ARM-PICK-TILT-UNREACHABLE", {"reason": "tilt_unreachable_height_unknown", **info})
                self.last_pick_tilted = False
                return height_required
        if isinstance(plan, MotionResult):
            return plan
        v0, t, q_hover, down, closing = plan.v0, plan.t, plan.q_hover, plan.down, plan.closing
        fail = MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
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
            self.grip = {"tilted": bool(tilt_ok), "tip_h": float(t), "obj_h": float(obj.height_mm), "obj_w": float(obj.width_mm), "held_gap": float(self._gap_mm(self._q().gripper))}  # 놓기가 같은 기울기·높이로 내려가도록 기억한다 (11.3.2)
            near = min(self.known_objects, key=lambda o: math.hypot(o[0] - obj.x_mm, o[1] - obj.y_mm), default=None)  # 집었다: 가장 가까운 한 항목만 장애물에서 뺀다(붙어 있는 이웃은 남긴다). GUI 자동 실행은 집는 물체까지 알려 준다. no_object·집기 실패면 그대로 둔다
            if near is not None and math.hypot(near[0] - obj.x_mm, near[1] - obj.y_mm) <= 40.0:
                self.known_objects.remove(near)
            self.ctl.hold_gripper(max(lo_g, self._q().gripper - self.cfg.grip_preload_units))
            self._set_grip_torque(self.cfg.gripper_hold_torque)
        else:
            self._restore_grip_torque()
        here = self.kin.fk(self._q())
        up = self._line(self._q(), replace(here, z_mm=here.z_mm + self.cfg.approach_height_mm))  # 5. 자세는 그대로 수직으로 상승
        if up is None:
            self.grip = None  # 집기 실패: 기억을 지운다 (11.3.2)
            return fail
        r = self._run([up])
        if r.status != "ok":
            self.grip = None
        if contact or r.status != "ok":
            return r
        self._run([self._gripper(self.cfg.gripper_open)])  # 접근 높이까지 올라온 뒤에 연다(바닥에서는 열지 않는다): 턱 사이에 끼인 것을 푼다
        return MotionResult("no_object", r.duration_s, r.final_tool)

    def _gap_mm(self, v: float) -> float:
        """그리퍼 값 v에서의 턱 끝 벌림(mm). 폭 표 gripper_width_mm를 거꾸로 읽는다."""
        pts = sorted((mm, val) for val, mm in self.cfg.gripper_width_mm)
        return float(np.interp(v, [p[1] for p in pts], [p[0] for p in pts]))

    def _sym_gap_mm(self) -> float:
        """대칭 상한 틈(11.3.2): 손목이 grasp_tilt_deg까지만 기울어서 이보다 더 벌리면 두 턱이 대칭이 아니다."""
        return self._gap_mm(2.0 * self.cfg.grasp_tilt_deg / VALUE_TO_Q[1])

    def _jaw_angles(self, q: JointState) -> tuple[float, float]:
        """(고정 턱 각도 α_f = τ, 가동 턱 각도 α_m = θ - τ): 수직 아래에서 각 턱의 바깥쪽으로 잰 각도 (11.3.2)."""
        tau = -90.0 - self.kin.fk(q).pitch_deg
        return tau, VALUE_TO_Q[1] * q.gripper - tau

    def _opening_path(self):
        """놓을 때 여는 경로: 차례로 실행할 궤적 목록, 못 풀면 None (arm-software-spec 11.3.3).
        경로 종류를 ① tilted ② released_then_tilted ③ fixed_tilt 순서로 시도하고(종류마다 여유를 place_clearance_mm에서 5mm씩 줄여 최소 10mm까지),
        처음 안전 검사를 넘은 것을 쓴다. 모두 안 되면 None(④)이고 opening_fallback_reason에 이유가 있다. 건너뛰거나 못 쓴 종류는 opening_skipped에 남는다."""
        ctx = self._place_ctx or {}
        q = self._q()
        v0 = q.gripper
        gap0 = self._gap_mm(v0)
        cap = self._gap_mm(self.cfg.joint_limits_deg["gripper"][1])  # 그리퍼 최대 벌림
        self.opening_fallback_reason, self.opening_info, self.opening_skipped = "gripper_already_open", None, []
        if gap0 >= cap - 1.0:
            return None
        p = self.kin.fk(q)
        f, m = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, v0)  # -90° 고정 대신 지금 손 pitch로 중심과 낮은 턱 끝 높이를 구한다
        mid = (f[:2] + m[:2]) / 2
        env = SimpleNamespace(center=SimpleNamespace(x_mm=p.x_mm + float(mid[0]), y_mm=p.y_mm + float(mid[1])), yaw=p.yaw_deg, tip_z=p.z_mm + float(min(f[2], m[2])),
                              v0=v0, gap0=gap0, cap=cap, sym=self._sym_gap_mm(), tau_s=self._tilt(v0) if ctx.get("tilted") else 0.0, obj_h=float(ctx.get("obj_h", 0.0)), need_lift=float(ctx.get("need_lift") or LIFT_MARGIN_MM), box_w=float(ctx.get("obj_w") or gap0))  # 상자 폭: 물체 폭을 알면 그 값, 모르면 쥔 틈 (11.3.3)
        if self._place_ctx is not None:
            self._place_ctx["center"] = env.center  # 떠날 때(_leave)도 열림 경로 검사(_can_leave)와 같은 상자 중심을 쓴다
        last = "ik_failed_along_opening_path"
        for route in ROUTES:
            why = self._route_skip(route, ctx, env)
            if why is None:
                trajs, why = self._try_route(route, env)
                if trajs is not None:
                    return trajs
                last = why
            self.opening_skipped.append({"route": route, "reason": why})
        self.opening_fallback_reason = last
        return None

    @staticmethod
    def _route_skip(route: str, ctx: dict, env) -> str | None:
        """이 종류를 아예 쓸 수 없는 이유(없으면 None)."""
        if route == "fixed_tilt":
            return None
        if ctx.get("legacy", True):
            return "legacy_pose"
        if route == "tilted" and not ctx.get("tilted"):
            return "not_tilted_grip"
        if route == "released_then_tilted":
            if ctx.get("tilted"):
                return "tilted_grip"
            if not ctx.get("tilt_allowed"):
                return "tilt_not_allowed"
            if env.obj_h <= 0:
                return "height_unknown"
        if env.sym - env.gap0 < MIN_SYM_ROOM_MM:
            return "gap_too_wide"  # 큰 물체: 대칭 상한까지 20mm도 못 벌린다
        return None

    def _try_route(self, route: str, env):
        """여유를 줄여 가며 경로를 만들고 안전 검사를 넘는 첫 것을 (궤적 목록, None)으로, 못 넘으면 (None, 이유)."""
        clearance, last, seen = self.cfg.place_clearance_mm, "ik_failed_along_opening_path", set()
        self.opening_blocked_by = None  # 앞 경로 종류의 값이 남지 않게
        top = env.cap if route == "fixed_tilt" else env.sym
        while True:
            gap1 = min(env.gap0 + 2.0 * clearance, top)
            if gap1 not in seen:  # 상한에 걸려 끝 틈이 같으면 다시 계산하지 않는다
                seen.add(gap1)
                v1 = self.gripper_for_width(gap1)
                if abs(v1 - env.v0) < 1.0:
                    return None, "gripper_already_open"
                trajs = self._build_route(route, env, v1, gap1)
                if trajs is None:
                    last = "ik_failed_along_opening_path"
                else:
                    why, stats = self._scan_opening(trajs, route, env)
                    if why is None:
                        self.opening_info = {"route": route, "clearance_mm": clearance, "requested_clearance_mm": self.cfg.place_clearance_mm, "gap_held_mm": env.gap0, **stats}
                        return trajs, None
                    last = why if why == "tilt_hits_object" else "clearance"
                    self.opening_blocked_by = why
            if clearance <= MIN_CLEARANCE_MM:
                return None, last
            clearance = max(MIN_CLEARANCE_MM, clearance - CLEARANCE_STEP_MM)

    def _build_route(self, route: str, env, v1: float, gap1: float):
        T = self.cfg.place_open_s
        c, yaw, z, v0, g0 = env.center, env.yaw, env.tip_z, env.v0, env.gap0
        if route == "tilted":  # 닫는 경로를 거꾸로: 기울기는 지금 값에서 tilt(v)로 커진다
            p = self._closing_path(self._q(), c, yaw, True, v0, v1, g0, gap1, z, duration_s=T)
            return None if p is None else [p]
        if route == "released_then_tilted":
            gA = g0 + self.cfg.place_release_mm
            vA = self.gripper_for_width(gA)
            a = self._closing_path(self._q(), c, yaw, False, v0, vA, g0, gA, z, duration_s=0.25 * T)  # A: 기울기 0으로 수직 놓기
            if a is None:
                return None
            # B: 벌리며 기울인다. 배율 k는 대칭 오차 θ(1-k)가 B 시작 값에서 0까지 곧게 줄도록 정한다(k를 v에 선형으로 두면 θ가 커지는 앞부분에서 오차가 먼저 늘어난다, 11.3.4 "줄어들기만 한다")
            th_a = VALUE_TO_Q[1] * vA
            k_of = lambda v: 1.0 - th_a * (1.0 - (v - vA) / (v1 - vA)) / (VALUE_TO_Q[1] * v)
            b = self._closing_path(JointState(*a.pts[-1]), c, yaw, True, vA, v1, gA, gap1, z, duration_s=0.75 * T, tilt_scale=k_of)
            return None if b is None else [a, b]
        tau = env.tau_s  # fixed_tilt: 기울기를 지금 값으로 고정한 채 중심을 지키며 벌린다(τ_s = 0이면 첫판과 같다)
        scale = (lambda v: tau / self._tilt(v) if self._tilt(v) > 1e-6 else 0.0) if tau > 0 else (1.0, 1.0)
        p = self._closing_path(self._q(), c, yaw, tau > 0, v0, v1, g0, gap1, z, duration_s=T, tilt_scale=scale)
        return None if p is None else [p]

    def _scan_opening(self, trajs, route: str, env):
        """경로의 모든 단계를 본다: 안전 검사(check_goal), 알려진 물체와 두 턱 끝, released_then_tilted는 B 구간의 쥔 물체 상자와 두 턱 선분.
        (이유 | None, 기록용 값 dict)."""
        steps = [(JointState(*pt), route == "released_then_tilted" and ti == 1) for ti, tr in enumerate(trajs) for i, pt in enumerate(tr.pts) if not (ti > 0 and i == 0)]
        prev, sym, taus = None, 0.0, []
        for st, in_b in steps:
            p = self.kin.fk(st)
            tau = -90.0 - p.pitch_deg
            taus.append(tau)
            if prev is not None:
                if why := self.ctl.safety.check_goal(st, prev):
                    return why, {}
                f, m = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, st.gripper)
                for cx, cy, r in [*self.placed, *self.known_objects]:
                    if any(math.hypot(p.x_mm + float(t[0]) - cx, p.y_mm + float(t[1]) - cy) < r + JAW_HALF_MM for t in (f, m)):
                        return f"object@({cx:.0f},{cy:.0f})", {}
                if in_b and self._box_gap(st, p, env) < BOX_GAP_MM:
                    return "tilt_hits_object", {}
            if in_b or route != "released_then_tilted":
                sym = max(sym, abs(2.0 * tau - VALUE_TO_Q[1] * st.gripper))
            prev = st
        if not self._can_leave(prev, env.need_lift, SimpleNamespace(center=env.center, box_w=env.box_w, obj_h=env.obj_h or NEIGHBOUR_H_MM)):
            return "no_lift_after_opening", {}  # 다 벌린 자세에서 필요한 만큼 올라갈 수 없으면(손목 한계) 이 끝 틈은 쓰지 않고 여유를 줄이거나 다음 경로로 간다
        return None, {"tilt_start_deg": round(float(taus[0]), 1), "tilt_end_deg": round(float(taus[-1]), 1), "sym_err_max_deg": round(float(sym), 2)}

    def _box_gap(self, st: JointState, p: ToolPose, env) -> float:
        """쥔 물체 상자(폭 = 쥔 틈, 높이 obj_h, 바닥 = 작업 면 위 place_drop_mm)와 두 턱 선분 사이의 최소 거리(mm). 턱 축이 들어간 수직 평면에서 2D로 본다 (11.3.3)."""
        (fa, fb), (ma, mb) = self.gripper.jaw_segments(p.pitch_deg, p.yaw_deg, st.gripper)
        f, m = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, st.gripper)
        e = (m - f)[:2]
        e = e / np.linalg.norm(e) if np.linalg.norm(e) > 1e-6 else np.array([math.cos(math.radians(p.yaw_deg)), math.sin(math.radians(p.yaw_deg))])
        base, o = np.array([p.x_mm, p.y_mm, p.z_mm]), np.array([env.center.x_mm, env.center.y_mm])
        lo, hi, half = self.cfg.place_drop_mm, self.cfg.place_drop_mm + env.obj_h, env.box_w / 2.0
        t = np.linspace(0.0, 1.0, 41)[:, None]

        def dist(a, b) -> float:
            pts = [np.array([(P[:2] - o) @ e, P[2] - self.cfg.table_z_mm]) for P in (base + a, base + b)]
            line = pts[0] + (pts[1] - pts[0]) * t
            du, dh = np.maximum(np.abs(line[:, 0]) - half, 0.0), np.maximum(np.maximum(lo - line[:, 1], line[:, 1] - hi), 0.0)
            return float(np.hypot(du, dh).min())

        return min(dist(fa, fb), dist(ma, mb))

    def _emit_opening(self, trajs, q_a: JointState, q_b: JointState, seconds: float) -> None:
        """ARM-OPENING-PATH: 계획 값(경로 처음·끝)과 실제 값(벌림 시작·끝에 읽은 관절로 FK한 고정 턱 끝 이동, 두 턱 중간점 이동, 턱 각도, 걸린 초) (REQ-049, 11.3)."""
        a, b = JointState(*trajs[0].pts[0]), JointState(*trajs[-1].pts[-1])
        pa, pb = self.kin.fk(a), self.kin.fk(b)
        move = (float(pb.x_mm - pa.x_mm), float(pb.y_mm - pa.y_mm), float(pb.z_mm - pa.z_mm))

        def tips(st):
            p = self.kin.fk(st)
            f, m = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, st.gripper)
            base = np.array([p.x_mm, p.y_mm, p.z_mm])
            return base + np.asarray(f, float), base + np.asarray(m, float)

        fa, ma = tips(q_a)
        fb, mb = tips(q_b)
        info = self.opening_info or {}
        (fa_deg, ma_deg), (fb_deg, mb_deg) = self._jaw_angles(q_a), self._jaw_angles(q_b)
        self.ctl._emit("opening_path", "INFO", "ARM-OPENING-PATH", {
            "route": info.get("route"), "gap_start_mm": round(self._gap_mm(a.gripper), 1), "gap_end_mm": round(self._gap_mm(b.gripper), 1),
            "hand_move_mm": [round(m_, 2) for m_ in move], "hand_move_xy_mm": round(math.hypot(move[0], move[1]), 2),
            "steps": sum(len(t.pts) - 1 for t in trajs), "duration_s": round(sum(float(t.duration_s) for t in trajs), 2), "gripper_start": round(a.gripper, 1), "gripper_end": round(b.gripper, 1),
            "clearance_mm": info.get("clearance_mm"), "requested_clearance_mm": info.get("requested_clearance_mm"),
            "tilt_start_deg": info.get("tilt_start_deg"), "tilt_end_deg": info.get("tilt_end_deg"), "sym_err_max_deg": info.get("sym_err_max_deg"),
            "jaw_angle_fixed_deg": [round(fa_deg, 1), round(fb_deg, 1)], "jaw_angle_moving_deg": [round(ma_deg, 1), round(mb_deg, 1)],
            "actual_sym_err_end_deg": round(abs(fb_deg - mb_deg), 2), "yaw_deg": round(pa.yaw_deg, 1), "skipped": self.opening_skipped,
            "hover_mm": (self._place_ctx or {}).get("hover_mm"),
            "actual_fixed_tip_move_mm": round(float(np.linalg.norm(fb[:2] - fa[:2])), 2),  # 수평(작업대 xy) 이동: 고정 턱 끝이 얼마나 비켜 가는지
            "actual_mid_move_mm": round(float(np.linalg.norm(((fb + mb) / 2 - (fa + ma) / 2)[:2])), 2),  # 두 턱 중간점의 수평 이동(제자리에 머물러야 한다). 가동 턱 끝은 벌어지며 올라가므로 높이는 따로
            "actual_mid_rise_mm": round(float(((fb + mb) / 2 - (fa + ma) / 2)[2]), 2),
            "actual_s": round(seconds, 2)})

    def place(self, zone: str) -> MotionResult:
        """배출 구역에 놓는다. 사각형 구역(`w_mm·h_mm·slot_mm`)이면 비어 있는 칸 가운데 로봇 원점에 가까운 칸부터, 내려가는 계획이 풀리는 첫 칸에 놓고(11.3.5),
        빈 칸이 없으면 움직이지 않고 stopped(drop_zone_full)로 멈춘다. 옛 점 형식(x_mm·y_mm만)은 그 한 점에 놓는다(이미 있으면 ARM-PLACE-SAME-SPOT 경고)."""
        z = self.ws.drop_zones[zone]
        slots = self.ws.zone_slots(zone)
        if slots is None:
            return self.place_at(z["x_mm"], z["y_mm"])
        half = float(z["slot_mm"]) / 2.0
        free = [p for p in slots if not any(math.hypot(o[0] - p[0], o[1] - p[1]) < half for o in self.placed)]
        if not free:
            self.ctl._emit("drop_zone_full", "ERROR", "ARM-DROP-ZONE-FULL", {"zone": zone, "slots": len(slots)})
            return MotionResult("stopped", 0.0, ToolPose(0, 0, 0, 0), reason=f"drop_zone_full: 배출 구역 {zone}의 칸 {len(slots)}개가 모두 차서 놓지 않았다. 손에 물체를 쥔 채이니 비운 뒤 다시 한다")
        r = None
        for x, y in free:
            r = self.place_at(x, y)
            released = self.grip is None  # 벌림을 시작한 뒤에는 다른 칸으로 바꾸지 않는다
            if r.status == "ok" or released or not (r.status == "ik_fail" or (r.status == "stopped" and (r.reason or "").startswith("low_hover"))):
                return r
        return r

    def _place_yaw(self, x_mm: float, y_mm: float) -> float:
        """놓는 손 방향: 단동식이면 놓는 점에서 팔이 뻗은 방향(턱 축이 방사 방향이라 기울일 수 있다), 아니면 0 (11.3.3)."""
        if self.gripper.name != "single_jaw":
            return 0.0
        xr, yr = frames.table_to_robot(x_mm, y_mm, self.kin.base)
        yaw = self._jaw_yaw(self.kin.base.yaw_deg + math.degrees(math.atan2(yr, xr)))
        return yaw if self.gripper.tilt_allowed(x_mm, y_mm, yaw) else 0.0

    def _heights(self) -> list[float]:
        """수직 이동 높이 후보: approach_height_mm부터 5mm씩 10mm까지. 전부 기울인 자세는 높이 올릴수록 손목 한계(wrist_flex ±101°)에 막혀(배출 구역은 15mm까지만 닿는다) 닿는 가장 높은 것을 쓴다 (11.3.3의 호버 높이를 현실에 맞춤)."""
        return [float(h) for h in range(int(self.cfg.approach_height_mm), 9, -5)] or [float(self.cfg.approach_height_mm)]

    def _hits_objects(self, states, held_r: float = 0.0, tolerate_start: bool = True) -> str | None:
        """낮게 지나는(낮은 턱 끝이 이웃 높이 + 여유보다 낮은) 상태에서 두 턱 끝이, held_r > 0이면 쥔 물체(두 턱 끝 중점, 반지름 held_r)도, 알려진 물체 원(placed·known_objects)에 들어가면 이유 문자열.
        첫 자세에서 이미 겹쳐 있는 원(들어올린 자세가 낮아 바로 옆 이웃과 겹쳐 시작할 수 있다)은 그 겹침이 더 깊어지지 않는 동안만 허용한다: 이웃을 쓸고 가면 충돌이다. tolerate_start=False면 허용하지 않는다(기울기 풀기는 집은 직후가 아니다)."""
        circles = [*self.placed, *self.known_objects]
        states = list(states)
        start = {}  # 원 -> 첫 자세에서의 여유 거리(음수 = 겹침)
        for i, st in enumerate(states if circles else []):
            p = self.kin.fk(st)
            f, m = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, st.gripper)
            if p.z_mm + float(min(f[2], m[2])) - self.cfg.table_z_mm >= NEIGHBOUR_H_MM + MIN_CLEARANCE_MM:
                continue
            tips = [(p.x_mm + float(t[0]), p.y_mm + float(t[1])) for t in (f, m)]
            mid = ((tips[0][0] + tips[1][0]) / 2, (tips[0][1] + tips[1][1]) / 2)
            for c in circles:
                cx, cy, r = c
                d = min(min(math.hypot(t[0] - cx, t[1] - cy) for t in tips) - (r + JAW_HALF_MM), (math.hypot(mid[0] - cx, mid[1] - cy) - (r + held_r + 2.0)) if held_r else 1e9)
                if i == 0 and tolerate_start:
                    start[c] = d
                if d < 0 and d < start.get(c, 0.0) - 1.0:  # 새로 겹치거나, 처음부터 겹친 것이 더 깊어진다
                    return f"object@({cx:.0f},{cy:.0f})"
        return None

    def _hover_need(self, target: tuple[float, float] | None) -> float:
        """hover 높이의 하한(mm): 쥔 물체 바닥(= place_drop_mm + hover)이 놓는 점 반경 100mm 안의 알려진 물체(placed·known_objects) 가운데 가장 높은 것보다 10mm 이상 위여야 한다 (11.3.3).
        이웃이 없으면 0. 높이를 모르는 이웃은 NEIGHBOUR_H_MM."""
        if target is None:
            return 0.0
        hs = [self.placed_height.get((o[0], o[1]), NEIGHBOUR_H_MM) if o in self.placed else NEIGHBOUR_H_MM
              for o in [*self.placed, *self.known_objects] if math.hypot(o[0] - target[0], o[1] - target[1]) <= HOVER_NEIGHBOUR_R_MM]
        return max(hs) + LIFT_MARGIN_MM - self.cfg.place_drop_mm if hs else 0.0

    def _descent(self, release: ToolPose, held_r: float = 0.0, target: tuple[float, float] | None = None):
        """(호버 관절, 하강 궤적, 호버 높이) 또는 못 풀면 None. 호버는 같은 자세를 위로 올린 것. 호버로 가는 관절 이동과 하강이 알려진 물체와 부딪히는 높이는 쓰지 않는다(낮은 호버는 검사를 넘을 때만).
        호버 하한(_hover_need)을 지키는 값이 없으면 None이고 self.hover_need_mm가 그 하한이다(> 10이면 _place_at이 stopped(low_hover)로 멈춘다)."""
        q0 = np.array(astuple(self._q()), float)
        self.descent_blocked_by = None
        need = self._hover_need(target)
        self.hover_need_mm = need
        heights = self._heights() if need <= 0 else [float(h) for h in range(int(max(self.cfg.approach_height_mm, need)), int(need) - 1, -5)] or [float(need)]
        for h in heights:
            q_hover = self._ik(replace(release, z_mm=release.z_mm + h))
            if q_hover is None or (down := self._line(q_hover, release)) is None:
                continue
            move = [JointState(*(q0 + (np.array(astuple(q_hover), float) - q0) * k)) for k in np.linspace(0.0, 1.0, 9)]
            self.descent_blocked_by = self._hits_objects([*move, *(JointState(*pt) for pt in down.pts)], held_r)
            if self.descent_blocked_by is None:
                return q_hover, down, h
        return None

    def _untilt_goal(self, q_from: JointState, rise: float, held=None) -> JointState | None:
        """q_from에서 벌린 틈을 유지한 채 기울기를 풀며 같은 중점 위로 rise(mm) 올라가는 목표 자세. 기울기 변화가 커서 직선(MoveL)은 해가 튀므로 관절 보간으로 가는데,
        q_from에서 가까운 해로 풀고, 지나는 자세(가장 크게 도는 관절 SWING_STEP_DEG마다 하나)에서 방금 놓은 물체 상자(held: center·box_w·obj_h)와 두 턱 선분이 BOX_GAP_MM 안으로 들어오거나
        알려진 이웃 물체·check_goal(관절 한계·바닥·작업 영역·장면)에 걸리면 None. 경로 모양(휘두름·내려감)은 제한하지 않는다: 턱이 물체를 치지 않고 관절 한계 안이면 된다(사용자 지시 11-12)."""
        p = self.kin.fk(q_from)
        f, m = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, q_from.gripper)
        mid = (f[:2] + m[:2]) / 2
        up = self._hand_pose(p.x_mm + float(mid[0]), p.y_mm + float(mid[1]), p.yaw_deg, q_from.gripper, False, p.z_mm + float(min(f[2], m[2])) + rise)
        self.untilt_rejected = None
        goal = self.kin.ik(up, q_from)
        if goal is None:
            self.untilt_rejected = "ik"
            return None
        goal = replace(goal, gripper=q_from.gripper)
        q0, q1 = np.array(astuple(q_from), float), np.array(astuple(goal), float)
        swing = float(np.max(np.abs((q1 - q0)[:5])))
        n = max(9, math.ceil(swing / SWING_STEP_DEG) + 1)  # 가장 크게 도는 관절이 SWING_STEP_DEG마다 한 자세씩 지나도록 촘촘히 본다
        states = [JointState(*(q0 + (q1 - q0) * k)) for k in np.linspace(0.0, 1.0, n)]
        gaps = [self._box_gap(st, self.kin.fk(st), held) for st in states] if held is not None else []
        if gaps and min(gaps) < BOX_GAP_MM:
            self.untilt_rejected = f"box:{min(gaps):.0f}mm"
        elif (why := self._hits_objects(states, tolerate_start=False)) is not None:
            self.untilt_rejected = f"object:{why}"
        elif (why := next((w for a, b in itertools.pairwise(states) if (w := self.ctl.safety.check_goal(b, a))), None)) is not None:
            self.untilt_rejected = f"check_goal:{why}"
        if self.untilt_rejected:
            return None
        return goal

    def _leave_plan(self, q_from: JointState, need: float, held=None):
        """q_from에서 need(mm) 올라가는 계획 (곧게 올라가는 궤적 또는 None, 기울기를 푸는 목표 또는 None, 곧게 올라가는 높이). 못 올라가면 None.
        곧게 need 이상 올라가면 (궤적, None, 높이). 아니면 올라갈 수 있는 만큼 곧게 올라간 뒤(없으면 0) 기울기를 풀며 마저 올라간다(_untilt_goal). _leave와 열림 경로 검사(_can_leave)가 같은 계획을 쓴다."""
        self.untilt_rejected = None
        p = self.kin.fk(q_from)
        for h in self._lift_heights(need):
            if (u := self._line(q_from, replace(p, z_mm=p.z_mm + h))) is not None:
                return u, None, h
        partial = next(((h, u) for h in self._heights() if h < need and (u := self._line(q_from, replace(p, z_mm=p.z_mm + h))) is not None), None)
        for lifted, up in ([(partial[0], partial[1])] if partial else []) + [(0.0, None)]:  # 곧게 올라갈 수 있는 만큼 올라간 뒤 풀기, 안 되면 끝 자세에서 바로 풀기
            q_mid = q_from if up is None else JointState(*up.pts[-1])
            if abs(self.kin.fk(q_mid).pitch_deg + 90.0) < 1.0:  # 이미 수직이면 풀 기울기가 없다
                self.untilt_rejected = "already_vertical"
                continue
            if (goal := self._untilt_goal(q_mid, need - lifted, held)) is not None:
                return up, goal, lifted
        return None

    def _can_leave(self, st: JointState, need: float, held=None) -> bool:
        """이 끝 자세에서 놓은 뒤 need(mm)만큼 올라갈 수 있는가: _leave와 같은 계획(_leave_plan)이 있는가."""
        return self._leave_plan(st, need, held) is not None

    def _lift_heights(self, need: float) -> list[float]:
        """놓은 뒤 상승 높이 후보: max(approach_height_mm, need)에서 5mm씩 need까지 (11.3.3)."""
        top = max(float(self.cfg.approach_height_mm), need)
        hs = [float(h) for h in range(int(top), int(need) - 1, -5)]
        return hs if hs and hs[-1] <= need + 1e-9 else [*hs, float(need)]

    def _low_lift(self, lifted: float, need: float) -> MotionResult:
        """올라갈 수 있는 높이가 최소(need)에 못 미침: 다음 수평 이동을 하지 않고 그 자리에서 멈춘다 (11.3.3)."""
        self.ctl._emit("place_low_lift", "ERROR", "ARM-PLACE-LOW-LIFT", {"lift_mm": lifted, "needed_mm": need, "rejected_by": self.untilt_rejected or "no_plan"})  # 어느 검사가 기울기 풀기를 거절했는지
        return MotionResult("stopped", 0.0, ToolPose(0, 0, 0, 0), reason=f"low_lift: 놓은 뒤 {need:.0f}mm 이상 올라가야 하는데 {lifted:.0f}mm만 올라갈 수 있다")

    def _leave(self, held_gap: float, ctx: dict) -> MotionResult:
        """놓은 뒤 떠나는 수직 상승 (11.3.3 5번). need = 물체 높이 + 10mm(모르면 t_g + 30mm). 계획(_leave_plan): 곧게 need 이상 올라가거나, 올라갈 수 있는 만큼 곧게 올라간 뒤 기울기를 풀며 마저 올라간다
        (직선은 해가 튀어 관절 보간, 지나는 자세는 방금 놓은 물체 상자·알려진 물체·check_goal 검사). 계획이 없으면 ARM-PLACE-LOW-LIFT(ERROR)와 stopped(low_lift)로 끝낸다. 계획은 지금 실제 관절에서 다시 세운다."""
        need = float(ctx.get("need_lift") or 0.0)
        held = SimpleNamespace(center=ctx.get("center") or SimpleNamespace(x_mm=ctx["target"][0], y_mm=ctx["target"][1]), box_w=float(ctx.get("obj_w") or held_gap), obj_h=float(ctx.get("obj_h") or NEIGHBOUR_H_MM))  # 방금 놓은 물체 상자
        plan = self._leave_plan(self._q(), need, held)
        if plan is None:  # 풀 길이 없다: 곧게 올라갈 수 있는 만큼은 올라가(턱을 놓은 물체 옆에서 떼) 멈춘다
            q, p, lifted = self._q(), self.kin.fk(self._q()), 0.0
            for h in self._heights():
                if h < need and (u := self._line(q, replace(p, z_mm=p.z_mm + h))) is not None:
                    if (r := self._run([u])).status != "ok":
                        return r  # E-STOP·fault는 low_lift로 덮지 않는다
                    lifted = h
                    break
            return self._low_lift(lifted, need)
        up, goal, lifted = plan
        if goal is None:
            return self._run([up])
        if up is not None:
            r = self._run([up])
            if r.status != "ok":
                return r
            goal = self._untilt_goal(self._q(), need - lifted, held)  # 실제로 올라간 자세에서 다시 푼다
            if goal is None:
                return self._low_lift(lifted, need)
        return self._run([self._joint_move(goal)])

    def place_at(self, x_mm: float, y_mm: float) -> MotionResult:
        """작업대 점(x_mm, y_mm)에 놓는다. 배출 구역(place)도 이것을 거친다. 집기 기억(self.grip)은 벌림을 시작할 때 지운다: 그 전에 실패하면(하강 중 정지 등) 다시 놓을 때 쓴다."""
        try:
            return self._place_at(x_mm, y_mm)
        finally:
            self._place_ctx = None

    def _place_at(self, x_mm: float, y_mm: float) -> MotionResult:
        same = [o for o in self.placed if math.hypot(o[0] - x_mm, o[1] - y_mm) <= SAME_SPOT_MM]
        if same:  # 같은 자리에 이미 놓은 물체가 있다(배출 구역은 점 하나): 피할 수 없어 위에 놓게 된다. 설계 결정 전까지 막지 않고 경고만 남긴다 (REQ-056 후속, [기획 요청])
            self.ctl._emit("place_same_spot", "WARN", "ARM-PLACE-SAME-SPOT", {"x_mm": x_mm, "y_mm": y_mm, "count": len(same)})
            self.placed = [o for o in self.placed if o not in same]
        fail = MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        v = self._q().gripper
        grip, plan, legacy = self.grip, None, None
        if grip is None:
            legacy = "no_grip_memory"
        else:  # 새 방식(11.3.3): 집을 때와 같은 기울기로, 낮은 턱 끝을 t_g + place_drop_mm에 내린다
            yaw = self._place_yaw(x_mm, y_mm)
            allowed = self.gripper.tilt_allowed(x_mm, y_mm, yaw)
            tilted = bool(grip["tilted"]) and allowed
            release = self._hand_pose(x_mm, y_mm, yaw, v, tilted, self.cfg.table_z_mm + grip["tip_h"] + self.cfg.place_drop_mm)
            plan = self._descent(release, self._gap_mm(v) / 2.0, (x_mm, y_mm))
            legacy = None if plan else "ik_failed_new_pose"
        if legacy:  # 옛 방식: yaw 0, 기울기 0, 손 기준점 높이 = table_z + place_height_mm. 한쪽만 열리는 그리퍼라 놓는 순간의 두 턱 끝 중간점이 점에 오도록 손 xy를 보정한다 (결정 0033)
            tilted_grip = bool(grip and grip["tilted"])  # 기울여 쥔 물체가 옛 방식(수직 손, 손 높이 place_height_mm)으로 놓이면 기운 채·높이에서 놓일 수 있다: 경고로 남긴다
            self.ctl._emit("place_legacy", "WARN" if tilted_grip else "INFO", "ARM-PLACE-LEGACY", {"reason": legacy, "grip_tilted": tilted_grip})
            hand = self._hand_pose(x_mm, y_mm, 0.0, v, False, 0.0)
            release = ToolPose(hand.x_mm, hand.y_mm, self.cfg.table_z_mm + self.cfg.place_height_mm, 0.0)
            plan = self._descent(release, self._gap_mm(v) / 2.0, (x_mm, y_mm))
            if plan is None and self.hover_need_mm > MIN_CLEARANCE_MM and not self.descent_blocked_by:  # 이웃 물체 위로 10mm를 지키는 hover가 없다
                self.ctl._emit("place_low_hover", "ERROR", "ARM-PLACE-LOW-HOVER", {"x_mm": x_mm, "y_mm": y_mm, "needed_hover_mm": self.hover_need_mm})
                return MotionResult("stopped", 0.0, ToolPose(0, 0, 0, 0), reason=f"low_hover: 이웃 물체 위로 10mm를 지키려면 hover가 {self.hover_need_mm:.0f}mm 이상이어야 하는데 닿지 않는다")
            if plan is None:
                return replace(fail, reason=f"known_object: 놓을 자리가 알려진 물체와 겹친다 ({self.descent_blocked_by})") if self.descent_blocked_by else fail
            tilted, allowed = False, False
        self.last_place_correction_mm = (float(release.x_mm - x_mm), float(release.y_mm - y_mm))
        self._place_ctx = {"target": (x_mm, y_mm), "legacy": bool(legacy), "tilted": tilted, "tilt_allowed": allowed, "obj_h": 0.0 if legacy else grip["obj_h"],
                           "obj_w": 0.0 if legacy or not grip else grip.get("obj_w", 0.0),
                           "need_lift": (grip["obj_h"] + LIFT_MARGIN_MM) if grip and grip["obj_h"] > 0 else ((grip["tip_h"] if grip else self.cfg.grasp_height_mm) + LIFT_UNKNOWN_EXTRA_MM)}  # 놓은 뒤 상승의 최소: 물체 높이 + 10mm, 모르면 t_g + 30mm (11.3.3)
        q_hover, down, self._place_ctx["hover_mm"] = plan
        r = self._run([self._joint_move(q_hover), down])
        if r.status != "ok":
            return r
        try:
            self._restore_grip_torque()  # 놓는다: 예압 고정을 풀고 조임 한계를 원래 값으로(이후 닫는 일이 없다)
            self.grip = None  # 벌림을 시작한다: 집기 기억은 여기서 지운다 (11.3.2)
            held_gap = self._gap_mm(self._q().gripper)
            trajs = self._opening_path()  # 열리는 동안 손을 같이 보내 물체 위치를 지킨다
            if trajs is None:  # ④ 못 풀면 제자리에서 그냥 연다: 조용히 넘어가지 않고 이유를 남긴다 (REQ-036, 11.3)
                data = {"reason": self.opening_fallback_reason, "gripper": float(self._q().gripper), "skipped": self.opening_skipped}
                if self.opening_fallback_reason == "clearance":
                    data["blocked_by"] = self.opening_blocked_by
                self.ctl._emit("opening_path_fallback", "WARN", "ARM-OPENING-PATH-FALLBACK", data)
            q_a, t0 = self._q(), time.perf_counter()
            r = self._run(trajs or [self._gripper(self.cfg.gripper_open)])
            if trajs is not None and len(trajs[0].pts) > 1:
                self._emit_opening(trajs, q_a, self._q(), time.perf_counter() - t0)
            if r.status != "ok":
                return r
            # 열면서 손이 옮겨졌으니 원래 하강 경로를 거꾸로 타면 낮은 높이에서 release 자리로 되돌아가며 물체를 민다: 그 틈과 기울기를 유지한 채 지금 자리에서 수직으로 올린다.
            # 그리퍼는 연 채로 둔다(닫기는 호출한 쪽이 호버 높이 이상 올라온 뒤에 한다)
            try:
                return self._leave(held_gap, self._place_ctx)  # 방금 놓은 물체는 떠난 뒤에 placed에 넣는다: 떠나는 길(턱 중점이 놓는 점 12mm 안)이 그 물체에 걸리지 않게 (11-12 리뷰, _can_leave는 놓기 전이라 이미 그랬다)
            finally:
                if held_gap < self._gap_mm(min(self.cfg.gripper_open, self.cfg.joint_limits_deg["gripper"][1])) - 1.0:  # 그리퍼가 이미 열린 값(gripper_open)까지 열려 있었으면 쥔 물체가 없었다(드라이런·집기 실패): 놓은 물체로 세지 않는다
                    self.placed.append((x_mm, y_mm, held_gap / 2.0))  # 다음 놓기의 벌림이 이 물체를 피하도록 알려진 물체로 둔다
                    self.placed_height[(x_mm, y_mm)] = float(self._place_ctx.get("obj_h") or NEIGHBOUR_H_MM)
        finally:
            self._restore_grip_torque()  # 예외가 나도 조임 한계를 원래 값으로
