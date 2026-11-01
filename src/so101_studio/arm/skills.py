"""기능: home, park, pick, place (arm-software-spec.md 6.6). 동작 계획은 MoveJ(관절 보간)와 MoveL(직선)의 조합이다."""
import math
import time
from dataclasses import astuple, replace

import numpy as np

from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .trajectory import LineTrajectory, QuinticShape, cfg_move_j, cfg_move_l

SEEDS = [JointState(0, -30, 30, 60, 0, 0), JointState(0, -60, 60, 30, 0, 0), JointState(0, 0, 60, 30, 0, 0)]


class Skills:
    def __init__(self, ctl, kin, cfg, ws, speed_pct: float | None = None):
        self.ctl, self.kin, self.cfg, self.ws = ctl, kin, cfg, ws
        self.speed = speed_pct if speed_pct is not None else cfg.speed_pct_default

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

    def _closing_path(self, q_start: JointState, grasp: ToolPose, m: tuple[float, float], gap0: float, gap_end: float):
        """닫으면서 손을 가동 턱 쪽으로 같이 보내는 한 개의 경로. 틈이 d만큼 줄 때 손은 d/2만큼 간다.
        그러면 틈의 중심이 물체 중심에 계속 머물러서, 닫는 동안 내내 가운데에 있는 물체를 양쪽에서 잡는 모양이 된다
        (고정 턱은 +m 쪽으로 d/2, 가동 턱은 -m 쪽으로 d/2 다가온다)."""
        length = (gap0 - gap_end) / 2
        n = max(1, math.ceil(length / 3.0))
        pts, q = [np.array(astuple(replace(q_start, gripper=self.gripper_for_width(gap0))), float)], q_start
        for i in range(1, n + 1):
            s = length * i / n
            q = self.kin.ik(replace(grasp, x_mm=grasp.x_mm + s * m[0], y_mm=grasp.y_mm + s * m[1]), q)
            if q is None:
                return None
            pts.append(np.array(astuple(replace(q, gripper=self.gripper_for_width(gap0 - 2 * s))), float))
        return LineTrajectory(np.array(pts), QuinticShape(max(1.0, length / 8.0)))  # 손 8mm/s, 틈 16mm/s로 천천히

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

    def pick(self, obj: DetectedObject) -> MotionResult:
        """고정 턱 1개 + 가동 턱 1개라서 닫으면 물체가 고정 턱 쪽으로 밀린다. URDF 기하로 보면 FK 기준점(gripper_frame)은
        고정 턱 안쪽 면 위에 있고 가동 턱은 방향 m=(cos yaw, sin yaw) 쪽이다. 그래서 (결정 0009, 0010)
        (1) 틈을 벌리고(기본: 최대) 그 틈의 중심이 물체에 오도록 목표를 (벌림/2)만큼 -m 쪽으로 옮겨 내려간다 (NVIDIA 포럼의 grasp_center 방식),
        (2) 닫을 때 틈이 d만큼 줄면 손을 +m 쪽으로 d/2만큼 같이 보내 중심을 유지하고(F1 Hand '닫으며 중심 쪽으로 이동'),
        (3) 그리퍼가 명령보다 앞서 멈추면(접촉) 그 자리에서 정지한다. 물체 크기를 몰라도 된다."""
        yaw = self._jaw_yaw((obj.angle_deg or 0.0) + 90.0)  # 턱 축이 물체 긴 변에 수직이 되도록 (spec 6.6: 물체 각도 + 90°)
        m = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        known = obj.width_mm > 0
        gap_min = min(mm for _, mm in self.cfg.gripper_width_mm)
        gap0 = self.cfg.grasp_max_open_mm if self.cfg.grasp_open_policy == "max" or not known else obj.width_mm + self.cfg.gripper_margin_mm
        gap_end = max(obj.width_mm - 10.0, gap_min) if known else gap_min  # 알면 폭보다 10mm 더 조이는 지점까지(접촉 감지에 필요한 명령 차이 4값 ≈ 6mm 이상), 모르면 끝까지
        shift = -gap0 / 2 + self.cfg.grasp_bias_mm
        grasp = ToolPose(obj.x_mm + shift * m[0], obj.y_mm + shift * m[1], self.cfg.grasp_height_mm, yaw)
        hover = replace(grasp, z_mm=grasp.z_mm + self.cfg.approach_height_mm)
        fail = MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        q_hover = self._ik(hover)
        if q_hover is None:
            return fail
        q_open = replace(q_hover, gripper=self.gripper_for_width(gap0))
        down = self._line(q_open, grasp) if self.kin.ik(grasp, q_open) is not None else None
        if down is None:
            return fail
        closing = self._closing_path(JointState(*down.pts[-1]), grasp, m, gap0, gap_end)
        if closing is None:
            return fail
        r = self._run([self._joint_move(q_hover)])  # 1. 접근 위치로
        if r.status != "ok":
            return r
        r = self._run([self._gripper(q_open.gripper), down])  # 2. 열기 3. 하강
        if r.status != "ok":
            return r
        self.ctl.driver.set_gripper_torque_limit(self.cfg.gripper_squeeze_torque)  # 조임 힘을 낮춰 얇은 물체가 미끄러져 빠지지 않게
        try:
            contact, r = self._close_until_contact(closing)  # 4. 닫으며 손을 같이 보낸다
        finally:
            self.ctl.driver.set_gripper_torque_limit(500)
        if r.status not in ("ok", "stopped"):
            return r
        here = self.kin.fk(self._q())
        up = self._line(self._q(), replace(here, z_mm=hover.z_mm))  # 5. 수직으로 상승
        if up is None:
            return fail
        r = self._run([up])
        return r if r.status != "ok" or contact else MotionResult("no_object", r.duration_s, r.final_tool)

    def place(self, zone: str) -> MotionResult:
        z = self.ws.drop_zones[zone]
        release = ToolPose(z["x_mm"], z["y_mm"], self.cfg.place_height_mm, 0.0)
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
        r = self._run([self._gripper(self.cfg.gripper_open)])
        if r.status != "ok":
            return r
        return self._run([down.reversed(self._q().gripper)])
