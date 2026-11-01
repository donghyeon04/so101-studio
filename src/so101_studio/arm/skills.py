"""기능: home, park, pick, place (arm-software-spec.md 6.6). 동작 계획은 MoveJ(관절 보간)와 MoveL(직선)의 조합이다."""
import math
from dataclasses import replace

import numpy as np

from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .trajectory import cfg_move_j, cfg_move_l

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

    def pick(self, obj: DetectedObject) -> MotionResult:
        """고정 턱 1개 + 가동 턱 1개라서 닫으면 물체가 고정 턱 쪽으로 밀린다. URDF 기하로 보면 FK 기준점(gripper_frame)은
        고정 턱 안쪽 면 위에 있고 가동 턱은 방향 m=(cos yaw, sin yaw) 쪽이다. 그래서
        (1) 턱을 폭+여유로 열고 턱 중심이 물체에 오도록 목표를 (벌림/2)만큼 -m 쪽으로 옮겨 내려가고 (NVIDIA 포럼의 grasp_center 방식),
        (2) 닫기 직전에 손을 +m 쪽으로 여유/2만큼 밀어 고정 턱 면을 물체 면에 붙인 뒤 가동 턱만 닫는다 (F1 Hand '닫으며 중심 쪽으로 이동', 고정 턱 기준)."""
        yaw = self._jaw_yaw((obj.angle_deg or 0.0) + 90.0)  # 턱 축이 물체 긴 변에 수직이 되도록 (spec 6.6: 물체 각도 + 90°)
        m = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        gap = obj.width_mm + self.cfg.gripper_margin_mm
        shift = -gap / 2 + self.cfg.grasp_bias_mm  # 턱 중심이 물체 중심에 오도록
        grasp = ToolPose(obj.x_mm + shift * m[0], obj.y_mm + shift * m[1], self.cfg.grasp_height_mm, yaw)
        nudge_mm = self.cfg.gripper_margin_mm / 2
        touch = replace(grasp, x_mm=grasp.x_mm + nudge_mm * m[0], y_mm=grasp.y_mm + nudge_mm * m[1])
        hover = replace(grasp, z_mm=grasp.z_mm + self.cfg.approach_height_mm)
        q_hover = self._ik(hover)
        if q_hover is None:
            return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        q_open = replace(q_hover, gripper=self.gripper_for_width(gap))
        q_down = self.kin.ik(grasp, q_open)
        down = self._line(q_open, grasp) if q_down is not None else None
        if down is None:
            return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        r = self._run([self._joint_move(q_hover)])  # 1. 접근 위치로
        if r.status != "ok":
            return r
        r = self._run([self._gripper(q_open.gripper), down])  # 2. 열기 3. 하강
        if r.status != "ok":
            return r
        nudge = self._line(self._q(), touch)  # 4. 고정 턱 면이 물체에 닿도록 +m 쪽으로
        if nudge is None:
            return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        r = self._run([nudge])
        if r.status != "ok":
            return r
        r = self._run([self._gripper(self.cfg.joint_limits_deg["gripper"][0])])  # 5. 닫기
        if r.status != "ok":
            return r
        return self._run([down.reversed(self._q().gripper)])  # 6. 내려간 경로로 상승 (IK를 다시 풀지 않는다)

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
