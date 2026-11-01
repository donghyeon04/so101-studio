"""기능: home, park, pick, place (arm-software-spec.md 6.6). 동작 계획은 MoveJ(관절 보간)와 MoveL(직선)의 조합이다."""
from dataclasses import replace

import numpy as np

from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .trajectory import move_j, move_l

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
        return move_j(self._q(), goal, self.cfg.joint_v_max_deg_s * self.speed / 100, self.cfg.a_max_deg_s2)

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
        return move_l(self.kin, self.kin.fk(start_q), goal, start_q, self.cfg.tool_v_max_mm_s * self.speed / 100)

    def gripper_for_width(self, width_mm: float) -> float:
        """벌어진 폭(mm) -> 그리퍼 값. 대응표는 robot.yaml의 gripper_width_mm (T7에서 측정, 지금은 자리표시)."""
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        lo, hi = self.cfg.joint_limits_deg["gripper"]
        return float(np.clip(np.interp(width_mm, [p[0] for p in pts], [p[1] for p in pts]), lo, hi))

    def home(self) -> MotionResult:
        return self._run([self._joint_move(JointState(*self.cfg.poses["home"]))])

    def park(self) -> MotionResult:
        return self._run([self._joint_move(JointState(*self.cfg.poses["park"]))])

    def pick(self, obj: DetectedObject) -> MotionResult:
        yaw = obj.angle_deg or 0.0  # 턱이 벌어지는 방향이 긴 변에 수직이 되도록: tool yaw = 물체 각도 (spec 6.6의 +90°와 같은 뜻)
        grasp = ToolPose(obj.x_mm, obj.y_mm, self.cfg.grasp_height_mm, yaw)
        hover = replace(grasp, z_mm=grasp.z_mm + self.cfg.approach_height_mm)
        q_hover = self._ik(hover)
        if q_hover is None:
            return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0))
        q_open = replace(q_hover, gripper=self.gripper_for_width(obj.width_mm + self.cfg.gripper_margin_mm))
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
        r = self._run([self._gripper(self.cfg.joint_limits_deg["gripper"][0])])  # 4. 닫기
        if r.status != "ok":
            return r
        up = self._line(self._q(), hover)  # 5. 상승
        return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0)) if up is None else self._run([up])

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
        up = self._line(self._q(), hover)
        return MotionResult("ik_fail", 0.0, ToolPose(0, 0, 0, 0)) if up is None else self._run([up])
