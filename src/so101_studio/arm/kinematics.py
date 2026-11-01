"""기구학: LeRobot RobotKinematics(placo) 래퍼 (arm-software-spec.md 6.2).

ToolPose는 작업대 좌표계(mm, 도). 로봇 원점은 작업대 면 위(z=0)에 있다고 본다.
pitch는 도구 축(gripper z축)의 수평면 기준 각도(-90 = 수직 하강), yaw는 가동 턱 쪽을 향하는 턱 축의 방향각이다
(URDF 기하로 확인: 고정 턱 안쪽 면이 gripper_frame 점이고, 턱은 도구 프레임 x축을 따라 벌어지며 가동 턱은 -x 쪽이다 → moving_jaw_dir = (cos yaw, sin yaw)).
shortcut: wrist_roll을 yaw로 읽으므로 yaw는 pitch가 -90에 가까울 때만 의미가 있다. 기울여 집기가 필요하면 확장.
"""
import math

import numpy as np
from lerobot.model.kinematics import RobotKinematics

from ..core import frames
from ..core.types import JointState, ToolPose

# URDF의 base_link 원점은 어깨 회전축보다 38.8mm 뒤에 있다(shoulder_pan 조인트 origin). 로봇 원점은 회전축 기준(spec 4절).
PAN_AXIS_X_MM = 38.8353
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _rotation(pitch_deg: float, yaw_deg: float) -> np.ndarray:
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    z = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p)])
    jaw = np.array([-math.sin(y), math.cos(y), 0.0])  # 턱 축은 항상 수평
    x = np.cross(jaw, z)
    return np.column_stack([x, jaw, z])


class Kinematics:
    def __init__(self, urdf_path: str, base: frames.Pose2D | None = None):
        self.base = base or frames.Pose2D(0, 0, 0)
        self._k = RobotKinematics(urdf_path, "gripper_frame_link", joint_names=ARM_JOINTS)
        # URDF 그리퍼 프레임은 도구 축에 대해 약 3° 돌아가 있다. 영점 자세 기준으로 상수 보정한다.
        self._C = _rotation(0, 0).T @ self._k.forward_kinematics(np.zeros(5))[:3, :3]

    def fk(self, q: JointState) -> ToolPose:
        T = self._k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))
        x, y, z = (T[:3, 3] * 1000).tolist()
        x -= PAN_AXIS_X_MM
        R = T[:3, :3] @ self._C.T
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 2]))))
        yaw = math.degrees(math.atan2(-R[0, 1], R[1, 1]))  # 도구 y축에서 구한 값. 수직 하강이면 턱 축(-x)의 방향각과 같다
        return frames.tool_robot_to_table(ToolPose(x, y, z, yaw, pitch), self.base)

    def moving_jaw_dir(self, q: JointState) -> tuple[float, float]:
        """고정 턱에서 가동 턱으로 향하는 방향(작업대 좌표계, 수평 성분의 단위 벡터)."""
        v = -(self._k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))[:3, :3] @ self._C.T)[:, 0]
        c, s = math.cos(math.radians(self.base.yaw_deg)), math.sin(math.radians(self.base.yaw_deg))
        x, y = c * v[0] - s * v[1], s * v[0] + c * v[1]
        n = math.hypot(x, y)
        return x / n, y / n

    def ik(self, pose: ToolPose, seed: JointState, tol_mm: float = 2.0, max_iter: int = 30,
           orientation_weight: float = 0.1) -> JointState | None:
        r = frames.tool_table_to_robot(pose, self.base)
        T = np.eye(4)
        T[:3, :3] = _rotation(r.pitch_deg, r.yaw_deg) @ self._C
        T[:3, 3] = np.array([r.x_mm + PAN_AXIS_X_MM, r.y_mm, r.z_mm]) / 1000
        q = np.array([getattr(seed, j) for j in ARM_JOINTS], float)
        for _ in range(max_iter):  # placo는 호출 1번에 한 걸음만 가므로 수렴할 때까지 반복
            nxt = self._k.inverse_kinematics(q, T, orientation_weight=orientation_weight)[:5]
            done = np.abs(nxt - q).max() < 1e-3
            q = nxt
            if done:
                break
        out = JointState(*q.tolist(), seed.gripper)
        got = self.fk(out)  # 해가 목표에 닿았는지 확인 (수치 해가 어긋나면 None)
        if math.dist((got.x_mm, got.y_mm, got.z_mm), (pose.x_mm, pose.y_mm, pose.z_mm)) > tol_mm:
            return None
        return out
