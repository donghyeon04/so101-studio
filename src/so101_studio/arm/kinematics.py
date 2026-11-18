"""기구학: LeRobot RobotKinematics(placo) 래퍼 (arm-software-spec.md 6.2).

ToolPose는 작업대 좌표계(mm, 도). 로봇 원점은 작업대 면 위(z=0)에 있다고 본다.
pitch는 도구 축(gripper z축)의 수평면 기준 각도(-90 = 수직 하강), yaw는 가동 턱 쪽을 향하는 턱 축의 방향각이다
(URDF 기하로 확인: 고정 턱 안쪽 면이 gripper_frame 점이고, 턱은 도구 프레임 x축을 따라 벌어지며 가동 턱은 -x 쪽이다 → moving_jaw_dir = (cos yaw, sin yaw)).
shortcut: wrist_roll을 yaw로 읽으므로 yaw는 pitch가 -90에 가까울 때만 의미가 있다. 기울여 집기가 필요하면 확장.
"""
import functools
import math
import threading

import numpy as np
from lerobot.model.kinematics import RobotKinematics

from ..core import frames
from ..core.types import JointState, ToolPose

# URDF의 base_link 원점은 어깨 회전축보다 38.8mm 뒤에 있다(shoulder_pan 조인트 origin). 로봇 원점은 회전축 기준(spec 4절).
PAN_AXIS_X_MM = 38.8353
# URDF 메쉬로 계산한 가동 턱 끝 궤적(gripper_link 좌표, mm): 관절각 q에서 턱 끝의 x(가동 턱 쪽 +)와 아래로의 거리 b=-z.
# 고정 턱 끝은 (x=-7.9, b=105.4). gripper_frame 점은 (x=-7.9, b=98.1)이다.
JAW_Q_DEG = [-10, 0, 20, 40, 60, 80, 100]
JAW_TIP_A = [-5.2, 8.9, 37.6, 64.3, 85.6, 99.0, 102.9]
JAW_TIP_B = [102.2, 105.4, 104.3, 93.5, 74.2, 48.8, 20.3]
FIXED_TIP = (-7.9, 105.4)
FRAME_B = 98.1
VALUE_TO_Q = (-10.0, 1.214)  # 그리퍼 값 v -> 관절각 q = -10 + 1.214 v (카드 시험 2점에 맞춘 가정)
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _rotation(pitch_deg: float, yaw_deg: float) -> np.ndarray:
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    z = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p)])
    jaw = np.array([-math.sin(y), math.cos(y), 0.0])  # 턱 축은 항상 수평
    x = np.cross(jaw, z)
    return np.column_stack([x, jaw, z])


def locked(f):
    """placo 풀이기(kin._k)는 호출마다 내부 상태를 바꿔서 스레드 둘이 같이 쓰면 IK가 가끔 틀린 해를 낸다(부하에서 ik_fail). Kinematics(또는 .kin이 있는 그리퍼)의 lock으로 한 번에 하나만 쓴다."""
    @functools.wraps(f)
    def wrapper(self, *a, **kw):
        with (self.lock if hasattr(self, "lock") else self.kin.lock):
            return f(self, *a, **kw)
    return wrapper


class Kinematics:
    def __init__(self, urdf_path: str, base: frames.Pose2D | None = None, limits: dict | None = None):
        self.base = base or frames.Pose2D(0, 0, 0)
        self.lock = threading.RLock()
        self._k = RobotKinematics(urdf_path, "gripper_frame_link", joint_names=ARM_JOINTS)
        for j in ARM_JOINTS if limits else []:  # 풀이기가 우리 관절 한계 안에서 해를 찾게 한다(한계 밖 해를 만들고 나중에 거부하지 않도록)
            self._k.robot.set_joint_limits(j, math.radians(limits[j][0]), math.radians(limits[j][1]))
        # URDF 그리퍼 프레임은 도구 축에 대해 약 3° 돌아가 있다. 영점 자세 기준으로 상수 보정한다.
        self._C = _rotation(0, 0).T @ self._k.forward_kinematics(np.zeros(5))[:3, :3]
        from .gripper import (
            SingleJawGripper,  # gripper.py가 이 모듈의 상수를 가져다 쓰므로 순환을 피해 여기서 읽는다
        )

        self.gripper = SingleJawGripper(self)  # 기하만 쓰는 기본 그리퍼. Skills가 설정을 가진 것으로 바꿔 끼운다

    @locked
    def fk(self, q: JointState) -> ToolPose:
        T = self._k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))
        x, y, z = (T[:3, 3] * 1000).tolist()
        x -= PAN_AXIS_X_MM
        R = T[:3, :3] @ self._C.T
        yaw = math.degrees(math.atan2(-R[0, 1], R[1, 1]))
        h = R[0, 2] * math.cos(math.radians(yaw)) + R[1, 2] * math.sin(math.radians(yaw))  # 도구 축의 수평 성분(진행 방향 기준)
        pitch = math.degrees(math.atan2(R[2, 2], h))  # 부호를 살린다: -90 수직, -115는 가동 턱 반대쪽으로 25° 기울임  # 도구 y축에서 구한 값. 수직 하강이면 턱 축(-x)의 방향각과 같다
        return frames.tool_robot_to_table(ToolPose(x, y, z, yaw, pitch), self.base)

    def jaw_lowest_z(self, q: JointState) -> float:
        """턱 메쉬의 가장 낮은 꼭짓점 높이(모델 z, mm). 구현은 그리퍼에 있다 (arm/gripper.py)."""
        return self.gripper.lowest_z(q)

    def jaw_tip_offsets(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float):
        """기준점(gripper_frame)에서 본 두 턱 끝의 위치 벡터(작업대 좌표계, mm). 구현은 그리퍼에 있다 (arm/gripper.py)."""
        return self.gripper.jaw_tips(pitch_deg, yaw_table_deg, gripper_value)

    def tcp_offset(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float) -> np.ndarray:
        """기준점에서 손끝(TCP)까지의 벡터(작업대 좌표계, mm). 단동식은 고정 턱 끝, 병렬은 두 턱 끝 가운데(결정 0043-1: 벌림에 따라 움직이지 않는다).
        좌표 이동·tool_position·follow_path(service/targets.tip_vector)와 link_points가 이 한 곳을 쓴다."""
        f, m = self.jaw_tip_offsets(pitch_deg, yaw_table_deg, gripper_value)
        return (np.asarray(f, float) + np.asarray(m, float)) / 2 if self.gripper.name == "parallel" else np.asarray(f, float)

    def jaw_tips_at(self, q: JointState) -> tuple[np.ndarray, np.ndarray]:
        """두 턱 끝의 절대 위치(작업대 xy, 모델 z, mm). 병렬은 실제 손 방향으로 만들어 wrist_roll을 돌린 기운 자세(6.6.1)에서도 맞다."""
        if hasattr(self.gripper, "tips_at"):
            return self.gripper.tips_at(q)
        p = self.fk(q)
        f, m = self.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, q.gripper)
        o = np.array([p.x_mm, p.y_mm, p.z_mm])
        return o + np.asarray(f, float), o + np.asarray(m, float)

    @locked
    def link_points(self, q: JointState) -> list[tuple[float, float]]:
        """어깨(위팔 관절)·팔꿈치·손목·손끝(tcp_offset: 단동식 고정 턱 끝, 병렬 두 턱 끝 가운데)의 작업대 xy(mm). 레이더가 팔이 가린 자리를 지울 때 쓴다."""
        tip = self.fk(q)
        f = self.tcp_offset(tip.pitch_deg, tip.yaw_deg, q.gripper)
        self._k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))  # 프레임 위치는 마지막 FK 상태에서 읽는다
        out = []
        for name in ("upper_arm_link", "lower_arm_link", "wrist_link"):
            t = self._k.robot.get_T_world_frame(name)[:3, 3] * 1000
            out.append(frames.robot_to_table(float(t[0]) - PAN_AXIS_X_MM, float(t[1]), self.base))
        out.append((tip.x_mm + float(f[0]), tip.y_mm + float(f[1])))
        return out

    @locked
    def tool_axes(self, q: JointState) -> np.ndarray:
        """도구 프레임의 x·y·z 축(열)을 작업대 좌표계 벡터로 돌려준다. z = 도구 앞(턱 끝 방향), x = 턱이 벌어지는 축(가동 턱은 -x 쪽)."""
        R = self._k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))[:3, :3] @ self._C.T
        c, s = math.cos(math.radians(self.base.yaw_deg)), math.sin(math.radians(self.base.yaw_deg))
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R

    @locked
    def moving_jaw_dir(self, q: JointState) -> tuple[float, float]:
        """고정 턱에서 가동 턱으로 향하는 방향(작업대 좌표계, 수평 성분의 단위 벡터)."""
        v = -(self._k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))[:3, :3] @ self._C.T)[:, 0]
        c, s = math.cos(math.radians(self.base.yaw_deg)), math.sin(math.radians(self.base.yaw_deg))
        x, y = c * v[0] - s * v[1], s * v[0] + c * v[1]
        n = math.hypot(x, y)
        return x / n, y / n

    @locked
    def step(self, q: JointState, dx_mm: float, dy_mm: float, dz_mm: float, dyaw_deg: float = 0.0, tol_mm: float = 0.5, dpitch_deg: float = 0.0) -> JointState | None:
        """지금 자세의 방향을 유지한 채 끝을 (dx, dy, dz)(작업대 좌표계 mm)만큼 옮기고 수직축으로 dyaw만큼 돌린다(원격 조종용).
        pitch·yaw로 방향을 다시 만들지 않으므로 손목이 돌아가 있어도(wrist_roll) 동작한다."""
        arm = np.array([getattr(q, j) for j in ARM_JOINTS], float)
        T = self._k.forward_kinematics(arm).copy()
        c, s = math.cos(math.radians(self.base.yaw_deg)), math.sin(math.radians(self.base.yaw_deg))
        d_robot = np.array([c * dx_mm + s * dy_mm, -s * dx_mm + c * dy_mm, dz_mm]) / 1000.0  # 작업대 -> 로봇 좌표
        T[:3, 3] += d_robot
        if dyaw_deg:
            a = math.radians(dyaw_deg)
            Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
            T[:3, :3] = Rz @ T[:3, :3]
        if dpitch_deg:  # 턱이 벌어지는 수평축을 기준으로 도구 앞(손끝 방향)을 위·아래로 기울인다. 양수 = 수평 쪽으로 든다(-90 아래 -> 0 수평)
            jaw = (T[:3, :3] @ self._C.T)[:, 1]  # 도구 y축 = 항상 수평인 턱 축(fk의 yaw가 이 축에서 나온다)
            jaw = np.array([jaw[0], jaw[1], 0.0]) / (math.hypot(jaw[0], jaw[1]) or 1.0)
            a = -math.radians(dpitch_deg)  # 축 방향 때문에 부호를 뒤집는다: 양수 dpitch가 fk의 pitch를 늘린다
            K = np.array([[0, -jaw[2], jaw[1]], [jaw[2], 0, -jaw[0]], [-jaw[1], jaw[0], 0]])
            Rp = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)
            T[:3, :3] = Rp @ T[:3, :3]
        # 5축이라 도구가 수평일 때는 방향을 그대로 둔 채 옆으로 옮길 수 없다. 방향 가중치를 점점 낮춰(위치 우선) 풀리는 첫 해를 쓴다
        for w in (0.1, 0.01, 0.0):
            cur = arm
            self._sync(cur, q.gripper)
            for _ in range(8):
                nxt = self._k.inverse_kinematics(cur, T, orientation_weight=w)[:5]
                done = np.abs(nxt - cur).max() < 1e-3
                cur = nxt
                if done:
                    break
            got = self._k.forward_kinematics(cur)[:3, 3]
            if np.linalg.norm(got - T[:3, 3]) * 1000 <= tol_mm:
                return JointState(*cur.tolist(), q.gripper)
        return None

    def _sync(self, arm: np.ndarray, gripper_value: float) -> None:
        """풀이기를 시작하기 전에 팔·그리퍼 관절과 링크 변환(update_kinematics)을 이 자세로 맞춘다. placo는 한 걸음을 마지막 update_kinematics의 변환으로 계산하고 그리퍼 관절도
        움직이는 변수로 보므로, 그 사이에 다른 호출(상태 스레드의 fk, 턱 높이 계산)이 남긴 값에 따라 해가 달라져 부하에서 가끔 ik_fail이 났다."""
        self.gripper.set_joints(self._k.robot, gripper_value)
        self._k.forward_kinematics(arm)

    @locked
    def ik(self, pose: ToolPose, seed: JointState, tol_mm: float = 3.0, max_iter: int = 30,
           orientation_weight: float = 0.1) -> JointState | None:
        r = frames.tool_table_to_robot(pose, self.base)
        T = np.eye(4)
        T[:3, :3] = _rotation(r.pitch_deg, r.yaw_deg) @ self._C
        T[:3, 3] = np.array([r.x_mm + PAN_AXIS_X_MM, r.y_mm, r.z_mm]) / 1000
        q = np.array([getattr(seed, j) for j in ARM_JOINTS], float)
        self._sync(q, seed.gripper)
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
