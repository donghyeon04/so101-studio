"""그리퍼 인터페이스와 단동식 핑거 그리퍼 구현 (work-queue 5-3 2단계, 동작 변경 없음).

단동식(SO-101 기본): 고정 턱 1개 + 가동 턱 1개라 한쪽 턱만 열리고, 두 턱 끝의 중간점이 손 기준점(gripper_frame)에서 멀어서
집기·놓기에서 손 위치를 보정해야 한다(결정 0009~0012, 0033). 이 보정·기울기·경로·메쉬 지식을 전부 여기 둔다.
다른 그리퍼(병렬 등)는 같은 이름의 메서드를 가진 클래스를 더하면 된다: 병렬은 needs_centering False, tilt 0, narrowing 0, 닫기·열기 경로가 그리퍼 축만 움직임."""
import functools
import math
import struct
from dataclasses import astuple, replace
from pathlib import Path
from typing import Protocol

import numpy as np

from ..core import frames
from ..core.types import JointState, ToolPose
from .kinematics import (
    ARM_JOINTS,
    FIXED_TIP,
    FRAME_B,
    JAW_Q_DEG,
    JAW_TIP_A,
    JAW_TIP_B,
    VALUE_TO_Q,
    _rotation,
)

JAW_PIVOT = (20.24, 23.45)  # 가동 턱 회전축 (JAW_TIP_A/B 표의 원 맞춤, 반지름 82.7mm, 잔차 0.05mm 이하): 턱 선분 검사용 (arm-software-spec 11.3.3)
from .trajectory import LineTrajectory, QuinticShape


class Gripper(Protocol):
    name: str
    needs_centering: bool  # 두 턱 끝 중간점이 손 기준점과 달라서 집기·놓기에서 손 위치를 보정해야 하는가

    def width_to_value(self, width_mm: float) -> float: ...  # 벌림 폭(mm) -> 그리퍼 값
    def jaw_tips(self, pitch_deg: float, yaw_deg: float, v: float): ...  # 기준점에서 본 두 턱 끝 (작업대 좌표계 mm)
    def center_offset(self, pitch_deg: float, yaw_deg: float, v: float) -> np.ndarray: ...  # 기준점에서 본 두 턱 끝 중간점
    def set_joints(self, robot, v: float) -> None: ...  # placo 로봇의 그리퍼 관절(들)을 그리퍼 값 v에 맞춘다
    def lowest_z(self, q: JointState) -> float: ...  # 턱 메쉬의 가장 낮은 점(모델 z, mm)
    def tilt(self, v: float) -> float: ...  # 그리퍼 값 v에서의 손 기울기(°)
    def narrowing(self, v: float, tilt_ok: bool) -> float: ...  # 턱 끝에서 위로 1mm 갈 때 턱 사이 간격이 줄어드는 mm
    def hand_pose(self, ox: float, oy: float, yaw_t: float, v: float, tilt_ok: bool, tip_z: float, tilt_scale: float = 1.0) -> ToolPose: ...
    def closing_path(self, q_start: JointState, center, yaw_t: float, tilt_ok: bool, v0: float, v_end: float, gap0: float, gap_end: float, tip_z: float, speed: float = 1.0, duration_s: float | None = None, tilt_scale=(1.0, 1.0)): ...
    def jaw_segments(self, pitch_deg: float, yaw_deg: float, v: float): ...  # 두 턱 선분 ((고정 시작, 고정 끝), (가동 시작, 가동 끝)): 기준점에서 본 작업대 좌표 벡터(mm)


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _stl_vertices(path: Path) -> np.ndarray:
    b = path.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    tri = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return np.unique(tri["v"].reshape(-1, 3).astype(float).round(6), axis=0)


# 턱 메쉬(URDF 단위 m): (링크 이름, STL 파일, visual origin xyz, rpy). 고정 턱은 손 링크(gripper_link)의 wrist_roll_follower 메쉬에 있다
_JAW_VISUALS = (
    ("gripper_link", "wrist_roll_follower_so101_v1", (8.32667e-17, -0.000218214, 0.000949706), (-3.14159, -5.55112e-17, 0.0)),
    ("moving_jaw_so101_v1_link", "moving_jaw_so101_v1", (-5.55112e-17, -5.55112e-17, 0.0189), (9.53145e-17, 6.93889e-18, 1.24077e-24)),
)


@functools.lru_cache(maxsize=1)
def _jaw_meshes() -> tuple:
    root = Path(__file__).resolve().parents[3] / "assets" / "assets"
    out = []
    for link, stl, xyz, rpy in _JAW_VISUALS:
        v = _stl_vertices(root / f"{stl}.stl")
        out.append((link, v @ _rpy(*rpy).T + np.array(xyz)))
    return tuple(out)


class SingleJawGripper:
    name = "single_jaw"
    needs_centering = True

    def __init__(self, kin, cfg=None):
        """kin: 도구 프레임 보정(_C)·로봇 기준(base)·placo 로봇을 가진 Kinematics. cfg: 폭 표·기울기 설정(기하만 쓰면 없어도 된다)."""
        self.kin, self.cfg = kin, cfg
        self._jaw_ready = False

    # ---- 폭 <-> 값
    def width_to_value(self, width_mm: float) -> float:
        """벌어진 폭(mm) -> 그리퍼 값. 대응표는 robot.yaml의 gripper_width_mm (T7에서 측정)."""
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        lo, hi = self.cfg.joint_limits_deg["gripper"]
        return float(np.clip(np.interp(width_mm, [p[0] for p in pts], [p[1] for p in pts]), lo, hi))

    @property
    def min_gap_mm(self) -> float:
        return min(mm for _, mm in self.cfg.gripper_width_mm)

    # ---- 기하
    def jaw_tips(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float):
        """기준점(gripper_frame, 고정 턱 면)에서 본 두 턱 끝의 위치 벡터(작업대 좌표계, mm). 자세(pitch, yaw)와 그리퍼 값으로 정해진다."""
        q = VALUE_TO_Q[0] + VALUE_TO_Q[1] * gripper_value
        a_m = float(np.interp(q, JAW_Q_DEG, JAW_TIP_A))
        b_m = float(np.interp(q, JAW_Q_DEG, JAW_TIP_B))
        R = _rotation(pitch_deg, yaw_table_deg - self.kin.base.yaw_deg) @ self.kin._C  # 로봇 좌표계에서의 도구 프레임
        c, s = math.cos(math.radians(self.kin.base.yaw_deg)), math.sin(math.radians(self.kin.base.yaw_deg))
        Rt = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R
        # 도구 프레임 좌표: 가동 턱 쪽이 -x, 도구 축(턱 끝 방향)이 +z
        f = Rt @ np.array([-(FIXED_TIP[0] + 7.9), 0.0, FIXED_TIP[1] - FRAME_B])
        m = Rt @ np.array([-(a_m + 7.9), 0.0, b_m - FRAME_B])
        return f, m

    def jaw_segments(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float):
        """두 턱 선분(REQ-053): 고정 턱은 기준점 -> 고정 턱 끝, 가동 턱은 회전축 -> 가동 턱 끝. 기준점에서 본 작업대 좌표계 벡터(mm)."""
        f, m = self.jaw_tips(pitch_deg, yaw_table_deg, gripper_value)
        R = _rotation(pitch_deg, yaw_table_deg - self.kin.base.yaw_deg) @ self.kin._C
        c, s = math.cos(math.radians(self.kin.base.yaw_deg)), math.sin(math.radians(self.kin.base.yaw_deg))
        pivot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R @ np.array([-(JAW_PIVOT[0] + 7.9), 0.0, JAW_PIVOT[1] - FRAME_B])
        return (np.zeros(3), f), (pivot, m)

    def center_offset(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float) -> np.ndarray:
        """기준점에서 본 두 턱 끝 중간점(작업대 좌표계 mm)."""
        f, m = self.jaw_tips(pitch_deg, yaw_table_deg, gripper_value)
        return (f + m) / 2

    def set_joints(self, robot, v: float) -> None:
        """placo 로봇의 그리퍼 관절을 그리퍼 값 v에 맞춘다(단동식: gripper 회전 관절, rad)."""
        robot.set_joint("gripper", math.radians(min(max(VALUE_TO_Q[0] + VALUE_TO_Q[1] * v, -10.0), 100.0)))  # URDF gripper 관절 범위

    def lowest_z(self, q: JointState) -> float:
        """고정 턱(손 메쉬)과 움직이는 턱 메쉬의 가장 낮은 꼭짓점 높이(모델 z, mm; fk의 z_mm와 같은 기준). 기울기와 벌림을 반영한 실제 턱 몸체 아래 모서리까지 본다 (REQ-036)."""
        meshes = _jaw_meshes()  # 메쉬를 먼저 읽는다(관절을 맞춘 뒤 처음 읽을 때 첫 결과가 틀리게 나온 일이 있었다)
        k = self.kin._k
        robot = k.robot
        arm = np.array([getattr(q, j) for j in ARM_JOINTS], float)
        for _ in range(1 if self._jaw_ready else 2):  # placo는 gripper 관절이 처음 들어간 첫 갱신에서 링크 변환이 낡은 채 나온다: 처음에는 두 번 맞춘다
            k.forward_kinematics(arm)
            self.set_joints(robot, q.gripper)
            robot.update_kinematics()
        self._jaw_ready = True
        low = []
        for link, verts in meshes:
            T = robot.get_T_world_frame(link)
            low.append(float((verts @ T[2, :3] + T[2, 3]).min()) * 1000.0)
        return min(low)

    # ---- 기울기
    def tilt_allowed(self, x_mm: float, y_mm: float, yaw_t: float) -> bool:
        """턱 축이 팔이 뻗은 방향(방사 방향)과 거의 나란할 때만 기울일 수 있다."""
        if self.cfg.grasp_tilt_deg <= 0:
            return False
        xr, yr = frames.table_to_robot(x_mm, y_mm, self.kin.base)
        radial = math.degrees(math.atan2(yr, xr))
        diff = (yaw_t - self.kin.base.yaw_deg - radial + 90.0) % 180.0 - 90.0  # 턱 축은 180° 대칭
        return abs(diff) <= self.cfg.grasp_tilt_tol_deg

    def tilt(self, v: float) -> float:
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

    def narrowing(self, v: float, tilt_ok: bool) -> float:
        """턱 끝에서 위로 1mm 올라갈 때 턱 사이 간격이 줄어드는 mm: tan(기울기) + tan(벌림각 - 기울기) (결정 0042)."""
        tau = self.tilt(v) if tilt_ok else 0.0
        return math.tan(math.radians(tau)) + math.tan(math.radians(VALUE_TO_Q[1] * v - tau))

    # ---- 손 자세와 경로
    def hand_pose(self, ox: float, oy: float, yaw_t: float, v: float, tilt_ok: bool, tip_z: float, tilt_scale: float = 1.0) -> ToolPose:
        """두 턱 끝의 중간점이 (ox, oy)에 오고 낮은 턱 끝이 tip_z에 오는 손 자세(기준점 gripper_frame의 위치)."""
        pitch = -90.0 - (self.tilt(v) * tilt_scale if tilt_ok else 0.0)
        f, m = self.jaw_tips(pitch, yaw_t, v)
        mid = (f[:2] + m[:2]) / 2
        return ToolPose(ox - mid[0], oy - mid[1], tip_z - min(f[2], m[2]), yaw_t, pitch)

    def closing_path(self, q_start: JointState, center, yaw_t: float, tilt_ok: bool, v0: float, v_end: float, gap0: float, gap_end: float, tip_z: float, speed: float = 1.0, duration_s: float | None = None,
                     tilt_scale=(1.0, 1.0)):
        """닫으면서 손의 자세와 위치를 매 단계 다시 계산한 한 개의 경로: 두 턱 끝의 중간점이 center에 계속 머물고,
        기울기는 닫히는 정도에 맞춰 풀린다(고정 턱은 +m 쪽으로, 가동 턱은 -m 쪽으로 같이 다가온다). 열 때도 같은 경로를 쓴다. 못 풀면 None.
        tilt_scale: 단계마다 기울기에 곱할 배율. (k0, k1)이면 그리퍼 값에 비례해 선형으로 바뀌고(기본 (1, 1)은 그대로), 함수면 v -> k (REQ-053)."""
        n = max(2, math.ceil(abs(v0 - v_end) / 1.5))
        q, pts = q_start, [np.array(astuple(replace(q_start, gripper=v0)), float)]
        for i in range(1, n + 1):
            v = v0 + (v_end - v0) * i / n
            k = tilt_scale(v) if callable(tilt_scale) else tilt_scale[0] + (tilt_scale[1] - tilt_scale[0]) * i / n
            q = self.kin.ik(self.hand_pose(center.x_mm, center.y_mm, yaw_t, v, tilt_ok, tip_z, k), q, tol_mm=3.0)
            if q is None:
                return None
            pts.append(np.array(astuple(replace(q, gripper=v)), float))
        return LineTrajectory(np.array(pts), QuinticShape(duration_s if duration_s is not None else max(1.0, abs(gap0 - gap_end) / 16.0) / speed))  # 틈 16mm/s로 천천히(gripper_speed_pct로 더 늦춘다). 놓기는 duration_s로 시간을 직접 정한다


@functools.lru_cache(maxsize=4)
def _parallel_meshes(urdf_file: str) -> dict:
    """설정 urdf(병렬 그리퍼 URDF)의 손 본체·클램프 두 개의 볼록 꼭짓점(gripper_link 프레임, m). 클램프는 닫힌(관절 0) 자리.
    링크 이름 gripper_link·clamp_right·clamp_left와 관절 gripper·gripper_left를 기대한다(docs/planning/arm-dev-status.md 22절)."""
    import xml.etree.ElementTree as ET

    import trimesh
    urdf = Path(urdf_file)
    urdf = urdf if urdf.is_absolute() else Path(__file__).resolve().parents[3] / urdf
    root = ET.parse(urdf).getroot()

    def vec(e, key):
        return np.array([float(x) for x in (e.get(key) if e is not None else "0 0 0").split()])

    out = {}
    for name, link, joint in (("hand", "gripper_link", None), ("right", "clamp_right", "gripper"), ("left", "clamp_left", "gripper_left")):
        vis = root.find(f"link[@name='{link}']/visual")
        mesh = vis.find("geometry/mesh")
        o = vis.find("origin")
        v = _stl_vertices(urdf.parent / mesh.get("filename")) * float(mesh.get("scale", "1 1 1").split()[0])
        v = v @ _rpy(*vec(o, "rpy")).T + vec(o, "xyz")
        if joint:
            v = v + vec(root.find(f"joint[@name='{joint}']/origin"), "xyz")
        out[name] = np.asarray(trimesh.convex.hull_points(v), float)
    return out


class ParallelGripper:
    """병렬 그리퍼(Robonine Parallel Gripper 등): 두 턱이 좌우 대칭으로 열린다 (결정 0043-1).
    TCP는 두 턱 중앙이라 손 위치 보정이 0, 기울기 0, 턱 간격은 높이에 따라 좁아지지 않는다(W <= G - 4). 닫기·열기는 그리퍼 축만 움직인다.
    폭<->서보값 표, 접촉·조임 기준, TCP에서 턱 끝까지 거리(gripper_tip_offset_mm)는 실물에서 재서 config/grippers/parallel.yaml에 채운다."""
    name = "parallel"
    needs_centering = False

    def __init__(self, kin, cfg=None):
        self.kin, self.cfg = kin, cfg

    def width_to_value(self, width_mm: float) -> float:
        pts = sorted((mm, v) for v, mm in self.cfg.gripper_width_mm)
        lo, hi = self.cfg.joint_limits_deg["gripper"]
        return float(np.clip(np.interp(width_mm, [p[0] for p in pts], [p[1] for p in pts]), lo, hi))

    def _gap(self, v: float) -> float:
        pts = sorted(self.cfg.gripper_width_mm)
        return float(np.interp(v, [p[0] for p in pts], [p[1] for p in pts]))

    @property
    def min_gap_mm(self) -> float:
        return min(mm for _, mm in self.cfg.gripper_width_mm)

    def jaw_tips(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float):
        """TCP(두 턱 중앙)에서 본 두 턱 끝: 도구 x축으로 -+간격/2, 도구 축(+z)으로 gripper_tip_offset_mm. 가동 턱 쪽(-x)을 앞에 둔다(단동식과 같은 순서)."""
        R = _rotation(pitch_deg, yaw_table_deg - self.kin.base.yaw_deg) @ self.kin._C
        c, s = math.cos(math.radians(self.kin.base.yaw_deg)), math.sin(math.radians(self.kin.base.yaw_deg))
        Rt = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R
        h, off = self._gap(gripper_value) / 2, self.cfg.gripper_tip_offset_mm if self.cfg else 0.0
        return Rt @ np.array([h, 0.0, off]), Rt @ np.array([-h, 0.0, off])

    def center_offset(self, pitch_deg: float, yaw_table_deg: float, gripper_value: float) -> np.ndarray:
        f, m = self.jaw_tips(pitch_deg, yaw_table_deg, gripper_value)
        return (f + m) / 2

    def set_joints(self, robot, v: float) -> None:
        """오른쪽 클램프(gripper)는 +간격/2, 왼쪽(gripper_left)은 -간격/2 (m). placo는 mimic을 몰라 둘 다 맞춘다. URDF에 없으면(단동식 URDF) 건너뛴다."""
        h = self._gap(v) / 2000.0
        names = set(robot.joint_names())
        for name, val in (("gripper", h), ("gripper_left", -h)):
            if name in names:
                robot.set_joint(name, val)

    def lowest_z(self, q: JointState) -> float:
        """손 본체와 두 클램프 메쉬의 가장 낮은 꼭짓점 높이(모델 z, mm). 클램프는 닫힌 자리에서 간격/2만큼 좌우로 벌린다 (assets/so101_parallel.urdf)."""
        meshes = _parallel_meshes(self.cfg.urdf)
        k = self.kin._k
        k.forward_kinematics(np.array([getattr(q, j) for j in ARM_JOINTS], float))
        T = k.robot.get_T_world_frame("gripper_link")
        h = self._gap(q.gripper) / 2000.0
        low = []
        for name, v in meshes.items():
            dx = {"hand": 0.0, "right": h, "left": -h}[name]
            low.append(float(((v @ T[2, :3]) + dx * T[2, 0]).min() + T[2, 3]))
        return min(low) * 1000.0

    def tilt_allowed(self, x_mm: float, y_mm: float, yaw_t: float) -> bool:
        return False

    def tilt(self, v: float) -> float:
        return 0.0

    def narrowing(self, v: float, tilt_ok: bool) -> float:
        return 0.0

    def hand_pose(self, ox: float, oy: float, yaw_t: float, v: float, tilt_ok: bool, tip_z: float, tilt_scale: float = 1.0) -> ToolPose:
        f, m = self.jaw_tips(-90.0, yaw_t, v)
        mid = (f[:2] + m[:2]) / 2  # 수직 하강이라 0
        return ToolPose(ox - mid[0], oy - mid[1], tip_z - min(f[2], m[2]), yaw_t, -90.0)

    def closing_path(self, q_start: JointState, center, yaw_t: float, tilt_ok: bool, v0: float, v_end: float, gap0: float, gap_end: float, tip_z: float, speed: float = 1.0, duration_s: float | None = None,
                     tilt_scale=(1.0, 1.0)):
        """손은 그대로 두고 그리퍼 축만 v0 -> v_end. 열 때도 같은 경로."""
        pts = np.array([np.array(astuple(replace(q_start, gripper=v0)), float), np.array(astuple(replace(q_start, gripper=v_end)), float)])
        return LineTrajectory(pts, QuinticShape(duration_s if duration_s is not None else max(1.0, abs(gap0 - gap_end) / 16.0) / speed))  # 틈 16mm/s로 천천히(gripper_speed_pct로 더 늦춘다). 놓기는 duration_s로 시간을 직접 정한다


def make_gripper(kin, cfg):
    """설정 gripper에 맞는 그리퍼 (config/grippers/<종류>.yaml)."""
    return {"single_jaw": SingleJawGripper, "parallel": ParallelGripper}[cfg.gripper](kin, cfg)
