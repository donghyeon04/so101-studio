"""장면 부품과 팔의 충돌 거리 (REQ-040, 결정 0043-5). coal로 팔 링크 메쉬(볼록 근사)와 장면 부품 사이 최단 거리를 잰다.
링크에 붙은 부품은 팔의 일부로 본다. 단위: 입출력 mm, 내부 coal은 m.
shortcut: 메쉬를 볼록 껍질로 근사한다(오목한 부품은 실제보다 크게 잡힌다: 안전한 쪽), 오목 부품을 정확히 보려면 BVH로 바꾼다."""
import functools
import math
import threading
import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import astuple
from pathlib import Path

import coal
import numpy as np
import trimesh
from lerobot.model.kinematics import RobotKinematics

from ..core import frames
from ..core.types import JointState
from .gripper import _rpy, _stl_vertices
from .kinematics import ARM_JOINTS, PAN_AXIS_X_MM


def _hull(vertices: np.ndarray):
    """꼭짓점(m) -> coal 볼록체."""
    v = coal.StdVec_Vec3s()
    for p in trimesh.convex.hull_points(vertices):
        v.append(np.asarray(p, float))
    return coal.Convex.convexHull(v, False, "")


def _T(T: np.ndarray):
    return coal.Transform3s(np.ascontiguousarray(T[:3, :3], float), np.ascontiguousarray(T[:3, 3], float))


@functools.lru_cache(maxsize=2)
def _shared(urdf_path: str):
    """(placo 로봇, 잠금, 링크 볼록체 목록)을 URDF 경로마다 한 번만 만든다."""
    return RobotKinematics(urdf_path, "gripper_frame_link", joint_names=ARM_JOINTS), threading.Lock(), SceneCollision._load_links(Path(urdf_path))


class SceneCollision:
    def __init__(self, kin, urdf_path: str, ws=None, table_z_mm: float = 0.0, z_floor_mm: float = 4.0):
        self.kin, self.ws, self.table_z_mm, self.z_floor_mm = kin, ws, table_z_mm, z_floor_mm
        # 제어 스레드(조그 IK)와 GUI 계획 스레드(check_path)가 kin._k를 같이 쓰므로 링크 변환은 kin과 따로 만든 placo 로봇으로 구하고, 그 로봇은 잠금으로 한 번에 하나만 쓴다.
        # URDF마다 한 번만 만들어 이 프로세스의 모든 SceneCollision이 나눠 쓴다(ArmService를 여러 번 만들어도 메모리가 늘지 않고 메쉬 읽기 1.2초도 한 번뿐)
        self._k, self._lock, self._links = _shared(str(urdf_path))
        self.scene = None
        self._parts: list = []  # 작업대 부품: (이름, coal 볼록체, 작업대 -> 부품 4x4 m)
        self._attached: list = []  # 링크 부품: (이름, 링크, coal 볼록체, 꼭짓점 m, 링크 -> 부품 4x4 m)
        self._ready = False
        self._memo: OrderedDict = OrderedDict()

    @staticmethod
    def _load_links(urdf: Path) -> list:
        """[(링크, coal 볼록체)]"""
        out = []
        for link in ET.parse(urdf).getroot().findall("link"):
            name = link.get("name")
            if name == "base_link":
                continue  # 받침은 움직이지 않는다
            for vis in link.findall("visual"):
                mesh = vis.find("geometry/mesh")
                if mesh is None:
                    continue
                o = vis.find("origin")
                xyz = [float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()]
                rpy = [float(x) for x in (o.get("rpy") if o is not None else "0 0 0").split()]
                v = _stl_vertices(urdf.parent / mesh.get("filename")) @ _rpy(*rpy).T + np.array(xyz)
                out.append((name, _hull(v)))
        return out

    def set_scene(self, scene, ws=None) -> None:
        """장면(또는 작업대 설정)이 바뀌면 다시 부른다."""
        with self._lock:
            self.scene, self._memo = scene, OrderedDict()
        if ws is not None:
            self.ws = ws
        self._parts, self._attached = [], []
        for p in scene.parts if scene else []:
            if not p.collide:
                continue
            v = np.asarray(p.mesh.vertices, float) / 1000.0
            T = p.T.copy()
            T[:3, 3] /= 1000.0
            if p.attach == "table":
                self._parts.append((p.name, _hull(v), T))
            else:
                self._attached.append((p.name, p.attach, _hull(v), v, T))

    def _table_to_world(self) -> np.ndarray:
        """작업대 프레임 -> placo 세계(m). 작업대 면은 모델 z = table_z_mm."""
        base = self.ws.base
        p0 = np.array(frames.table_to_robot(0.0, 0.0, base))
        px = np.array(frames.table_to_robot(1.0, 0.0, base)) - p0
        py = np.array(frames.table_to_robot(0.0, 1.0, base)) - p0
        T = np.eye(4)
        T[:2, 0], T[:2, 1] = px, py
        T[:3, 3] = [p0[0] + PAN_AXIS_X_MM, p0[1], self.table_z_mm]
        T[:3, 3] /= 1000.0
        return T

    def _pose(self, q: JointState):
        k = self._k
        arm = np.array([getattr(q, j) for j in ARM_JOINTS], float)
        for _ in range(1 if self._ready else 2):  # placo는 gripper 관절이 처음 들어간 첫 갱신에서 링크 변환이 낡은 채 나온다 (gripper.lowest_z와 같다)
            k.forward_kinematics(arm)
            self.kin.gripper.set_joints(k.robot, q.gripper)  # 그리퍼 종류마다 관절이 다르다(회전 rad | 직선 m)
            k.robot.update_kinematics()
        self._ready = True
        return k.robot

    def distance(self, q: JointState) -> tuple[float, str | None, str | None, float]:
        """(작업대 부품까지 가장 가까운 거리 mm, 그 부품 이름, 가장 낮게 내려온 링크 부품 이름, 그 부품의 작업대 면 위 최저 높이 mm).
        부품이 없으면 (inf, None, None, inf). 같은 자세는 기억해 둔다(check_path가 ref를 반복해 묻는다)."""
        key = astuple(q)
        with self._lock:
            if key in self._memo:
                self._memo.move_to_end(key)
                return self._memo[key]
            out = self._distance(q)
            self._memo[key] = out
            if len(self._memo) > 256:
                self._memo.popitem(last=False)  # 가장 오래 안 쓴 하나만 버린다
            return out

    def _distance(self, q: JointState) -> tuple[float, str | None, str | None, float]:
        if not self._parts and not self._attached:
            return math.inf, None, None, math.inf
        robot = self._pose(q)
        world = {}

        def link_T(name):
            if name not in world:
                world[name] = robot.get_T_world_frame(name)
            return world[name]

        geoms = [(h, _T(link_T(n))) for n, h in self._links]
        low_name, low_z = None, math.inf
        for name, link, h, v, T in self._attached:  # 링크에 붙은 부품은 팔의 일부
            Tw = link_T(link) @ T
            geoms.append((h, _T(Tw)))
            z = float(((v @ Tw[2, :3] + Tw[2, 3]) * 1000.0).min()) - self.table_z_mm
            if z < low_z:
                low_name, low_z = name, z
        T_tw = self._table_to_world()
        req, res = coal.DistanceRequest(), coal.DistanceResult()
        best, who = math.inf, None
        for name, ph, T in self._parts:
            Tp = _T(T_tw @ T)
            for h, Tg in geoms:
                res.clear()
                d = coal.distance(h, Tg, ph, Tp, req, res) * 1000.0
                if d < best:
                    best, who = d, name
        return best, who, low_name, low_z
