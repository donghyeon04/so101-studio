"""MuJoCo 로보나인 병렬 손 (mujoco-spec 13.10 M4b, 결정 0063).

menagerie 팔(어깨~손목)은 그대로 두고 단동식 손(고정 턱·움직이는 턱·손목 서보 메쉬)을 MjSpec에서 지운 뒤, 로보나인 손 본체와 클램프 둘(slide)을 붙인다.
좌표·치수는 assets/so101_parallel.urdf와 assets/assets/parallel/*.stl(gripper_link 프레임 mm)에서 그대로 읽는다. MuJoCo는 URDF mimic을 무시하므로 왼쪽 클램프는 equality joint로 오른쪽의 반대 방향에 묶는다.
충돌은 메쉬의 볼록 껍질이 실제보다 4~6.5배 커서 쓰지 않고, 정점을 z로 자른 조각(slab)의 경계 상자로 근사한다(시각은 원본 메쉬). 값 중 `HAND_*`·`CLAMP_*`는 실물 측정 전의 잠정값이다(runbook 2-10)."""
from functools import lru_cache
from itertools import pairwise

import numpy as np

from .sim_driver import REPO_ROOT

PARALLEL_DIR = REPO_ROOT / "assets" / "assets" / "parallel"
NOCAM_STL = REPO_ROOT / "assets" / "mujoco" / "parallel" / "parallel_hand_nocam.stl"  # parallel_hand.stl에서 손목 카메라 팔·렌즈를 뺀 파생(원본은 그대로)
CAMERA_Y_GL_MM = -36.0  # gripper_link y가 이보다 작은(-y쪽 끝) 부분이 카메라 마운트·카메라: 손 본체(서보·판)는 y -35 안쪽
_RX90 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])  # URDF visual rpy (1.570796, 0, 0)
# (메쉬 파일, visual origin xyz m): 손 본체는 gripper_link, 클램프는 각 관절 origin에서 본 값
HAND_MESH = ("parallel_hand", (-0.007062, 0.152066, 0.120627))  # visual origin(URDF). 메쉬 자체는 NOCAM_STL을 쓴다
CLAMPS = {  # 이름 -> (메쉬, visual origin, 관절 origin(gripper_link, m), 축 x에서 열리는 방향 부호)
    "clamp_right": ("clamp_right", (0.045072, -0.033971, 0.340704), (-0.052134, 0.186037, -0.220077), 1.0),
    "clamp_left": ("clamp_left", (-0.07989, -0.031809, 0.342905), (0.072828, 0.183875, -0.222278), -1.0),
}
STROKE_M = 0.042  # 클램프 한 쪽 최대 이동(URDF limit upper)
CLAMP_MASS_KG = 0.03  # URDF 값(관성 1e-5는 임시)
CLAMP_SLAB_MM = 10.0
HAND_SLABS = 3
SLIDE_DAMPING = 1.0
# 액추에이터(잠정): 힘 한계 10N은 URDF effort. kp는 1mm 오차에서 한계, kv는 30g 두 개를 임계 감쇠 근처로. 실물 조임 힘 측정(runbook 2-10) 뒤 바꾼다
GRIP_FORCE_N = 10.0
GRIP_KP = 10000.0
GRIP_KV = 20.0
EQ_SOLREF = (0.004, 1.0)  # 두 클램프가 접촉 때 한쪽만 밀리지 않게 단단하게(시간 상수 2 스텝)
PARALLEL_FRAME_POS = (0.00044, 0.00017, -0.1218)  # URDF gripper_frame_joint origin (gripper_link, m), rpy (0, pi, 0)
PARALLEL_FRAME_QUAT = (0.0, 0.0, 1.0, 0.0)


def slide_m(value: float, width_mm) -> float:
    """우리 그리퍼 값(0~100) -> 클램프 한 쪽 이동(m). 폭 표 [[값, 폭mm], …]에서 보간하고 반으로 나눈다(두 클램프가 같은 만큼 벌어진다)."""
    pts = sorted(width_mm)
    return float(np.interp(value, [p[0] for p in pts], [p[1] for p in pts])) / 2000.0


def value_of_slide(q: float, width_mm) -> float:
    """slide_m의 역."""
    pts = sorted((w, v) for v, w in width_mm)
    return float(np.interp(q * 2000.0, [p[0] for p in pts], [p[1] for p in pts]))


def make_nocam_stl(out=NOCAM_STL):
    """parallel_hand.stl에서 gripper_link y < CAMERA_Y_GL_MM인 부분(카메라 마운트·렌즈 달린 카메라)을 평면으로 잘라 파생 STL로 쓴다(경계를 걸친 삼각형도 평면에서 자른다: 면 단위로 빼면 y −43까지 남아 브래킷과 2mm 겹쳤다). y_gl(mm) = 152.066 − z_stl(URDF visual rpy 1.570796, origin y 0.152066). (잘라낸 면 수, 남은 면 수)를 돌려준다."""
    import trimesh

    m = trimesh.load(PARALLEL_DIR / "parallel_hand.stl", force="mesh")
    n0 = len(m.faces)
    sub = m.slice_plane([0, 0, 152.066 - CAMERA_Y_GL_MM], [0, 0, -1], cap=False)  # z_stl가 이보다 작은 쪽(= y_gl가 큰 쪽)을 남긴다
    out.parent.mkdir(parents=True, exist_ok=True)
    sub.export(out)
    return n0 - len(sub.faces), len(sub.faces)


def _load(mesh: str, vis):
    """STL(mm) -> 그 링크 body 프레임(m): URDF visual rpy (1.570796, 0, 0)와 origin을 적용한다. 손 본체의 body는 gripper_link, 클램프의 body는 관절 origin(링크 프레임)이다."""
    import trimesh

    m = trimesh.load(NOCAM_STL if mesh == "parallel_hand" else PARALLEL_DIR / f"{mesh}.stl", force="mesh")
    m.apply_scale(0.001)
    m.vertices = m.vertices @ _RX90.T + np.array(vis)
    return m


@lru_cache(maxsize=1)
def hand_geometry() -> dict:
    """{'hand'|'clamp_right'|'clamp_left': {verts, faces, boxes}} — 정점·면과 충돌 상자 [(중심, 반길이)](body 프레임 m). STL 읽기·계산은 한 번만."""
    out = {}
    m = _load(*HAND_MESH)
    v = m.vertices
    out["hand"] = {"verts": v, "faces": m.faces, "boxes": _slabs(v, np.linspace(float(v[:, 2].min()), float(v[:, 2].max()), HAND_SLABS + 1))}
    for name, (mesh, vis, _joint, _sign) in CLAMPS.items():
        m = _load(mesh, vis)
        v = m.vertices
        z0, z1 = float(v[:, 2].min()), float(v[:, 2].max())
        out[name] = {"verts": v, "faces": m.faces, "boxes": _slabs(v, np.linspace(z0, z1, max(1, round((z1 - z0) * 1000.0 / CLAMP_SLAB_MM)) + 1))}
    return out


def _slabs(verts: np.ndarray, edges: np.ndarray) -> list:
    boxes = []
    for lo, hi in pairwise(edges):
        sel = verts[(verts[:, 2] >= lo - 1e-9) & (verts[:, 2] <= hi + 1e-9)]
        if len(sel) == 0:
            continue
        b0, b1 = sel.min(axis=0), sel.max(axis=0)
        boxes.append(((b0 + b1) / 2.0, (b1 - b0) / 2.0))
    return boxes


def attach_parallel_hand(spec, width_mm, jaw_condim: int, jaw_friction) -> None:
    """spec의 단동식 손을 로보나인 손으로 바꾼다. 관절 이름 gripper(slide, m)·액추에이터 gripper는 그대로 쓴다. 같은 이름의 단동식 관절·액추에이터를 지운다."""
    import mujoco

    geo = hand_geometry()
    spec.delete(spec.actuator("gripper"))
    spec.delete(spec.body("moving_jaw_so101_v1"))
    gripper = spec.body("gripper")
    for g in list(gripper.geoms):  # 고정 턱·손목 서보 메쉬·상자 전부(로보나인 손 메쉬가 서보까지 포함)
        spec.delete(g)

    def add_mesh_geoms(body, key: str, collide_kw: dict) -> None:
        d = geo[key]
        spec.add_mesh(name=f"hand_mesh:{key}", uservert=np.asarray(d["verts"], float).reshape(-1).tolist(), userface=np.asarray(d["faces"], int).reshape(-1).tolist())
        body.add_geom(name=f"hand_visual:{key}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"hand_mesh:{key}", contype=0, conaffinity=0, group=2, rgba=[0.75, 0.75, 0.78, 1.0], mass=0)
        for i, (c, h) in enumerate(d["boxes"]):
            body.add_geom(name=f"hand_col:{key}:{i}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=c.tolist(), size=np.maximum(h, 1e-4).tolist(), group=3, mass=0, **collide_kw)

    add_mesh_geoms(gripper, "hand", {})
    jaw_kw = {"priority": 1, "condim": jaw_condim, "friction": list(jaw_friction)}
    for name, (_m, _v, joint, sign) in CLAMPS.items():
        b = gripper.add_body(name=name, pos=list(joint))
        b.explicitinertial = 1
        b.mass = CLAMP_MASS_KG
        b.inertia = [1e-5, 1e-5, 1e-5]
        j = b.add_joint(name="gripper" if sign > 0 else "gripper_left", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], damping=SLIDE_DAMPING,
                        range=[-0.0005, STROKE_M + 0.0005] if sign > 0 else [-STROKE_M - 0.0005, 0.0005])
        j.limited = mujoco.mjtLimited.mjLIMITED_TRUE
        add_mesh_geoms(b, name, jaw_kw)
    spec.add_equality(name="clamp_mimic", type=mujoco.mjtEq.mjEQ_JOINT, name1="gripper_left", name2="gripper", data=[0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], solref=list(EQ_SOLREF))
    spec.add_exclude(name="clamps", bodyname1="clamp_right", bodyname2="clamp_left")  # 닫힌 자리에서 두 클램프가 0.2mm 겹쳐 있다(URDF 값): 서로의 접촉은 뺀다
    a = spec.add_actuator(name="gripper", target="gripper", trntype=mujoco.mjtTrn.mjTRN_JOINT)
    a.gaintype, a.biastype = mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = GRIP_KP
    a.biasprm[:3] = [0.0, -GRIP_KP, -GRIP_KV]
    a.ctrllimited = a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
    a.ctrlrange = [-0.0005, STROKE_M + 0.0005]
    a.forcerange = [-GRIP_FORCE_N, GRIP_FORCE_N]
    a.group = 0
