"""MuJoCo 모델 읽기 (arm-software-spec 13.1·13.4, 결정 0053, M1).

menagerie `robotstudio_so101`(assets/mujoco/so101/so101.xml, 원본 그대로)을 MjSpec으로 읽어 작업대·장면 부품·물체를 더하고, 턱 충돌 geom의 접촉값과 전역 옵션을 정한 `MjModel`을 만든다.
월드 좌표는 URDF `base_link` 프레임(m)이라 우리 기구학 모델 좌표와 같다. 좌표 변환 두 함수와 관절 대응표 `JOINT_MAP`은 이 파일의 상수·공개 함수다.
`kinematics.py`·`collision.py`·`scene3d.py` 안의 같은 계산은 바꾸지 않는다: tests/test_mujoco_model.py가 두 경로가 같은지 확인한다."""
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..core import frames
from .kinematics import PAN_AXIS_X_MM, VALUE_TO_Q

MODEL_XML = Path(__file__).resolve().parents[3] / "assets" / "mujoco" / "so101" / "so101.xml"

# 우리 관절 -> (MuJoCo 관절 이름, 부호, 오프셋 도). MuJoCo 각도(도) = 부호 * 우리 각도 + 오프셋. 값은 menagerie 관절 이름과 tests/test_mujoco_model.py(gripper_frame 위치 대조)로 정했다.
# 그리퍼는 우리 값 v가 아니라 관절각 q = VALUE_TO_Q[0] + VALUE_TO_Q[1]*v(도)에 같은 식을 적용한다(`gripper_value_to_deg`).
JOINT_MAP = {
    "shoulder_pan": ("shoulder_pan", 1.0, 0.0),
    "shoulder_lift": ("shoulder_lift", 1.0, 0.0),
    "elbow_flex": ("elbow_flex", 1.0, 0.0),
    "wrist_flex": ("wrist_flex", 1.0, 0.0),
    "wrist_roll": ("wrist_roll", 1.0, 0.0),
    "gripper": ("gripper", 1.0, 0.0),
}

# gripper_frame site: 우리 URDF gripper_frame_joint의 origin을 그대로 (gripper_link 좌표, m / rpy (0, pi, 0) = y축으로 180°)
GRIPPER_FRAME_POS = (-0.0079, -0.000218121, -0.0981274)
GRIPPER_FRAME_QUAT = (0.0, 0.0, 1.0, 0.0)  # (w, x, y, z)

# 받침 충돌 geom: 원본 menagerie base body에는 충돌 geom이 없다(전부 visual). 받침 visual 메쉬들의 경계 상자(base 프레임, m)를 상자로 더한다.
BASE_BOX_POS = (0.0136, 0.0, 0.0238)
BASE_BOX_HALF = (0.0699, 0.0555, 0.0680)

# 장면 부품이 붙는 URDF 링크 -> MJCF body 이름
LINK_BODY = {
    "base_link": "base", "shoulder_link": "shoulder", "upper_arm_link": "upper_arm", "lower_arm_link": "lower_arm",
    "wrist_link": "wrist", "gripper_link": "gripper", "moving_jaw_so101_v1_link": "moving_jaw_so101_v1",
}
JAW_BODIES = {"jaw_fixed": "gripper", "jaw_moving": "moving_jaw_so101_v1"}  # 접촉 보고의 역할 이름 -> body
JAW_CONDIM = 4
JAW_FRICTION = (1.0, 0.02, 0.001)
DEFAULT_OBJECT_HEIGHT_MM = 40.0
TABLE_FRICTION = (1.0, 0.005, 0.0001)


@dataclass
class SimObject:
    """MuJoCo 월드에 놓는 상자 물체 (작업대 좌표 mm, 높이는 상자 바닥에서 위)."""
    name: str
    x_mm: float
    y_mm: float
    angle_deg: float = 0.0
    width_mm: float = 30.0
    length_mm: float = 100.0
    height_mm: float = 0.0  # 0 = 모름: 상자 높이 DEFAULT_OBJECT_HEIGHT_MM
    mass_g: float = 30.0


def gripper_value_to_deg(v: float) -> float:
    """우리 그리퍼 값(0~100) -> URDF/MuJoCo 그리퍼 관절각(도)."""
    return VALUE_TO_Q[0] + VALUE_TO_Q[1] * v


def table_to_model_mm(x_mm: float, y_mm: float, h_mm: float, base: frames.Pose2D, table_z_mm: float) -> tuple[float, float, float]:
    """작업대 점(작업 면 위 높이 h) -> 모델(base_link) 좌표 mm. frames.table_to_robot로 pan 축 원점의 로봇 좌표로 바꾸고 x에 PAN_AXIS_X_MM를 더하고 z = table_z + h."""
    xr, yr = frames.table_to_robot(x_mm, y_mm, base)
    return xr + PAN_AXIS_X_MM, yr, table_z_mm + h_mm


def model_to_table_mm(x_mm: float, y_mm: float, z_mm: float, base: frames.Pose2D, table_z_mm: float) -> tuple[float, float, float]:
    """table_to_model_mm의 역: 모델 좌표 mm -> (작업대 x, y, 작업 면 위 높이)."""
    x, y = frames.robot_to_table(x_mm - PAN_AXIS_X_MM, y_mm, base)
    return x, y, z_mm - table_z_mm


def _quat_z(deg: float) -> np.ndarray:
    h = math.radians(deg) / 2.0
    return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])


def _mat_quat(R: np.ndarray) -> np.ndarray:
    import mujoco

    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def load_model(scene=None, objects=(), *, base: frames.Pose2D, table_z_mm: float):
    """MjModel을 만든다: 원본 so101.xml + 받침 상자 + gripper_frame site + 작업대 평면 + 장면 부품 + 물체, 턱 충돌 geom 접촉값(condim 4, 마찰), 전역 옵션(타원 원뿔, impratio 10, implicitfast, timestep 2ms).
    scene는 core.scene.Scene 또는 None, objects는 SimObject 목록(이름이 서로 달라야 한다). `mujoco`가 없으면 ImportError."""
    import mujoco

    spec = mujoco.MjSpec.from_file(str(MODEL_XML))
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.timestep = 0.002

    gripper = spec.body("gripper")
    gripper.add_site(name="gripper_frame", pos=list(GRIPPER_FRAME_POS), quat=list(GRIPPER_FRAME_QUAT), group=3)
    spec.body("base").add_geom(name="base_box", type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(BASE_BOX_POS), size=list(BASE_BOX_HALF), group=3, rgba=[0.5, 0.5, 0.5, 0.0])

    for body_name in JAW_BODIES.values():  # 턱 충돌 geom(원본의 collision_gripper 클래스, group 3·4)의 접촉값만 고친다
        for g in spec.body(body_name).geoms:
            if g.group in (3, 4) and g.name != "base_box":
                g.condim = JAW_CONDIM
                g.friction = list(JAW_FRICTION)

    world = spec.worldbody
    table_z = table_z_mm / 1000.0
    world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[0, 0, table_z], size=[2.0, 2.0, 0.01], friction=list(TABLE_FRICTION), rgba=[0.8, 0.8, 0.8, 1.0])

    if scene is not None:
        _add_scene(spec, scene, base, table_z_mm)
    seen = set()
    for o in objects:
        if o.name in seen:
            raise ValueError(f"물체 이름이 겹친다: {o.name!r}")
        seen.add(o.name)
        _add_object(spec, o, base, table_z_mm)
    return spec.compile()


def _add_object(spec, o: SimObject, base, table_z_mm: float) -> None:
    import mujoco

    height = o.height_mm if o.height_mm > 0 else DEFAULT_OBJECT_HEIGHT_MM
    x, y, z = table_to_model_mm(o.x_mm, o.y_mm, height / 2.0, base, table_z_mm)  # 상자 가운데 높이(바닥이 작업 면에 닿게)
    b = spec.worldbody.add_body(name=o.name, pos=[x / 1000, y / 1000, z / 1000], quat=list(_quat_z(o.angle_deg - base.yaw_deg)))
    b.add_freejoint()
    b.add_geom(name=f"{o.name}_box", type=mujoco.mjtGeom.mjGEOM_BOX, size=[o.length_mm / 2000, o.width_mm / 2000, height / 2000], mass=o.mass_g / 1000.0,
               friction=[1.0, 0.005, 0.0001], rgba=[0.9, 0.4, 0.1, 1.0])


def _add_scene(spec, scene, base, table_z_mm: float) -> None:
    """장면 부품: attach가 table이면 월드 고정 body, 링크면 그 링크 body의 자식. collide: false는 보이기만(contype=0 conaffinity=0). box 또는 메쉬(STL, mm)."""
    import mujoco

    for p in scene.parts:
        T = np.asarray(p.T, float)
        if p.attach == "table":
            pos = np.array(table_to_model_mm(T[0, 3], T[1, 3], T[2, 3], base, table_z_mm)) / 1000.0
            quat = _mat_quat(_rotz(-base.yaw_deg) @ T[:3, :3])  # 작업대 축 -> 모델 축 회전(로봇 yaw만큼)
            parent = spec.worldbody
        else:
            body = LINK_BODY.get(p.attach)
            if body is None:
                raise ValueError(f"장면 부품 {p.name}: 알 수 없는 링크 {p.attach!r}")
            pos, quat, parent = T[:3, 3] / 1000.0, _mat_quat(T[:3, :3]), spec.body(body)
        b = parent.add_body(name=f"scene:{p.name}", pos=list(pos), quat=list(quat))
        lo, hi = p.mesh.bounds
        kw = {} if p.collide else {"contype": 0, "conaffinity": 0}
        if len(p.mesh.faces) <= 12:  # 상자 메쉬(12면)는 상자 geom으로
            half = (hi - lo) / 2000.0
            center = (hi + lo) / 2000.0
            b.add_geom(name=f"scene:{p.name}_geom", type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(center), size=list(half), **kw)
        else:
            mesh_name = f"scene_mesh:{p.name}"
            spec.add_mesh(name=mesh_name, uservert=(p.mesh.vertices / 1000.0).astype(float).reshape(-1).tolist(), userface=p.mesh.faces.astype(int).reshape(-1).tolist())
            b.add_geom(name=f"scene:{p.name}_geom", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh_name, **kw)


def _rotz(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
