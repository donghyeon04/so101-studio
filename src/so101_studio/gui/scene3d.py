"""Viser 3D 장면: URDF 팔 두 개(실제, 계산 자세), 작업대, 물체, 배출 구역, 좌표축 (gui-spec 4·8절).
장면 좌표는 작업대 좌표를 m로 옮긴 것이고, z = 0이 작업대 면이다."""

import math
from pathlib import Path

import numpy as np
import trimesh
import viser
import viser.transforms as tf
import yourdfpy
from viser.extras import ViserUrdf

from ..arm.kinematics import ARM_JOINTS, PAN_AXIS_X_MM, VALUE_TO_Q
from ..core.frames import robot_to_table
from . import theme

HOST = "127.0.0.1"
DEFAULT_PORT = 8081
TABLE_T_M = 0.01
OBJECT_H_M = 0.03
PREVIEW_ALPHA = 0.4
MESH_CELL_M = 0.002
DEFAULT_PICK_H_MM = 40.0  # 작업대를 클릭했을 때의 높이. 0이면 턱 끝이 작업대에 닿는다
# (카메라 위치, 보는 점, 위쪽). 작업대 600x400mm 기준
VIEWS = {
    "위": ((0.3, 0.2, 0.55), (0.3, 0.2, 0.0), (0.0, 1.0, 0.0)),
    "앞": ((0.3, -0.45, 0.12), (0.3, 0.2, 0.1), (0.0, 0.0, 1.0)),
    "옆": ((-0.4, 0.2, 0.12), (0.3, 0.2, 0.1), (0.0, 0.0, 1.0)),
}


def table_hit(origin, direction) -> tuple[float, float] | None:
    """클릭 광선과 작업대 면(z = 0)의 교점을 작업대 mm로. 면을 향하지 않으면 None."""
    if direction[2] >= 0 or origin[2] <= 0:
        return None
    t = -origin[2] / direction[2]
    return (origin[0] + t * direction[0]) * 1000, (origin[1] + t * direction[1]) * 1000


def urdf_cfg(q, names) -> np.ndarray:
    """JointState(도, 그리퍼는 값)를 URDF 관절각(rad)으로. names는 URDF의 관절 순서."""
    deg = {j: getattr(q, j) for j in ARM_JOINTS} | {"gripper": VALUE_TO_Q[0] + VALUE_TO_Q[1] * q.gripper}
    return np.radians([deg[n] for n in names])


def simplify(mesh, cell_m: float = MESH_CELL_M) -> trimesh.Trimesh:
    """격자 한 칸 안의 꼭짓점을 하나로 합친다. 원본 STL은 면이 40만 개라 브라우저로 보내는 데 오래 걸린다."""
    cells, index = np.unique(np.round(mesh.vertices / cell_m).astype(np.int64), axis=0, return_inverse=True)
    index = index.ravel()
    vertices = np.zeros((len(cells), 3))
    np.add.at(vertices, index, mesh.vertices)
    vertices /= np.bincount(index)[:, None]
    faces = index[mesh.faces]
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return trimesh.Trimesh(vertices, faces[keep], process=False, face_colors=mesh.visual.main_color)


def _load_urdf(path: Path) -> yourdfpy.URDF:
    """팔마다 따로 읽는다. ViserUrdf가 관절값을 URDF 객체에 써서, 나눠 쓰면 스레드끼리 섞인다."""
    urdf = yourdfpy.URDF.load(path, build_scene_graph=True, load_meshes=True, mesh_dir=str(path.parent))
    for name, mesh in list(urdf.scene.geometry.items()):
        urdf.scene.geometry[name] = simplify(mesh)
    return urdf


def _yaw(deg: float) -> np.ndarray:
    return tf.SO3.from_z_radians(math.radians(deg)).wxyz


class Scene3D:
    def __init__(self, cfg, ws, kin, port: int = DEFAULT_PORT):
        self.server = viser.ViserServer(host=HOST, port=port, label="SO-101 Studio", verbose=False)
        self.server.gui.configure_theme(
            dark_mode=False, show_logo=False, show_share_button=False, brand_color=theme.rgb(theme.ACCENT)
        )
        self.server.gui.main_panel.minimize()  # 조작은 NiceGUI 쪽에서 한다. 이 패널은 3D를 가리기만 한다
        self._kin, self._table_z = kin, cfg.table_z_mm
        scene = self.server.scene
        w, h = ws.table_mm["x"] / 1000, ws.table_mm["y"] / 1000
        scene.add_box(
            "/table", color=theme.rgb(theme.CARD), dimensions=(w, h, TABLE_T_M), position=(w / 2, h / 2, -TABLE_T_M / 2)
        )
        scene.add_grid(
            "/grid", width=w, height=h, cell_size=0.05, section_size=0.1, position=(w / 2, h / 2, 0.0005),
            cell_color=theme.rgb(theme.LINE), section_color=theme.rgb(theme.CONTROL_BORDER),
        )
        for name, zone in ws.drop_zones.items():
            scene.add_box(
                f"/zones/{name}", color=theme.rgb(theme.OK), dimensions=(0.08, 0.08, 0.001), opacity=0.35,
                position=(zone["x_mm"] / 1000, zone["y_mm"] / 1000, 0.001),
            )

        base = ws.base
        scene.add_frame("/axes/table", axes_length=0.06, axes_radius=0.002)
        scene.add_frame(
            "/axes/robot", axes_length=0.06, axes_radius=0.002,
            position=(base.x_mm / 1000, base.y_mm / 1000, 0.001), wxyz=_yaw(base.yaw_deg),
        )
        self._tool = scene.add_frame("/axes/tool", axes_length=0.04, axes_radius=0.0015)

        # URDF 원점은 어깨 회전축에서 PAN_AXIS_X_MM 뒤에 있다 (Kinematics.fk와 같은 오프셋)
        ox, oy = robot_to_table(-PAN_AXIS_X_MM, 0.0, base)
        root = {"position": (ox / 1000, oy / 1000, -self._table_z / 1000), "wxyz": _yaw(base.yaw_deg)}
        urdf_path = Path(cfg.urdf)
        scene.add_frame("/real", show_axes=False, **root)
        self._real = ViserUrdf(self.server, _load_urdf(urdf_path), root_node_name="/real")
        self._preview_root = scene.add_frame("/preview", show_axes=False, visible=False, **root)
        self._preview = ViserUrdf(
            self.server, _load_urdf(urdf_path), root_node_name="/preview",
            mesh_color_override=(*(c / 255 for c in theme.rgb(theme.PREVIEW)), PREVIEW_ALPHA),
        )
        self._names = self._real.get_actuated_joint_names()
        self.picked = (0, 0.0, 0.0, 0.0)  # (번호, x, y, 높이). 번호가 바뀌면 새 점이다
        self._handle = scene.add_transform_controls(
            "/handle", scale=0.09, disable_rotations=True, depth_test=False, visible=True
        )
        self._handle.on_update(self._on_drag)
        scene.on_click()(self._on_click)
        cam = self.server.initial_camera
        cam.position, cam.look_at = (w / 2, -0.22, 0.34), (w / 2, h / 2 - 0.02, 0.06)

    @property
    def url(self) -> str:
        return f"http://{HOST}:{self.server.get_port()}"

    def update_status(self, status) -> None:
        """status_stream 콜백. 실제 팔과 툴 좌표축을 지금 자세로 옮긴다."""
        t = status.tool
        with self.server.atomic():
            self._real.update_cfg(urdf_cfg(status.joints, self._names))
            self._tool.position = (t.x_mm / 1000, t.y_mm / 1000, (t.z_mm - self._table_z) / 1000)
            self._tool.wxyz = tf.SO3.from_matrix(self._kin.tool_axes(status.joints)).wxyz

    def set_preview(self, joints) -> None:
        """계산 자세를 보여 준다. None이면 숨긴다."""
        self._preview_root.visible = joints is not None
        if joints is not None:
            self._preview.update_cfg(urdf_cfg(joints, self._names))

    def set_objects(self, objects) -> None:
        """물체 목록을 다시 그린다. objects는 DetectedObject들이다."""
        scene = self.server.scene
        with self.server.atomic():
            scene.remove_by_name("/objects")
            scene.add_frame("/objects", show_axes=False)
            for o in objects:
                scene.add_box(
                    f"/objects/{o.id}", color=theme.rgb(theme.MUTED),
                    dimensions=(o.length_mm / 1000, o.width_mm / 1000, OBJECT_H_M),
                    position=(o.x_mm / 1000, o.y_mm / 1000, OBJECT_H_M / 2), wxyz=_yaw(o.angle_deg),
                )

    def pick(self, x_mm: float, y_mm: float, h_mm: float) -> None:
        """작업대 클릭이나 손잡이로 고른 점을 남긴다. Viser 스레드에서 불리고, 화면 타이머가 picked로 읽는다."""
        self.picked = (self.picked[0] + 1, x_mm, y_mm, h_mm)

    def _on_click(self, event) -> None:
        hit = table_hit(event.ray_origin, event.ray_direction)
        if hit is not None:
            self.pick(hit[0], hit[1], DEFAULT_PICK_H_MM)

    def _on_drag(self, _event) -> None:
        x, y, z = np.asarray(self._handle.position) * 1000
        self.pick(float(x), float(y), max(float(z), 0.0))

    def show_target(self, x_mm: float, y_mm: float, h_mm: float, ok: bool, tool_path=None) -> None:
        """고른 점(닿으면 강조색, 안 닿으면 위험색)과 손끝 경로(점선)를 그린다. tool_path는 preview_path의 tool 목록이다."""
        scene = self.server.scene
        position = (x_mm / 1000, y_mm / 1000, h_mm / 1000)
        with self.server.atomic():
            scene.add_icosphere(
                "/target", radius=0.008, position=position, color=theme.rgb(theme.ACCENT if ok else theme.DANGER)
            )
            self._handle.position = position
            scene.remove_by_name("/path")
            if tool_path:
                points = np.array([(p["x_mm"], p["y_mm"], p["z_mm"] - self._table_z) for p in tool_path]) / 1000
                dashes = np.stack([points[:-1:2], points[1::2][: len(points[:-1:2])]], axis=1)
                if len(dashes):
                    scene.add_line_segments(
                        "/path", dashes, theme.rgb(theme.PREVIEW), thickness=3.0, thickness_units="screen"
                    )

    def clear_target(self) -> None:
        scene = self.server.scene
        with self.server.atomic():
            scene.remove_by_name("/target")
            scene.remove_by_name("/path")
            self.set_preview(None)

    def set_view(self, name: str) -> None:
        """시점을 위/앞/옆으로 옮긴다. 열려 있는 3D 칸 모두에 적용된다."""
        position, look_at, up = VIEWS[name]
        for client in self.server.get_clients().values():
            with client.atomic():
                client.camera.up_direction = up
                client.camera.position = position
                client.camera.look_at = look_at

    def close(self) -> None:
        self.server.stop()
